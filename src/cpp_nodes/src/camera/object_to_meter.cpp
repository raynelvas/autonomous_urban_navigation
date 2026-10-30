#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp> 
#include <Eigen/Dense>       

#include "custom_interfaces/msg/detection_array.hpp"
#include "custom_interfaces/msg/detection_meter_array.hpp" 
#include "custom_interfaces/msg/detection_meter.hpp"
#include "custom_interfaces/msg/lane_model.hpp" 
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <std_msgs/msg/bool.hpp>

using json = nlohmann::json;

class DetectorToCloudNode : public rclcpp::Node {
public:
    DetectorToCloudNode() : Node("detector_to_cloud_transform") {        
        this->declare_parameter("config_path", "");
        std::string config_path = this->get_parameter("config_path").as_string();
        load_calibration(config_path);

        // --- SPEED TABLE LOAD ---
        for (int i = 0; i <= 9; ++i) {
            std::string param_name = "speed_configs.class_" + std::to_string(i);
            this->declare_parameter(param_name, std::vector<double>({}));
            
            auto values = this->get_parameter(param_name).as_double_array();
            if (values.size() == 7) {
                //[danger_d_min, danger_d_max, danger_v_min, occupation_min, warning_d_min, warning_d_max, warning_v_min]
                speed_map_[i] = {values[0], values[1], values[2], values[3], values[4], values[5], values[6]};
            }
        }

        this->declare_parameter("temporal_config.threshold_confirm", 3);
        this->declare_parameter("temporal_config.max_miss_frames", 2);
        this->declare_parameter("temporal_config.dist_threshold", 0.8);
        this->declare_parameter("temporal_config.inconsistency_threshold", 2);

        threshold_confirm_ = this->get_parameter("temporal_config.threshold_confirm").as_int();
        max_miss_frames_ = this->get_parameter("temporal_config.max_miss_frames").as_int();
        dist_threshold_ = this->get_parameter("temporal_config.dist_threshold").as_double();
        inconsistency_threshold_ = this->get_parameter("temporal_config.inconsistency_threshold").as_int();
        
        this->declare_parameter("min_distance", 0.0);
        this->declare_parameter("max_distance", 30.0);
        min_dist_ = this->get_parameter("min_distance").as_double();
        max_dist_ = this->get_parameter("max_distance").as_double();

        // 1. Detection Subscription
        sub_detections_ = this->create_subscription<custom_interfaces::msg::DetectionArray>(
            "/detection/results", 10,
            std::bind(&DetectorToCloudNode::process_callback, this, std::placeholders::_1));

        // 2. Subscription to the Filtered Lane Model
        sub_lane_ = this->create_subscription<custom_interfaces::msg::LaneModel>(
            "/lane/model_filtered", 10,
            [this](const custom_interfaces::msg::LaneModel::SharedPtr msg) {
                last_lane_ = msg;
            });

        pub_speed_ = this->create_publisher<std_msgs::msg::Float32>("/alpha/vision", 10);
        pub_active_ = this->create_publisher<std_msgs::msg::Bool>("/active/vision", 10);
        pub_meter_detections_ = this->create_publisher<custom_interfaces::msg::DetectionMeterArray>("/detection/meter_results", 10);

        // Publish active status
        auto active_msg = std_msgs::msg::Bool();
        active_msg.data = false; 
        pub_active_->publish(active_msg);
    }

private:
    // Data structure for speeds
    struct SpeedConfig { 
        double d_min_p, d_max_p, v_min_p, ocup_min_p, d_min_pre, d_max_pre, v_min_pre; 
    };

    struct TrackCandidate {
        int last_id;
        double last_x;
        double last_y;
        int hit_count;      // Consecutive frames appearing
        int miss_count;     // Consecutive frames disappearing
        bool confirmed;     // If it's already a real object
        int class_id;       // Save class for later processing
        uint32_t u1, v1, u2, v2;
        int track_id;
    };

    std::vector<TrackCandidate> candidates_;

    struct TemporalState {
        double last_distance = 0.0;
        double last_danger = 0.0;
        int inconsistency_count = 0;
        bool initialized = false;
    };

    std::unordered_map<int, TemporalState> temporal_map_;

    // Control Parameters
    int threshold_confirm_;
    int max_miss_frames_;
    double dist_threshold_;
    int inconsistency_threshold_;

    // Structure for storing information about each detected object
    struct ObjectInfo {
        int class_id;
        int tracking_id;           // ID de tracking 
        double danger_occupancy;    // Percentage of occupancy in danger zone (0-100)
        double warning_occupancy;   // Percentage of occupancy in warning zone (0-100)
        double speed_percentage;    // Percentage of speed calculated (0-1)
        double distance;            // Longitudinal distance X
        
        // Default constructor
        ObjectInfo() : class_id(-1), tracking_id(-1), danger_occupancy(0.0), 
                    warning_occupancy(0.0), speed_percentage(1.0), distance(0.0) {}
    };

    Eigen::Vector2d project_pixel(double u, double v) {
        Eigen::Vector3d pixel(u, v, 1.0);
        Eigen::Vector3d ground = homography_matrix_ * pixel;
        if (std::abs(ground.z()) < 1e-6) return Eigen::Vector2d(-1.0, -1.0); 
        return Eigen::Vector2d(ground.x() / ground.z(), ground.y() / ground.z());
    }

    // ---INTERSECTION CALCULATION ---
    ObjectInfo calculate_occupancy_percentages(const Eigen::Vector2d& center, double radius, int class_id, int tracking_id = -1) {
        ObjectInfo info;
        info.class_id = class_id;
        info.tracking_id = tracking_id;
        info.distance = center.x();

        if (!last_lane_) {
            info.danger_occupancy = 0.0;
            info.warning_occupancy = 0.0;
            return info;
        }

        // 1. Define the object's cross-section (Y-axis)
        double obj_y_min = center.y() - radius;
        double obj_y_max = center.y() + radius;

        // 2. Define intervals for the zones
        // Danger Zone: Fixed between -0.5 and 0.5 (Total width = 1.0m)
        double danger_min = -0.5;
        double danger_max = 0.5;
        double danger_width = 1.0;

        // Warning Zone: Based on the lane model
        double y_lane_center = last_lane_->curvature * std::pow(center.x(), 2) + 
                            std::tan(last_lane_->yaw) * center.x() + 
                            last_lane_->d_lat;
        double half_w = last_lane_->lane_width / 2.0;
        double precaution_min = y_lane_center - half_w;
        double precaution_max = y_lane_center + half_w;
        double precaution_width = last_lane_->lane_width;

        // 3. Calculate Overlaps
        auto get_overlap = [](double min1, double max1, double min2, double max2) {
            return std::max(0.0, std::min(max1, max2) - std::max(min1, min2));
        };

        double overlap_danger = get_overlap(obj_y_min, obj_y_max, danger_min, danger_max);
        double overlap_precaution = get_overlap(obj_y_min, obj_y_max, precaution_min, precaution_max);

        // 4.Calculate OCCUPATION percentages for the area
        info.danger_occupancy = (overlap_danger / danger_width);
        info.warning_occupancy = (overlap_precaution / precaution_width);

        // Limit to 100% (if the object is wider than the lane)
        info.danger_occupancy = std::min(1.0, info.danger_occupancy);
        info.warning_occupancy = std::min(1.0, info.warning_occupancy);

        if (!last_lane_) {
            info.danger_occupancy = 0.0;
        }

        return info;
    }

    // --- SPEED PERCENTAGE CALCULATION USING LINEAR CLAMPING ---
    double compute_speed_percentage(double distance, double d_min, double d_max, double v_min) {        
        // Calculate ratio using linear clamping
        double ratio = compute_ratio(distance, d_min, d_max);
        
        // Apply clamp between v_min and 1.0
        double speed = std::max(v_min, std::min(1.0, ratio));
        
        return speed;
    }

    double compute_ratio(double distance, double d_min, double d_max) {
        // Special case: if d_min and d_max are zero, it is constant speed
        if (d_min == 0.0 && d_max == 0.0) {
            return 1.0;
        }
        
        // Avoid division by zero
        if (d_max <= d_min) {
            return 1.0;
        }
        
        // Calculate ratio using linear clamping
        double ratio = (distance - d_min) / (d_max - d_min);

        return std::max(0.0, std::min(1.0, ratio));
    }

    void update_candidates(const custom_interfaces::msg::DetectionArray::SharedPtr msg) {
        for (auto& cand : candidates_) {
            cand.miss_count++; 
        }

        for (const auto& det : msg->detections) {

            if (det.class_id >= 6 && det.class_id <= 9) continue;
            // Project the base point of the detection to have X, Y in meters
            Eigen::Vector2d pos = project_pixel((det.u1 + det.u2) / 2.0, det.v2);
            if (pos.x() < 0) continue; // Projection error

            bool matched = false;
            for (auto& cand : candidates_) {
                double dist = std::sqrt(std::pow(pos.x() - cand.last_x, 2) + 
                                    std::pow(pos.y() - cand.last_y, 2));

                // If the ID matches OR is very close spatially
                if (det.track_id == cand.last_id || dist < dist_threshold_) {
                    cand.last_id = det.track_id;
                    cand.last_x = pos.x();
                    cand.last_y = pos.y();
                    cand.class_id = det.class_id;
                    cand.hit_count++;
                    cand.miss_count = 0;

                    cand.u1 = det.u1; cand.v1 = det.v1;
                    cand.u2 = det.u2; cand.v2 = det.v2;
                    cand.track_id = det.track_id;

                    if (cand.hit_count >= threshold_confirm_) {
                        cand.confirmed = true;
                    }
                    matched = true;
                    break;
                }
            }

            // If it's a completely new item
            if (!matched) {
                candidates_.push_back({
                    (int)det.track_id, pos.x(), pos.y(), 1, 0, false, (int)det.class_id,
                    det.u1, det.v1, det.u2, det.v2, (int)det.track_id
                });
            }
        }

        // Cleaning: Eliminating candidates who have been lost for a long time
        candidates_.erase(
            std::remove_if(candidates_.begin(), candidates_.end(),
                [this](const TrackCandidate& c) { 
                    return c.miss_count > max_miss_frames_; 
                }), 
            candidates_.end()
        );
    }

    bool is_inconsistent(int track_id, double distance, double danger) {
        auto& state = temporal_map_[track_id];

        if (!state.initialized) {
            state.last_distance = distance;
            state.last_danger = danger;
            state.initialized = true;
            return false;
        }
        if (distance < state.last_distance && danger < state.last_danger) {
            state.inconsistency_count++;
        } else {
            state.inconsistency_count = 0;
        }

        state.last_distance = distance;
        state.last_danger = danger;

        return state.inconsistency_count >= inconsistency_threshold_;
    }

    void process_callback(const custom_interfaces::msg::DetectionArray::SharedPtr msg) {
        auto meter_array_msg = custom_interfaces::msg::DetectionMeterArray();
        meter_array_msg.header = msg->header;

        // 1. Update temporary memory (N-Frames + Spatial)
        update_candidates(msg);

        // Vector to store information about all detected objects
        std::vector<ObjectInfo> objects_info;

        // --- A: DIRECT PROCESSING (Classes 6, 7, 8, 9) ---
        for (const auto& det : msg->detections) {
            if (det.class_id == 8 || det.class_id == 9) {
                ObjectInfo info;
                info.class_id = det.class_id;
                info.tracking_id = det.track_id;
                info.distance = 0.0; // We don't care about the exact physical distance for speed.

                auto it = speed_map_.find(det.class_id);
                if (it != speed_map_.end()) {
                    const SpeedConfig& cfg = it->second;
                    // We directly assign the configured minimum speed
                    info.speed_percentage = cfg.v_min_p; 
                } else {
                    info.speed_percentage = 1.0;
                }
                
                objects_info.push_back(info);
            
            }
            else if (det.class_id == 6 || det.class_id == 7) {
                // 1. Project the necessary corners to calculate the width (v2 is the ground)
                Eigen::Vector2d left  = project_pixel(det.u1, det.v2);
                Eigen::Vector2d right = project_pixel(det.u2, det.v2);

                // --- Store in message in meters ---
                custom_interfaces::msg::DetectionMeter det_meter;
                det_meter.class_id = det.class_id;
                det_meter.track_id = det.track_id;
                det_meter.x1 = left.x();
                det_meter.y1 = left.y();
                det_meter.x2 = right.x();
                det_meter.y2 = right.y();
                meter_array_msg.detections.push_back(det_meter);
                // -------------------------------------------------
                
                // 2. Calculate the center and dynamic radius
                Eigen::Vector2d center = (left + right) / 2.0;
                double radius = (left - right).norm() / 2.0; // Actual radius based on the width of the box

                if (center.x() < min_dist_ || center.x() > max_dist_) continue;

                // 3. Calculate occupancy using the calculated radius
                ObjectInfo info = calculate_occupancy_percentages(center, radius, det.class_id, det.track_id);

                // 4. Filter: We only apply the ramp if there is occupancy in the danger zone
                auto it = speed_map_.find(det.class_id);
                if (it != speed_map_.end() && info.danger_occupancy > 0.0) { 
                    const SpeedConfig& cfg = it->second;
                    info.speed_percentage = compute_speed_percentage(
                        center.x(), cfg.d_min_p, cfg.d_max_p, cfg.v_min_p);
                } else {
                    info.speed_percentage = 1.0;
                }
                
                objects_info.push_back(info);
            }
        }

        // --- EXISTING DETECTIONS ---
        for (const auto& det : candidates_) {
            if (!det.confirmed) continue;

            
            if (det.class_id >= 0 && det.class_id <= 5) {
                Eigen::Vector2d center(det.last_x, det.last_y);
                Eigen::Vector2d left   = project_pixel(det.u1, det.v2);
                Eigen::Vector2d right  = project_pixel(det.u2, det.v2);
                if (center.x() < min_dist_ || center.x() > max_dist_) continue;

                // ---  Store confirmed candidates in the message in meters ---
                custom_interfaces::msg::DetectionMeter det_meter;
                det_meter.class_id = det.class_id;
                det_meter.track_id = det.track_id;
                det_meter.x1 = left.x();
                det_meter.y1 = left.y();
                det_meter.x2 = right.x();
                det_meter.y2 = right.y();
                meter_array_msg.detections.push_back(det_meter);
                // -----------------------------------------------------------------------

                double radius = (left - right).norm() / 2.0;
                if (det.class_id == 0 && radius < 1.0) radius *= 1.2;

                // Create ObjectInfo for mobile object
                ObjectInfo info = calculate_occupancy_percentages(center, radius, det.class_id, det.track_id);
                bool inconsistent = false;
                inconsistent = is_inconsistent(det.track_id, center.x(), info.danger_occupancy);
                
                // Get speed settings for this class
                auto it = speed_map_.find(det.class_id);
                if (it != speed_map_.end()) {
                    const SpeedConfig& cfg = it->second;
                    info.speed_percentage = 1.0;

                    if (info.danger_occupancy > cfg.ocup_min_p) {

                        if (inconsistent) {
                            info.speed_percentage = 1.0;
                        } else {
                            info.speed_percentage = compute_speed_percentage(
                                center.x(), cfg.d_min_p, cfg.d_max_p, cfg.v_min_p);
                        }

                    }
                    else if (info.warning_occupancy > 0.01) {
                        // ZONA DE PRECAUCIÓN:
                        // 1. Calculate what speed would be required per distance.
                        double ratio_distancia = compute_ratio(center.x(), cfg.d_min_pre, cfg.d_max_pre);
                        double velocidad_por_distancia = cfg.v_min_pre + (ratio_distancia * (1.0 - cfg.v_min_pre));

                        // 2. We apply occupancy as a mixing factor
                        info.speed_percentage = 1.0 - (info.warning_occupancy * (1.0 - velocidad_por_distancia));
                        
                        // Ensure that it does not fall below v_min_pre due to precision errors
                        info.speed_percentage = std::max(cfg.v_min_pre, info.speed_percentage);
                    }
                }
                
                objects_info.push_back(info);
            }
        }

        double velocidad = 1.0; 
        // --- SHOW INFORMATION FOR ALL OBJECTS ---
        for (const auto& obj : objects_info) {
            velocidad = std::min(velocidad, obj.speed_percentage); // Convert to percentage to display
        }
        
        // 1. Publish speed
        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = static_cast<float>(velocidad);
        pub_speed_->publish(speed_msg);

        // 2. Publish active state
        auto active_msg = std_msgs::msg::Bool();
        active_msg.data = true; 

        // 3. Publish the detections transformed to meters
        pub_meter_detections_->publish(meter_array_msg);
        pub_active_->publish(active_msg);
    }
 
    void load_calibration(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), " The calibration file could not be opened in: %s", path.c_str());
            return; 
        }
        json calib = json::parse(f);
        double cam_z = calib["camera_z"], cam_x = calib["camera_x"], cam_y = calib["camera_y"];
        double roll = calib["camera_roll"];
        double fx = calib["intrinsics"]["fx"], fy = calib["intrinsics"]["fy"];
        double cx = calib["intrinsics"]["cx"], cy = calib["intrinsics"]["cy"];
        Eigen::Matrix3d K; K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        Eigen::Matrix3d R_align; R_align << 0, -1, 0, 0, 0, -1, 1, 0, 0;
        double cp = std::cos(roll), sp = std::sin(roll);
        Eigen::Matrix3d R_roll; R_roll << 1, 0, 0, 0, cp, sp, 0, -sp, cp;
        Eigen::Matrix3d R = R_roll * R_align;
        Eigen::Vector3d t = -R * Eigen::Vector3d(cam_x, cam_y, cam_z);
        Eigen::Matrix3d H; H.col(0) = K * R.col(0); H.col(1) = K * R.col(1); H.col(2) = K * t;
        homography_matrix_ = H.inverse();
    }
    
    // The MAP is now a member of the class (accessible in process_callback)
    std::map<int, SpeedConfig> speed_map_;

    Eigen::Matrix3d homography_matrix_; 
    double min_dist_, max_dist_;
    
    // Subscribers
    rclcpp::Subscription<custom_interfaces::msg::DetectionArray>::SharedPtr sub_detections_;
    rclcpp::Subscription<custom_interfaces::msg::LaneModel>::SharedPtr sub_lane_;
    
    // Storage for the lane
    custom_interfaces::msg::LaneModel::SharedPtr last_lane_;
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_active_;
    rclcpp::Publisher<custom_interfaces::msg::DetectionMeterArray>::SharedPtr pub_meter_detections_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorToCloudNode>());
    rclcpp::shutdown();
    return 0;
}