#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp> 
#include <Eigen/Dense>       

#include "custom_interfaces/msg/pixel_point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using json = nlohmann::json;

class PixelToMeterNode : public rclcpp::Node {
public:
    PixelToMeterNode() : Node("pixel_to_meter_transform") {        
        // Load full calibration
        this->declare_parameter("config_path", "");
        std::string config_path = this->get_parameter("config_path").as_string();
        load_calibration(config_path);

        this->declare_parameter("min_distance", 1.0);
        this->declare_parameter("max_distance", 50.0);

        min_dist_ = this->get_parameter("min_distance").as_double();
        max_dist_ = this->get_parameter("max_distance").as_double();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        sub_pixels_ = this->create_subscription<custom_interfaces::msg::PixelPoint>(
            "/lane/pixel_candidates", qos,
            std::bind(&PixelToMeterNode::process_callback, this, std::placeholders::_1));

        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lane/meter_candidates", 10);
    }

private:
    void load_calibration(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            return;
        }
        json calib = json::parse(f);

        cam_z = calib["camera_z"];
        cam_x = calib["camera_x"];
        cam_y = calib["camera_y"];
        roll = calib["camera_roll"];
        fx_ = calib["intrinsics"]["fx"];
        fy_ = calib["intrinsics"]["fy"];
        cx_ = calib["intrinsics"]["cx"];
        cy_ = calib["intrinsics"]["cy"];
        
        // 1. Intrinsic matrix K
        Eigen::Matrix3d K;
        K << fx_,  0, cx_,
             0, fy_, cy_,
             0,  0,   1;

        // ROS Alignment -> Camera
        Eigen::Matrix3d R_align;
        R_align << 0, -1,  0,
                0,  0, -1,
                1,  0,  0;

        // Roll (rotation around X_cam)
        // Negative roll indicates that the camera is tilted downwards, pointing more towards the ground.
        double cp = std::cos(roll);
        double sp = std::sin(roll);

        Eigen::Matrix3d R_roll;
        R_roll << 1,  0,   0,
                0,  cp,  sp,
                0, -sp,  cp;

        // World rotation -> camera
        Eigen::Matrix3d R = R_roll * R_align;

        // Position camera in world (base_footprint)
        Eigen::Vector3d C(cam_x, cam_y, cam_z);

        // Translation OpenCV
        Eigen::Vector3d t = -R * C;

        // Homography construction assuming Z=0 plane in world
        Eigen::Matrix3d H;
        H.col(0) = K * R.col(0);
        H.col(1) = K * R.col(1);
        H.col(2) = K * t;

        homography_matrix_ = H.inverse();
    }

    void process_callback(const custom_interfaces::msg::PixelPoint::SharedPtr msg) {
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = msg->header;
        cloud_msg->header.frame_id = "base_footprint";
        
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(msg->u.size());

        if (msg->u.empty()) {
            modifier.resize(0);
            pub_cloud_->publish(*cloud_msg);
            return; 
        }

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

        size_t valid_count = 0;

        for (size_t i = 0; i < msg->u.size(); ++i) {
            Eigen::Vector3d pixel(msg->u[i], msg->v[i], 1.0);
            
            // footprint = H * Píxel
            Eigen::Vector3d ground_points = homography_matrix_ * pixel;

            // Normalize homogeneous coordinates (divide by W)
            if (std::abs(ground_points.z()) < 1e-6) continue;
            
            double xw = ground_points.x() / ground_points.z();
            double yw = ground_points.y() / ground_points.z();
            
            if (xw >= min_dist_ && xw <= max_dist_) {
                *iter_x = static_cast<float>(xw);
                *iter_y = static_cast<float>(yw);
                *iter_z = 0.0f;

                ++iter_x; ++iter_y; ++iter_z;
                valid_count++;
            }
        }

        if (valid_count > 0) {
            modifier.resize(valid_count);
            pub_cloud_->publish(*cloud_msg);
        } else {
            // Publish empty cloud if there are no valid points
            modifier.resize(0);
            pub_cloud_->publish(*cloud_msg);
        }
    }

    // CLASS VARIABLES
    double cam_z, cam_x, cam_y, roll, fx_, fy_, cx_, cy_;
    double min_dist_, max_dist_;
    Eigen::Matrix3d homography_matrix_; 

    rclcpp::Subscription<custom_interfaces::msg::PixelPoint>::SharedPtr sub_pixels_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixelToMeterNode>());
    rclcpp::shutdown();
    return 0;
}