#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdlib>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node"), running_(true) {        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", qos);
        
        int device_id = this->declare_parameter("device", 2);
        std::string device_path = "/dev/video" + std::to_string(device_id);

        cap_.open(device_id, cv::CAP_V4L2); 

        if (!cap_.isOpened()) {
            return;
        }

        // Hardware Configuration
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        apply_v4l2_settings(device_path);

        capture_thread_ = std::thread([this]() {
            cv::Mat raw_frame;
            
            while (rclcpp::ok() && running_) {
                if (cap_.read(raw_frame)) {
                    if (raw_frame.empty()) continue;
                    process_and_publish_direct(raw_frame);
                }
            }
        });
    }

    ~CameraNode() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (cap_.isOpened()) cap_.release();
    }

private:
    void apply_v4l2_settings(const std::string & device) {
        std::string base_cmd = "v4l2-ctl -d " + device + " --set-ctrl ";
        
        system((base_cmd + "focus_automatic_continuous=0").c_str());
        system((base_cmd + "focus_absolute=0").c_str());
        
        // Dynamic adjustment: auto_exposure=3 is aperture priority
        system((base_cmd + "auto_exposure=3").c_str()); 
        system((base_cmd + "auto_exposure_priority=0").c_str()); 

        system((base_cmd + "white_balance_automatic=0").c_str());
        system((base_cmd + "white_balance_temperature=3700").c_str());
        system((base_cmd + "brightness=128").c_str());
        system((base_cmd + "gain=0").c_str());
    }

    void process_and_publish_direct(const cv::Mat & raw_frame) {
        // 1. Resize
        cv::resize(raw_frame, resized_frame_, cv::Size(512, 288), 0, 0, cv::INTER_AREA);

        // 2. Compress and Publish
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";
        msg->format = "jpeg";
        
        cv::imencode(".jpg", resized_frame_, msg->data, compression_params_);
        publisher_->publish(std::move(msg));
    }

    cv::VideoCapture cap_;
    std::atomic<bool> running_;
    std::thread capture_thread_;
    cv::Mat resized_frame_;
    
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::vector<int> compression_params_ = {cv::IMWRITE_JPEG_QUALITY, 85};
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}