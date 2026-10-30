#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <atomic>
#include <fstream>

using json = nlohmann::json;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), running_(true)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", qos);

        std::string pipeline =
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=1920,height=1080,framerate=30/1 ! "
            "jpegparse ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw,width=512,height=288,format=BGRx ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false";

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "The camera could not be opened withGStreamer");
            return;
        }
        capture_thread_ = std::thread([this]() {
            cv::Mat frame;
            while (rclcpp::ok() && running_) {
                if (!cap_.read(frame) || frame.empty())
                    continue;

                process_and_publish(frame);
            }
        });
    }

    ~CameraNode()
    {
        running_ = false;
        if (capture_thread_.joinable())
            capture_thread_.join();

        if (cap_.isOpened())
            cap_.release();
    }

private:
    void process_and_publish(const cv::Mat& frame)
    {
        cv::Mat output;
        output = frame;

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            output
        ).toImageMsg();

        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";

        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> running_;

    cv::Mat map1_, map2_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}