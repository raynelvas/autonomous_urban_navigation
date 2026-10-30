#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include "custom_interfaces/msg/pixel_point.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
} gLogger;

class TwinLiteNetNode : public rclcpp::Node {
public:
    TwinLiteNetNode() : Node("twinlite_lane_refiner") {
        this->declare_parameter("engine_path", "/home/robot/Downloads/twinlite_FAST_32bit.engine");
        
        input_h_ = 288;
        input_w_ = 512;
        
        // All handling on Host is float (4 bytes)
        input_host_buffer_.resize(3 * input_h_ * input_w_);
        output_host_buffer_.resize(2 * input_h_ * input_w_); 

        load_engine(this->get_parameter("engine_path").as_string());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", qos, std::bind(&TwinLiteNetNode::image_callback, this, std::placeholders::_1));

        publisher_refined_ = this->create_publisher<custom_interfaces::msg::PixelPoint>(
            "/lane/pixel_candidates", 1);
    }

    ~TwinLiteNetNode() {
        cudaStreamDestroy(stream_);
        cudaFree(device_buffers_[0]);
        cudaFree(device_buffers_[1]);
    }

private:
    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el motor: %s", path.c_str());
            return;
        }
        file.seekg(0, std::ios::end);
        size_t size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> data(size);
        file.read(data.data(), size);

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        context_ = engine_->createExecutionContext();

        // We reserved sizeof(float) because the engine interface is FP32
        cudaMalloc(&device_buffers_[0], 3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&device_buffers_[1], 2 * input_h_ * input_w_ * sizeof(float));
        cudaStreamCreate(&stream_);
        
        context_->setTensorAddress("input", device_buffers_[0]);
        context_->setTensorAddress("lane_lines", device_buffers_[1]);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        //auto t_start = std::chrono::high_resolution_clock::now();

        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (frame.empty()) return;

        // 1. Pre-process
        cv::Mat resized, float_img;
        cv::resize(frame, resized, cv::Size(input_w_, input_h_));
        cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
        resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

        std::vector<cv::Mat> channels(3);
        cv::split(float_img, channels);
        for (int i = 0; i < 3; ++i) {
            memcpy(input_host_buffer_.data() + i * input_h_ * input_w_,
                   channels[i].data, input_h_ * input_w_ * sizeof(float));
        }

        // 2. Inference
        cudaMemcpyAsync(device_buffers_[0], input_host_buffer_.data(), 
                        input_host_buffer_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        
        context_->enqueueV3(stream_);

        cudaMemcpyAsync(output_host_buffer_.data(), device_buffers_[1], 
                        output_host_buffer_.size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        
        cudaStreamSynchronize(stream_);

        // 3. Post-processing
        float* lane_channel_ptr = output_host_buffer_.data() + (input_h_ * input_w_);
        cv::Mat mask_raw(input_h_, input_w_, CV_32FC1, lane_channel_ptr);
        
        cv::Mat mask_bin;
        cv::threshold(mask_raw, mask_bin, 0.5, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8UC1);

        auto out_msg = custom_interfaces::msg::PixelPoint();
        out_msg.header = msg->header;

        float scale_x = (float)frame.cols / input_w_;
        float scale_y = (float)frame.rows / input_h_;

        for (int y = 0; y < mask_bin.rows; y += 1) {
            const uint8_t* row = mask_bin.ptr<uint8_t>(y);
            int start_x = -1;
            for (int x = 0; x < mask_bin.cols; ++x) {
                if (row[x] > 0) {
                    if (start_x == -1) start_x = x;
                } else if (start_x != -1) {
                    int width = x - start_x;
                    out_msg.u.push_back((start_x + width / 2) * scale_x);
                    out_msg.v.push_back(y * scale_y);
                    start_x = -1;
                }
            }
        }
        publisher_refined_->publish(out_msg);

        // Measure total cycle time
        //auto t_end = std::chrono::high_resolution_clock::now();
        //double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Optimized cycle: %.2f ms", ms);
    }

    int input_h_, input_w_;
    std::vector<float> input_host_buffer_, output_host_buffer_;
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    void* device_buffers_[2];
    cudaStream_t stream_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::PixelPoint>::SharedPtr publisher_refined_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwinLiteNetNode>());
    rclcpp::shutdown();
    return 0;
}