#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include <cuda_fp16.h> 
#include <fstream>
#include <iostream>
#include <vector>
#include "custom_interfaces/msg/pixel_point.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TensorRT-FP16] " << msg << std::endl;
    }
} gLogger;

class TwinLiteSegmenter : public rclcpp::Node {
public:
    TwinLiteSegmenter() : Node("segmenter_twinlite") {
        input_h_ = 288;
        input_w_ = 512;
        num_classes_ = 2;

        // Reserve Pinned Memory using __half
        cudaMallocHost((void**)&input_buffer_, 3 * input_h_ * input_w_ * sizeof(float));
        cudaMallocHost((void**)&output_buffer_, num_classes_ * input_h_ * input_w_ * sizeof(float));

        // 1. Declare the parameter
        this->declare_parameter<std::string>("engine_path", "");
        
        // 2. Get the parameter value
        std::string engine_path = this->get_parameter("engine_path").as_string();

        // 3. Use the variable instead of the static patha
        if (engine_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No engine path was provided. Use the 'engine_path' parameter'");
        } else {
            load_engine(engine_path);
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image_raw/compressed", qos,
            std::bind(&TwinLiteSegmenter::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<custom_interfaces::msg::PixelPoint>("/lane/pixel_candidates", 10);

    }

    ~TwinLiteSegmenter() {
        if (context_) delete context_;
        if (engine_) delete engine_;
        if (runtime_) delete runtime_;
        cudaStreamDestroy(stream_);
        cudaFree(buffers_[0]);
        cudaFree(buffers_[1]);
        cudaFreeHost(input_buffer_);
        cudaFreeHost(output_buffer_);
    }

private:
    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.good()) return;
        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> data(size);
        file.read(data.data(), size);

        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        context_ = engine_->createExecutionContext();

        for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
            auto name = engine_->getIOTensorName(i);
            if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) input_name = name;
            else output_name = name;
        }

        // GPU buffers also in FP16
        cudaMalloc(&buffers_[0], 3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&buffers_[1], num_classes_ * input_h_ * input_w_ * sizeof(float));
        cudaStreamCreate(&stream_);
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        //auto t_start = std::chrono::high_resolution_clock::now();
        // 1. Decode and Pre-process 
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        cv::Mat resized, float_img;
        cv::resize(frame, resized, cv::Size(input_w_, input_h_));
        cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
        
        // Direct normalization to Float32
        resized.convertTo(float_img, CV_32FC3, 1.0f / 255.0f);

        // Quick copy without manual conversion to half
        int vol = input_h_ * input_w_;
        std::vector<cv::Mat> channels(3);
        cv::split(float_img, channels);
        for (int i = 0; i < 3; ++i) {
            memcpy(input_buffer_ + i * vol, channels[i].data, vol * sizeof(float));
        }

        //auto t_2 = std::chrono::high_resolution_clock::now();
        // 2. Inference
        cudaMemcpyAsync(buffers_[0], input_buffer_, 3 * vol * sizeof(float), cudaMemcpyHostToDevice, stream_);

        context_->setTensorAddress(input_name.c_str(), buffers_[0]);
        context_->setTensorAddress(output_name.c_str(), buffers_[1]);
        // -----------------------------------

        context_->enqueueV3(stream_);
        cudaMemcpyAsync(output_buffer_, buffers_[1], num_classes_ * vol * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        //auto t_3 = std::chrono::high_resolution_clock::now();
        // 3. Post-processing
        float* lane_ptr = output_buffer_ + vol; 
        cv::Mat mask_raw(input_h_, input_w_, CV_32FC1, lane_ptr);
        
        cv::Mat mask_bin;
        cv::threshold(mask_raw, mask_bin, 0.5, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8UC1);

        custom_interfaces::msg::PixelPoint out_msg;
        out_msg.header = msg->header;
        float scale_x = (float)frame.cols / input_w_;
        float scale_y = (float)frame.rows / input_h_;

        for (int y = 0; y < mask_bin.rows; y++) {
            const uint8_t* row = mask_bin.ptr<uint8_t>(y);
            int start_x = -1;
            for (int x = 0; x < mask_bin.cols; ++x) {
                if (row[x] > 0) {
                    if (start_x == -1) start_x = x;
                } else if (start_x != -1) {
                    int width = x - start_x;
                    out_msg.u.push_back(static_cast<int>((start_x + width / 2) * scale_x));
                    out_msg.v.push_back(static_cast<int>(y * scale_y));
                    start_x = -1;
                }
            }
        }
        publisher_->publish(out_msg);
        // Measure total cycle time
        /*auto t_end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        double ms_1 = std::chrono::duration<double, std::milli>(t_2 - t_start).count();
        double ms_2 = std::chrono::duration<double, std::milli>(t_3 - t_2).count();
        double ms_3 = std::chrono::duration<double, std::milli>(t_end - t_3).count();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Total time: %.2f ms (Pre-proc: %.2f ms, Inf: %.2f ms, Post-proc: %.2f ms)", ms, ms_1, ms_2, ms_3);*/
    }

    int input_h_, input_w_, num_classes_;
    std::string input_name, output_name;
    float *input_buffer_, *output_buffer_;
    nvinfer1::IRuntime* runtime_{nullptr};
    nvinfer1::ICudaEngine* engine_{nullptr};
    nvinfer1::IExecutionContext* context_{nullptr};
    void* buffers_[2];
    cudaStream_t stream_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::PixelPoint>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwinLiteSegmenter>());
    rclcpp::shutdown();
    return 0;
}