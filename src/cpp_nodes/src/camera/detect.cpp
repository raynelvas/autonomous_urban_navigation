#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include "custom_interfaces/msg/detection_array.hpp"
#include "custom_interfaces/msg/detection.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

struct TrackedObject {
    int id; int class_id; cv::Rect bbox; cv::Point2f center; int lost_frames;
};
class YoloFinalNode : public rclcpp::Node {
public:
    YoloFinalNode() : Node("detect_node") {
        this->declare_parameter<std::string>("engine_path", "/home/robot/Downloads/detect_yolo_512x288_ORIN.engine");
        
        // Dimension parameters
        input_h_ = 288; input_w_ = 512;
        num_classes_ = 10; num_anchors_ = 3024;
        conf_threshold_ = 0.35f; nms_threshold_ = 0.45f;

        cudaMallocHost((void**)&input_host_ptr_, 3 * input_h_ * input_w_ * sizeof(float));
        cudaMallocHost((void**)&output_host_ptr_, (4 + num_classes_) * num_anchors_ * sizeof(float));

        load_engine(this->get_parameter("engine_path").as_string());

        class_thresholds_.resize(10, 0.35f);  
        this->declare_parameter<std::vector<double>>("class_thresholds", std::vector<double>());
        auto thresholds_param = this->get_parameter("class_thresholds").as_double_array();
        
        if (thresholds_param.size() == 10) {
            for (int i = 0; i < 10; i++) {
                class_thresholds_[i] = static_cast<float>(thresholds_param[i]);
            }
        } 

        pub_ = create_publisher<custom_interfaces::msg::DetectionArray>("/detection/results", 10);
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(1).best_effort(),
            std::bind(&YoloFinalNode::callback, this, std::placeholders::_1));

    }

    ~YoloFinalNode() {
        cudaStreamDestroy(stream_);
        cudaFree(dev_buffers_[0]); cudaFree(dev_buffers_[1]);
        cudaFreeHost(input_host_ptr_); cudaFreeHost(output_host_ptr_);
    }

private:
    std::vector<float> class_thresholds_;
    std::vector<TrackedObject> active_tracks_;
    int next_id_ = 1;
    const int MAX_LOST_FRAMES = 10; // Tolerates 10 frames of "disappearance"
    
    int update_tracker(const cv::Rect& box, int class_id) {
        int best_idx = -1;
        float best_score = 0.0f;
        cv::Point2f current_center(box.x + box.width/2.0f, box.y + box.height/2.0f);

        for (size_t i = 0; i < active_tracks_.size(); ++i) {
            if (active_tracks_[i].class_id != class_id) continue;
            float iou = (float)(box & active_tracks_[i].bbox).area() / (box | active_tracks_[i].bbox).area();
            float dist = cv::norm(current_center - active_tracks_[i].center);
            float score = (iou * 0.7f) + (1.0f / (1.0f + dist) * 0.3f);

            if (score > 0.4f && score > best_score) {
                best_score = score;
                best_idx = i;
            }
        }

        if (best_idx != -1) {
            active_tracks_[best_idx].bbox = box;
            active_tracks_[best_idx].center = current_center;
            active_tracks_[best_idx].lost_frames = 0;
            return active_tracks_[best_idx].id;
        } else {
            active_tracks_.push_back({next_id_++, class_id, box, current_center, 0});
            return active_tracks_.back().id;
        }
    }

    void load_engine(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        file.seekg(0, file.end); size_t size = file.tellg(); file.seekg(0, file.beg);
        std::vector<char> data(size); file.read(data.data(), size);

        initLibNvInferPlugins(&gLogger, "");
        runtime_ = nvinfer1::createInferRuntime(gLogger);
        engine_ = runtime_->deserializeCudaEngine(data.data(), size);
        context_ = engine_->createExecutionContext();

        cudaMalloc(&dev_buffers_[0], 3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&dev_buffers_[1], (4 + num_classes_) * num_anchors_ * sizeof(float));
        cudaStreamCreate(&stream_);
        
        context_->setTensorAddress("images", dev_buffers_[0]);
        context_->setTensorAddress("output0", dev_buffers_[1]);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        //auto t0 = std::chrono::high_resolution_clock::now();

        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (frame.empty()) return;

        // 1. LETTERBOX
        float scale = std::min((float)input_w_ / frame.cols, (float)input_h_ / frame.rows);
        int nw = frame.cols * scale; int nh = frame.rows * scale;
        int dx = (input_w_ - nw) / 2; int dy = (input_h_ - nh) / 2;

        cv::Mat canvas(input_h_, input_w_, CV_8UC3, cv::Scalar(114,114,114));
        cv::Mat resized; cv::resize(frame, resized, cv::Size(nw, nh));
        resized.copyTo(canvas(cv::Rect(dx, dy, nw, nh)));

        // 2. PRE-PROCESS
        float* ptr = input_host_ptr_;
        for (int c = 2; c >= 0; --c) { // BGR to RGB and Normalization in a single step
            for (int i = 0; i < input_h_; ++i) {
                uchar* row_ptr = canvas.ptr<uchar>(i);
                for (int j = 0; j < input_w_; ++j) {
                    *ptr++ = row_ptr[j * 3 + c] / 255.0f;
                }
            }
        }

        // 3. INFERENCE
        cudaMemcpyAsync(dev_buffers_[0], input_host_ptr_, 3 * input_h_ * input_w_ * sizeof(float), cudaMemcpyHostToDevice, stream_);
        context_->enqueueV3(stream_);
        cudaMemcpyAsync(output_host_ptr_, dev_buffers_[1], (4 + num_classes_) * num_anchors_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        cudaStreamSynchronize(stream_);

        // 4. POST-PROCESS
        std::vector<cv::Rect> boxes;
        std::vector<float> confs;
        std::vector<int> class_ids;

        for (int i = 0; i < num_anchors_; ++i) {
            float max_s = 0; int id = -1;
            for (int c = 0; c < num_classes_; ++c) {
                float s = output_host_ptr_[(4 + c) * num_anchors_ + i];
                if (s > max_s) { max_s = s; id = c; }
            }

            if (max_s > class_thresholds_[id]) {
                float cx = output_host_ptr_[0*num_anchors_+i];
                float cy = output_host_ptr_[1*num_anchors_+i];
                float w  = output_host_ptr_[2*num_anchors_+i];
                float h  = output_host_ptr_[3*num_anchors_+i];

                int x = static_cast<int>(std::round((cx - w/2.0f - dx) / scale));
                int y = static_cast<int>(std::round((cy - h/2.0f - dy) / scale));
                int width  = static_cast<int>(std::round(w / scale));
                int height = static_cast<int>(std::round(h / scale));

                // Clamp
                x = std::max(0, std::min(x, frame.cols - 1));
                y = std::max(0, std::min(y, frame.rows - 1));
                width  = std::max(0, std::min(width, frame.cols - x));
                height = std::max(0, std::min(height, frame.rows - y));

                if (width > 5 && height > 5) {
                    boxes.emplace_back(x, y, width, height);
                    confs.push_back(max_s);
                    class_ids.push_back(id);
                }
            }
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confs, conf_threshold_, nms_threshold_, indices);

        custom_interfaces::msg::DetectionArray det_msg;
        det_msg.header = msg->header;
        for (int idx : indices) {
            custom_interfaces::msg::Detection d;
            d.class_id = class_ids[idx];
            d.confidence = confs[idx];
            d.u1 = boxes[idx].x; d.v1 = boxes[idx].y;
            d.u2 = boxes[idx].x + boxes[idx].width; d.v2 = boxes[idx].y + boxes[idx].height;
            d.track_id = update_tracker(boxes[idx], d.class_id);
            det_msg.detections.push_back(d);
        }

        active_tracks_.erase(std::remove_if(active_tracks_.begin(), active_tracks_.end(),
            [this](TrackedObject& t){ t.lost_frames++; return t.lost_frames > MAX_LOST_FRAMES; }), 
            active_tracks_.end());

        pub_->publish(det_msg);
    }

    int input_h_, input_w_, num_classes_, num_anchors_;
    float conf_threshold_, nms_threshold_;
    float *input_host_ptr_, *output_host_ptr_;
    void* dev_buffers_[2];
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    cudaStream_t stream_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<custom_interfaces::msg::DetectionArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloFinalNode>());
    rclcpp::shutdown();
    return 0;
}