#include "libfreenect.hpp"

#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>

class ImageStreamKinectV1 : public Freenect::FreenectDevice {
public:
    ImageStreamKinectV1(freenect_context *_ctx, int _index)
        : Freenect::FreenectDevice(_ctx, _index), gamma_(2048),
    depth_image_(cv::Size(640,480), CV_16UC1),
    rgb_image_(cv::Size(640,480), CV_8UC3, cv::Scalar(0)),
    own_image_(cv::Size(640,480), CV_8UC3, cv::Scalar(0)) {
        setDepthFormat(FREENECT_DEPTH_REGISTERED);
    }

    // Do not call directly even in child
    void VideoCallback(void* rgb_data, uint32_t timestamp) {
        rgb_mutex_.lock();
        uint8_t* rgb = static_cast<uint8_t*>(rgb_data);
        rgb_image_.data = rgb;
        new_rgb_frame_ = true;
        rgb_mutex_.unlock();
    };

    // Do not call directly even in child
    void DepthCallback(void* depth_data, uint32_t timestamp) {
        std::cout << "Depth callback" << std::endl;
        depth_mutex_.lock();
        uint16_t* depth = static_cast<uint16_t*>(depth_data);
        depth_image_.data = (unsigned char*) depth;
        new_depth_frame_ = true;
        depth_mutex_.unlock();
    }

    bool getVideo(cv::Mat& output) {
        rgb_mutex_.lock();
        if(new_rgb_frame_) {
            cv::cvtColor(rgb_image_, output, cv::COLOR_RGB2BGR);
            new_rgb_frame_ = false;
            rgb_mutex_.unlock();
            return true;
        } else {
            rgb_mutex_.unlock();
            return false;
        }
    }

    bool getDepth(cv::Mat& output) {
        depth_mutex_.lock();
        if(new_depth_frame_) {
            depth_image_.copyTo(output);
            new_depth_frame_ = false;
            depth_mutex_.unlock();
            return true;
        } else {
            depth_mutex_.unlock();
            return false;
        }
    }

public:
    bool new_rgb_frame_ = false;
    bool new_depth_frame_ = false;

private:
    std::vector<uint8_t> depth_buffer_;
    std::vector<uint8_t> rgb_buffer_;
    std::vector<uint16_t> gamma_;
    cv::Mat depth_image_;
    cv::Mat rgb_image_;
    cv::Mat own_image_;
    std::mutex rgb_mutex_;
    std::mutex depth_mutex_;
};
