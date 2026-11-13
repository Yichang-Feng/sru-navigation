#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class Depth16UC1To32FC1Filled : public rclcpp::Node
{
public:
  Depth16UC1To32FC1Filled()
  : Node("depth_preprocess_realsense")
  {
    max_depth_m_ = 6.0f;
    max_depth_mm_ = static_cast<int>(max_depth_m_ * 1000.0f);

    spatial_ksize_ = 5;
    spatial_sigma_ = 0.03;

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();
    qos.durability_volatile();

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_rect_raw",
      qos,
      std::bind(&Depth16UC1To32FC1Filled::callback, this, _1)
    );

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_rect_32fc1",
      10
    );
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->encoding != "16UC1") {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat depth_mm;
    cv_ptr->image.convertTo(depth_mm, CV_32F);

    cv::Mat depth_m = depth_mm * 0.001f;

    // invalid mask
    cv::Mat invalid =
      (depth_mm <= 0.0f) | (depth_mm >= static_cast<float>(max_depth_mm_));

    depth_m.setTo(0.0f, invalid);

    // ===== 方向性 hole filling：横向 =====
    for (int y = 0; y < depth_m.rows; ++y) {
      float* row = depth_m.ptr<float>(y);

      std::vector<int> valid_idx;
      std::vector<float> valid_val;

      for (int x = 0; x < depth_m.cols; ++x) {
        if (row[x] > 0.0f) {
          valid_idx.push_back(x);
          valid_val.push_back(row[x]);
        }
      }

      if (valid_idx.size() < 2) continue;

      for (int x = 0; x < depth_m.cols; ++x) {
        if (row[x] == 0.0f) {
          auto it = std::lower_bound(valid_idx.begin(), valid_idx.end(), x);
          if (it == valid_idx.begin() || it == valid_idx.end()) continue;

          int i1 = *(it - 1);
          int i2 = *it;
          float v1 = row[i1];
          float v2 = row[i2];

          float t = float(x - i1) / float(i2 - i1);
          row[x] = v1 + t * (v2 - v1);
        }
      }
    }

    // ===== 方向性 hole filling：纵向 =====
    for (int x = 0; x < depth_m.cols; ++x) {
      std::vector<int> valid_idx;
      std::vector<float> valid_val;

      for (int y = 0; y < depth_m.rows; ++y) {
        float v = depth_m.at<float>(y, x);
        if (v > 0.0f) {
          valid_idx.push_back(y);
          valid_val.push_back(v);
        }
      }

      if (valid_idx.size() < 2) continue;

      for (int y = 0; y < depth_m.rows; ++y) {
        if (depth_m.at<float>(y, x) == 0.0f) {
          auto it = std::lower_bound(valid_idx.begin(), valid_idx.end(), y);
          if (it == valid_idx.begin() || it == valid_idx.end()) continue;

          int i1 = *(it - 1);
          int i2 = *it;
          float v1 = depth_m.at<float>(i1, x);
          float v2 = depth_m.at<float>(i2, x);

          float t = float(y - i1) / float(i2 - i1);
          depth_m.at<float>(y, x) = v1 + t * (v2 - v1);
        }
      }
    }

    // ===== 双边滤波 =====
    cv::Mat depth_filtered;
    cv::bilateralFilter(
    depth_m,
    depth_filtered,
    spatial_ksize_,
    spatial_sigma_,
    5
    );

    depth_m = depth_filtered;

    auto out_msg =
      cv_bridge::CvImage(msg->header, "32FC1", depth_m).toImageMsg();

    pub_->publish(*out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  float max_depth_m_;
  int max_depth_mm_;

  int spatial_ksize_;
  double spatial_sigma_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Depth16UC1To32FC1Filled>());
  rclcpp::shutdown();
  return 0;
}