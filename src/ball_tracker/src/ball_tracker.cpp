#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class BallTracker : public rclcpp::Node {
public:
  BallTracker() : Node("ball_tracker") {
    using std::placeholders::_1;

    // Parameters (can be tuned here) I made script moehaha
    h_min_ = 36; h_max_ = 96;
    s_min_ = 56; s_max_ = 255;
    v_min_ = 21; v_max_ = 255;
    size_min_px_ = 10; size_max_px_ = 200;

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, std::bind(&BallTracker::image_callback, this, _1));

    ball_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/light_position", 10);

    RCLCPP_INFO(this->get_logger(), "Ball tracker initialized");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_pub_;

  int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
  int size_min_px_, size_max_px_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat bgr_image, hsv_image, mask;

    try {
      bgr_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
      return;
    }

    // Convert to HSV
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    // Thresholding
    cv::inRange(hsv_image, cv::Scalar(h_min_, s_min_, v_min_),
                           cv::Scalar(h_max_, s_max_, v_max_), mask);

    // Morphology
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Blob detection
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float best_size = 0;
    cv::Point2f best_center;

    for (const auto &cnt : contours) {
      float radius;
      cv::Point2f center;
      cv::minEnclosingCircle(cnt, center, radius);
      float size = radius * 2;

      if (size > size_min_px_ && size < size_max_px_ && size > best_size) {
        best_size = size;
        best_center = center;
      }
    }

    if (best_size > 0) {
      geometry_msgs::msg::Point p;
      p.x = best_center.x;
      p.y = best_center.y;
      p.z = best_size;  // Could be used for size filtering

      RCLCPP_INFO(this->get_logger(), "Ball at x=%.1f y=%.1f size=%.1f", p.x, p.y, p.z);
      ball_pub_->publish(p);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallTracker>());
  rclcpp::shutdown();
  return 0;
}
