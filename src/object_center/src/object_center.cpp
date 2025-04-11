//==============================================================
// Filename : object_center.cpp
// Authors  : Leonie Hoekstra (s2831465), Moritz Hofmann (s2836459)
// Group    : 30
// Description :
//==============================================================

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class ObjectCenter : public rclcpp::Node {
public:
  ObjectCenter() : Node("object_center") {
    this->declare_parameter("h_min", 68);
    this->declare_parameter("h_max", 95);
    this->declare_parameter("s_min", 221);
    this->declare_parameter("s_max", 255);
    this->declare_parameter("v_min", 131);
    this->declare_parameter("v_max", 198);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {imageCallback(msg);});
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("light_position", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    int h_min = this->get_parameter("h_min").as_int();
    int h_max = this->get_parameter("h_max").as_int();
    int s_min = this->get_parameter("s_min").as_int();
    int s_max = this->get_parameter("s_max").as_int();
    int v_min = this->get_parameter("v_min").as_int();
    int v_max = this->get_parameter("v_max").as_int();

    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);


    cv::Mat mask;
    cv::inRange(
      hsv_image,
      cv::Scalar(h_min, s_min, v_min),  
      cv::Scalar(h_max, s_max, v_max), 
      mask
    );

    // If needed? 
    // cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    // cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

    // Compute image moments to find the centre of the ball
    cv::Moments M = cv::moments(mask, true);
    int cX = 0, cY = 0;
    if (M.m00 != 0) {
      cX = static_cast<int>(M.m10 / M.m00);
      cY = static_cast<int>(M.m01 / M.m00);
    } else {
      RCLCPP_INFO(this->get_logger(), "No object detected based on made HSV filter hehe.");
      cX = -1;  
      cY = -1;
    }

    RCLCPP_INFO(this->get_logger(), "Center of Object: (%d, %d)", cX, cY);

    geometry_msgs::msg::Point point_msg;
    point_msg.x = cX;
    point_msg.y = cY;
    point_msg.z = 0.0;

    publisher_->publish(point_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectCenter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}