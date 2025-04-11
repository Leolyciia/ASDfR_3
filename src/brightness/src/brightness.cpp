//==============================================================
// Filename : brightness.cpp
// Authors  : Leonie Hoekstra (s2831465), Moritz Hofmann (s2836459)
// Group    : 30
// Description : 
//==============================================================

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class BrightnessDetector : public rclcpp::Node {
public:
  BrightnessDetector() : Node("brightness_detector") {

    this->declare_parameter("threshold", 150);
    // lambda, 10 -> how many messages are kept until new messages, then call imageCall with string Msg 
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCall(msg); });
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("/brightness_status", 10);
  }

private:
  void imageCall(const sensor_msgs::msg::Image::SharedPtr msg) {

    if (msg->encoding != "bgr8") {
      RCLCPP_ERROR(this->get_logger(), "We should look how many bits it instead is: %s", msg->encoding.c_str());
      return;
    }
    
    double total_intensity = 0.0;
    size_t pixel_count = msg->height * msg->width;
    for (size_t i = 0; i < msg->data.size(); i += 3) {
      uint8_t b = msg->data[i];
      uint8_t g = msg->data[i + 1];
      uint8_t r = msg->data[i + 2];
      // Found this formula on wikipedia
      double intensity = 0.299 * r + 0.587 * g + 0.114 * b;
      total_intensity += intensity;
    }
    double avg_brightness = total_intensity / pixel_count;

    int threshold = this->get_parameter("threshold").as_int();
    
    std_msgs::msg::String status_msg;
    if (avg_brightness > threshold) {
      status_msg.data = "Light ON";
    } else {
      status_msg.data = "Light OFF";
    }
    
    RCLCPP_INFO(this->get_logger(), "Brightness info: %.2f, Status: %s", avg_brightness, status_msg.data.c_str());
    publisher_->publish(status_msg);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BrightnessDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
