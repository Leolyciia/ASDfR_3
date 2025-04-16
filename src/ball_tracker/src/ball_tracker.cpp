// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include "cv_bridge/cv_bridge.h"
// #include <opencv2/opencv.hpp>

// class BallTracker : public rclcpp::Node {
// public:
//   BallTracker() : Node("ball_tracker") {
//     using std::placeholders::_1;

//     // Parameters (can be tuned here) I made script moehaha
//     h_min_ = 40; h_max_ = 101;
//     s_min_ = 61; s_max_ = 255;
//     v_min_ = 128; v_max_ = 255;
//     size_min_px_ = 100; size_max_px_ = 1600;

//     image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, std::bind(&BallTracker::image_callback, this, _1));

//     ball_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/light_position", 10);

//     RCLCPP_INFO(this->get_logger(), "Ball tracker initialized");
//   }

// private:
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_pub_;

//   int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
//   int size_min_px_, size_max_px_;

//   void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     cv::Mat bgr_image, hsv_image, mask;

//     try {
//       bgr_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
//     } catch (cv_bridge::Exception &e) {
//       RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
//       return;
//     }

//     // Convert to HSV
//     cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

//     // Get the size of the image
//     int image_width = hsv_image.cols;
//     // cv::Point2f target_x = image_width / 2;
//     // int image_height = hsv_image.rows;
//     // RCLCPP_INFO(this->get_logger(), "Image size: width=%d, height=%d", image_width, image_height);

//     // Thresholding
//     cv::inRange(hsv_image, cv::Scalar(h_min_, s_min_, v_min_),
//                            cv::Scalar(h_max_, s_max_, v_max_), mask);

//     // Morphology
//     cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
//     cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
//     cv::imshow("HSV Mask", mask); // Creates/updates a window named "HSV Mask" showing the mask
//     cv::waitKey(1);

//     // Blob detection
//     std::vector<std::vector<cv::Point>> contours;
//     cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//     float best_size = 0;
//     cv::Point2f best_center;

//     for (const auto &cnt : contours) {
//       float radius;
//       cv::Point2f center;
//       cv::minEnclosingCircle(cnt, center, radius);
//       float size = radius * 2;

//       if (size > size_min_px_ && size < size_max_px_ && best_size < size) {
//         best_size = size;
//         best_center = center;
//       }
//       // else {
//       //   best_size = 0;
//       // }
//       RCLCPP_INFO(this->get_logger(), "Best Size:%.1f", best_size);
//       // best_center = center;
//       // best_size = size;
//     }

//     // if (best_size > 0) {
//       geometry_msgs::msg::Point p;
//       p.x = best_center.x;
//       p.y = best_center.y;
//       p.z = best_size;  // Could be used for size filtering

//       RCLCPP_INFO(this->get_logger(), "Ball at x=%.1f y=%.1f size=%.1f", p.x, p.y, p.z);
//       ball_pub_->publish(p);
//     // }
//   }
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<BallTracker>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "ball_tracker.hpp" 

#include <rclcpp/rclcpp.hpp>             
#include <geometry_msgs/msg/point.hpp>  
#include "cv_bridge/cv_bridge.h"         
#include <opencv2/opencv.hpp>            
#include <opencv2/highgui.hpp>           
#include <opencv2/imgproc.hpp>           

#include <string>                       
#include <cstdio>                        

namespace ball_tracker
{

// Define the constructor which belongs to the BallTracker class
BallTracker::BallTracker() : Node("ball_tracker") {
  using std::placeholders::_1;

  // Parameters
  h_min_ = 37; h_max_ = 94;
  s_min_ = 26; s_max_ = 255;
  v_min_ = 113; v_max_ = 255;
  size_min_px_ = 30; size_max_px_ = 1600; 

  // Initialise subscription
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10, std::bind(&BallTracker::image_callback, this, _1));

  // Initialise publisher
  ball_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/light_position", 10);

  RCLCPP_INFO(this->get_logger(), "Ball tracker initialized");
}

// image_callback function is defined which belongs to the BallTracker class
void BallTracker::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv::Mat bgr_image, hsv_image, mask;

  // ROS image message -> OpenCV image
  try {
    bgr_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
    return;
  }

  cv::Mat display_image = bgr_image.clone();

  // BGR -> HSV 
  cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

  // HSV thresholding for getting the mask
  cv::inRange(hsv_image, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);

  // Cleaning up the mask
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  // Find contours in the mask
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  float best_size = 0;
  cv::Point2f best_center(0.0f, 0.0f); // Initialise best_center

  // Find the best contour
  for (const auto &cnt : contours) {
    float radius;
    cv::Point2f center;
    cv::minEnclosingCircle(cnt, center, radius);
    float size = radius * 2; // Diameter

    // Check if current contour meets size criteria and is the best so far
    if (size >= size_min_px_ && size <= size_max_px_) {
      if (size > best_size) {
        best_size = size;
        best_center = center;
      }
    }
  }

  // Draw the best circle and publish if one was found
  if (best_size > 0) {
    // Draw circle and center marker
    int radius_px = static_cast<int>(best_size / 2.0f);
    cv::circle(display_image, best_center, radius_px, cv::Scalar(0, 255, 0), 2);
    cv::line(display_image, cv::Point(best_center.x - 5, best_center.y), cv::Point(best_center.x + 5, best_center.y), cv::Scalar(0, 255, 0), 1);
    cv::line(display_image, cv::Point(best_center.x, best_center.y - 5), cv::Point(best_center.x, best_center.y + 5), cv::Scalar(0, 255, 0), 1);

    // Add size text
    char text_buffer[50];
    snprintf(text_buffer, sizeof(text_buffer), "Size: %.1f px", best_size);
    std::string size_text = text_buffer;
    cv::Point text_origin(10, 25);
    cv::putText(display_image, size_text, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

    // Publish the detected point
    geometry_msgs::msg::Point p;
    p.x = best_center.x;
    p.y = best_center.y;
    p.z = best_size; // Diameter

    RCLCPP_INFO(this->get_logger(), "Ball found: x=%.1f y=%.1f size=%.1f", p.x, p.y, p.z);
    ball_pub_->publish(p);
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No valid ball detected."); // Log less frequently
    geometry_msgs::msg::Point p_no_ball;
    p_no_ball.x = -1.0;
    p_no_ball.y = -1.0;
    p_no_ball.z = 0.0; // no ball detected
    ball_pub_->publish(p_no_ball);
  }

  // Display the camera output with contour and size
  cv::imshow("Detection Output", display_image);
  cv::waitKey(1);
}

} // namespace ball_tracker

// Main function 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ball_tracker::BallTracker>();
  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}

