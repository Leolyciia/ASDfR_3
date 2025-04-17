// #ifndef BALL_TRACKER_HPP_
// #define BALL_TRACKER_HPP_

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include "cv_bridge/cv_bridge.h"
// #include <opencv2/opencv.hpp> 

// namespace ball_tracker
// {

// class BallTracker : public rclcpp::Node {
// public:
//   /**
//    * @brief Construct a new ball tracker object
//    */
//   BallTracker();

// private:
//   /**
//    * @brief Callback function for processing incoming image messages.
//    * @param msg Shared pointer to the incoming sensor_msgs::msg::Image message.
//    */
//   void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

//   // ROS 2 subscribers and publishers
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_pub_;

//   // Parameters for the HSV thresholding and filtering based on size
//   int h_min_, h_max_;
//   int s_min_, s_max_;
//   int v_min_, v_max_;
//   int size_min_px_, size_max_px_;
// };

// } // namespace ball_tracker

// #endif // BALL_TRACKER_HPP_


#ifndef BALL_TRACKER_HPP_
#define BALL_TRACKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

namespace ball_tracker
{

class BallTracker : public rclcpp::Node {
public:
  BallTracker();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_pub_;

  int h_min_, h_max_;
  int s_min_, s_max_;
  int v_min_, v_max_;
  int size_min_px_, size_max_px_;

  cv::Ptr<cv::SimpleBlobDetector> blob_detector_;
};

} // namespace ball_tracker

#endif // BALL_TRACKER_HPP_
