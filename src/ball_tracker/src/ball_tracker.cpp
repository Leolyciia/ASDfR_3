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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
// Make sure highgui is included for imshow and waitKey
#include <opencv2/highgui.hpp>

class BallTracker : public rclcpp::Node {
public:
    BallTracker() : Node("ball_tracker") {
        using std::placeholders::_1;

        // Parameters (updated based on your last snippet)
        h_min_ = 40; h_max_ = 101;
        s_min_ = 61; s_max_ = 255;
        v_min_ = 128; v_max_ = 255;
        size_min_px_ = 30; size_max_px_ = 1600; // Using the values from your latest code post

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&BallTracker::image_callback, this, _1));

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

        // --- Create a copy of the image to draw on ---
        cv::Mat display_image = bgr_image.clone();

        // Convert to HSV
        cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

        // Thresholding
        cv::inRange(hsv_image, cv::Scalar(h_min_, s_min_, v_min_),
                    cv::Scalar(h_max_, s_max_, v_max_), mask);

        // Morphology
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // --- Display the intermediate mask (optional) ---
        // cv::imshow("HSV Mask", mask); // Keep or comment out as needed

        // Blob detection
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        float best_size = 0;
        cv::Point2f best_center;

        // --- Find the best contour/circle ---
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
        // Note: The RCLCPP_INFO_THROTTLE inside the loop was potentially misleading,
        // it printed the current best_size after *every* contour check.
        // It's better to log the final result after the loop.

        // --- Draw the best circle if one was found ---
        if (best_size > 0) {
            // Calculate radius (integer needed for drawing function)
            int radius_px = static_cast<int>(best_size / 2.0f);
            // Draw the circle on the display image (Green color, thickness 2)
            cv::circle(display_image, best_center, radius_px, cv::Scalar(0, 255, 0), 2);
            // Draw a small cross at the center for better visibility
            cv::line(display_image, cv::Point(best_center.x - 5, best_center.y), cv::Point(best_center.x + 5, best_center.y), cv::Scalar(0, 255, 0), 1);
            cv::line(display_image, cv::Point(best_center.x, best_center.y - 5), cv::Point(best_center.x, best_center.y + 5), cv::Scalar(0, 255, 0), 1);

            // --- Add text for size ---
            char text_buffer[50]; // Buffer to hold the formatted string
            // Format the string: "Size: XX.X px"
            snprintf(text_buffer, sizeof(text_buffer), "Size: %.1f px", best_size);
            std::string size_text = text_buffer;

            // Define text position (e.g., top-left corner)
            cv::Point text_origin(10, 25); // 10px from left, 25px from top

            // Define font properties
            int font_face = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.7;
            cv::Scalar font_color(255, 255, 0); // Cyan BGR color
            int font_thickness = 2;

            // Put the text on the image
            cv::putText(display_image, size_text, text_origin, font_face, font_scale, font_color, font_thickness);
            // --- End text adding ---

            // Publish the detected point
            geometry_msgs::msg::Point p;
            p.x = best_center.x;
            p.y = best_center.y;
            p.z = best_size; // Diameter

            // Log the final detected ball info
            RCLCPP_INFO(this->get_logger(), "Ball found: x=%.1f y=%.1f size=%.1f", p.x, p.y, p.z);
            ball_pub_->publish(p);
        } else {
            // No valid ball found in this frame
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No valid ball detected."); // Log less frequently when nothing is found
             // Publish point indicating no detection (e.g., size = 0 or -1)
             geometry_msgs::msg::Point p_no_ball;
             p_no_ball.x = -1.0; // Or image center?
             p_no_ball.y = -1.0; // Or image center?
             p_no_ball.z = 0.0;  // Indicate size 0 for no detection
             ball_pub_->publish(p_no_ball);
        }

        // --- Display the image with drawings ---
        cv::imshow("Detection Output", display_image);
        // --- Process GUI events for ALL imshow windows ---
        cv::waitKey(1);

    }
}; // End class BallTracker

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallTracker>());
    // Add window destruction (optional but good practice)
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}