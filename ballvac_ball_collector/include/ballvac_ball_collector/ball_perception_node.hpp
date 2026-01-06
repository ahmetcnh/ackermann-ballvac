/**
 * @file ball_perception_node.hpp
 * @brief Header for ball perception node - detects colored balls using camera + OpenCV
 * 
 * This node subscribes to camera images and uses HSV color thresholding to detect
 * colored balls (red, green, blue, yellow, orange, purple). For each detected ball,
 * it computes the bearing (angle from image center) and apparent size.
 */

#ifndef BALLVAC_BALL_COLLECTOR__BALL_PERCEPTION_NODE_HPP_
#define BALLVAC_BALL_COLLECTOR__BALL_PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ballvac_msgs/msg/ball_detection.hpp>
#include <ballvac_msgs/msg/ball_detection_array.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>
#include <map>

namespace ballvac_ball_collector
{

/**
 * @brief Structure to hold HSV color range for ball detection
 */
struct ColorRange
{
    std::string name;           // Color name (e.g., "red", "green")
    cv::Scalar lower1;          // Lower HSV bound (primary range)
    cv::Scalar upper1;          // Upper HSV bound (primary range)
    cv::Scalar lower2;          // Lower HSV bound (secondary range, for red which wraps around)
    cv::Scalar upper2;          // Upper HSV bound (secondary range)
    bool has_secondary;         // Whether this color has two ranges (like red)
};

/**
 * @brief Structure to hold detection result for a single ball
 */
struct BallDetectionResult
{
    std::string color;
    cv::Point2f center;
    float radius;
    float confidence;
};

/**
 * @class BallPerceptionNode
 * @brief ROS 2 node that detects colored balls in camera images
 */
class BallPerceptionNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes the node with parameters and subscriptions
     */
    explicit BallPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    ~BallPerceptionNode() = default;

private:
    /**
     * @brief Callback for incoming camera images
     * @param msg The image message from the camera
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Initialize the HSV color ranges for each ball color
     */
    void initialize_color_ranges();

    /**
     * @brief Detect balls of a specific color in the image
     * @param hsv_image The image converted to HSV color space
     * @param color_range The color range to detect
     * @return Vector of detection results for this color
     */
    std::vector<BallDetectionResult> detect_color(
        const cv::Mat & hsv_image,
        const ColorRange & color_range);

    /**
     * @brief Calculate bearing angle from image center
     * @param center_x X coordinate of ball center in image
     * @param image_width Width of the image
     * @return Bearing angle in radians (positive = right, negative = left)
     */
    float calculate_bearing(float center_x, int image_width);

    // ROS 2 communication
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<ballvac_msgs::msg::BallDetectionArray>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    // Parameters
    std::string image_topic_;
    std::string detection_topic_;
    bool publish_debug_image_;
    int min_contour_area_;
    double camera_hfov_;  // Horizontal field of view in radians

    // Color ranges for detection
    std::vector<ColorRange> color_ranges_;

    // Morphological operation kernels
    cv::Mat erode_kernel_;
    cv::Mat dilate_kernel_;
};

}  // namespace ballvac_ball_collector

#endif  // BALLVAC_BALL_COLLECTOR__BALL_PERCEPTION_NODE_HPP_
