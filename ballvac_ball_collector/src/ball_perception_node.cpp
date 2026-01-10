/**
 * @file ball_perception_node.cpp
 * @brief Implementation of ball perception node using OpenCV for color detection
 * 
 * This node detects colored balls (red, green, blue, yellow, orange, purple) in
 * camera images using HSV color thresholding, morphological operations, and
 * contour detection. It publishes detected balls with their bearing and size.
 */

#include "ballvac_ball_collector/ball_perception_node.hpp"

#include <algorithm>
#include <cmath>

namespace ballvac_ball_collector
{

// =============================================================================
// Constructor
// =============================================================================

BallPerceptionNode::BallPerceptionNode(const rclcpp::NodeOptions & options)
: Node("ball_perception_node", options)
{
    // -------------------------------------------------------------------------
    // Declare and get parameters
    // -------------------------------------------------------------------------
    
    // Image topic - default based on the repo's ros_gz_bridge configuration
    this->declare_parameter<std::string>("image_topic", "/camera/front_raw");
    image_topic_ = this->get_parameter("image_topic").as_string();
    
    // Output topic for detections
    this->declare_parameter<std::string>("detection_topic", "/ball_detections");
    detection_topic_ = this->get_parameter("detection_topic").as_string();
    
    // Debug image publishing
    this->declare_parameter<bool>("publish_debug_image", true);
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
    
    // Minimum contour area to filter noise (in pixels^2)
    this->declare_parameter<int>("min_contour_area", 100);
    min_contour_area_ = this->get_parameter("min_contour_area").as_int();
    
    // Camera horizontal field of view (from model.sdf: 1.658 rad for front camera)
    this->declare_parameter<double>("camera_hfov", 1.658);
    camera_hfov_ = this->get_parameter("camera_hfov").as_double();

    // -------------------------------------------------------------------------
    // Initialize color ranges for ball detection
    // -------------------------------------------------------------------------
    initialize_color_ranges();

    // -------------------------------------------------------------------------
    // Create morphological operation kernels
    // -------------------------------------------------------------------------
    erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));

    // -------------------------------------------------------------------------
    // Create ROS 2 publishers and subscribers
    // -------------------------------------------------------------------------
    
    // Subscriber for camera images
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&BallPerceptionNode::image_callback, this, std::placeholders::_1));
    
    // Publisher for ball detections
    detection_pub_ = this->create_publisher<ballvac_msgs::msg::BallDetectionArray>(
        detection_topic_, 10);
    
    // Publisher for debug image (if enabled)
    if (publish_debug_image_)
    {
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/ball_perception/debug_image", 10);
    }

    // -------------------------------------------------------------------------
    // Log startup information
    // -------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Ball Perception Node Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Detection topic: %s", detection_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Debug image: %s", publish_debug_image_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Min contour area: %d px^2", min_contour_area_);
    RCLCPP_INFO(this->get_logger(), "  Camera HFOV: %.3f rad", camera_hfov_);
    RCLCPP_INFO(this->get_logger(), "  Detecting colors: red, green, blue, cyan, orange, yellow, purple");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

// =============================================================================
// Initialize HSV color ranges for each ball color
// =============================================================================

void BallPerceptionNode::initialize_color_ranges()
{
    // HSV ranges tuned for typical colored balls in simulation
    // H: 0-179, S: 0-255, V: 0-255 in OpenCV
    
    // RED - needs two ranges because red wraps around in HSV
    // Range 1: Low red (0-10)
    // Range 2: High red (160-179)
    ColorRange red;
    red.name = "red";
    red.lower1 = cv::Scalar(0, 70, 50);
    red.upper1 = cv::Scalar(10, 255, 255);
    red.lower2 = cv::Scalar(160, 70, 50);
    red.upper2 = cv::Scalar(179, 255, 255);
    red.has_secondary = true;
    color_ranges_.push_back(red);

    // GREEN
    ColorRange green;
    green.name = "green";
    green.lower1 = cv::Scalar(35, 100, 100);
    green.upper1 = cv::Scalar(85, 255, 255);
    green.has_secondary = false;
    color_ranges_.push_back(green);

    // BLUE
    ColorRange blue;
    blue.name = "blue";
    blue.lower1 = cv::Scalar(100, 100, 100);
    blue.upper1 = cv::Scalar(130, 255, 255);
    blue.has_secondary = false;
    color_ranges_.push_back(blue);

    // CYAN/TURQUOISE - bright blue-green color
    ColorRange cyan;
    cyan.name = "cyan";
    cyan.lower1 = cv::Scalar(80, 100, 100);  // Hue 80-100 for cyan
    cyan.upper1 = cv::Scalar(100, 255, 255);
    cyan.has_secondary = false;
    color_ranges_.push_back(cyan);

    // ORANGE - between red and yellow
    ColorRange orange;
    orange.name = "orange";
    orange.lower1 = cv::Scalar(11, 70, 50);
    orange.upper1 = cv::Scalar(21, 255, 255);
    orange.has_secondary = false;
    color_ranges_.push_back(orange);

    // YELLOW - starts after orange range
    ColorRange yellow;
    yellow.name = "yellow";
    yellow.lower1 = cv::Scalar(22, 100, 100);  // Start at 22 to avoid orange overlap
    yellow.upper1 = cv::Scalar(35, 255, 255);
    yellow.has_secondary = false;
    color_ranges_.push_back(yellow);

    // PURPLE/MAGENTA
    ColorRange purple;
    purple.name = "purple";
    purple.lower1 = cv::Scalar(130, 100, 100);
    purple.upper1 = cv::Scalar(160, 255, 255);
    purple.has_secondary = false;
    color_ranges_.push_back(purple);
}

// =============================================================================
// Image callback - main processing function
// =============================================================================

void BallPerceptionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // -------------------------------------------------------------------------
    // Convert ROS image to OpenCV format
    // -------------------------------------------------------------------------
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat & frame = cv_ptr->image;
    
    // Check if image is valid
    if (frame.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Received empty image");
        return;
    }

    // -------------------------------------------------------------------------
    // Convert to HSV color space for color detection
    // -------------------------------------------------------------------------
    cv::Mat hsv_image;
    cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);

    // -------------------------------------------------------------------------
    // Detect balls for each color
    // -------------------------------------------------------------------------
    std::vector<BallDetectionResult> all_detections;
    
    for (const auto & color_range : color_ranges_)
    {
        auto detections = detect_color(hsv_image, color_range);
        all_detections.insert(all_detections.end(), detections.begin(), detections.end());
    }

    // -------------------------------------------------------------------------
    // Create and publish detection message
    // -------------------------------------------------------------------------
    ballvac_msgs::msg::BallDetectionArray detection_array_msg;
    detection_array_msg.header = msg->header;
    detection_array_msg.header.frame_id = "camera_front";

    for (const auto & det : all_detections)
    {
        ballvac_msgs::msg::BallDetection ball_msg;
        ball_msg.header = msg->header;
        ball_msg.name = "ball_" + det.color;
        ball_msg.color = det.color;
        ball_msg.bearing = calculate_bearing(det.center.x, frame.cols);
        ball_msg.apparent_size = det.radius;
        ball_msg.confidence = det.confidence;
        ball_msg.center_x = static_cast<int>(det.center.x);
        ball_msg.center_y = static_cast<int>(det.center.y);
        
        detection_array_msg.detections.push_back(ball_msg);
    }

    detection_pub_->publish(detection_array_msg);

    // -------------------------------------------------------------------------
    // Publish debug image if enabled
    // -------------------------------------------------------------------------
    if (publish_debug_image_ && debug_image_pub_)
    {
        cv::Mat debug_frame = frame.clone();
        
        // Draw detections on debug image
        for (const auto & det : all_detections)
        {
            // Choose color for drawing based on detected color
            cv::Scalar draw_color;
            if (det.color == "red") draw_color = cv::Scalar(0, 0, 255);
            else if (det.color == "green") draw_color = cv::Scalar(0, 255, 0);
            else if (det.color == "blue") draw_color = cv::Scalar(255, 0, 0);
            else if (det.color == "yellow") draw_color = cv::Scalar(0, 255, 255);
            else if (det.color == "orange") draw_color = cv::Scalar(0, 165, 255);
            else if (det.color == "purple") draw_color = cv::Scalar(255, 0, 255);
            else draw_color = cv::Scalar(255, 255, 255);

            // Draw circle around detection
            cv::circle(debug_frame, det.center, static_cast<int>(det.radius), 
                       draw_color, 2);
            
            // Draw center point
            cv::circle(debug_frame, det.center, 3, draw_color, -1);
            
            // Draw label
            std::string label = det.color + " r=" + std::to_string(static_cast<int>(det.radius));
            cv::putText(debug_frame, label,
                        cv::Point(det.center.x - 30, det.center.y - det.radius - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2);
        }
        
        // Draw image center crosshair
        int cx = frame.cols / 2;
        int cy = frame.rows / 2;
        cv::line(debug_frame, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy),
                 cv::Scalar(255, 255, 255), 1);
        cv::line(debug_frame, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20),
                 cv::Scalar(255, 255, 255), 1);

        // Convert back to ROS message and publish
        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_frame).toImageMsg();
        debug_image_pub_->publish(*debug_msg);
    }

    // Log detection summary periodically
    if (!all_detections.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "Detected %zu balls", all_detections.size());
    }
}

// =============================================================================
// Detect balls of a specific color
// =============================================================================

std::vector<BallDetectionResult> BallPerceptionNode::detect_color(
    const cv::Mat & hsv_image,
    const ColorRange & color_range)
{
    std::vector<BallDetectionResult> results;

    // -------------------------------------------------------------------------
    // Create mask for this color
    // -------------------------------------------------------------------------
    cv::Mat mask;
    cv::inRange(hsv_image, color_range.lower1, color_range.upper1, mask);
    
    // For colors like red that wrap around HSV, combine two ranges
    if (color_range.has_secondary)
    {
        cv::Mat mask2;
        cv::inRange(hsv_image, color_range.lower2, color_range.upper2, mask2);
        cv::bitwise_or(mask, mask2, mask);
    }

    // -------------------------------------------------------------------------
    // Apply morphological operations to reduce noise
    // -------------------------------------------------------------------------
    // Erode to remove small noise
    cv::erode(mask, mask, erode_kernel_, cv::Point(-1, -1), 2);
    // Dilate to restore blob size and fill small holes
    cv::dilate(mask, mask, dilate_kernel_, cv::Point(-1, -1), 2);

    // -------------------------------------------------------------------------
    // Find contours
    // -------------------------------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // -------------------------------------------------------------------------
    // Find the largest contour that meets our criteria
    // -------------------------------------------------------------------------
    double max_area = 0;
    int best_idx = -1;
    
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        
        // Filter by minimum area
        if (area < min_contour_area_)
        {
            continue;
        }
        
        // Check circularity (balls should be roughly circular)
        double perimeter = cv::arcLength(contours[i], true);
        double circularity = 4.0 * M_PI * area / (perimeter * perimeter);
        
        // Accept contours with circularity > 0.5 (perfect circle = 1.0)
        // Increased from 0.3 to filter out rectangular wall/obstacle detections
        if (circularity < 0.5)
        {
            continue;
        }
        
        if (area > max_area)
        {
            max_area = area;
            best_idx = static_cast<int>(i);
        }
    }

    // -------------------------------------------------------------------------
    // If we found a valid contour, compute detection result
    // -------------------------------------------------------------------------
    if (best_idx >= 0)
    {
        BallDetectionResult result;
        result.color = color_range.name;
        
        // Fit minimum enclosing circle
        cv::minEnclosingCircle(contours[best_idx], result.center, result.radius);
        
        // Compute confidence based on circularity and area
        double perimeter = cv::arcLength(contours[best_idx], true);
        double circularity = 4.0 * M_PI * max_area / (perimeter * perimeter);
        result.confidence = static_cast<float>(std::min(1.0, circularity));
        
        results.push_back(result);
    }

    return results;
}

// =============================================================================
// Calculate bearing angle from image center
// =============================================================================

float BallPerceptionNode::calculate_bearing(float center_x, int image_width)
{
    // Calculate offset from image center as fraction (-0.5 to 0.5)
    float offset_fraction = (center_x - (image_width / 2.0f)) / image_width;
    
    // Convert to angle based on camera horizontal FOV
    // Positive bearing = ball is to the right
    // Negative bearing = ball is to the left
    float bearing = offset_fraction * static_cast<float>(camera_hfov_);
    
    return bearing;
}

}  // namespace ballvac_ball_collector

// =============================================================================
// Main function
// =============================================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ballvac_ball_collector::BallPerceptionNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
