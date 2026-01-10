/**
 * @file exploration_collector_node.cpp
 * @brief Implementation of exploration and collection node
 * 
 * Explores the room using wall-following and frontier-based exploration,
 * builds a map using SLAM, then collects balls.
 */

#include "ballvac_ball_collector/exploration_collector_node.hpp"
#include <cmath>
#include <algorithm>
#include <tf2/utils.h>

namespace ballvac_ball_collector
{

// =============================================================================
// Constructor
// =============================================================================

ExplorationCollectorNode::ExplorationCollectorNode(const rclcpp::NodeOptions & options)
: Node("exploration_collector_node", options),
  current_state_(ExplorationState::WAITING),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0),
  have_pose_(false),
  have_frontier_(false),
  exploration_progress_(0.0),
  delete_pending_(false),
  balls_collected_(0),
  recovery_phase_(0),
  recovery_turn_dir_(1.0),
  last_x_(0.0),
  last_y_(0.0)
{
    // -------------------------------------------------------------------------
    // Declare parameters
    // -------------------------------------------------------------------------
    
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("cmd_topic", "/cmd_vel_in");
    this->declare_parameter<std::string>("detection_topic", "/ball_detections");
    this->declare_parameter<std::string>("delete_service", "/world/ball_arena/remove");
    
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    cmd_topic_ = this->get_parameter("cmd_topic").as_string();
    detection_topic_ = this->get_parameter("detection_topic").as_string();
    delete_service_ = this->get_parameter("delete_service").as_string();
    
    this->declare_parameter<double>("exploration_speed", 0.4);
    this->declare_parameter<double>("collection_speed", 0.3);
    this->declare_parameter<double>("max_steer", 0.5);
    this->declare_parameter<double>("obstacle_stop_distance", 0.5);
    this->declare_parameter<double>("obstacle_slow_distance", 1.5);
    this->declare_parameter<double>("frontier_min_size", 5);
    this->declare_parameter<double>("exploration_threshold", 85.0);
    
    exploration_speed_ = this->get_parameter("exploration_speed").as_double();
    collection_speed_ = this->get_parameter("collection_speed").as_double();
    max_steer_ = this->get_parameter("max_steer").as_double();
    obstacle_stop_distance_ = this->get_parameter("obstacle_stop_distance").as_double();
    obstacle_slow_distance_ = this->get_parameter("obstacle_slow_distance").as_double();
    frontier_min_size_ = this->get_parameter("frontier_min_size").as_double();
    exploration_threshold_ = this->get_parameter("exploration_threshold").as_double();
    
    // -------------------------------------------------------------------------
    // Initialize timing
    // -------------------------------------------------------------------------
    exploration_start_time_ = this->now();
    last_progress_time_ = this->now();
    recovery_start_time_ = this->now();
    
    current_target_.valid = false;
    
    // -------------------------------------------------------------------------
    // Create subscribers
    // -------------------------------------------------------------------------
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ExplorationCollectorNode::scan_callback, this, std::placeholders::_1));
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 10,
        std::bind(&ExplorationCollectorNode::map_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&ExplorationCollectorNode::odom_callback, this, std::placeholders::_1));
    
    detection_sub_ = this->create_subscription<ballvac_msgs::msg::BallDetectionArray>(
        detection_topic_, 10,
        std::bind(&ExplorationCollectorNode::detection_callback, this, std::placeholders::_1));
    
    // -------------------------------------------------------------------------
    // Create publishers and clients
    // -------------------------------------------------------------------------
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    
    delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>(delete_service_);
    
    // -------------------------------------------------------------------------
    // Control timer
    // -------------------------------------------------------------------------
    
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ExplorationCollectorNode::control_loop, this));
    
    // -------------------------------------------------------------------------
    // Log startup
    // -------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Exploration Collector Node Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  Phase 1: Explore room and build map");
    RCLCPP_INFO(this->get_logger(), "  Phase 2: Collect balls using map");
    RCLCPP_INFO(this->get_logger(), "  Exploration threshold: %.0f%%", exploration_threshold_);
    RCLCPP_INFO(this->get_logger(), "========================================");
}

// =============================================================================
// Callbacks
// =============================================================================

void ExplorationCollectorNode::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
}

void ExplorationCollectorNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_map_ = msg;
    
    // Calculate exploration progress
    if (latest_map_)
    {
        exploration_progress_ = calculate_exploration_progress();
        
        // Log progress periodically
        static int log_counter = 0;
        if (++log_counter >= 100)  // Every ~5 seconds at 20Hz
        {
            log_counter = 0;
            RCLCPP_INFO(this->get_logger(), 
                "Exploration progress: %.1f%% (state: %s)",
                exploration_progress_, state_to_string(current_state_).c_str());
        }
    }
}

void ExplorationCollectorNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_yaw_);
    
    have_pose_ = true;
}

void ExplorationCollectorNode::detection_callback(
    const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg)
{
    // During exploration, we can opportunistically collect balls we see
    // During collection phase, actively seek balls
    
    if (current_state_ == ExplorationState::COLLECTING ||
        current_state_ == ExplorationState::EXPLORING)
    {
        if (!msg->detections.empty() && !current_target_.valid)
        {
            // Select largest ball
            const auto & best = *std::max_element(
                msg->detections.begin(), msg->detections.end(),
                [](const auto & a, const auto & b) {
                    return a.apparent_size < b.apparent_size;
                });
            
            current_target_.name = best.name;
            current_target_.color = best.color;
            current_target_.bearing = best.bearing;
            current_target_.size = best.apparent_size;
            current_target_.valid = true;
            
            // If exploring and we see a ball, switch to approaching
            if (current_state_ == ExplorationState::EXPLORING)
            {
                RCLCPP_INFO(this->get_logger(), 
                    "Ball spotted during exploration: %s (size=%.1f)",
                    current_target_.color.c_str(), current_target_.size);
                transition_to(ExplorationState::APPROACHING);
            }
        }
        else if (current_target_.valid && !msg->detections.empty())
        {
            // Update target tracking
            for (const auto & det : msg->detections)
            {
                if (det.color == current_target_.color)
                {
                    current_target_.bearing = det.bearing;
                    current_target_.size = det.apparent_size;
                    break;
                }
            }
        }
    }
}

// =============================================================================
// Control loop
// =============================================================================

void ExplorationCollectorNode::control_loop()
{
    switch (current_state_)
    {
        case ExplorationState::WAITING:
            execute_waiting();
            break;
        case ExplorationState::EXPLORING:
            execute_exploring();
            break;
        case ExplorationState::RETURNING:
            execute_returning();
            break;
        case ExplorationState::MAP_COMPLETE:
            execute_map_complete();
            break;
        case ExplorationState::COLLECTING:
            execute_collecting();
            break;
        case ExplorationState::APPROACHING:
            execute_approaching();
            break;
        case ExplorationState::RECOVERING:
            execute_recovering();
            break;
    }
}

// =============================================================================
// State: WAITING - Wait for SLAM to initialize
// =============================================================================

void ExplorationCollectorNode::execute_waiting()
{
    publish_cmd_vel(0.0, 0.0);
    
    if (latest_map_ && latest_scan_ && have_pose_)
    {
        RCLCPP_INFO(this->get_logger(), "SLAM initialized, starting exploration");
        exploration_start_time_ = this->now();
        transition_to(ExplorationState::EXPLORING);
    }
}

// =============================================================================
// State: EXPLORING - Explore using wall-following
// =============================================================================

void ExplorationCollectorNode::execute_exploring()
{
    // Check if exploration is complete
    if (exploration_progress_ >= exploration_threshold_)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Exploration complete! Progress: %.1f%%", exploration_progress_);
        transition_to(ExplorationState::MAP_COMPLETE);
        return;
    }
    
    // Use wall-following behavior for exploration
    wall_follow();
    
    // Check for stuck condition
    double dist_moved = std::sqrt(
        std::pow(robot_x_ - last_x_, 2) + std::pow(robot_y_ - last_y_, 2));
    
    if (dist_moved > 0.1)
    {
        last_progress_time_ = this->now();
        last_x_ = robot_x_;
        last_y_ = robot_y_;
    }
    else
    {
        double stuck_time = (this->now() - last_progress_time_).seconds();
        if (stuck_time > 5.0)
        {
            RCLCPP_WARN(this->get_logger(), "Stuck during exploration, recovering");
            transition_to(ExplorationState::RECOVERING);
        }
    }
}

// =============================================================================
// State: RETURNING - Return to a frontier (not used in wall-following)
// =============================================================================

void ExplorationCollectorNode::execute_returning()
{
    // For simplicity, just go back to exploring
    transition_to(ExplorationState::EXPLORING);
}

// =============================================================================
// State: MAP_COMPLETE - Transition to collection
// =============================================================================

void ExplorationCollectorNode::execute_map_complete()
{
    publish_cmd_vel(0.0, 0.0);
    
    double exploration_time = (this->now() - exploration_start_time_).seconds();
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "MAP COMPLETE!");
    RCLCPP_INFO(this->get_logger(), "  Exploration time: %.1f seconds", exploration_time);
    RCLCPP_INFO(this->get_logger(), "  Coverage: %.1f%%", exploration_progress_);
    RCLCPP_INFO(this->get_logger(), "  Now switching to ball collection mode");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // Short pause then start collecting
    rclcpp::sleep_for(std::chrono::seconds(2));
    transition_to(ExplorationState::COLLECTING);
}

// =============================================================================
// State: COLLECTING - Search for and collect balls
// =============================================================================

void ExplorationCollectorNode::execute_collecting()
{
    // If we have a target, approach it
    if (current_target_.valid)
    {
        transition_to(ExplorationState::APPROACHING);
        return;
    }
    
    // Otherwise, continue moving to find balls
    static int navigating_log_counter = 0;
    if (++navigating_log_counter >= 40)  // Every ~2 seconds at 20Hz
    {
        navigating_log_counter = 0;
        RCLCPP_INFO(this->get_logger(), "Navigating to search for balls");
    }
    wall_follow();
}

// =============================================================================
// State: APPROACHING - Approach detected ball
// =============================================================================

void ExplorationCollectorNode::execute_approaching()
{
    if (!current_target_.valid)
    {
        RCLCPP_INFO(this->get_logger(), "Lost target, returning to previous state");
        if (exploration_progress_ >= exploration_threshold_)
            transition_to(ExplorationState::COLLECTING);
        else
            transition_to(ExplorationState::EXPLORING);
        return;
    }
    
    static int approaching_log_counter = 0;
    if (++approaching_log_counter >= 20)  // Every ~1 second at 20Hz
    {
        approaching_log_counter = 0;
        RCLCPP_INFO(this->get_logger(),
            "Approaching ball: %s (bearing=%.2f size=%.1f)",
            current_target_.color.c_str(), current_target_.bearing, current_target_.size);
    }

    float linear_vel = collection_speed_;
    float angular_vel = -2.0f * current_target_.bearing;  // Proportional steering
    angular_vel = std::clamp(angular_vel, -static_cast<float>(max_steer_), 
                             static_cast<float>(max_steer_));
    
    // Check obstacles
    float min_range;
    if (check_obstacles(min_range))
    {
        if (min_range < 0.5)
        {
            // Very close - try to collect
            collect_ball();
            return;
        }
        
        // Blend avoidance
        float avoid_steer = compute_avoidance_steering();
        angular_vel = 0.6f * avoid_steer + 0.4f * angular_vel;
        linear_vel *= 0.5f;
    }
    
    // Check if ball is large enough to collect
    if (current_target_.size > 120.0f)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Ball close enough (size=%.1f), collecting", current_target_.size);
        collect_ball();
        return;
    }
    
    publish_cmd_vel(linear_vel, angular_vel);
}

// =============================================================================
// State: RECOVERING
// =============================================================================

void ExplorationCollectorNode::execute_recovering()
{
    double time_in_recover = (this->now() - recovery_start_time_).seconds();
    
    if (recovery_phase_ == 0)
    {
        // Back up
        if (time_in_recover < 1.5)
        {
            publish_cmd_vel(-0.3, 0.0);
        }
        else
        {
            recovery_phase_ = 1;
        }
    }
    else if (recovery_phase_ == 1)
    {
        // Turn
        if (time_in_recover < 3.0)
        {
            publish_cmd_vel(0.0, recovery_turn_dir_ * max_steer_);
        }
        else
        {
            // Done
            last_progress_time_ = this->now();
            last_x_ = robot_x_;
            last_y_ = robot_y_;
            recovery_turn_dir_ *= -1.0f;  // Alternate direction
            
            if (exploration_progress_ >= exploration_threshold_)
                transition_to(ExplorationState::COLLECTING);
            else
                transition_to(ExplorationState::EXPLORING);
        }
    }
}

// =============================================================================
// State transition
// =============================================================================

void ExplorationCollectorNode::transition_to(ExplorationState new_state)
{
    if (new_state == current_state_) return;
    
    RCLCPP_INFO(this->get_logger(), "State: %s -> %s",
        state_to_string(current_state_).c_str(),
        state_to_string(new_state).c_str());
    
    if (new_state == ExplorationState::RECOVERING)
    {
        recovery_start_time_ = this->now();
        recovery_phase_ = 0;
    }
    
    current_state_ = new_state;
}

std::string ExplorationCollectorNode::state_to_string(ExplorationState state)
{
    switch (state)
    {
        case ExplorationState::WAITING: return "WAITING";
        case ExplorationState::EXPLORING: return "EXPLORING";
        case ExplorationState::RETURNING: return "RETURNING";
        case ExplorationState::MAP_COMPLETE: return "MAP_COMPLETE";
        case ExplorationState::COLLECTING: return "COLLECTING";
        case ExplorationState::APPROACHING: return "APPROACHING";
        case ExplorationState::RECOVERING: return "RECOVERING";
        default: return "UNKNOWN";
    }
}

// =============================================================================
// Wall following behavior
// =============================================================================

void ExplorationCollectorNode::wall_follow()
{
    if (!latest_scan_)
    {
        publish_cmd_vel(0.0, 0.0);
        return;
    }
    
    float min_range;
    bool obstacle = check_obstacles(min_range);
    
    float linear_vel = exploration_speed_;
    float angular_vel = 0.0;
    
    if (obstacle)
    {
        angular_vel = compute_avoidance_steering();
        
        if (min_range < obstacle_stop_distance_)
        {
            linear_vel = 0.0;
        }
        else
        {
            linear_vel = exploration_speed_ * (min_range / obstacle_slow_distance_);
        }
    }
    else
    {
        // Gentle curve to the right for wall following
        angular_vel = -0.15;
    }
    
    publish_cmd_vel(linear_vel, angular_vel);
}

// =============================================================================
// Check for obstacles
// =============================================================================

bool ExplorationCollectorNode::check_obstacles(float & min_range)
{
    if (!latest_scan_)
    {
        min_range = 10.0;
        return false;
    }
    
    min_range = std::numeric_limits<float>::max();
    size_t num_readings = latest_scan_->ranges.size();
    
    // Check front 90 degrees
    size_t start = num_readings * 3 / 8;
    size_t end = num_readings * 5 / 8;
    
    for (size_t i = start; i < end; ++i)
    {
        float r = latest_scan_->ranges[i];
        if (std::isfinite(r) && r > latest_scan_->range_min)
        {
            min_range = std::min(min_range, r);
        }
    }
    
    return min_range < obstacle_slow_distance_;
}

// =============================================================================
// Compute avoidance steering
// =============================================================================

float ExplorationCollectorNode::compute_avoidance_steering()
{
    if (!latest_scan_) return 0.0;
    
    size_t num_readings = latest_scan_->ranges.size();
    
    // Sum ranges on left and right
    float left_sum = 0.0, right_sum = 0.0;
    int left_count = 0, right_count = 0;
    
    // Left side (first quarter)
    for (size_t i = 0; i < num_readings / 4; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]))
        {
            left_sum += latest_scan_->ranges[i];
            left_count++;
        }
    }
    
    // Right side (last quarter)
    for (size_t i = num_readings * 3 / 4; i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]))
        {
            right_sum += latest_scan_->ranges[i];
            right_count++;
        }
    }
    
    float left_avg = left_count > 0 ? left_sum / left_count : 10.0;
    float right_avg = right_count > 0 ? right_sum / right_count : 10.0;
    
    // Turn towards more open space
    if (left_avg > right_avg)
    {
        return max_steer_;  // Turn left
    }
    else
    {
        return -max_steer_;  // Turn right
    }
}

// =============================================================================
// Calculate exploration progress
// =============================================================================

double ExplorationCollectorNode::calculate_exploration_progress()
{
    if (!latest_map_) return 0.0;
    
    int known_cells = 0;
    int total_cells = latest_map_->info.width * latest_map_->info.height;
    
    for (const auto & cell : latest_map_->data)
    {
        if (cell != -1)  // -1 is unknown
        {
            known_cells++;
        }
    }
    
    return 100.0 * static_cast<double>(known_cells) / static_cast<double>(total_cells);
}

// =============================================================================
// Publish velocity
// =============================================================================

void ExplorationCollectorNode::publish_cmd_vel(float linear, float angular)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_vel_pub_->publish(msg);
}

// =============================================================================
// Collection functions
// =============================================================================

void ExplorationCollectorNode::collect_ball()
{
    publish_cmd_vel(0.0, 0.0);
    
    if (!current_target_.valid || delete_pending_)
    {
        if (!current_target_.valid)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot collect: no valid target");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Cannot collect: delete already pending");
        }
        return;
    }
    
    std::string entity_name = get_entity_name(current_target_.color);
    delete_entity(entity_name);
}

void ExplorationCollectorNode::delete_entity(const std::string & name)
{
    if (!delete_client_->service_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Delete service not ready");
        current_target_.valid = false;
        return;
    }
    
    delete_pending_ = true;
    
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = name;
    
    RCLCPP_INFO(this->get_logger(), "Collecting ball: %s", name.c_str());
    
    auto future = delete_client_->async_send_request(
        request,
        std::bind(&ExplorationCollectorNode::delete_callback, this, std::placeholders::_1));
}

void ExplorationCollectorNode::delete_callback(
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future)
{
    delete_pending_ = false;
    
    try
    {
        auto response = future.get();
        if (response->success)
        {
            balls_collected_++;
            RCLCPP_INFO(this->get_logger(), 
                "Collected %s ball! Total: %d",
                current_target_.color.c_str(), balls_collected_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "Failed to collect %s ball: delete service returned failure",
                current_target_.color.c_str());
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), "Delete error: %s", e.what());
    }
    
    current_target_.valid = false;
    
    if (exploration_progress_ >= exploration_threshold_)
        transition_to(ExplorationState::COLLECTING);
    else
        transition_to(ExplorationState::EXPLORING);
}

std::string ExplorationCollectorNode::get_entity_name(const std::string & color)
{
    // Try launched ball names first, then static names
    return "ball_" + color;
}

}  // namespace ballvac_ball_collector

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ballvac_ball_collector::ExplorationCollectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
