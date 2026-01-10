/**
 * @file ball_collector_node.cpp
 * @brief Implementation of ball collector node - FSM for autonomous ball collection
 * 
 * This node implements a finite state machine (FSM) that:
 * - SEARCH: Wanders the environment looking for colored balls while avoiding obstacles
 * - APPROACH: Navigates towards a detected ball using proportional steering
 * - COLLECT: When close enough, calls DeleteEntity service to remove ball from Gazebo
 * - RECOVER: Backs up and turns when stuck or after collection failure
 * 
 * Control is done via geometry_msgs/Twist on /cmd_vel_in topic (Ackermann plugin).
 * Entity deletion uses ros_gz_interfaces/srv/DeleteEntity service.
 */

#include "ballvac_ball_collector/ball_collector_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ballvac_ball_collector
{

// =============================================================================
// Constructor
// =============================================================================

BallCollectorNode::BallCollectorNode(const rclcpp::NodeOptions & options)
: Node("ball_collector_node", options),
  current_state_(CollectorState::EXPLORE),
  recover_phase_(0),
  recover_turn_direction_(1.0),
  last_min_front_range_(10.0),
  stuck_count_(0),
  search_steering_direction_(1.0),
  delete_pending_(false),
  following_wall_(false),
  wall_follow_distance_(0.8),
  wall_follow_side_(1)  // Start following right wall
{
    // -------------------------------------------------------------------------
    // Declare and get parameters
    // -------------------------------------------------------------------------
    
    // Topic parameters
    this->declare_parameter<std::string>("scan_topic", "/scan");
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    
    this->declare_parameter<std::string>("detection_topic", "/ball_detections");
    detection_topic_ = this->get_parameter("detection_topic").as_string();
    
    this->declare_parameter<std::string>("cmd_topic", "/cmd_vel_in");
    cmd_topic_ = this->get_parameter("cmd_topic").as_string();
    
    // Delete entity service - the world name should match the SDF world name
    // Format: /world/<world_name>/remove
    this->declare_parameter<std::string>("delete_service", "/world/my_world/remove");
    delete_service_ = this->get_parameter("delete_service").as_string();
    
    // Spawn entity service - Format: /world/<world_name>/create
    this->declare_parameter<std::string>("spawn_service", "/world/my_world/create");
    spawn_service_ = this->get_parameter("spawn_service").as_string();
    
    // Control parameters
    this->declare_parameter<double>("collect_distance_m", 0.5);
    collect_distance_m_ = this->get_parameter("collect_distance_m").as_double();
    
    this->declare_parameter<double>("obstacle_stop_m", 0.5);
    obstacle_stop_m_ = this->get_parameter("obstacle_stop_m").as_double();
    
    this->declare_parameter<double>("obstacle_slow_m", 1.0);
    obstacle_slow_m_ = this->get_parameter("obstacle_slow_m").as_double();
    
    this->declare_parameter<double>("search_speed", 0.5);
    search_speed_ = this->get_parameter("search_speed").as_double();
    
    this->declare_parameter<double>("approach_speed", 0.4);
    approach_speed_ = this->get_parameter("approach_speed").as_double();
    
    this->declare_parameter<double>("max_steer", 0.5);
    max_steer_ = this->get_parameter("max_steer").as_double();
    
    this->declare_parameter<double>("control_rate", 20.0);
    control_rate_ = this->get_parameter("control_rate").as_double();
    
    // Approach parameters
    this->declare_parameter<double>("steering_gain", 2.0);
    steering_gain_ = this->get_parameter("steering_gain").as_double();
    
    // Ball radius threshold for collection (larger radius = closer ball)
    // This depends on camera resolution and ball size in simulation
    // Higher value = robot must get closer before collecting
    this->declare_parameter<double>("approach_radius_threshold", 150.0);
    approach_radius_threshold_ = this->get_parameter("approach_radius_threshold").as_double();
    
    // Recovery parameters
    this->declare_parameter<double>("recover_duration", 2.0);
    recover_duration_ = this->get_parameter("recover_duration").as_double();
    
    this->declare_parameter<double>("recover_speed", 0.3);
    recover_speed_ = this->get_parameter("recover_speed").as_double();
    
    // Stuck detection parameters
    this->declare_parameter<double>("stuck_timeout", 3.0);
    stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
    
    this->declare_parameter<double>("stuck_distance_threshold", 0.05);
    stuck_distance_threshold_ = this->get_parameter("stuck_distance_threshold").as_double();
    
    // Target lost timeout
    this->declare_parameter<double>("target_lost_timeout", 2.0);
    target_lost_timeout_ = this->get_parameter("target_lost_timeout").as_double();

    // Ball radius filtering - reject detections outside this range
    // Too small = noise/far away, too large = false positive (sky/walls)
    this->declare_parameter<double>("min_ball_radius", 15.0);
    min_ball_radius_ = this->get_parameter("min_ball_radius").as_double();
    
    this->declare_parameter<double>("max_ball_radius", 160.0);
    max_ball_radius_ = this->get_parameter("max_ball_radius").as_double();

    // Cooldown after collection attempt (seconds)
    this->declare_parameter<double>("collection_cooldown", 1.0);
    collection_cooldown_ = this->get_parameter("collection_cooldown").as_double();

    // Potential Field parameters - for intelligent obstacle avoidance
    this->declare_parameter<double>("repulsive_gain", 2.5);
    repulsive_gain_ = this->get_parameter("repulsive_gain").as_double();
    
    this->declare_parameter<double>("attractive_gain", 0.8);
    attractive_gain_ = this->get_parameter("attractive_gain").as_double();
    
    this->declare_parameter<double>("influence_distance", 3.5);
    influence_distance_ = this->get_parameter("influence_distance").as_double();
    
    this->declare_parameter<double>("critical_distance", 1.0);
    critical_distance_ = this->get_parameter("critical_distance").as_double();

    // -------------------------------------------------------------------------
    // Initialize target ball
    // -------------------------------------------------------------------------
    target_ball_.valid = false;
    target_ball_.name = "";
    last_collection_time_ = this->now() - rclcpp::Duration::from_seconds(10.0);
    target_ball_.color = "";
    target_ball_.bearing = 0.0;
    target_ball_.apparent_size = 0.0;

    // -------------------------------------------------------------------------
    // Initialize random number generator for spawn positions
    // -------------------------------------------------------------------------
    std::random_device rd;
    rng_ = std::mt19937(rd());

    // -------------------------------------------------------------------------
    // Initialize timing
    // -------------------------------------------------------------------------
    last_detection_time_ = this->now();
    search_direction_change_time_ = this->now();
    recover_start_time_ = this->now();
    last_progress_time_ = this->now();

    // -------------------------------------------------------------------------
    // Create ROS 2 publishers and subscribers
    // -------------------------------------------------------------------------
    
    // LiDAR subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&BallCollectorNode::scan_callback, this, std::placeholders::_1));
    
    // Ball detection subscriber
    detection_sub_ = this->create_subscription<ballvac_msgs::msg::BallDetectionArray>(
        detection_topic_,
        10,
        std::bind(&BallCollectorNode::detection_callback, this, std::placeholders::_1));
    
    // Velocity command publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    
    // Delete entity service client
    delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>(delete_service_);
    
    // Spawn entity service client
    spawn_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(spawn_service_);

    // -------------------------------------------------------------------------
    // Create control loop timer
    // -------------------------------------------------------------------------
    auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
        std::bind(&BallCollectorNode::control_loop, this));

    // -------------------------------------------------------------------------
    // Log startup information
    // -------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Ball Collector Node Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Topics:");
    RCLCPP_INFO(this->get_logger(), "  Scan: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Detections: %s", detection_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Cmd vel: %s", cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Delete service: %s", delete_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Spawn service: %s", spawn_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  Collect distance: %.2f m", collect_distance_m_);
    RCLCPP_INFO(this->get_logger(), "  Obstacle stop: %.2f m", obstacle_stop_m_);
    RCLCPP_INFO(this->get_logger(), "  Search speed: %.2f m/s", search_speed_);
    RCLCPP_INFO(this->get_logger(), "  Approach speed: %.2f m/s", approach_speed_);
    RCLCPP_INFO(this->get_logger(), "  Max steer: %.2f rad/s", max_steer_);
    RCLCPP_INFO(this->get_logger(), "  Radius threshold: %.1f px", approach_radius_threshold_);
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // Check if delete service is available
    if (!delete_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(this->get_logger(), 
            "Delete entity service '%s' not available. "
            "Ball deletion will fail until service is ready. "
            "Make sure Gazebo is running with the correct world name.",
            delete_service_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Delete entity service is available.");
    }
}

// =============================================================================
// LiDAR scan callback
// =============================================================================

void BallCollectorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
}

// =============================================================================
// Ball detection callback
// =============================================================================

void BallCollectorNode::detection_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg)
{
    // Update detection time
    last_detection_time_ = this->now();
    
    // If we're in EXPLORE state and got detections, check if ball is IN OUR WAY (close and centered)
    if (current_state_ == CollectorState::EXPLORE && !msg->detections.empty())
    {
        // Only approach balls that are:
        // 1. Large enough (close to us)
        // 2. Centered (in our path, bearing close to 0)
        for (const auto & det : msg->detections)
        {
            // Filter: ball must be reasonably close (large radius)
            if (det.apparent_size < 40.0)  // Too far away, ignore
            {
                continue;
            }
            
            // Filter: ball must be in front of us (small bearing)
            if (std::abs(det.bearing) > 0.4)  // ~23 degrees - not in our path
            {
                continue;
            }
            
            // Filter by radius range
            if (det.apparent_size < min_ball_radius_ || det.apparent_size > max_ball_radius_)
            {
                continue;
            }
            
            // Check cooldown
            double time_since_collection = (this->now() - last_collection_time_).seconds();
            if (time_since_collection < collection_cooldown_)
            {
                continue;
            }
            
            // This ball is in our way - approach it
            target_ball_.valid = true;
            target_ball_.name = det.name;
            target_ball_.color = det.color;
            target_ball_.bearing = det.bearing;
            target_ball_.apparent_size = det.apparent_size;
            target_ball_.last_seen = this->now();
            
            RCLCPP_INFO(this->get_logger(), 
                "Ball in path: '%s' (bearing=%.2f, radius=%.1f) - approaching",
                det.name.c_str(), det.bearing, det.apparent_size);
            
            transition_to(CollectorState::APPROACH);
            return;
        }
    }
    // If we're in APPROACH state, update target tracking
    else if (current_state_ == CollectorState::APPROACH && target_ball_.valid)
    {
        // Look for our target ball in the detections
        bool found_target = false;
        for (const auto & det : msg->detections)
        {
            if (det.color == target_ball_.color)
            {
                // Filter by radius - reject false positives
                if (det.apparent_size < min_ball_radius_ || det.apparent_size > max_ball_radius_)
                {
                    RCLCPP_DEBUG(this->get_logger(), 
                        "Tracking filter: '%s' radius %.1f outside range",
                        det.name.c_str(), det.apparent_size);
                    continue;
                }
                
                // Update target information
                target_ball_.bearing = det.bearing;
                target_ball_.apparent_size = det.apparent_size;
                target_ball_.last_seen = this->now();
                found_target = true;
                
                // Check if we're close enough to collect
                if (det.apparent_size > approach_radius_threshold_)
                {
                    RCLCPP_INFO(this->get_logger(), 
                        "Ball '%s' radius %.1f > threshold %.1f, transitioning to COLLECT",
                        target_ball_.name.c_str(), det.apparent_size, approach_radius_threshold_);
                    transition_to(CollectorState::COLLECT);
                }
                break;
            }
        }
        
        // If target not found, check if we should give up
        if (!found_target)
        {
            double time_since_seen = (this->now() - target_ball_.last_seen).seconds();
            if (time_since_seen > target_lost_timeout_)
            {
                RCLCPP_WARN(this->get_logger(), 
                    "Lost track of target '%s', returning to EXPLORE",
                    target_ball_.name.c_str());
                target_ball_.valid = false;
                transition_to(CollectorState::EXPLORE);
            }
        }
    }
}

// =============================================================================
// Main control loop
// =============================================================================

void BallCollectorNode::control_loop()
{
    // Execute behavior based on current state
    switch (current_state_)
    {
        case CollectorState::EXPLORE:
            execute_explore();
            break;
        case CollectorState::APPROACH:
            execute_approach();
            break;
        case CollectorState::COLLECT:
            execute_collect();
            break;
        case CollectorState::RECOVER:
            execute_recover();
            break;
    }
}

// =============================================================================
// EXPLORE state - Potential Field based obstacle avoidance
// =============================================================================

void BallCollectorNode::execute_explore()
{
    if (!latest_scan_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "EXPLORE: No LiDAR data received yet!");
        publish_cmd_vel(0.0, 0.0);
        return;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    if (num_readings == 0)
    {
        publish_cmd_vel(0.0, 0.0);
        return;
    }
    
    // =========================================================================
    // Use Potential Field method to find best direction
    // =========================================================================
    
    float linear_vel = 0.0f;
    float angular_vel = 0.0f;
    
    // Random exploration direction (changes periodically)
    double time_since_change = (this->now() - search_direction_change_time_).seconds();
    if (time_since_change > 5.0)
    {
        std::uniform_real_distribution<float> turn_dist(-0.3f, 0.3f);
        search_steering_direction_ = turn_dist(rng_);
        search_direction_change_time_ = this->now();
    }
    
    // Use potential field to navigate
    bool path_safe = compute_potential_field_velocity(
        static_cast<float>(search_steering_direction_), linear_vel, angular_vel);
    
    if (!path_safe)
    {
        // All directions blocked - try to find any gap
        float best_direction = find_best_gap(0.0f);
        
        if (std::abs(best_direction) > 2.5f)  // No good gap found
        {
            // Completely blocked - go to recovery
            RCLCPP_WARN(this->get_logger(), "EXPLORE: Completely blocked, entering RECOVER");
            recover_turn_direction_ = (best_direction > 0) ? 1.0f : -1.0f;
            transition_to(CollectorState::RECOVER);
            return;
        }
        
        // Turn towards the gap
        linear_vel = 0.0f;
        angular_vel = std::clamp(best_direction * 1.5f, 
                                 static_cast<float>(-max_steer_), 
                                 static_cast<float>(max_steer_));
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "EXPLORE: Finding gap, turning %.2f rad", angular_vel);
    }
    
    // Clamp velocities
    linear_vel = std::clamp(linear_vel, 0.0f, static_cast<float>(search_speed_));
    angular_vel = std::clamp(angular_vel, static_cast<float>(-max_steer_), 
                             static_cast<float>(max_steer_));
    
    last_progress_time_ = this->now();
    
    publish_cmd_vel(linear_vel, angular_vel);
}

// =============================================================================
// APPROACH state - navigate towards detected ball (IGNORE obstacles near ball)
// =============================================================================

void BallCollectorNode::execute_approach()
{
    if (!target_ball_.valid)
    {
        RCLCPP_WARN(this->get_logger(), "APPROACH: No valid target, returning to EXPLORE");
        transition_to(CollectorState::EXPLORE);
        return;
    }
    
    // Check if target was lost
    double time_since_seen = (this->now() - target_ball_.last_seen).seconds();
    if (time_since_seen > target_lost_timeout_)
    {
        RCLCPP_WARN(this->get_logger(), 
            "APPROACH: Target '%s' lost for %.1f seconds, returning to EXPLORE",
            target_ball_.name.c_str(), time_since_seen);
        target_ball_.valid = false;
        transition_to(CollectorState::EXPLORE);
        return;
    }
    
    // -------------------------------------------------------------------------
    // BALL IS VISIBLE - Go directly towards it, ignore obstacles in ball direction
    // The ball itself shows up as an obstacle in LiDAR, so we must ignore it!
    // -------------------------------------------------------------------------
    
    float linear_vel = approach_speed_;
    float angular_vel = 0.0f;
    
    // Simple proportional steering towards ball
    angular_vel = -steering_gain_ * target_ball_.bearing;
    angular_vel = std::clamp(angular_vel, static_cast<float>(-max_steer_), 
                             static_cast<float>(max_steer_));
    
    // Check distance to ball using LiDAR in ball's direction
    float ball_distance = std::numeric_limits<float>::max();
    if (latest_scan_)
    {
        size_t num_readings = latest_scan_->ranges.size();
        float angle_min = latest_scan_->angle_min;
        float angle_increment = latest_scan_->angle_increment;
        
        // Get LiDAR reading in direction of ball
        int ball_idx = static_cast<int>((target_ball_.bearing - angle_min) / angle_increment);
        ball_idx = std::clamp(ball_idx, 0, static_cast<int>(num_readings - 1));
        
        // Check a narrow window around ball direction (this is likely the ball)
        for (int i = std::max(0, ball_idx - 5); 
             i < std::min(static_cast<int>(num_readings), ball_idx + 5); ++i)
        {
            if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
            {
                ball_distance = std::min(ball_distance, latest_scan_->ranges[i]);
            }
        }
    }
    
    // Ball apparent size indicates how close we are (bigger = closer)
    // If ball is large in image OR lidar shows something close in ball direction -> approach!
    bool ball_is_close = (target_ball_.apparent_size > 50.0f) || (ball_distance < 2.0f);
    
    if (ball_is_close)
    {
        // Ball is visible and close - GO DIRECTLY TO IT, ignore "obstacles"
        // The obstacle in front IS the ball!
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "APPROACH: Ball close! (radius=%.1f, lidar_dist=%.2f) - going direct",
            target_ball_.apparent_size, ball_distance);
        
        // Slow down as we get very close
        if (ball_distance < 1.0f)
        {
            linear_vel = approach_speed_ * 0.5f;
        }
        
        // Check if close enough to collect
        if (ball_distance < collect_distance_m_ || 
            target_ball_.apparent_size > approach_radius_threshold_)
        {
            RCLCPP_INFO(this->get_logger(), 
                "APPROACH: Ready to collect! (dist=%.2f, radius=%.1f)",
                ball_distance, target_ball_.apparent_size);
            transition_to(CollectorState::COLLECT);
            return;
        }
    }
    else
    {
        // Ball is far - use potential field but with reduced obstacle avoidance
        float pf_linear = 0.0f;
        float pf_angular = 0.0f;
        
        bool path_safe = compute_potential_field_velocity(
            target_ball_.bearing, pf_linear, pf_angular);
        
        if (!path_safe)
        {
            // Only avoid if ball is NOT in that direction
            // The "obstacle" might be the ball itself!
            float best_gap = find_best_gap(target_ball_.bearing);
            
            // If best gap is close to ball direction, just go to ball
            if (std::abs(best_gap - target_ball_.bearing) < 0.5f)
            {
                // Gap is near ball - probably the ball, go there
                linear_vel = approach_speed_ * 0.7f;
            }
            else if (std::abs(best_gap) > 2.5f)
            {
                // Truly blocked - but if we can see the ball, try anyway
                linear_vel = approach_speed_ * 0.3f;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "APPROACH: Path seems blocked but ball visible, continuing");
            }
            else
            {
                // Navigate towards gap
                linear_vel = approach_speed_ * 0.5f;
                angular_vel = std::clamp(best_gap * 0.8f, 
                                         static_cast<float>(-max_steer_), 
                                         static_cast<float>(max_steer_));
            }
        }
        else
        {
            // Path is safe, blend potential field with direct approach
            linear_vel = 0.7f * pf_linear + 0.3f * static_cast<float>(approach_speed_);
            angular_vel = 0.5f * pf_angular + 0.5f * (-steering_gain_ * target_ball_.bearing);
        }
    }
    
    // Clamp velocities
    linear_vel = std::clamp(linear_vel, 0.0f, static_cast<float>(approach_speed_));
    angular_vel = std::clamp(angular_vel, static_cast<float>(-max_steer_), 
                             static_cast<float>(max_steer_));
    
    // -------------------------------------------------------------------------
    // Stuck detection
    // -------------------------------------------------------------------------
    static float last_approach_range = 10.0f;
    float range_change = std::abs(linear_vel - last_approach_range);
    
    if (range_change > 0.01f)
    {
        last_progress_time_ = this->now();
        last_approach_range = linear_vel;
    }
    else
    {
        double time_stuck = (this->now() - last_progress_time_).seconds();
        if (time_stuck > stuck_timeout_)
        {
            stuck_count_++;
            RCLCPP_WARN(this->get_logger(), 
                "APPROACH: Stuck for %.1fs, entering RECOVER", time_stuck);
            target_ball_.valid = false;
            recover_turn_direction_ = (target_ball_.bearing > 0) ? -1.0f : 1.0f;
            transition_to(CollectorState::RECOVER);
            return;
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
        "APPROACH: Target '%s', bearing=%.2f, vel=(%.2f, %.2f)",
        target_ball_.name.c_str(), target_ball_.bearing, linear_vel, angular_vel);
    
    publish_cmd_vel(linear_vel, angular_vel);
}

// =============================================================================
// COLLECT state - delete ball entity from Gazebo
// =============================================================================

void BallCollectorNode::execute_collect()
{
    // Stop the robot
    publish_cmd_vel(0.0, 0.0);
    
    if (!target_ball_.valid)
    {
        RCLCPP_WARN(this->get_logger(), "COLLECT: No valid target");
        transition_to(CollectorState::EXPLORE);
        return;
    }
    
    // Don't send multiple delete requests
    if (delete_pending_)
    {
        return;
    }
    
    // Get entity name and attempt deletion
    std::string entity_name = get_entity_name(target_ball_.color);
    
    RCLCPP_INFO(this->get_logger(), 
        "COLLECT: Attempting to delete entity '%s'", entity_name.c_str());
    
    delete_entity(entity_name);
}

// =============================================================================
// RECOVER state - back up and turn after failure or getting stuck
// =============================================================================

void BallCollectorNode::execute_recover()
{
    double time_in_recover = (this->now() - recover_start_time_).seconds();
    
    // Much longer recovery time based on stuck count (more aggressive escape)
    // Base: 3.5 seconds, increases with stuck count up to 6 seconds
    double base_duration = 3.5;
    double extended_duration = base_duration * (1.0 + 0.3 * std::min(stuck_count_, 5));
    
    // Check for obstacles behind us
    float min_rear_range = 10.0;
    if (latest_scan_)
    {
        // Check rear (around 180 degrees from center)
        size_t num_readings = latest_scan_->ranges.size();
        size_t rear_center = num_readings / 2;
        size_t check_width = num_readings / 6;  // Check 60 degree arc behind
        
        for (size_t i = rear_center - check_width; i < rear_center + check_width; ++i)
        {
            if (i < num_readings && std::isfinite(latest_scan_->ranges[i]))
            {
                min_rear_range = std::min(min_rear_range, latest_scan_->ranges[i]);
            }
        }
    }
    
    // Recovery has 4 phases for better escape:
    // Phase 0: Full reverse (40% time)
    // Phase 1: Reverse + turn (25% time)
    // Phase 2: Forward + turn opposite direction (25% time)
    // Phase 3: Forward straight to clear (10% time)
    
    double phase0_end = extended_duration * 0.40;
    double phase1_end = extended_duration * 0.65;
    double phase2_end = extended_duration * 0.90;
    
    if (recover_phase_ == 0)
    {
        // Phase 0: Full reverse
        if (time_in_recover < phase0_end)
        {
            // If obstacle behind, skip to turn phase
            if (min_rear_range < 0.25)
            {
                RCLCPP_WARN(this->get_logger(), 
                    "RECOVER: Obstacle behind (%.2f m), skipping to turn", min_rear_range);
                recover_phase_ = 1;
            }
            else
            {
                float backup_speed = recover_speed_ * 1.2;  // Faster backup
                if (min_rear_range < 0.4)
                {
                    backup_speed *= 0.5;
                }
                publish_cmd_vel(-backup_speed, 0.0);
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "RECOVER Phase 0: Full reverse (%.1f/%.1f s), rear=%.2f",
                    time_in_recover, phase0_end, min_rear_range);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "RECOVER: Phase 0 complete, starting turn");
            recover_phase_ = 1;
        }
    }
    else if (recover_phase_ == 1)
    {
        // Phase 1: Reverse + hard turn
        if (time_in_recover < phase1_end)
        {
            publish_cmd_vel(-recover_speed_ * 0.5, recover_turn_direction_ * max_steer_ * 1.2);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "RECOVER Phase 1: Reverse + turn (dir=%.1f)",
                recover_turn_direction_);
        }
        else
        {
            recover_phase_ = 2;
        }
    }
    else if (recover_phase_ == 2)
    {
        // Phase 2: Forward + turn in OPPOSITE direction to swing around
        if (time_in_recover < phase2_end)
        {
            publish_cmd_vel(recover_speed_ * 0.7, -recover_turn_direction_ * max_steer_ * 0.8);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "RECOVER Phase 2: Forward + opposite turn");
        }
        else
        {
            recover_phase_ = 3;
        }
    }
    else
    {
        // Phase 3: Forward straight to clear and complete
        if (time_in_recover < extended_duration)
        {
            publish_cmd_vel(recover_speed_ * 0.8, 0.0);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "RECOVER Phase 3: Forward straight");
        }
        else
        {
            // Recovery complete
            RCLCPP_INFO(this->get_logger(), 
                "RECOVER: Complete (%.1f s), returning to EXPLORE", time_in_recover);
            
            // Reset progress tracking
            last_progress_time_ = this->now();
            last_min_front_range_ = 10.0;
            
            // Change wall follow side to try different path
            wall_follow_side_ = -wall_follow_side_;
            
            target_ball_.valid = false;
            transition_to(CollectorState::EXPLORE);
        }
    }
}

// =============================================================================
// State transition
// =============================================================================

void BallCollectorNode::transition_to(CollectorState new_state)
{
    if (new_state == current_state_)
    {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
        state_to_string(current_state_).c_str(),
        state_to_string(new_state).c_str());
    
    // Handle state entry actions
    switch (new_state)
    {
        case CollectorState::EXPLORE:
            // Reset stuck tracking when entering explore
            stuck_count_ = 0;
            last_progress_time_ = this->now();
            break;
        case CollectorState::APPROACH:
            // Reset progress tracking when starting a new approach
            last_progress_time_ = this->now();
            last_min_front_range_ = 10.0;
            break;
        case CollectorState::COLLECT:
            delete_pending_ = false;
            break;
        case CollectorState::RECOVER:
            recover_start_time_ = this->now();
            recover_phase_ = 0;
            // Random recovery turn direction
            recover_turn_direction_ = (rand() % 2 == 0) ? 1.0f : -1.0f;
            break;
    }
    
    current_state_ = new_state;
}

// =============================================================================
// Select target ball from detections
// =============================================================================

bool BallCollectorNode::select_target(const ballvac_msgs::msg::BallDetectionArray & detections)
{
    if (detections.detections.empty())
    {
        return false;
    }
    
    // Check cooldown
    double time_since_collection = (this->now() - last_collection_time_).seconds();
    if (time_since_collection < collection_cooldown_)
    {
        RCLCPP_DEBUG(this->get_logger(), "In cooldown, %.1f s remaining",
            collection_cooldown_ - time_since_collection);
        return false;
    }
    
    // Find the best target: prefer larger radius (closer) and smaller bearing (more centered)
    const ballvac_msgs::msg::BallDetection * best = nullptr;
    float best_score = -std::numeric_limits<float>::max();
    
    for (const auto & det : detections.detections)
    {
        // Filter by radius - reject too small (far/noise) or too large (false positive)
        if (det.apparent_size < min_ball_radius_ || det.apparent_size > max_ball_radius_)
        {
            RCLCPP_DEBUG(this->get_logger(), 
                "Filtered '%s': radius %.1f outside range [%.1f, %.1f]",
                det.name.c_str(), det.apparent_size, min_ball_radius_, max_ball_radius_);
            continue;
        }
        
        // Skip already collected balls
        if (collected_balls_.count(det.color) > 0)
        {
            continue;
        }
        
        // Score based on size (larger = better) and bearing (centered = better)
        // Radius typically ranges from ~10 to ~150 depending on distance
        // Bearing ranges from -0.8 to 0.8 radians typically
        float score = det.apparent_size - 50.0f * std::abs(det.bearing);
        
        if (score > best_score)
        {
            best_score = score;
            best = &det;
        }
    }
    
    if (best == nullptr)
    {
        RCLCPP_DEBUG(this->get_logger(), "No valid targets after filtering");
        return false;
    }
    
    // Set as target
    target_ball_.valid = true;
    target_ball_.name = best->name;
    target_ball_.color = best->color;
    target_ball_.bearing = best->bearing;
    target_ball_.apparent_size = best->apparent_size;
    target_ball_.last_seen = this->now();
    
    RCLCPP_INFO(this->get_logger(), 
        "Selected target: '%s' (color=%s, bearing=%.2f, radius=%.1f)",
        target_ball_.name.c_str(), target_ball_.color.c_str(),
        target_ball_.bearing, target_ball_.apparent_size);
    
    return true;
}

// =============================================================================
// Check for obstacles in front using LiDAR
// =============================================================================

bool BallCollectorNode::check_obstacle_front(float & min_range)
{
    if (!latest_scan_)
    {
        min_range = std::numeric_limits<float>::max();
        return false;
    }
    
    // Check front sector (roughly +/- 30 degrees from center)
    // LiDAR has 720 samples covering 360 degrees (2*pi radians)
    // Center is at index 0 (or 360 for 0 degrees forward)
    
    int num_samples = latest_scan_->ranges.size();
    float angle_increment = latest_scan_->angle_increment;
    float angle_min = latest_scan_->angle_min;
    
    // Define front sector: +/- 30 degrees = +/- 0.52 radians
    float front_sector_angle = 0.52;
    
    min_range = std::numeric_limits<float>::max();
    
    for (int i = 0; i < num_samples; i++)
    {
        float angle = angle_min + i * angle_increment;
        
        // Check if angle is in front sector (around 0 degrees)
        // Handle wrap-around at +/- pi
        if (std::abs(angle) < front_sector_angle || std::abs(angle) > (M_PI - front_sector_angle))
        {
            float range = latest_scan_->ranges[i];
            
            // Ignore invalid readings
            if (std::isfinite(range) && range > latest_scan_->range_min && 
                range < latest_scan_->range_max)
            {
                min_range = std::min(min_range, range);
            }
        }
    }
    
    return min_range < obstacle_stop_m_;
}

// =============================================================================
// Compute steering to avoid obstacles
// =============================================================================

float BallCollectorNode::compute_obstacle_avoidance_steering()
{
    if (!latest_scan_)
    {
        return 0.0f;
    }
    
    // Compare left and right sectors to determine which way to turn
    int num_samples = latest_scan_->ranges.size();
    float angle_increment = latest_scan_->angle_increment;
    float angle_min = latest_scan_->angle_min;
    
    float left_sum = 0.0f;
    float right_sum = 0.0f;
    int left_count = 0;
    int right_count = 0;
    
    for (int i = 0; i < num_samples; i++)
    {
        float angle = angle_min + i * angle_increment;
        float range = latest_scan_->ranges[i];
        
        // Ignore invalid readings
        if (!std::isfinite(range) || range < latest_scan_->range_min || 
            range > latest_scan_->range_max)
        {
            continue;
        }
        
        // Consider front 180 degrees
        if (std::abs(angle) < M_PI / 2.0)
        {
            if (angle > 0)  // Left side (positive angles)
            {
                left_sum += range;
                left_count++;
            }
            else  // Right side (negative angles)
            {
                right_sum += range;
                right_count++;
            }
        }
    }
    
    // Average distances
    float left_avg = (left_count > 0) ? left_sum / left_count : 10.0f;
    float right_avg = (right_count > 0) ? right_sum / right_count : 10.0f;
    
    // Turn towards the side with more space
    // Positive angular velocity = turn left (counterclockwise)
    // Negative angular velocity = turn right (clockwise)
    if (left_avg > right_avg)
    {
        return static_cast<float>(max_steer_);  // Turn left
    }
    else
    {
        return static_cast<float>(-max_steer_);  // Turn right
    }
}

// =============================================================================
// Publish velocity command
// =============================================================================

void BallCollectorNode::publish_cmd_vel(float linear, float angular)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = angular;
    
    cmd_vel_pub_->publish(cmd);
}

// =============================================================================
// Get Gazebo entity name for a ball color
// =============================================================================

std::string BallCollectorNode::get_entity_name(const std::string & color)
{
    // Entity names match the SDF model names (simple format: ball_color)
    return "ball_" + color;
}

// =============================================================================
// Delete entity from Gazebo
// =============================================================================

void BallCollectorNode::delete_entity(const std::string & entity_name)
{
    if (!delete_client_->service_is_ready())
    {
        RCLCPP_ERROR(this->get_logger(), 
            "Delete service '%s' not available", delete_service_.c_str());
        transition_to(CollectorState::RECOVER);
        return;
    }
    
    delete_pending_ = true;
    
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = entity_name;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    
    RCLCPP_INFO(this->get_logger(), "Sending delete request for '%s'", entity_name.c_str());
    
    auto future = delete_client_->async_send_request(
        request,
        std::bind(&BallCollectorNode::delete_entity_callback, this, std::placeholders::_1));
}

// =============================================================================
// Delete entity callback
// =============================================================================

void BallCollectorNode::delete_entity_callback(
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future)
{
    delete_pending_ = false;
    last_collection_time_ = this->now();  // Start cooldown
    
    try
    {
        auto response = future.get();
        
        if (response->success)
        {
            std::string collected_color = target_ball_.color;
            
            RCLCPP_INFO(this->get_logger(), 
                "Successfully deleted '%s' - Ball collected!",
                target_ball_.name.c_str());
            
            // Increment collection counter for this color
            ball_collect_count_[collected_color]++;
            
            // Log progress
            RCLCPP_INFO(this->get_logger(), 
                "Collected %s ball #%d - spawning new one at random position",
                collected_color.c_str(), ball_collect_count_[collected_color]);
            
            // Spawn a new ball of the same color at a different position
            spawn_ball(collected_color);
            
            // Clear target and go back to exploration
            target_ball_.valid = false;
            transition_to(CollectorState::EXPLORE);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to delete '%s': service returned false - might be a false detection",
                target_ball_.name.c_str());
            
            // Clear target - probably a false detection
            target_ball_.valid = false;
            transition_to(CollectorState::EXPLORE);
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), 
            "Exception in delete callback: %s", e.what());
        target_ball_.valid = false;
        transition_to(CollectorState::RECOVER);
    }
}

// =============================================================================
// Spawn a new ball at random position
// =============================================================================

void BallCollectorNode::spawn_ball(const std::string & color)
{
    if (!spawn_client_->service_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), 
            "Spawn service '%s' not available, cannot respawn ball",
            spawn_service_.c_str());
        return;
    }
    
    double x, y;
    get_random_spawn_position(x, y);
    
    auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    // Use simple naming: ball_color (matches launcher format)
    std::string entity_name = "ball_" + color;
    request->entity_factory.name = entity_name;
    request->entity_factory.allow_renaming = false;  // Don't allow rename - only one per color
    request->entity_factory.sdf = generate_ball_sdf(color, entity_name);
    request->entity_factory.pose.position.x = x;
    request->entity_factory.pose.position.y = y;
    request->entity_factory.pose.position.z = 0.1;
    
    RCLCPP_INFO(this->get_logger(), 
        "Spawning new %s ball at position (%.2f, %.2f)", 
        color.c_str(), x, y);
    
    auto future = spawn_client_->async_send_request(
        request,
        std::bind(&BallCollectorNode::spawn_entity_callback, this, std::placeholders::_1));
}

// =============================================================================
// Spawn entity callback
// =============================================================================

void BallCollectorNode::spawn_entity_callback(
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future)
{
    try
    {
        auto response = future.get();
        
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully spawned new ball!");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to spawn ball: service returned false");
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), 
            "Exception in spawn callback: %s", e.what());
    }
}

// =============================================================================
// Generate SDF string for a colored ball
// =============================================================================

std::string BallCollectorNode::generate_ball_sdf(const std::string & color, const std::string & entity_name)
{
    // Define RGB colors for each ball
    std::string ambient, diffuse, emissive;
    
    if (color == "red") {
        ambient = "1.0 0.0 0.0 1"; diffuse = "1.0 0.0 0.0 1"; emissive = "0.1 0.0 0.0 1";
    } else if (color == "green") {
        ambient = "0.0 1.0 0.0 1"; diffuse = "0.0 1.0 0.0 1"; emissive = "0.0 0.1 0.0 1";
    } else if (color == "blue") {
        ambient = "0.0 0.0 1.0 1"; diffuse = "0.0 0.0 1.0 1"; emissive = "0.0 0.0 0.1 1";
    } else if (color == "yellow") {
        ambient = "1.0 1.0 0.0 1"; diffuse = "1.0 1.0 0.0 1"; emissive = "0.1 0.1 0.0 1";
    } else if (color == "cyan") {
        ambient = "0.0 1.0 1.0 1"; diffuse = "0.0 1.0 1.0 1"; emissive = "0.0 0.1 0.1 1";
    } else if (color == "purple") {
        ambient = "0.6 0.0 0.8 1"; diffuse = "0.6 0.0 0.8 1"; emissive = "0.06 0.0 0.08 1";
    } else {
        ambient = "1.0 1.0 1.0 1"; diffuse = "1.0 1.0 1.0 1"; emissive = "0.1 0.1 0.1 1";
    }
    
    std::string sdf = R"(<?xml version="1.0"?>
<sdf version="1.8">
  <model name=")" + entity_name + R"(">
    <static>false</static>
    <link name="ball_link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.00004</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00004</iyy><iyz>0</iyz><izz>0.00004</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.1</radius></sphere></geometry>
        <material>
          <ambient>)" + ambient + R"(</ambient>
          <diffuse>)" + diffuse + R"(</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>)" + emissive + R"(</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
)";
    
    return sdf;
}

// =============================================================================
// Get random spawn position for a ball
// =============================================================================

void BallCollectorNode::get_random_spawn_position(double & x, double & y)
{
    // Define spawn area (adjust these based on your world)
    // Avoiding the center where the robot usually starts
    std::uniform_real_distribution<double> dist_x(1.5, 6.0);
    std::uniform_real_distribution<double> dist_y(-3.0, 3.0);
    
    x = dist_x(rng_);
    y = dist_y(rng_);
    
    // Add some randomness to avoid spawning exactly where previous ball was
    std::uniform_real_distribution<double> offset(-0.5, 0.5);
    x += offset(rng_);
    y += offset(rng_);
}

// =============================================================================
// Potential Field based velocity computation
// =============================================================================

bool BallCollectorNode::compute_potential_field_velocity(
    float target_bearing, float & linear_vel, float & angular_vel)
{
    if (!latest_scan_)
    {
        linear_vel = 0.0f;
        angular_vel = 0.0f;
        return false;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    float angle_min = latest_scan_->angle_min;
    float angle_increment = latest_scan_->angle_increment;
    
    // =========================================================================
    // Calculate repulsive force from all obstacles
    // =========================================================================
    float repulsive_x = 0.0f;  // Force pushing robot away from obstacles
    float repulsive_y = 0.0f;
    float min_obstacle_dist = std::numeric_limits<float>::max();
    
    for (size_t i = 0; i < num_readings; ++i)
    {
        float range = latest_scan_->ranges[i];
        
        // Skip invalid readings
        if (!std::isfinite(range) || range < 0.1f || range > latest_scan_->range_max)
        {
            continue;
        }
        
        float angle = angle_min + i * angle_increment;
        
        // Only consider front hemisphere (-90 to +90 degrees)
        if (std::abs(angle) > M_PI / 2.0)
        {
            continue;
        }
        
        // Track minimum obstacle distance
        min_obstacle_dist = std::min(min_obstacle_dist, range);
        
        // Calculate repulsive force if within influence distance
        if (range < influence_distance_)
        {
            // Repulsive force magnitude: inversely proportional to distance squared
            // F = gain * (1/d - 1/d_max) * (1/d^2) pointing away from obstacle
            float force_magnitude = repulsive_gain_ * 
                (1.0f / range - 1.0f / static_cast<float>(influence_distance_)) * 
                (1.0f / (range * range));
            
            // Critical distance: very high force
            if (range < critical_distance_)
            {
                force_magnitude *= 5.0f;  // Much stronger repulsion when very close
            }
            
            // Direction: away from obstacle (opposite of obstacle direction)
            // Obstacle is at angle 'angle' from robot, so force pushes in opposite direction
            repulsive_x += force_magnitude * (-std::cos(angle));
            repulsive_y += force_magnitude * (-std::sin(angle));
        }
    }
    
    // =========================================================================
    // Calculate attractive force towards target
    // =========================================================================
    float attractive_x = attractive_gain_ * std::cos(target_bearing);
    float attractive_y = attractive_gain_ * std::sin(target_bearing);
    
    // =========================================================================
    // Combine forces
    // =========================================================================
    float total_x = attractive_x + repulsive_x;
    float total_y = attractive_y + repulsive_y;
    
    // Calculate desired direction
    float desired_direction = std::atan2(total_y, total_x);
    // Force magnitude can be used for speed scaling if needed
    (void)std::sqrt(total_x * total_x + total_y * total_y);
    
    // =========================================================================
    // Convert to velocity commands
    // =========================================================================
    
    // Check if path is blocked (very close obstacle)
    bool path_blocked = (min_obstacle_dist < critical_distance_);
    
    if (path_blocked)
    {
        linear_vel = 0.0f;
        angular_vel = static_cast<float>(max_steer_) * 
                      ((desired_direction > 0) ? 1.0f : -1.0f);
        
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "POTENTIAL FIELD: Path blocked! Obstacle at %.2f m, turning %.2f",
            min_obstacle_dist, angular_vel);
        
        return false;
    }
    
    // Scale linear velocity based on obstacle proximity and force direction
    // Go slower when obstacles are closer
    float proximity_factor = std::min(1.0f, 
        (min_obstacle_dist - static_cast<float>(critical_distance_)) / 
        (static_cast<float>(influence_distance_) - static_cast<float>(critical_distance_)));
    proximity_factor = std::max(0.1f, proximity_factor);
    
    // Only move forward if desired direction is roughly forward
    if (std::abs(desired_direction) < M_PI / 2.0)
    {
        // Forward movement scaled by how aligned we are with desired direction
        float direction_factor = std::cos(desired_direction);
        linear_vel = static_cast<float>(search_speed_) * proximity_factor * 
                     std::max(0.2f, direction_factor);
    }
    else
    {
        // Need to turn, don't move forward
        linear_vel = 0.0f;
    }
    
    // Angular velocity proportional to desired direction
    angular_vel = std::clamp(
        desired_direction * static_cast<float>(steering_gain_),
        static_cast<float>(-max_steer_),
        static_cast<float>(max_steer_));
    
    RCLCPP_DEBUG(this->get_logger(),
        "POTENTIAL FIELD: target=%.2f, force=(%.2f,%.2f), dir=%.2f, vel=(%.2f,%.2f), min_obs=%.2f",
        target_bearing, total_x, total_y, desired_direction, linear_vel, angular_vel, min_obstacle_dist);
    
    return true;
}

// =============================================================================
// Find best gap/opening in LiDAR scan
// =============================================================================

float BallCollectorNode::find_best_gap(float preferred_direction)
{
    if (!latest_scan_)
    {
        return preferred_direction;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    float angle_min = latest_scan_->angle_min;
    float angle_increment = latest_scan_->angle_increment;
    
    // Find gaps (openings) in the scan
    float best_gap_angle = 0.0f;
    float best_gap_score = -1000.0f;
    
    // Scan through front hemisphere in windows
    const int window_size = 20;  // Number of readings per window
    const float min_gap_distance = 1.5f;  // Minimum distance to consider as a gap
    
    for (size_t i = 0; i < num_readings - window_size; i += window_size / 2)
    {
        float center_angle = angle_min + (i + window_size / 2) * angle_increment;
        
        // Only consider front hemisphere
        if (std::abs(center_angle) > M_PI / 2.0)
        {
            continue;
        }
        
        // Calculate average distance in this window
        float avg_distance = 0.0f;
        float min_distance = std::numeric_limits<float>::max();
        int valid_count = 0;
        
        for (size_t j = i; j < i + window_size && j < num_readings; ++j)
        {
            float range = latest_scan_->ranges[j];
            if (std::isfinite(range) && range > 0.1f)
            {
                avg_distance += range;
                min_distance = std::min(min_distance, range);
                valid_count++;
            }
        }
        
        if (valid_count > 0)
        {
            avg_distance /= valid_count;
        }
        else
        {
            continue;  // No valid readings in window
        }
        
        // Score this gap:
        // - Prefer gaps with larger minimum distance
        // - Prefer gaps closer to preferred direction
        // - Prefer gaps closer to front (center)
        float distance_score = min_distance * 2.0f + avg_distance;
        float direction_score = -std::abs(center_angle - preferred_direction) * 0.5f;
        float center_score = -std::abs(center_angle) * 0.3f;
        
        float total_score = distance_score + direction_score + center_score;
        
        // Only consider if minimum distance is sufficient
        if (min_distance > min_gap_distance && total_score > best_gap_score)
        {
            best_gap_score = total_score;
            best_gap_angle = center_angle;
        }
    }
    
    // If no good gap found, return a large value to indicate blocked
    if (best_gap_score < -500.0f)
    {
        RCLCPP_WARN(this->get_logger(), "No good gap found!");
        return (preferred_direction > 0) ? 3.0f : -3.0f;  // Return large angle
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Best gap at %.2f rad (score=%.2f, preferred=%.2f)",
        best_gap_angle, best_gap_score, preferred_direction);
    
    return best_gap_angle;
}

// =============================================================================
// Check if a direction is safe to travel
// =============================================================================

bool BallCollectorNode::is_direction_safe(float angle, float safe_distance)
{
    if (!latest_scan_)
    {
        return false;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    float angle_min = latest_scan_->angle_min;
    float angle_increment = latest_scan_->angle_increment;
    
    // Convert angle to index
    int center_idx = static_cast<int>((angle - angle_min) / angle_increment);
    center_idx = std::clamp(center_idx, 0, static_cast<int>(num_readings - 1));
    
    // Check a window around the direction
    const int check_window = 10;
    
    for (int i = center_idx - check_window; i <= center_idx + check_window; ++i)
    {
        if (i < 0 || i >= static_cast<int>(num_readings))
        {
            continue;
        }
        
        float range = latest_scan_->ranges[i];
        if (std::isfinite(range) && range < safe_distance)
        {
            return false;  // Obstacle too close in this direction
        }
    }
    
    return true;
}

}  // namespace ballvac_ball_collector

// =============================================================================
// Main function
// =============================================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ballvac_ball_collector::BallCollectorNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
