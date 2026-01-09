/**
 * @file nav_ball_collector_node.cpp
 * @brief Implementation of navigation-enabled ball collector node
 * 
 * This node uses Nav2 for intelligent path planning to reach balls
 * while avoiding obstacles based on the map created during SLAM.
 */

#include "ballvac_ball_collector/nav_ball_collector_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ballvac_ball_collector
{

// =============================================================================
// Constructor
// =============================================================================

NavBallCollectorNode::NavBallCollectorNode(const rclcpp::NodeOptions & options)
: Node("nav_ball_collector_node", options),
  current_state_(NavCollectorState::IDLE),
  pose_received_(false),
  navigation_in_progress_(false),
  navigation_succeeded_(false),
  nav2_ready_(false),
  consecutive_rejections_(0),
  exploration_waypoint_index_(0),
  recover_phase_(0),
  recover_turn_direction_(1.0),
  last_min_front_range_(10.0),
  stuck_count_(0),
  delete_pending_(false),
  delete_attempt_(0)
{
    // -------------------------------------------------------------------------
    // Declare and get parameters
    // -------------------------------------------------------------------------
    
    // Topic parameters
    this->declare_parameter<std::string>("scan_topic", "/scan");
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    
    this->declare_parameter<std::string>("detection_topic", "/ball_detections");
    detection_topic_ = this->get_parameter("detection_topic").as_string();
    
    this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    cmd_topic_ = this->get_parameter("cmd_topic").as_string();
    
    this->declare_parameter<std::string>("odom_topic", "/odom");
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    
    this->declare_parameter<std::string>("delete_service", "/world/my_world/remove");
    delete_service_ = this->get_parameter("delete_service").as_string();
    
    this->declare_parameter<std::string>("spawn_service", "/world/my_world/create");
    spawn_service_ = this->get_parameter("spawn_service").as_string();
    
    // Fleet coordination parameters
    this->declare_parameter<std::string>("robot_id", "ballvac");
    robot_id_ = this->get_parameter("robot_id").as_string();
    
    this->declare_parameter<bool>("use_fleet_coordinator", false);
    use_fleet_coordinator_ = this->get_parameter("use_fleet_coordinator").as_bool();
    
    this->declare_parameter<std::string>("assignment_topic", "/assignment");
    assignment_topic_ = this->get_parameter("assignment_topic").as_string();
    
    this->declare_parameter<std::string>("robot_status_topic", "/fleet/robot_status");
    robot_status_topic_ = this->get_parameter("robot_status_topic").as_string();
    
    // Exploration bounds (default to 20m x 20m)
    this->declare_parameter<double>("exploration_min_x", -10.0);
    exploration_min_x_ = this->get_parameter("exploration_min_x").as_double();
    
    this->declare_parameter<double>("exploration_max_x", 10.0);
    exploration_max_x_ = this->get_parameter("exploration_max_x").as_double();
    
    this->declare_parameter<double>("exploration_min_y", -10.0);
    exploration_min_y_ = this->get_parameter("exploration_min_y").as_double();
    
    this->declare_parameter<double>("exploration_max_y", 10.0);
    exploration_max_y_ = this->get_parameter("exploration_max_y").as_double();
    
    // Frame parameters
    this->declare_parameter<std::string>("map_frame", "map");
    map_frame_ = this->get_parameter("map_frame").as_string();
    
    this->declare_parameter<std::string>("robot_frame", "ballvac");
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    
    this->declare_parameter<std::string>("camera_frame", "ballvac/camera_link");
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    
    // Control parameters
    this->declare_parameter<double>("collect_distance_m", 0.15);  // CLOSER: Stop 15cm before ball for collection
    collect_distance_m_ = this->get_parameter("collect_distance_m").as_double();
    
    this->declare_parameter<double>("obstacle_stop_m", 0.7);  // CRITICAL: Never go closer than this to walls
    obstacle_stop_m_ = this->get_parameter("obstacle_stop_m").as_double();
    
    this->declare_parameter<double>("obstacle_slow_m", 1.8);  // Start slowing at this distance
    obstacle_slow_m_ = this->get_parameter("obstacle_slow_m").as_double();
    
    this->declare_parameter<double>("obstacle_avoid_m", 3.0);  // Start steering at this distance
    obstacle_avoid_m_ = this->get_parameter("obstacle_avoid_m").as_double();
    
    this->declare_parameter<double>("approach_speed", 1.2);  // Slower for better ball approach control
    approach_speed_ = this->get_parameter("approach_speed").as_double();
    
    this->declare_parameter<double>("max_steer", 3.2);  // INCREASED: Sharper turns for wall avoidance
    max_steer_ = this->get_parameter("max_steer").as_double();
    
    this->declare_parameter<double>("control_rate", 30.0);  // INCREASED: Faster control loop
    control_rate_ = this->get_parameter("control_rate").as_double();

    this->declare_parameter<double>("pose_log_interval", 1.0);
    pose_log_interval_ = this->get_parameter("pose_log_interval").as_double();

    this->declare_parameter<bool>("respawn_balls", false);
    respawn_balls_ = this->get_parameter("respawn_balls").as_bool();
    
    // Approach parameters
    this->declare_parameter<double>("steering_gain", 4.0);  // INCREASED: More responsive steering
    steering_gain_ = this->get_parameter("steering_gain").as_double();
    
    this->declare_parameter<double>("approach_radius_threshold", 120.0);  // Higher = ball must be closer
    approach_radius_threshold_ = this->get_parameter("approach_radius_threshold").as_double();
    
    this->declare_parameter<double>("nav_to_approach_distance", 0.6);  // Switch to direct approach at 0.6m
    nav_to_approach_distance_ = this->get_parameter("nav_to_approach_distance").as_double();
    
    // Recovery parameters
    this->declare_parameter<double>("recover_duration", 1.5);  // Faster recovery
    recover_duration_ = this->get_parameter("recover_duration").as_double();
    
    this->declare_parameter<double>("recover_speed", 1.2);  // INCREASED: Faster reverse
    recover_speed_ = this->get_parameter("recover_speed").as_double();
    
    // Ball detection parameters
    this->declare_parameter<double>("min_ball_radius", 15.0);
    min_ball_radius_ = this->get_parameter("min_ball_radius").as_double();
    
    this->declare_parameter<double>("max_ball_radius", 160.0);
    max_ball_radius_ = this->get_parameter("max_ball_radius").as_double();
    
    this->declare_parameter<double>("collection_cooldown", 1.0);
    collection_cooldown_ = this->get_parameter("collection_cooldown").as_double();
    
    this->declare_parameter<double>("target_lost_timeout", 6.0);  // HYSTERESIS: Keep target for 6s even if not visible
    target_lost_timeout_ = this->get_parameter("target_lost_timeout").as_double();

    this->declare_parameter<double>("ball_visible_timeout", 0.6);
    ball_visible_timeout_ = this->get_parameter("ball_visible_timeout").as_double();
    
    // Camera parameters for distance estimation
    this->declare_parameter<double>("camera_fov_horizontal", 1.3962634);  // 80 degrees
    camera_fov_horizontal_ = this->get_parameter("camera_fov_horizontal").as_double();
    
    this->declare_parameter<double>("camera_resolution_width", 640.0);
    camera_resolution_width_ = this->get_parameter("camera_resolution_width").as_double();
    
    this->declare_parameter<double>("ball_actual_diameter", 0.15);  // 15cm ball
    ball_actual_diameter_ = this->get_parameter("ball_actual_diameter").as_double();
    
    // Focal length = (image_width / 2) / tan(fov/2)
    camera_focal_length_ = (camera_resolution_width_ / 2.0) / std::tan(camera_fov_horizontal_ / 2.0);
    
    // Exploration parameters
    this->declare_parameter<double>("exploration_waypoint_distance", 1.5);  // Reduced from 2.0 for closer waypoints
    exploration_waypoint_distance_ = this->get_parameter("exploration_waypoint_distance").as_double();
    
    this->declare_parameter<double>("exploration_timeout", 15.0);  // Reduced from 30.0 for faster waypoint switching
    exploration_timeout_ = this->get_parameter("exploration_timeout").as_double();

    this->declare_parameter<bool>("explore_with_nav2", true);
    explore_with_nav2_ = this->get_parameter("explore_with_nav2").as_bool();
    
    this->declare_parameter<double>("nav_goal_retry_cooldown", 1.0);
    nav_goal_retry_cooldown_ = this->get_parameter("nav_goal_retry_cooldown").as_double();

    this->declare_parameter<double>("wander_bias_interval", 3.0);
    wander_bias_interval_ = this->get_parameter("wander_bias_interval").as_double();

    this->declare_parameter<double>("wander_bias_max", 0.6);
    wander_bias_max_ = this->get_parameter("wander_bias_max").as_double();

    // -------------------------------------------------------------------------
    // Initialize target ball
    // -------------------------------------------------------------------------
    target_ball_.valid = false;
    target_ball_.position_known = false;
    last_collection_time_ = this->now() - rclcpp::Duration::from_seconds(10.0);
    
    // Initialize fleet coordination state
    has_active_assignment_ = false;
    last_heartbeat_time_ = this->now();

    // -------------------------------------------------------------------------
    // Initialize TF2
    // -------------------------------------------------------------------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // -------------------------------------------------------------------------
    // Initialize random number generator
    // -------------------------------------------------------------------------
    std::random_device rd;
    rng_ = std::mt19937(rd());

    // -------------------------------------------------------------------------
    // Initialize timing
    // -------------------------------------------------------------------------
    last_progress_time_ = this->now();
    recover_start_time_ = this->now();
    exploration_start_time_ = this->now();
    last_goal_rejection_time_ = this->now() - rclcpp::Duration::from_seconds(60.0);  // Allow goals immediately
    last_nav_result_time_ = this->now() - rclcpp::Duration::from_seconds(60.0);  // Allow goals immediately
    last_wander_bias_time_ = this->now();
    wander_bias_ = 0.0;
    
    // -------------------------------------------------------------------------
    // Initialize corner escape state
    // -------------------------------------------------------------------------
    in_corner_escape_ = false;
    corner_escape_start_time_ = this->now();
    escape_target_angle_ = 0.0f;
    escape_phase_ = 0;
    last_escape_x_ = 0.0;
    last_escape_y_ = 0.0;
    consecutive_stuck_count_ = 0;
    same_direction_attempts_ = 0;
    last_escape_direction_ = 0.0f;
    blocked_directions_.clear();
    blocked_directions_clear_time_ = this->now();

    // -------------------------------------------------------------------------
    // Create ROS 2 communication
    // -------------------------------------------------------------------------
    
    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&NavBallCollectorNode::scan_callback, this, std::placeholders::_1));
    
    detection_sub_ = this->create_subscription<ballvac_msgs::msg::BallDetectionArray>(
        detection_topic_,
        10,
        std::bind(&NavBallCollectorNode::detection_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        10,
        std::bind(&NavBallCollectorNode::odom_callback, this, std::placeholders::_1));

    deleted_ball_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/fleet/ball_deleted",
        rclcpp::QoS(10).reliable(),
        std::bind(&NavBallCollectorNode::deleted_ball_callback, this, std::placeholders::_1));
    
    // Fleet coordination subscribers/publishers
    if (use_fleet_coordinator_)
    {
        assignment_sub_ = this->create_subscription<ballvac_msgs::msg::RobotAssignment>(
            assignment_topic_,
            rclcpp::QoS(10).reliable().transient_local(),
            std::bind(&NavBallCollectorNode::assignment_callback, this, std::placeholders::_1));
        
        // Subscribe to ball registry to know which balls are claimed by others
        ball_registry_sub_ = this->create_subscription<ballvac_msgs::msg::BallRegistry>(
            "/fleet/ball_registry",
            rclcpp::QoS(10).reliable().transient_local(),
            std::bind(&NavBallCollectorNode::ball_registry_callback, this, std::placeholders::_1));
        
        robot_status_pub_ = this->create_publisher<ballvac_msgs::msg::RobotStatus>(
            robot_status_topic_,
            rclcpp::QoS(50).reliable());
        
        // Heartbeat timer - publish status every 2 seconds
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&NavBallCollectorNode::heartbeat_timer_callback, this));
    }
    
    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
    ball_deleted_pub_ = this->create_publisher<std_msgs::msg::String>("/fleet/ball_deleted", 10);
    
    // Action client for Nav2 NavigateToPose
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    // Service clients
    delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>(delete_service_);
    spawn_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(spawn_service_);

    // -------------------------------------------------------------------------
    // Create control loop timer
    // -------------------------------------------------------------------------
    auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
        std::bind(&NavBallCollectorNode::control_loop, this));

    if (pose_log_interval_ > 0.0)
    {
        auto pose_log_period = std::chrono::duration<double>(pose_log_interval_);
        pose_log_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(pose_log_period),
            std::bind(&NavBallCollectorNode::pose_log_timer_callback, this));
    }

    // -------------------------------------------------------------------------
    // Log startup information
    // -------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Nav Ball Collector Node Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Using Nav2 for path planning!");
    if (use_fleet_coordinator_)
    {
        RCLCPP_INFO(this->get_logger(), "Fleet Coordination: ENABLED");
        RCLCPP_INFO(this->get_logger(), "  Robot ID: %s", robot_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Assignment topic: %s", assignment_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Status topic: %s", robot_status_topic_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Fleet Coordination: DISABLED (standalone mode)");
    }
    RCLCPP_INFO(this->get_logger(), "Exploration bounds: [%.1f, %.1f] x [%.1f, %.1f]",
        exploration_min_x_, exploration_max_x_, exploration_min_y_, exploration_max_y_);
    RCLCPP_INFO(this->get_logger(), "Topics:");
    RCLCPP_INFO(this->get_logger(), "  Scan: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Detections: %s", detection_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Odom: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Frames:");
    RCLCPP_INFO(this->get_logger(), "  Map: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Robot: %s", robot_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  Nav to approach dist: %.2f m", nav_to_approach_distance_);
    RCLCPP_INFO(this->get_logger(), "  Collect distance: %.2f m", collect_distance_m_);
    RCLCPP_INFO(this->get_logger(), "  Camera focal length: %.2f px", camera_focal_length_);
    RCLCPP_INFO(this->get_logger(), "========================================");
}

// =============================================================================
// Callbacks
// =============================================================================

void NavBallCollectorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
}

void NavBallCollectorNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_odom_ = msg;
    
    // Update current pose
    current_pose_.header = msg->header;
    current_pose_.header.frame_id = map_frame_;  // We'll transform if needed
    current_pose_.pose = msg->pose.pose;
    pose_received_ = true;
}

void NavBallCollectorNode::deleted_ball_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::string & deleted_name = msg->data;
    if (deleted_name.empty())
    {
        return;
    }

    collected_balls_.insert(deleted_name);

    bool target_matches = target_ball_.valid &&
        (target_ball_.name == deleted_name ||
         deleted_name == (target_ball_.name + "_2"));

    if (target_matches)
    {
        RCLCPP_INFO(this->get_logger(),
            "Target '%s' was deleted by another robot - returning to explore",
            target_ball_.name.c_str());
        cancel_navigation();
        target_ball_.valid = false;
        transition_to(NavCollectorState::EXPLORING);
    }
}

void NavBallCollectorNode::pose_log_timer_callback()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!pose_received_ || !latest_odom_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Pose log: waiting for odometry data...");
        return;
    }

    const auto & pose = latest_odom_->pose.pose;
    double qz = pose.orientation.z;
    double qw = pose.orientation.w;
    double yaw = 2.0 * std::atan2(qz, qw);

    RCLCPP_INFO(this->get_logger(),
        "Robot %s pose: x=%.2f y=%.2f yaw=%.2f",
        robot_id_.c_str(),
        pose.position.x,
        pose.position.y,
        yaw);
}

// =============================================================================
// Fleet Coordination Callbacks and Functions
// =============================================================================

void NavBallCollectorNode::assignment_callback(const ballvac_msgs::msg::RobotAssignment::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Check if this assignment is for us
    if (msg->robot_id != robot_id_)
    {
        return;
    }
    
    if (msg->has_assignment)
    {
        // New or updated assignment
        bool is_new = !has_active_assignment_ || 
                      current_assignment_.ball_id != msg->ball_id;
        
        current_assignment_ = *msg;
        has_active_assignment_ = true;
        
        if (is_new)
        {
            RCLCPP_INFO(this->get_logger(), 
                "Received assignment: ball '%s' (%s) at (%.2f, %.2f)",
                msg->ball_id.c_str(), msg->ball_color.c_str(),
                msg->goal_pose.pose.position.x, msg->goal_pose.pose.position.y);
            
            // Set target ball from assignment
            target_ball_.valid = true;
            target_ball_.name = msg->ball_id;
            target_ball_.color = msg->ball_color;
            target_ball_.world_pose = msg->goal_pose;
            target_ball_.position_known = true;
            target_ball_.last_seen = this->now();
            
            // Cancel any current navigation and start new one
            if (navigation_in_progress_)
            {
                cancel_navigation();
            }
            
            // Publish claim to coordinator
            publish_claim(msg->ball_id);
            
            transition_to(NavCollectorState::NAVIGATING);
        }
    }
    else
    {
        // Assignment cleared
        if (has_active_assignment_)
        {
            RCLCPP_INFO(this->get_logger(), "Assignment cleared");
            has_active_assignment_ = false;
            current_assignment_ = ballvac_msgs::msg::RobotAssignment();
            target_ball_.valid = false;
            
            if (navigation_in_progress_)
            {
                cancel_navigation();
            }
            
            transition_to(NavCollectorState::EXPLORING);
        }
    }
}

void NavBallCollectorNode::heartbeat_timer_callback()
{
    publish_robot_status(ballvac_msgs::msg::RobotStatus::ACTION_HEARTBEAT);
}

void NavBallCollectorNode::publish_robot_status(uint8_t action, const std::string & action_ball_id)
{
    if (!use_fleet_coordinator_ || !robot_status_pub_)
    {
        return;
    }
    
    auto msg = ballvac_msgs::msg::RobotStatus();
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_;
    msg.robot_id = robot_id_;
    
    // Current pose
    if (latest_odom_)
    {
        msg.pose.header.stamp = this->now();
        msg.pose.header.frame_id = map_frame_;
        msg.pose.pose = latest_odom_->pose.pose;
    }
    
    // Current state mapping
    switch (current_state_)
    {
        case NavCollectorState::IDLE:
            msg.state = ballvac_msgs::msg::RobotStatus::IDLE;
            break;
        case NavCollectorState::EXPLORING:
            msg.state = ballvac_msgs::msg::RobotStatus::IDLE;
            break;
        case NavCollectorState::NAVIGATING:
            msg.state = ballvac_msgs::msg::RobotStatus::NAVIGATING;
            break;
        case NavCollectorState::APPROACHING:
            msg.state = ballvac_msgs::msg::RobotStatus::APPROACHING;
            break;
        case NavCollectorState::COLLECTING:
            msg.state = ballvac_msgs::msg::RobotStatus::COLLECTING;
            break;
        case NavCollectorState::RECOVERING:
            msg.state = ballvac_msgs::msg::RobotStatus::STUCK;
            break;
    }
    
    msg.assigned_ball_id = has_active_assignment_ ? current_assignment_.ball_id : "";
    msg.is_operational = true;
    msg.navigation_ready = nav_to_pose_client_->wait_for_action_server(std::chrono::milliseconds(10));
    
    msg.action = action;
    msg.action_ball_id = action_ball_id;
    
    robot_status_pub_->publish(msg);
}

void NavBallCollectorNode::publish_claim(const std::string & ball_id)
{
    publish_robot_status(ballvac_msgs::msg::RobotStatus::ACTION_CLAIM, ball_id);
}

void NavBallCollectorNode::publish_collected(const std::string & ball_id)
{
    publish_robot_status(ballvac_msgs::msg::RobotStatus::ACTION_COLLECTED, ball_id);
    has_active_assignment_ = false;
    current_assignment_ = ballvac_msgs::msg::RobotAssignment();
}

void NavBallCollectorNode::publish_lost(const std::string & ball_id)
{
    publish_robot_status(ballvac_msgs::msg::RobotStatus::ACTION_LOST, ball_id);
    has_active_assignment_ = false;
    current_assignment_ = ballvac_msgs::msg::RobotAssignment();
}

void NavBallCollectorNode::ball_registry_callback(const ballvac_msgs::msg::BallRegistry::SharedPtr msg)
{
    // Update our local map of claimed balls from ball states
    claimed_balls_.clear();
    
    for (const auto & ball_state : msg->balls)
    {
        // Track claimed balls
        if (ball_state.state == 1 && !ball_state.claimed_by_robot.empty())  // CLAIMED=1
        {
            claimed_balls_[ball_state.ball_id] = ball_state.claimed_by_robot;
        }
    }
}

bool NavBallCollectorNode::is_ball_claimed_by_other(const std::string & ball_id)
{
    auto it = claimed_balls_.find(ball_id);
    if (it != claimed_balls_.end())
    {
        // Ball is claimed - check if claimed by another robot (not us)
        return it->second != robot_id_;
    }
    return false;
}

// =============================================================================
// Detection Callback (with fleet coordination support)
// =============================================================================

void NavBallCollectorNode::detection_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Skip if no detections
    if (msg->detections.empty())
    {
        return;
    }
    
    // Allow detections even if Nav2 isn't ready (LiDAR-only mode still collects)
    
    // Check cooldown
    double time_since_collection = (this->now() - last_collection_time_).seconds();
    if (time_since_collection < collection_cooldown_)
    {
        return;
    }
    
    // ==========================================================================
    // Fleet coordination mode: only pursue assigned balls
    // ==========================================================================
    if (use_fleet_coordinator_)
    {
        // In IDLE or EXPLORING, wait for coordinator assignments only
        if (current_state_ == NavCollectorState::IDLE || 
            current_state_ == NavCollectorState::EXPLORING)
        {
            return;
        }
        
        // In NAVIGATING or APPROACHING, track the assigned ball
        if (current_state_ == NavCollectorState::NAVIGATING || 
            current_state_ == NavCollectorState::APPROACHING)
        {
            if (!has_active_assignment_ || !target_ball_.valid)
            {
                return;
            }

            // If another robot already claimed this ball, back off immediately
            if (is_ball_claimed_by_other(target_ball_.name))
            {
                RCLCPP_WARN(this->get_logger(),
                    "Assigned ball '%s' is now claimed by another robot - abandoning",
                    target_ball_.name.c_str());
                has_active_assignment_ = false;
                current_assignment_ = ballvac_msgs::msg::RobotAssignment();
                target_ball_.valid = false;
                cancel_navigation();
                transition_to(NavCollectorState::EXPLORING);
                return;
            }
            
            // Look for our assigned ball
            bool found_target = false;
            const ballvac_msgs::msg::BallDetection * best_det = nullptr;
            double best_score = std::numeric_limits<double>::max();
            for (const auto & det : msg->detections)
            {
                // Match by exact name first, otherwise by color
                if (det.apparent_size < min_ball_radius_ || det.apparent_size > max_ball_radius_)
                {
                    continue;
                }
                
                if (det.name == target_ball_.name)
                {
                    best_det = &det;
                    best_score = 0.0;
                    break;
                }
                
                if (det.color == target_ball_.color)
                {
                    double score = std::abs(det.bearing - target_ball_.bearing);
                    if (!best_det || score < best_score)
                    {
                        best_det = &det;
                        best_score = score;
                    }
                }
            }

            if (best_det)
            {
                found_target = true;
                target_ball_.bearing = best_det->bearing;
                target_ball_.apparent_size = best_det->apparent_size;
                target_ball_.last_seen = this->now();
                target_ball_.estimated_distance = estimate_distance_from_size(best_det->apparent_size);
                target_ball_.world_pose = estimate_ball_world_pose(*best_det);
                target_ball_.name = best_det->name;  // Update name in case it changed
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Tracking assigned '%s': bearing=%.2f, size=%.1f, dist=%.2fm",
                    best_det->name.c_str(), best_det->bearing, best_det->apparent_size,
                    target_ball_.estimated_distance);
                
                // Switch to APPROACHING if close enough
                if (current_state_ == NavCollectorState::NAVIGATING &&
                    target_ball_.estimated_distance < nav_to_approach_distance_)
                {
                    RCLCPP_INFO(this->get_logger(), 
                        "Ball within %.2fm - switching to reactive approach",
                        target_ball_.estimated_distance);
                    cancel_navigation();
                    transition_to(NavCollectorState::APPROACHING);
                }
                
                // Check if close enough to collect
                if (best_det->apparent_size > approach_radius_threshold_)
                {
                    RCLCPP_INFO(this->get_logger(), 
                        "Ball '%s' radius %.1f > threshold %.1f - COLLECTING",
                        target_ball_.name.c_str(), best_det->apparent_size, approach_radius_threshold_);
                    cancel_navigation();
                    transition_to(NavCollectorState::COLLECTING);
                }
            }
            
            // Check if target was lost
            if (!found_target)
            {
                double time_since_seen = (this->now() - target_ball_.last_seen).seconds();
                
                // PART C: Hysteresis - don't abandon target too quickly
                // If we were tracking a ball successfully before, give it more time
                double effective_timeout = target_lost_timeout_;
                if (target_ball_.apparent_size > approach_radius_threshold_ * 0.3)
                {
                    // Ball was getting close, extend timeout significantly
                    effective_timeout = target_lost_timeout_ * 2.0;
                }
                
                if (time_since_seen > effective_timeout)
                {
                    RCLCPP_WARN(this->get_logger(), 
                        "Target '%s' lost for %.1f seconds - reporting to coordinator",
                        target_ball_.name.c_str(), time_since_seen);
                    publish_lost(target_ball_.name);
                    target_ball_.valid = false;
                    cancel_navigation();
                    transition_to(NavCollectorState::EXPLORING);
                }
                else if (time_since_seen > target_lost_timeout_ * 0.5)
                {
                    // PART C: Lock-on behavior - continue toward last known position
                    RCLCPP_DEBUG(this->get_logger(), 
                        "Target temporarily lost (%.1fs) - continuing to last known position",
                        time_since_seen);
                }
            }
        }
        return;
    }
    
    // ==========================================================================
    // Standalone mode: original behavior - autonomously select balls
    // ==========================================================================
    
    // In IDLE or EXPLORING state, look for balls to approach
    if (current_state_ == NavCollectorState::IDLE || 
        current_state_ == NavCollectorState::EXPLORING)
    {
        // Find the best ball (largest apparent size within valid range)
        const ballvac_msgs::msg::BallDetection * best_ball = nullptr;
        float best_size = 0.0f;
        
        for (const auto & det : msg->detections)
        {
            // Filter by radius
            if (det.apparent_size < min_ball_radius_ || det.apparent_size > max_ball_radius_)
            {
                continue;
            }
            
            // Skip already collected balls
            if (collected_balls_.count(det.name) > 0)
            {
                continue;
            }
            
            if (det.apparent_size > best_size)
            {
                best_size = det.apparent_size;
                best_ball = &det;
            }
        }
        
        if (best_ball != nullptr)
        {
            // Found a ball! Set as target
            target_ball_.valid = true;
            target_ball_.name = best_ball->name;
            target_ball_.color = best_ball->color;
            target_ball_.bearing = best_ball->bearing;
            target_ball_.apparent_size = best_ball->apparent_size;
            target_ball_.last_seen = this->now();
            
            // Estimate world position
            target_ball_.world_pose = estimate_ball_world_pose(*best_ball);
            target_ball_.position_known = true;
            target_ball_.estimated_distance = estimate_distance_from_size(best_ball->apparent_size);
            
            RCLCPP_INFO(this->get_logger(), 
                "Detected ball: '%s' (bearing=%.2f, size=%.1f, est_dist=%.2fm)",
                best_ball->name.c_str(), best_ball->bearing, best_ball->apparent_size,
                target_ball_.estimated_distance);
            
            // Cancel any ongoing exploration and start navigation
            if (navigation_in_progress_)
            {
                cancel_navigation();
            }
            
            // DIRECT APPROACH: Always go directly to ball when detected
            // Use reactive visual servoing instead of Nav2 planning for faster response
            transition_to(NavCollectorState::APPROACHING);
        }
    }
    // In NAVIGATING or APPROACHING state, update target tracking
    else if (current_state_ == NavCollectorState::NAVIGATING || 
             current_state_ == NavCollectorState::APPROACHING)
    {
        if (!target_ball_.valid)
        {
            return;
        }
        
        // Look for our target ball
        const ballvac_msgs::msg::BallDetection * best_det = nullptr;
        double best_score = std::numeric_limits<double>::max();
        for (const auto & det : msg->detections)
        {
            if (det.apparent_size < min_ball_radius_ || det.apparent_size > max_ball_radius_)
            {
                continue;
            }
            
            if (det.name == target_ball_.name)
            {
                best_det = &det;
                best_score = 0.0;
                break;
            }
            
            if (det.color == target_ball_.color)
            {
                double score = std::abs(det.bearing - target_ball_.bearing);
                if (!best_det || score < best_score)
                {
                    best_det = &det;
                    best_score = score;
                }
            }
        }
        
        if (best_det)
        {
            // Update target info
            target_ball_.bearing = best_det->bearing;
            target_ball_.apparent_size = best_det->apparent_size;
            target_ball_.last_seen = this->now();
            target_ball_.estimated_distance = estimate_distance_from_size(best_det->apparent_size);
            
            // Update world pose
            target_ball_.world_pose = estimate_ball_world_pose(*best_det);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Tracking '%s': bearing=%.2f, size=%.1f, dist=%.2fm",
                best_det->name.c_str(), best_det->bearing, best_det->apparent_size,
                target_ball_.estimated_distance);
            
            // Check if close enough to switch to APPROACHING
            if (current_state_ == NavCollectorState::NAVIGATING &&
                target_ball_.estimated_distance < nav_to_approach_distance_)
            {
                RCLCPP_INFO(this->get_logger(), 
                    "Ball within %.2fm - switching to reactive approach",
                    target_ball_.estimated_distance);
                cancel_navigation();
                transition_to(NavCollectorState::APPROACHING);
            }
            
            // Check if close enough to collect
            if (best_det->apparent_size > approach_radius_threshold_)
            {
                RCLCPP_INFO(this->get_logger(), 
                    "Ball '%s' radius %.1f > threshold %.1f - COLLECTING",
                    target_ball_.name.c_str(), best_det->apparent_size, approach_radius_threshold_);
                cancel_navigation();
                transition_to(NavCollectorState::COLLECTING);
            }
        }
    }
}

// =============================================================================
// Main control loop
// =============================================================================

void NavBallCollectorNode::control_loop()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    switch (current_state_)
    {
        case NavCollectorState::IDLE:
            execute_idle();
            break;
        case NavCollectorState::EXPLORING:
            execute_exploring();
            break;
        case NavCollectorState::NAVIGATING:
            execute_navigating();
            break;
        case NavCollectorState::APPROACHING:
            execute_approaching();
            break;
        case NavCollectorState::COLLECTING:
            execute_collecting();
            break;
        case NavCollectorState::RECOVERING:
            execute_recovering();
            break;
    }
}

// =============================================================================
// IDLE state - Wait for system ready
// =============================================================================

void NavBallCollectorNode::execute_idle()
{
    // Wait for necessary data
    if (!latest_scan_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "IDLE: Waiting for LiDAR data...");
        return;
    }
    
    if (!pose_received_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "IDLE: Waiting for odometry data...");
        return;
    }
    
    // Check if Nav2 is available - but DON'T block on it
    // We can operate in LiDAR-only mode without Nav2
    bool nav2_available = nav_to_pose_client_->wait_for_action_server(std::chrono::milliseconds(100));
    
    if (nav2_available)
    {
        nav2_ready_ = true;
        explore_with_nav2_ = true;
        RCLCPP_INFO(this->get_logger(), "System ready with Nav2! Starting exploration...");
    }
    else
    {
        // Nav2 not ready yet - use LiDAR-only mode
        nav2_ready_ = false;
        explore_with_nav2_ = false;
        RCLCPP_INFO(this->get_logger(), "System ready (LiDAR-only mode)! Starting exploration...");
    }
    
    transition_to(NavCollectorState::EXPLORING);
}

// =============================================================================
// EXPLORING state - FAST LiDAR-based exploration with corner escape
// =============================================================================

void NavBallCollectorNode::execute_exploring()
{
    // PRIORITY 1: If in corner escape mode, handle it first
    if (in_corner_escape_)
    {
        if (execute_lidar_escape())
        {
            // Escape complete
            in_corner_escape_ = false;
            consecutive_stuck_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "EXPLORING: Corner escape complete!");
        }
        return;
    }

    if (explore_with_nav2_ && navigation_succeeded_)
    {
        navigation_succeeded_ = false;
        navigation_in_progress_ = false;
    }
    
    // PRIORITY 2: Check if stuck in corner - ONLY in corner escape mode
    // Don't trigger stuck detection too often during normal navigation
    if (detect_corner_situation())  // Only check corners, not general stuck
    {
        consecutive_stuck_count_++;
        
        if (consecutive_stuck_count_ >= 10)  // Much higher threshold to avoid false positives
        {
            RCLCPP_WARN(this->get_logger(), 
                "EXPLORING: Corner/stuck detected! Starting LiDAR escape...");
            if (navigation_in_progress_)
            {
                cancel_navigation();
            }
            in_corner_escape_ = true;
            corner_escape_start_time_ = this->now();
            escape_phase_ = 0;
            
            // Find escape direction
            float escape_angle, escape_dist;
            if (find_escape_direction(escape_angle, escape_dist))
            {
                escape_target_angle_ = escape_angle;
                RCLCPP_INFO(this->get_logger(), 
                    "Escape direction found: angle=%.2f rad (%.1f deg), dist=%.2fm",
                    escape_angle, escape_angle * 180.0 / M_PI, escape_dist);
            }
            else
            {
                // No clear direction - just turn around
                escape_target_angle_ = M_PI;
                RCLCPP_WARN(this->get_logger(), "No clear escape - turning around");
            }
            return;
        }
    }
    else
    {
        consecutive_stuck_count_ = 0;
    }

    if (explore_with_nav2_)
    {
        if (navigation_in_progress_)
        {
            double elapsed = (this->now() - exploration_start_time_).seconds();
            if (elapsed > exploration_timeout_)
            {
                cancel_navigation();
                navigation_in_progress_ = false;
            }
            else
            {
                return;
            }
        }

        double since_last_result = (this->now() - last_nav_result_time_).seconds();
        if (since_last_result < nav_goal_retry_cooldown_)
        {
            return;
        }

        geometry_msgs::msg::PoseStamped goal = generate_exploration_goal();
        exploration_start_time_ = this->now();
        if (send_navigation_goal(goal))
        {
            RCLCPP_INFO(this->get_logger(),
                "EXPLORING: Nav2 waypoint (%.2f, %.2f)",
                goal.pose.position.x, goal.pose.position.y);
        }
        // CRITICAL: Always return in Nav2 mode to avoid falling through to manual control
        // Even if goal send fails (cooldown/rejection), wait for next cycle
        return;
    }
    
    // ===========================================================================
    // LiDAR-only exploration: steady forward motion + gentle steering bias
    // ===========================================================================

    float min_front, min_left, min_right;
    check_obstacle_sectors(min_front, min_left, min_right);

    float linear_vel = static_cast<float>(approach_speed_ * 0.8);
    float angular_vel = static_cast<float>(wander_bias_);

    const float WALL_EMERGENCY = obstacle_stop_m_ * 0.5f;
    const float WALL_DANGER = obstacle_stop_m_;
    const float WALL_CAUTION = obstacle_slow_m_;

    // Update wandering bias periodically to avoid spinning in place
    if ((this->now() - last_wander_bias_time_).seconds() > wander_bias_interval_)
    {
        std::uniform_real_distribution<double> bias_dist(-wander_bias_max_, wander_bias_max_);
        wander_bias_ = bias_dist(rng_);
        last_wander_bias_time_ = this->now();
    }

    // Boundary awareness - calculate ideal steering based on robot heading and boundary
    float boundary_steer = 0.0f;
    bool near_boundary = false;
    double boundary_escape_yaw = 0.0;  // Target yaw to face center
    if (latest_odom_)
    {
        double x = latest_odom_->pose.pose.position.x;
        double y = latest_odom_->pose.pose.position.y;
        const double safe_margin = 1.8;
        
        // Calculate center of exploration area
        double center_x = (exploration_min_x_ + exploration_max_x_) / 2.0;
        double center_y = (exploration_min_y_ + exploration_max_y_) / 2.0;
        
        // Check if near boundary
        bool near_min_x = x < exploration_min_x_ + safe_margin;
        bool near_max_x = x > exploration_max_x_ - safe_margin;
        bool near_min_y = y < exploration_min_y_ + safe_margin;
        bool near_max_y = y > exploration_max_y_ - safe_margin;
        
        if (near_min_x || near_max_x || near_min_y || near_max_y)
        {
            near_boundary = true;
            
            // Get current robot yaw
            double qz = latest_odom_->pose.pose.orientation.z;
            double qw = latest_odom_->pose.pose.orientation.w;
            double current_yaw = 2.0 * std::atan2(qz, qw);
            
            // Calculate desired yaw to face center
            boundary_escape_yaw = std::atan2(center_y - y, center_x - x);
            
            // Calculate angular difference (shortest path)
            double yaw_diff = boundary_escape_yaw - current_yaw;
            while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
            
            // Proportional steering toward center (reduced gain to prevent oscillation)
            boundary_steer = std::clamp(static_cast<float>(yaw_diff * 0.8), 
                                        static_cast<float>(-max_steer_ * 0.5f), 
                                        static_cast<float>(max_steer_ * 0.5f));
        }
    }

    auto steer_away = [&]() -> float {
        // Prioritize LiDAR-based steering over boundary steering
        if (min_left < min_right)
        {
            return -max_steer_ * 0.8f;
        }
        return max_steer_ * 0.8f;
    };

    if (min_front < WALL_EMERGENCY)
    {
        // Emergency: very close to wall - reverse with steering
        linear_vel = -recover_speed_ * 0.3f;  // REVERSE instead of crawling
        angular_vel = steer_away();
    }
    else if (min_front < WALL_DANGER)
    {
        // Danger: close to wall - slow down and steer hard
        linear_vel = 0.3f;
        angular_vel = steer_away() * 0.9f;
    }
    else if (min_front < WALL_CAUTION)
    {
        // Caution: approaching wall - reduce speed and steer
        linear_vel = 0.5f;
        angular_vel = steer_away() * 0.6f;
    }
    else if (min_left < WALL_CAUTION || min_right < WALL_CAUTION)
    {
        // Side walls nearby - gentle steering
        linear_vel = 0.7f;
        angular_vel = steer_away() * 0.3f;
    }
    else
    {
        // Open space - use boundary steering or wander bias
        if (near_boundary)
        {
            // Keep moving forward while gently steering toward center
            angular_vel = boundary_steer;
            linear_vel *= 0.8f;  // Slightly reduced speed near boundaries
        }
        else
        {
            angular_vel = static_cast<float>(wander_bias_);
        }
    }

    angular_vel = std::clamp(angular_vel, static_cast<float>(-max_steer_), static_cast<float>(max_steer_));
    
    // Update progress tracking
    if (latest_odom_)
    {
        double dx = latest_odom_->pose.pose.position.x - last_escape_x_;
        double dy = latest_odom_->pose.pose.position.y - last_escape_y_;
        double dist_moved = std::sqrt(dx*dx + dy*dy);
        
        if (dist_moved > 0.5)
        {
            last_escape_x_ = latest_odom_->pose.pose.position.x;
            last_escape_y_ = latest_odom_->pose.pose.position.y;
            last_progress_time_ = this->now();
        }
    }
    
    publish_cmd_vel(linear_vel, angular_vel);
}

// =============================================================================
// NAVIGATING state - Navigate to ball using Nav2
// =============================================================================

void NavBallCollectorNode::execute_navigating()
{
    if (!target_ball_.valid)
    {
        RCLCPP_WARN(this->get_logger(), "NAVIGATING: No valid target, returning to explore");
        if (use_fleet_coordinator_ && has_active_assignment_)
        {
            publish_lost(current_assignment_.ball_id);
        }
        transition_to(NavCollectorState::EXPLORING);
        return;
    }
    
    // Check if target was lost - PART C: Hysteresis in NAVIGATING state
    double time_since_seen = (this->now() - target_ball_.last_seen).seconds();
    
    // Calculate effective timeout with hysteresis
    double effective_timeout = target_lost_timeout_;
    if (target_ball_.position_known && target_ball_.estimated_distance < nav_to_approach_distance_ * 2.0)
    {
        // Getting close to ball - extend timeout
        effective_timeout = target_lost_timeout_ * 1.5;
    }
    
    if (time_since_seen > effective_timeout)
    {
        RCLCPP_WARN(this->get_logger(), 
            "NAVIGATING: Target '%s' lost for %.1f seconds",
            target_ball_.name.c_str(), time_since_seen);
        if (use_fleet_coordinator_)
        {
            publish_lost(target_ball_.name);
        }
        target_ball_.valid = false;
        cancel_navigation();
        transition_to(NavCollectorState::EXPLORING);
        return;
    }
    else if (time_since_seen > target_lost_timeout_ * 0.3)
    {
        // PART C: Lock-on - continue navigating to last known position
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "NAVIGATING: Target temporarily lost (%.1fs), continuing to last known position",
            time_since_seen);
    }
    
    // If navigation is not in progress, send goal to ball position
    if (!navigation_in_progress_ && target_ball_.position_known)
    {
        // If Nav2 is not ready, skip to APPROACHING mode directly
        if (!nav2_ready_)
        {
            RCLCPP_INFO(this->get_logger(), 
                "NAVIGATING: Nav2 not ready, switching to direct APPROACHING");
            transition_to(NavCollectorState::APPROACHING);
            return;
        }
        
        // Check if ball is near a wall - need special approach
        bool near_wall = is_ball_near_wall(target_ball_.world_pose);
        
        geometry_msgs::msg::PoseStamped goal;
        
        if (near_wall)
        {
            // Ball is near wall - calculate safe approach position
            goal = calculate_safe_approach_pose(target_ball_.world_pose);
            RCLCPP_INFO(this->get_logger(), 
                "Ball near wall! Using safe approach from (%.2f, %.2f)",
                goal.pose.position.x, goal.pose.position.y);
        }
        else
        {
            // Normal approach - goal slightly in front of ball
            goal = target_ball_.world_pose;
            
            // Offset goal back from ball towards robot
            if (latest_odom_)
            {
                double dx = goal.pose.position.x - latest_odom_->pose.pose.position.x;
                double dy = goal.pose.position.y - latest_odom_->pose.pose.position.y;
                double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist > 0.1)
            {
                // Place goal nav_to_approach_distance_ away from ball
                double offset = std::min(nav_to_approach_distance_, dist * 0.7);
                goal.pose.position.x -= offset * (dx / dist);
                goal.pose.position.y -= offset * (dy / dist);
                
                // Face towards the ball
                double yaw = std::atan2(dy, dx);
                goal.pose.orientation.z = std::sin(yaw / 2.0);
                goal.pose.orientation.w = std::cos(yaw / 2.0);
            }
            }
        }
        
        if (send_navigation_goal(goal))
        {
            RCLCPP_INFO(this->get_logger(), 
                "NAVIGATING: Sent goal near ball at (%.2f, %.2f)",
                goal.pose.position.x, goal.pose.position.y);
        }
        else
        {
            // Nav2 rejected goal - switch to direct approach
            RCLCPP_INFO(this->get_logger(), 
                "NAVIGATING: Nav2 rejected goal, switching to direct APPROACHING");
            transition_to(NavCollectorState::APPROACHING);
            return;
        }
    }
    
    // If navigation succeeded or we're close enough, switch to approaching
    if (navigation_succeeded_)
    {
        RCLCPP_INFO(this->get_logger(), "NAVIGATING: Goal reached, switching to APPROACHING");
        navigation_succeeded_ = false;
        transition_to(NavCollectorState::APPROACHING);
    }
}

// =============================================================================
// APPROACHING state - Reactive approach to ball
// =============================================================================

void NavBallCollectorNode::execute_approaching()
{
    if (!target_ball_.valid)
    {
        RCLCPP_WARN(this->get_logger(), "APPROACHING: No valid target, returning to EXPLORING");
        transition_to(NavCollectorState::EXPLORING);
        return;
    }
    
    // Check if target was lost - PART C: Hysteresis in APPROACHING state  
    double time_since_seen = (this->now() - target_ball_.last_seen).seconds();
    
    // In APPROACHING, we're close - give MORE time before giving up
    double effective_timeout = target_lost_timeout_ * 2.0;  // Double timeout when approaching
    if (target_ball_.apparent_size > approach_radius_threshold_ * 0.5)
    {
        // Very close to ball - even more hysteresis
        effective_timeout = target_lost_timeout_ * 3.0;
    }
    
    if (time_since_seen > effective_timeout)
    {
        RCLCPP_WARN(this->get_logger(), 
            "APPROACHING: Target '%s' lost for %.1f seconds, returning to EXPLORING",
            target_ball_.name.c_str(), time_since_seen);
        if (use_fleet_coordinator_)
        {
            publish_lost(target_ball_.name);
        }
        target_ball_.valid = false;
        transition_to(NavCollectorState::EXPLORING);
        return;
    }
    else if (time_since_seen > target_lost_timeout_ * 0.5)
    {
        // PART C: Lock-on - continue driving toward last known bearing
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "APPROACHING: Target temporarily lost (%.1fs), continuing approach to last bearing",
            time_since_seen);
    }
    
    bool ball_visible = time_since_seen <= ball_visible_timeout_;
    
    // Get LIDAR-based obstacle information
    float min_front_range = std::numeric_limits<float>::max();
    float min_left_range = std::numeric_limits<float>::max();
    float min_right_range = std::numeric_limits<float>::max();
    check_obstacle_sectors(min_front_range, min_left_range, min_right_range);
    
    float linear_vel = approach_speed_;
    float angular_vel = 0.0;
    
    // =========================================================================
    // If we don't currently see the ball, avoid walls and don't drive forward.
    // =========================================================================
    if (!ball_visible)
    {
        float avoid_steer = compute_obstacle_avoidance_steering();
        if (min_front_range < obstacle_stop_m_)
        {
            linear_vel = -approach_speed_ * 0.2f;
            angular_vel = avoid_steer;
        }
        else if (min_front_range < obstacle_slow_m_)
        {
            linear_vel = 0.0f;
            angular_vel = avoid_steer;
        }
        else
        {
            angular_vel = -steering_gain_ * target_ball_.bearing;
        }
        
        angular_vel = std::clamp(angular_vel, static_cast<float>(-max_steer_), 
                                 static_cast<float>(max_steer_));
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "APPROACHING: Ball not visible (%.1fs). Avoiding walls, vel=(%.2f, %.2f)",
            time_since_seen, linear_vel, angular_vel);
        
        publish_cmd_vel(linear_vel, angular_vel);
        return;
    }
    
    // =========================================================================
    // Check if close enough to STOP and COLLECT
    // IMPORTANT: Stop BEFORE hitting the ball!
    // =========================================================================
    
    // Check if ball is large enough (close enough) to collect
    if (target_ball_.apparent_size > approach_radius_threshold_)
    {
        RCLCPP_INFO(this->get_logger(), 
            "APPROACHING: Ball size %.1f > threshold %.1f - STOP and COLLECT",
            target_ball_.apparent_size, approach_radius_threshold_);
        publish_cmd_vel(0.0, 0.0);  // STOP immediately!
        transition_to(NavCollectorState::COLLECTING);
        return;
    }
    
    // If something is close in front AND we're looking at the ball direction, STOP and collect!
    if (min_front_range < collect_distance_m_ && std::abs(target_ball_.bearing) < 0.6)
    {
        RCLCPP_INFO(this->get_logger(), 
            "APPROACHING: Close object (%.2fm) in ball direction (bearing=%.2f) - STOP and COLLECT!",
            min_front_range, target_ball_.bearing);
        publish_cmd_vel(0.0, 0.0);  // STOP immediately!
        transition_to(NavCollectorState::COLLECTING);
        return;
    }
    
    // =========================================================================
    // Slow approach: Drive towards ball with DECREASING speed as we get closer
    // CRITICAL: Slow down to avoid pushing the ball!
    // =========================================================================
    
    // Proportional steering towards ball
    angular_vel = -steering_gain_ * target_ball_.bearing;
    angular_vel = std::clamp(angular_vel, static_cast<float>(-max_steer_), 
                             static_cast<float>(max_steer_));
    
    // Calculate speed based on distance to ball (from apparent size)
    // Larger apparent size = closer = slower speed
    float distance_factor = 1.0f - (target_ball_.apparent_size / (approach_radius_threshold_ * 1.5f));
    distance_factor = std::clamp(distance_factor, 0.15f, 1.0f);
    
    // Also slow down based on LiDAR front range
    float range_factor = std::clamp(static_cast<float>((min_front_range - collect_distance_m_) / 1.0), 0.15f, 1.0f);
    
    // Use the smaller of the two factors
    float slow_factor = std::min(distance_factor, range_factor);
    
    // Speed based on how centered the ball is
    if (std::abs(target_ball_.bearing) > 0.4)
    {
        // Ball is to the side - slow down more and turn
        linear_vel = approach_speed_ * 0.3f * slow_factor;
        angular_vel *= 1.5;
    }
    else
    {
        // Ball is ahead - approach with controlled speed
        linear_vel = approach_speed_ * 0.6f * slow_factor;
    }
    
    // Minimum speed to keep moving, but STOP if very close
    if (min_front_range < collect_distance_m_ + 0.3f)
    {
        // Very close - creep slowly
        linear_vel = std::min(linear_vel, 0.2f);
        RCLCPP_DEBUG(this->get_logger(), "APPROACHING: Creeping slowly, range=%.2fm", min_front_range);
    }
    
    // Clamp angular velocity
    angular_vel = std::clamp(angular_vel, static_cast<float>(-max_steer_), static_cast<float>(max_steer_));
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "APPROACHING: '%s' bearing=%.2f size=%.1f/%.1f vel=(%.2f, %.2f) front=%.2f",
        target_ball_.name.c_str(), target_ball_.bearing, 
        target_ball_.apparent_size, approach_radius_threshold_,
        linear_vel, angular_vel, min_front_range);
    
    publish_cmd_vel(linear_vel, angular_vel);
}

// =============================================================================
// COLLECTING state - Delete the ball entity
// =============================================================================

void NavBallCollectorNode::execute_collecting()
{
    if (!target_ball_.valid)
    {
        RCLCPP_WARN(this->get_logger(), "COLLECTING: No valid target");
        transition_to(NavCollectorState::EXPLORING);
        return;
    }
    
    // Stop the robot
    publish_cmd_vel(0.0, 0.0);
    
    // Delete the ball if not already pending
    // Camera saw ball close enough (apparent_size > threshold), so collect it
    if (!delete_pending_)
    {
        RCLCPP_INFO(this->get_logger(), 
            "COLLECTING: Deleting ball '%s' (size was %.1f)", 
            target_ball_.name.c_str(), target_ball_.apparent_size);
        delete_entity(target_ball_.name);
        delete_pending_ = true;
    }
}

// =============================================================================
// RECOVERING state - PART D: Diagonal Reverse Recovery for Ackermann
// Key principle: If escape LEFT, backup with RIGHT steering (and vice versa)
// This creates a diagonal reverse that pivots the robot away from the wall
// =============================================================================

void NavBallCollectorNode::execute_recovering()
{
    double elapsed = (this->now() - recover_start_time_).seconds();
    
    // Find escape direction if not set (Phase 0 - initialization)
    if (recover_phase_ == 0)
    {
        float escape_angle, escape_dist;
        if (find_escape_direction(escape_angle, escape_dist))
        {
            escape_target_angle_ = escape_angle;
            
            // PART D: Determine escape direction, then use OPPOSITE for reverse steering
            // If best escape is LEFT (positive angle), we need to steer RIGHT while reversing
            // This creates a diagonal reverse that swings the front toward the escape direction
            if (escape_angle > 0)
            {
                recover_turn_direction_ = -1.0f;  // Escape LEFT -> reverse steer RIGHT
                RCLCPP_INFO(this->get_logger(), 
                    "RECOVERING: Escape LEFT (%.1f deg) -> Diagonal reverse RIGHT",
                    escape_angle * 180.0 / M_PI);
            }
            else
            {
                recover_turn_direction_ = 1.0f;   // Escape RIGHT -> reverse steer LEFT
                RCLCPP_INFO(this->get_logger(), 
                    "RECOVERING: Escape RIGHT (%.1f deg) -> Diagonal reverse LEFT",
                    escape_angle * 180.0 / M_PI);
            }
        }
        else
        {
            // No clear escape - check which side has more space
            float min_front, min_left, min_right;
            check_obstacle_sectors(min_front, min_left, min_right);
            
            // PART D: Reverse steering opposite to the more open side
            if (min_left > min_right)
            {
                recover_turn_direction_ = -1.0f;  // More space LEFT -> reverse steer RIGHT
                escape_target_angle_ = M_PI / 2;
            }
            else
            {
                recover_turn_direction_ = 1.0f;   // More space RIGHT -> reverse steer LEFT
                escape_target_angle_ = -M_PI / 2;
            }
            RCLCPP_INFO(this->get_logger(), 
                "RECOVERING: Side space L=%.2f R=%.2f -> Diagonal reverse %s",
                min_left, min_right, recover_turn_direction_ > 0 ? "LEFT" : "RIGHT");
        }
        recover_phase_ = 1;
    }
    
    float min_front, min_left, min_right;
    check_obstacle_sectors(min_front, min_left, min_right);
    
    // PART D: Diagonal reverse recovery phases
    if (elapsed < recover_duration_ * 0.6)
    {
        // Phase 1: DIAGONAL REVERSE - backup with steering to swing front toward escape
        // This is the key maneuver for Ackermann vehicles
        float steer_intensity = std::min(static_cast<float>(std::abs(escape_target_angle_) / M_PI), 1.0f);
        float reverse_steer = recover_turn_direction_ * max_steer_ * (0.8f + steer_intensity * 0.7f);
        
        publish_cmd_vel(-recover_speed_ * 1.2, reverse_steer);
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "RECOVERING Phase 1: Diagonal reverse vel=%.2f steer=%.2f", 
            -recover_speed_ * 1.2, reverse_steer);
    }
    else if (elapsed < recover_duration_ * 0.85)
    {
        // Phase 2: Forward arc toward escape direction
        // Now steer in the SAME direction as escape (opposite of reverse phase)
        float forward_steer = -recover_turn_direction_ * max_steer_ * 1.2f;
        
        if (min_front > obstacle_stop_m_ * 0.7)
        {
            publish_cmd_vel(approach_speed_ * 0.6, forward_steer);
        }
        else
        {
            // Front still blocked - continue diagonal reverse
            publish_cmd_vel(-recover_speed_ * 0.8, recover_turn_direction_ * max_steer_);
        }
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "RECOVERING Phase 2: Forward arc, front=%.2f", min_front);
    }
    else if (elapsed < recover_duration_)
    {
        // Phase 3: Short forward burst to clear
        if (min_front > obstacle_stop_m_)
        {
            // Good clearance - go forward with slight steering toward open space
            float final_steer = -recover_turn_direction_ * max_steer_ * 0.4f;
            publish_cmd_vel(approach_speed_ * 0.8, final_steer);
        }
        else
        {
            // Still not clear - one more diagonal reverse
            publish_cmd_vel(-recover_speed_ * 0.6, recover_turn_direction_ * max_steer_ * 0.8f);
        }
    }
    else
    {
        // Recovery complete - check if we actually escaped
        if (min_front > obstacle_stop_m_ * 1.2)
        {
            RCLCPP_INFO(this->get_logger(), 
                "RECOVERING: Diagonal recovery complete! Front clear at %.2fm", min_front);
            transition_to(NavCollectorState::EXPLORING);
        }
        else
        {
            // Not clear yet - restart recovery with potentially different direction
            RCLCPP_WARN(this->get_logger(), 
                "RECOVERING: Front still blocked (%.2fm), trying again...", min_front);
            recover_phase_ = 0;  // Reset to find new escape direction
            recover_start_time_ = this->now();
            // Flip direction to try the other way
            escape_target_angle_ = -escape_target_angle_;
        }
    }
}

// =============================================================================
// State transition
// =============================================================================

void NavBallCollectorNode::transition_to(NavCollectorState new_state)
{
    if (new_state == current_state_)
    {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
        nav_state_to_string(current_state_).c_str(),
        nav_state_to_string(new_state).c_str());
    
    // Exit actions
    switch (current_state_)
    {
        case NavCollectorState::NAVIGATING:
        case NavCollectorState::EXPLORING:
            if (navigation_in_progress_)
            {
                cancel_navigation();
            }
            break;
        default:
            break;
    }
    
    // Entry actions
    switch (new_state)
    {
        case NavCollectorState::RECOVERING:
            recover_start_time_ = this->now();
            recover_phase_ = 0;
            break;
        case NavCollectorState::EXPLORING:
            navigation_in_progress_ = false;
            break;
        case NavCollectorState::NAVIGATING:
            navigation_in_progress_ = false;
            navigation_succeeded_ = false;
            break;
        default:
            break;
    }
    
    current_state_ = new_state;
    last_progress_time_ = this->now();
}

// =============================================================================
// Navigation functions
// =============================================================================

bool NavBallCollectorNode::send_navigation_goal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
    // Check if we're in a rejection cooldown period
    double rejection_cooldown = std::min(2.0 + consecutive_rejections_ * 0.5, 10.0);  // 2-10 seconds
    double time_since_rejection = (this->now() - last_goal_rejection_time_).seconds();
    if (consecutive_rejections_ > 0 && time_since_rejection < rejection_cooldown)
    {
        RCLCPP_DEBUG(this->get_logger(), 
            "In goal rejection cooldown (%.1fs remaining)",
            rejection_cooldown - time_since_rejection);
        return false;
    }
    
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::milliseconds(100)))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "IDLE: Waiting for Nav2 action server...");
        return false;
    }
    
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = map_frame_;
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavBallCollectorNode::navigate_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavBallCollectorNode::navigate_feedback_callback, this, 
            std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavBallCollectorNode::navigate_result_callback, this, std::placeholders::_1);
    
    // Mark as in progress immediately to prevent multiple goals
    navigation_in_progress_ = true;
    
    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    current_nav_goal_ = goal_pose;
    
    return true;
}

void NavBallCollectorNode::cancel_navigation()
{
    if (navigation_in_progress_ && current_goal_handle_)
    {
        RCLCPP_INFO(this->get_logger(), "Cancelling navigation");
        nav_to_pose_client_->async_cancel_goal(current_goal_handle_);
    }
    navigation_in_progress_ = false;
    current_goal_handle_ = nullptr;
}

void NavBallCollectorNode::navigate_goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
    if (!goal_handle)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        RCLCPP_WARN(this->get_logger(), "Navigation goal was rejected");
        navigation_in_progress_ = false;
        last_goal_rejection_time_ = this->now();
        last_nav_result_time_ = last_goal_rejection_time_;
        consecutive_rejections_++;
        if (explore_with_nav2_)
        {
            explore_with_nav2_ = false;
            nav2_ready_ = false;
            RCLCPP_WARN(this->get_logger(),
                "Nav2 goal rejected - switching to LiDAR exploration mode");
        }
        
        // Log if many rejections (Nav2 not ready)
        if (consecutive_rejections_ >= 5)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Nav2 appears not ready - %d consecutive rejections. Waiting...",
                consecutive_rejections_);
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Navigation goal accepted");
        navigation_in_progress_ = true;
        current_goal_handle_ = goal_handle;
        consecutive_rejections_ = 0;  // Reset on success
    }
}

void NavBallCollectorNode::navigate_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_DEBUG(this->get_logger(), 
        "Nav feedback: distance remaining = %.2f m",
        feedback->distance_remaining);
}

void NavBallCollectorNode::navigate_result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result)
{
    navigation_in_progress_ = false;
    current_goal_handle_ = nullptr;
    last_nav_result_time_ = this->now();
    
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            navigation_succeeded_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Navigation was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Navigation was canceled");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Navigation returned unknown result");
            break;
    }
}

geometry_msgs::msg::PoseStamped NavBallCollectorNode::generate_exploration_goal()
{
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = this->now();
    
    // Generate random waypoint in front of robot, constrained to exploration bounds
    if (latest_odom_)
    {
        double current_x = latest_odom_->pose.pose.position.x;
        double current_y = latest_odom_->pose.pose.position.y;
        
        // Get current yaw
        double qz = latest_odom_->pose.pose.orientation.z;
        double qw = latest_odom_->pose.pose.orientation.w;
        double current_yaw = 2.0 * std::atan2(qz, qw);
        
        // Generate random offset
        std::uniform_real_distribution<double> angle_dist(-M_PI/3, M_PI/3);
        std::uniform_real_distribution<double> dist_dist(1.5, exploration_waypoint_distance_);
        
        double angle_offset = angle_dist(rng_);
        double distance = dist_dist(rng_);
        
        double goal_yaw = current_yaw + angle_offset;
        double goal_x = current_x + distance * std::cos(goal_yaw);
        double goal_y = current_y + distance * std::sin(goal_yaw);
        
        // Clamp to exploration bounds (20m x 20m by default)
        goal_x = std::clamp(goal_x, exploration_min_x_, exploration_max_x_);
        goal_y = std::clamp(goal_y, exploration_min_y_, exploration_max_y_);
        
        // If clamped position is at boundary, face inward
        if (goal_x == exploration_min_x_ || goal_x == exploration_max_x_ ||
            goal_y == exploration_min_y_ || goal_y == exploration_max_y_)
        {
            // Face towards center
            goal_yaw = std::atan2(-goal_y, -goal_x);
        }
        
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.position.z = 0.0;
        
        goal.pose.orientation.z = std::sin(goal_yaw / 2.0);
        goal.pose.orientation.w = std::cos(goal_yaw / 2.0);
    }
    else
    {
        // Default if no odometry
        goal.pose.position.x = 1.0;
        goal.pose.position.y = 0.0;
        goal.pose.orientation.w = 1.0;
    }
    
    exploration_waypoint_index_++;
    return goal;
}

geometry_msgs::msg::PoseStamped NavBallCollectorNode::estimate_ball_world_pose(
    const ballvac_msgs::msg::BallDetection & detection)
{
    geometry_msgs::msg::PoseStamped world_pose;
    world_pose.header.frame_id = map_frame_;
    world_pose.header.stamp = this->now();
    
    // Estimate distance from apparent size
    double distance = estimate_distance_from_size(detection.apparent_size);
    
    // Ball bearing is angle from camera center
    double bearing = detection.bearing;
    
    // Calculate position relative to robot
    if (latest_odom_)
    {
        double robot_x = latest_odom_->pose.pose.position.x;
        double robot_y = latest_odom_->pose.pose.position.y;
        
        // Get robot yaw
        double qz = latest_odom_->pose.pose.orientation.z;
        double qw = latest_odom_->pose.pose.orientation.w;
        double robot_yaw = 2.0 * std::atan2(qz, qw);
        
        // Ball angle in world frame
        double ball_angle = robot_yaw - bearing;  // Negative because bearing is positive to right
        
        // Ball position in world frame
        world_pose.pose.position.x = robot_x + distance * std::cos(ball_angle);
        world_pose.pose.position.y = robot_y + distance * std::sin(ball_angle);
        world_pose.pose.position.z = 0.0;
        world_pose.pose.orientation.w = 1.0;
    }
    
    return world_pose;
}

double NavBallCollectorNode::estimate_distance_from_size(double apparent_size)
{
    // Using pinhole camera model:
    // apparent_size (pixels) = focal_length * actual_diameter / distance
    // distance = focal_length * actual_diameter / apparent_size
    
    if (apparent_size < 1.0)
    {
        return 10.0;  // Very far
    }
    
    double distance = (camera_focal_length_ * ball_actual_diameter_) / apparent_size;
    
    // Clamp to reasonable range
    return std::clamp(distance, 0.3, 15.0);
}

bool NavBallCollectorNode::transform_pose(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output,
    const std::string & target_frame)
{
    try
    {
        output = tf_buffer_->transform(input, target_frame, tf2::durationFromSec(0.5));
        return true;
    }
    catch (tf2::TransformException & ex)
    {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return false;
    }
}

// =============================================================================
// Helper functions
// =============================================================================

bool NavBallCollectorNode::check_obstacle_front(float & min_range)
{
    float min_left, min_right;
    check_obstacle_sectors(min_range, min_left, min_right);
    return min_range < obstacle_stop_m_;
}

void NavBallCollectorNode::check_obstacle_sectors(float & min_front, float & min_left, float & min_right)
{
    min_front = 10.0f;
    min_left = 10.0f;
    min_right = 10.0f;
    
    if (!latest_scan_)
    {
        return;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    if (num_readings == 0)
    {
        return;
    }
    
    float angle_min = latest_scan_->angle_min;
    float angle_increment = latest_scan_->angle_increment;
    
    auto angle_to_index = [&](float angle) -> size_t {
        angle = std::clamp(angle, angle_min, latest_scan_->angle_max);
        int idx = static_cast<int>((angle - angle_min) / angle_increment);
        return static_cast<size_t>(std::clamp(idx, 0, static_cast<int>(num_readings - 1)));
    };
    
    // Front sector: -30° to +30° (narrower for accurate front detection)
    size_t front_start = angle_to_index(-0.52f);
    size_t front_end = angle_to_index(0.52f);
    
    // Left sector: +30° to +120°
    size_t left_start = angle_to_index(0.52f);
    size_t left_end = angle_to_index(2.09f);
    
    // Right sector: -120° to -30°
    size_t right_start = angle_to_index(-2.09f);
    size_t right_end = angle_to_index(-0.52f);
    
    for (size_t i = front_start; i <= front_end && i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
        {
            min_front = std::min(min_front, latest_scan_->ranges[i]);
        }
    }
    
    for (size_t i = left_start; i <= left_end && i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
        {
            min_left = std::min(min_left, latest_scan_->ranges[i]);
        }
    }
    
    for (size_t i = right_start; i <= right_end && i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
        {
            min_right = std::min(min_right, latest_scan_->ranges[i]);
        }
    }
}

void NavBallCollectorNode::check_diagonal_sectors(float & min_front_left, float & min_front_right)
{
    min_front_left = 10.0f;
    min_front_right = 10.0f;
    
    if (!latest_scan_)
    {
        return;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    if (num_readings == 0)
    {
        return;
    }
    
    float angle_min = latest_scan_->angle_min;
    float angle_increment = latest_scan_->angle_increment;
    
    auto angle_to_index = [&](float angle) -> size_t {
        angle = std::clamp(angle, angle_min, latest_scan_->angle_max);
        int idx = static_cast<int>((angle - angle_min) / angle_increment);
        return static_cast<size_t>(std::clamp(idx, 0, static_cast<int>(num_readings - 1)));
    };
    
    // Front-left diagonal: +20° to +50° (for early wall detection)
    size_t fl_start = angle_to_index(0.35f);
    size_t fl_end = angle_to_index(0.87f);
    
    // Front-right diagonal: -50° to -20°
    size_t fr_start = angle_to_index(-0.87f);
    size_t fr_end = angle_to_index(-0.35f);
    
    for (size_t i = fl_start; i <= fl_end && i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
        {
            min_front_left = std::min(min_front_left, latest_scan_->ranges[i]);
        }
    }
    
    for (size_t i = fr_start; i <= fr_end && i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
        {
            min_front_right = std::min(min_front_right, latest_scan_->ranges[i]);
        }
    }
}

float NavBallCollectorNode::compute_obstacle_avoidance_steering()
{
    if (!latest_scan_)
    {
        return 0.0f;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    if (num_readings == 0)
    {
        return 0.0f;
    }
    
    float angle_min = latest_scan_->angle_min;
    float angle_increment = latest_scan_->angle_increment;
    
    auto angle_to_index = [&](float angle) -> size_t {
        angle = std::clamp(angle, angle_min, latest_scan_->angle_max);
        int idx = static_cast<int>((angle - angle_min) / angle_increment);
        return static_cast<size_t>(std::clamp(idx, 0, static_cast<int>(num_readings - 1)));
    };
    
    // Calculate min ranges for left and right
    size_t left_start = angle_to_index(0.0f);
    size_t left_end = angle_to_index(1.57f);
    size_t right_start = angle_to_index(-1.57f);
    size_t right_end = angle_to_index(0.0f);
    
    float min_left = 50.0f;
    float min_right = 50.0f;
    
    for (size_t i = left_start; i <= left_end && i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
            min_left = std::min(min_left, latest_scan_->ranges[i]);
    }
    
    for (size_t i = right_start; i <= right_end && i < num_readings; ++i)
    {
        if (std::isfinite(latest_scan_->ranges[i]) && latest_scan_->ranges[i] > 0.1f)
            min_right = std::min(min_right, latest_scan_->ranges[i]);
    }
    
    // Steer away from closer obstacle
    if (min_left < min_right)
    {
        return -max_steer_ * (1.0f - min_left / obstacle_slow_m_);  // Turn right
    }
    else
    {
        return max_steer_ * (1.0f - min_right / obstacle_slow_m_);  // Turn left
    }
}

// =============================================================================
// Corner Detection and Escape Functions
// =============================================================================

bool NavBallCollectorNode::detect_corner_situation()
{
    if (!latest_scan_)
    {
        return false;
    }
    
    float min_front, min_left, min_right;
    check_obstacle_sectors(min_front, min_left, min_right);
    
    // MUCH MORE STRICT corner detection - only trigger when truly stuck
    // Use obstacle_stop_m_ (0.7m) as the critical threshold instead of obstacle_slow_m_ (1.8m)
    bool front_blocked = min_front < obstacle_stop_m_ * 1.2;  // 0.84m - very close
    bool left_blocked = min_left < obstacle_stop_m_;           // 0.7m
    bool right_blocked = min_right < obstacle_stop_m_;         // 0.7m
    
    // ONLY trigger if truly stuck: front AND both sides blocked
    if (front_blocked && left_blocked && right_blocked)
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "True corner detected: front=%.2f, left=%.2f, right=%.2f",
            min_front, min_left, min_right);
        return true;
    }
    
    // ADDITIONAL: Detect spinning in place (yaw changing but position not)
    // This catches the loop behavior when robot is stuck turning
    if (latest_odom_)
    {
        static double last_check_x = 0.0;
        static double last_check_y = 0.0;
        static double last_check_yaw = 0.0;
        static double accumulated_yaw_change = 0.0;
        static rclcpp::Time last_check_time = this->now();
        
        double current_x = latest_odom_->pose.pose.position.x;
        double current_y = latest_odom_->pose.pose.position.y;
        double qz = latest_odom_->pose.pose.orientation.z;
        double qw = latest_odom_->pose.pose.orientation.w;
        double current_yaw = 2.0 * std::atan2(qz, qw);
        
        double elapsed = (this->now() - last_check_time).seconds();
        
        if (elapsed > 0.5)  // Check every 0.5 seconds
        {
            double pos_change = std::sqrt(
                std::pow(current_x - last_check_x, 2) + 
                std::pow(current_y - last_check_y, 2));
            
            double yaw_change = std::abs(current_yaw - last_check_yaw);
            // Normalize yaw change
            if (yaw_change > M_PI) yaw_change = 2 * M_PI - yaw_change;
            
            // If position barely changed but yaw changed significantly
            if (pos_change < 0.1 && yaw_change > 0.3)
            {
                accumulated_yaw_change += yaw_change;
                
                // If accumulated yaw change > 2*PI (full rotation) without moving
                if (accumulated_yaw_change > 2 * M_PI)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Spinning in place detected! Total rotation: %.1f deg, moved: %.2fm",
                        accumulated_yaw_change * 180.0 / M_PI, pos_change);
                    accumulated_yaw_change = 0.0;  // Reset
                    return true;  // Trigger escape
                }
            }
            else if (pos_change > 0.3)  // Actually moving - reset counter
            {
                accumulated_yaw_change = 0.0;
            }
            
            last_check_x = current_x;
            last_check_y = current_y;
            last_check_yaw = current_yaw;
            last_check_time = this->now();
        }
    }
    
    return false;
}

bool NavBallCollectorNode::find_escape_direction(float & best_angle, float & best_distance)
{
    if (!latest_scan_)
    {
        return false;
    }
    
    size_t num_readings = latest_scan_->ranges.size();
    if (num_readings == 0)
    {
        return false;
    }
    
    float angle_min = latest_scan_->angle_min;
    float angle_increment = latest_scan_->angle_increment;
    
    // Calculate direction toward center of exploration area (in robot frame)
    float center_direction = 0.0f;  // Direction to face toward center
    if (latest_odom_)
    {
        double x = latest_odom_->pose.pose.position.x;
        double y = latest_odom_->pose.pose.position.y;
        double center_x = (exploration_min_x_ + exploration_max_x_) / 2.0;
        double center_y = (exploration_min_y_ + exploration_max_y_) / 2.0;
        
        // Get robot yaw
        double qz = latest_odom_->pose.pose.orientation.z;
        double qw = latest_odom_->pose.pose.orientation.w;
        double robot_yaw = 2.0 * std::atan2(qz, qw);
        
        // Angle to center in world frame
        double angle_to_center = std::atan2(center_y - y, center_x - x);
        
        // Convert to robot frame
        center_direction = static_cast<float>(angle_to_center - robot_yaw);
        while (center_direction > M_PI) center_direction -= 2 * M_PI;
        while (center_direction < -M_PI) center_direction += 2 * M_PI;
    }

    // Find the widest gap (most open direction)
    best_angle = 0.0f;
    best_distance = 0.0f;
    
    // Sliding window for gap detection
    const int window_size = num_readings / 12;  // ~30 degree window
    float max_avg_distance = 0.0f;
    
    for (size_t center = window_size; center < num_readings - window_size; ++center)
    {
        float sum_distance = 0.0f;
        int valid_count = 0;
        
        for (int offset = -window_size; offset <= window_size; ++offset)
        {
            size_t idx = center + offset;
            if (std::isfinite(latest_scan_->ranges[idx]) && 
                latest_scan_->ranges[idx] > 0.1f &&
                latest_scan_->ranges[idx] < 50.0f)
            {
                sum_distance += latest_scan_->ranges[idx];
                valid_count++;
            }
        }
        
        if (valid_count > window_size)  // At least half the window has valid readings
        {
            float avg_distance = sum_distance / valid_count;
            
            // Prefer directions that are more to the side (better for turning)
            float angle = angle_min + center * angle_increment;
            float side_bonus = std::abs(angle) * 0.5f;  // Bonus for side directions
            
            // IMPORTANT: Add bonus for directions toward center
            float center_bonus = 0.0f;
            float angle_diff = std::abs(angle - center_direction);
            if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
            if (angle_diff < M_PI / 2)  // Within 90 degrees of center direction
            {
                center_bonus = (1.0f - angle_diff / (M_PI / 2)) * 2.0f;  // Up to 2m bonus
            }
            
            float effective_distance = avg_distance + side_bonus + center_bonus;
            
            if (effective_distance > max_avg_distance)
            {
                max_avg_distance = effective_distance;
                best_angle = angle;
                best_distance = avg_distance;
            }
        }
    }
    
    // Only return true if we found a reasonably open direction
    // Also check if this direction is blocked
    for (float blocked : blocked_directions_)
    {
        if (std::abs(best_angle - blocked) < 0.5f)
        {
            // This direction is blocked, find next best
            best_distance *= 0.3f;  // Penalize this direction heavily
        }
    }
    
    return best_distance > obstacle_stop_m_ * 1.5;
}

bool NavBallCollectorNode::execute_lidar_escape()
{
    double elapsed = (this->now() - corner_escape_start_time_).seconds();
    
    // Clear blocked directions after 10 seconds
    if ((this->now() - blocked_directions_clear_time_).seconds() > 10.0)
    {
        blocked_directions_.clear();
        blocked_directions_clear_time_ = this->now();
    }
    
    // PART D: Diagonal reverse escape timing - optimized for Ackermann
    const double diagonal_reverse_duration = 1.0;   // Diagonal reverse phase (longer)
    const double forward_arc_duration = 0.8;        // Forward arc phase
    const double forward_burst_duration = 1.0;      // Final forward burst
    const double total_duration = diagonal_reverse_duration + forward_arc_duration + forward_burst_duration;
    
    float min_front, min_left, min_right;
    check_obstacle_sectors(min_front, min_left, min_right);
    
    // Check if we're near map boundaries - if so, steer toward center
    float boundary_steer_override = 0.0f;
    bool near_boundary = false;
    if (latest_odom_)
    {
        double x = latest_odom_->pose.pose.position.x;
        double y = latest_odom_->pose.pose.position.y;
        
        // Define safe boundaries (smaller than exploration bounds)
        const double safe_margin = 2.0;  // 2m margin from boundaries
        double safe_min_x = exploration_min_x_ + safe_margin;
        double safe_max_x = exploration_max_x_ - safe_margin;
        double safe_min_y = exploration_min_y_ + safe_margin;
        double safe_max_y = exploration_max_y_ - safe_margin;
        
        // Check each boundary and calculate steering toward center
        if (x < safe_min_x) {
            boundary_steer_override = -max_steer_;  // Steer toward +X (right)
            near_boundary = true;
            RCLCPP_DEBUG(this->get_logger(), "BOUNDARY: Near -X edge, steering right");
        } else if (x > safe_max_x) {
            boundary_steer_override = max_steer_;   // Steer toward -X (left)
            near_boundary = true;
            RCLCPP_DEBUG(this->get_logger(), "BOUNDARY: Near +X edge, steering left");
        }
        
        if (y < safe_min_y) {
            boundary_steer_override = max_steer_;   // Steer toward +Y (left)
            near_boundary = true;
            RCLCPP_DEBUG(this->get_logger(), "BOUNDARY: Near -Y edge, steering left");
        } else if (y > safe_max_y) {
            boundary_steer_override = -max_steer_;  // Steer toward -Y (right)
            near_boundary = true;
            RCLCPP_DEBUG(this->get_logger(), "BOUNDARY: Near +Y edge, steering right");
        }
    }
    
    // IMPORTANT: For Ackermann steering when reversing:
    // - Steering RIGHT while going BACKWARD makes the FRONT go LEFT
    // - Steering LEFT while going BACKWARD makes the FRONT go RIGHT
    // So if escape direction is LEFT (positive), we need to steer LEFT while reversing
    // to make the front swing RIGHT, then when we go forward the front faces LEFT
    
    // Calculate escape steering - same direction as escape target
    float escape_steer_dir = (escape_target_angle_ > 0) ? 1.0f : -1.0f;
    
    // ALTERNATE direction if last escape was same direction (prevent same-side loops)
    if (std::abs(last_escape_direction_) > 0.1f && 
        (escape_steer_dir > 0) == (last_escape_direction_ > 0))
    {
        // Last escape was same direction - try opposite
        escape_steer_dir = -escape_steer_dir;
        RCLCPP_DEBUG(this->get_logger(), "ESCAPE: Alternating direction to prevent loop");
    }
    
    if (elapsed < diagonal_reverse_duration)
    {
        // Phase 1: REVERSE with steering - For Ackermann, steer SAME direction as desired front turn
        // This will swing the rear opposite, positioning front toward escape
        escape_phase_ = 1;
        float steer_intensity = std::min(1.0f, std::abs(escape_target_angle_) / (float)M_PI_2);
        
        // Use boundary override if near edge
        float reverse_steer = near_boundary ? boundary_steer_override : 
                              (escape_steer_dir * max_steer_ * (0.8f + steer_intensity * 0.6f));
        
        publish_cmd_vel(-recover_speed_ * 0.8, reverse_steer);
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "ESCAPE Phase 1: Reverse steer=%.2f (escape=%.1f deg, near_boundary=%d)", 
            reverse_steer, escape_target_angle_ * 180.0 / M_PI, near_boundary);
    }
    else if (elapsed < diagonal_reverse_duration + forward_arc_duration)
    {
        // Phase 2: FORWARD ARC - drive forward with OPPOSITE steering
        // After reversing with steer X, we go forward with steer -X to arc away
        escape_phase_ = 2;
        
        // Opposite of reverse steer direction for arc
        float forward_steer = near_boundary ? boundary_steer_override :
                              (-escape_steer_dir * max_steer_ * 1.0f);
        
        if (min_front > obstacle_stop_m_ * 0.8)
        {
            // Front clearing - forward arc
            publish_cmd_vel(approach_speed_ * 0.6, forward_steer);
        }
        else
        {
            // Still blocked - continue reversing with same steering
            float reverse_steer = near_boundary ? boundary_steer_override :
                                  (escape_steer_dir * max_steer_ * 0.8f);
            publish_cmd_vel(-recover_speed_ * 0.5, reverse_steer);
        }
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "ESCAPE Phase 2: Forward arc, front=%.2f, steer=%.2f", min_front, forward_steer);
    }
    else if (elapsed < total_duration)
    {
        // Phase 3: FORWARD BURST - drive toward open space
        escape_phase_ = 3;
        
        if (min_front > obstacle_stop_m_)
        {
            // Go forward with steering away from walls
            float steer = 0.0f;
            if (near_boundary)
            {
                steer = boundary_steer_override * 0.5f;
            }
            else if (min_left < obstacle_slow_m_ && min_right > min_left)
            {
                steer = -max_steer_ * 0.5f;  // Steer right (away from left wall)
            }
            else if (min_right < obstacle_slow_m_ && min_left > min_right)
            {
                steer = max_steer_ * 0.5f;   // Steer left (away from right wall)
            }
            
            publish_cmd_vel(approach_speed_ * 0.8, steer);
        }
        else
        {
            // Front blocked - another reverse attempt with opposite steering
            float reverse_steer = near_boundary ? boundary_steer_override :
                                  (-escape_steer_dir * max_steer_ * 0.7f);
            publish_cmd_vel(-recover_speed_ * 0.5, reverse_steer);
        }
    }
    else
    {
        // Escape attempt complete - check if successful
        if (min_front > obstacle_slow_m_)
        {
            RCLCPP_INFO(this->get_logger(), "ESCAPE: Success! Front clear at %.2fm", min_front);
            
            if (latest_odom_)
            {
                last_escape_x_ = latest_odom_->pose.pose.position.x;
                last_escape_y_ = latest_odom_->pose.pose.position.y;
            }
            last_progress_time_ = this->now();
            same_direction_attempts_ = 0;
            blocked_directions_.clear();
            
            return true;  // Escape complete!
        }
        else
        {
            // Failed - add this direction to blocked list
            blocked_directions_.push_back(escape_target_angle_);
            same_direction_attempts_++;
            
            RCLCPP_WARN(this->get_logger(), 
                "ESCAPE: Direction %.2f rad BLOCKED! Attempt #%d, trying different direction", 
                escape_target_angle_, same_direction_attempts_);
            
            // Find a completely different direction
            float best_angle = 0.0f;
            float best_dist = 0.0f;
            
            // Scan for the most open direction that's NOT blocked
            if (latest_scan_)
            {
                size_t num_readings = latest_scan_->ranges.size();
                float angle_min = latest_scan_->angle_min;
                float angle_increment = latest_scan_->angle_increment;
                
                for (size_t i = 0; i < num_readings; i += num_readings / 8)
                {
                    float angle = angle_min + i * angle_increment;
                    float range = latest_scan_->ranges[i];
                    
                    if (!std::isfinite(range) || range < 0.1f)
                        continue;
                    
                    // Check if this direction is blocked
                    bool is_blocked = false;
                    for (float blocked : blocked_directions_)
                    {
                        if (std::abs(angle - blocked) < 0.8f)
                        {
                            is_blocked = true;
                            break;
                        }
                    }
                    
                    if (!is_blocked && range > best_dist)
                    {
                        best_dist = range;
                        best_angle = angle;
                    }
                }
            }
            
            if (best_dist > obstacle_stop_m_)
            {
                escape_target_angle_ = best_angle;
                RCLCPP_INFO(this->get_logger(), 
                    "ESCAPE: Found unblocked direction: %.2f rad (%.2fm clear)", 
                    best_angle, best_dist);
            }
            else
            {
                // All directions seem blocked - try opposite of last attempt
                escape_target_angle_ = -escape_target_angle_;
                
                // If we've tried too many times, clear blocked list and start fresh
                if (same_direction_attempts_ >= 4)
                {
                    blocked_directions_.clear();
                    same_direction_attempts_ = 0;
                    RCLCPP_WARN(this->get_logger(), "ESCAPE: Clearing blocked list, starting fresh");
                }
            }
            
            corner_escape_start_time_ = this->now();
            escape_phase_ = 0;
            last_escape_direction_ = escape_target_angle_;
            
            return false;  // Continue escaping
        }
    }
    
    return false;  // Still escaping
}

bool NavBallCollectorNode::is_stuck()
{
    if (!latest_odom_)
    {
        return false;
    }
    
    // Check time since last progress
    double time_since_progress = (this->now() - last_progress_time_).seconds();
    
    // Consider stuck if no progress for 15 seconds (increased from 5)
    if (time_since_progress > 15.0)
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Stuck detection: %.1fs since progress", time_since_progress);
        return true;
    }
    
    // Also check if front is blocked for too long
    float min_front, min_left, min_right;
    check_obstacle_sectors(min_front, min_left, min_right);
    
    if (min_front < obstacle_stop_m_ && time_since_progress > 8.0)
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Stuck: front blocked (%.2fm) for %.1fs", min_front, time_since_progress);
        return true;
    }
    
    return false;
}

void NavBallCollectorNode::publish_cmd_vel(float linear, float angular)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
}

void NavBallCollectorNode::delete_entity(const std::string & entity_name)
{
    if (!delete_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Delete service not available");
        delete_pending_ = false;
        transition_to(NavCollectorState::RECOVERING);
        return;
    }
    
    // Store the base entity name and determine the name to try
    std::string name_to_try;
    if (delete_attempt_ == 0)
    {
        name_to_try = entity_name;
    }
    else
    {
        // Try with _2 suffix
        name_to_try = entity_name + "_2";
    }
    current_delete_name_ = name_to_try;
    
    RCLCPP_INFO(this->get_logger(), "Attempting to delete entity: %s (attempt %d)", 
        name_to_try.c_str(), delete_attempt_ + 1);
    
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = name_to_try;
    request->entity.type = 2;  // MODEL type
    
    delete_client_->async_send_request(request,
        std::bind(&NavBallCollectorNode::delete_entity_callback, this, std::placeholders::_1));
}

void NavBallCollectorNode::delete_entity_callback(
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    try
    {
        auto result = future.get();
        if (result->success)
        {
            // Reset delete attempts on success
            delete_attempt_ = 0;
            delete_pending_ = false;
            
            RCLCPP_INFO(this->get_logger(), 
                "Successfully collected ball '%s' (deleted: %s)!", 
                target_ball_.name.c_str(), current_delete_name_.c_str());

            // Publish deletion event for other robots (and optional respawn)
            auto delete_msg = std_msgs::msg::String();
            delete_msg.data = current_delete_name_;
            ball_deleted_pub_->publish(delete_msg);
            
            // Notify fleet coordinator (if enabled)
            if (use_fleet_coordinator_)
            {
                publish_collected(target_ball_.name);
            }
            
            // Track collection
            collected_balls_.insert(current_delete_name_);
            collected_balls_.insert(target_ball_.name);
            ball_collect_count_[target_ball_.color]++;
            last_collection_time_ = this->now();
            
            // Log stats
            int total = 0;
            for (const auto & pair : ball_collect_count_)
            {
                total += pair.second;
                RCLCPP_INFO(this->get_logger(), "  %s balls: %d", 
                    pair.first.c_str(), pair.second);
            }
            RCLCPP_INFO(this->get_logger(), "Total collected: %d", total);
            
            if (respawn_balls_)
            {
                // Spawn new ball with the same name that was deleted
                spawn_ball_with_name(target_ball_.color, current_delete_name_);
            }
            
            // Reset target and continue exploring
            target_ball_.valid = false;
            transition_to(NavCollectorState::EXPLORING);
        }
        else
        {
            // Try alternative name if first attempt failed
            if (delete_attempt_ < 1)
            {
                RCLCPP_WARN(this->get_logger(), 
                    "Failed to delete '%s', trying alternative name...", 
                    current_delete_name_.c_str());
                delete_attempt_++;
                delete_pending_ = true;  // Keep pending
                delete_entity(target_ball_.name);  // Try again with next variant
            }
            else
            {
                // Both attempts failed - count as collected anyway and move on
                RCLCPP_WARN(this->get_logger(), 
                    "Could not find ball entity to delete - marking as collected anyway");
                delete_attempt_ = 0;
                delete_pending_ = false;
                
                // Still count the collection
                ball_collect_count_[target_ball_.color]++;
                last_collection_time_ = this->now();
                
                if (respawn_balls_)
                {
                    // Spawn new ball with base name
                    spawn_ball(target_ball_.color);
                }
                
                target_ball_.valid = false;
                transition_to(NavCollectorState::EXPLORING);
            }
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), "Delete service exception: %s", e.what());
        delete_attempt_ = 0;
        delete_pending_ = false;
        transition_to(NavCollectorState::RECOVERING);
    }
}

void NavBallCollectorNode::spawn_ball(const std::string & color)
{
    std::string entity_name = get_entity_name(color);
    spawn_ball_with_name(color, entity_name);
}

void NavBallCollectorNode::spawn_ball_with_name(const std::string & color, const std::string & entity_name)
{
    if (!spawn_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Spawn service not available");
        return;
    }
    
    double x, y;
    get_random_spawn_position(x, y);
    
    auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    request->entity_factory.name = entity_name;
    request->entity_factory.allow_renaming = false;  // Keep exact name
    request->entity_factory.sdf = generate_ball_sdf(color, entity_name);
    request->entity_factory.pose.position.x = x;
    request->entity_factory.pose.position.y = y;
    request->entity_factory.pose.position.z = 0.1;
    
    spawn_client_->async_send_request(request,
        std::bind(&NavBallCollectorNode::spawn_entity_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Spawning new %s ball '%s' at (%.2f, %.2f)", 
        color.c_str(), entity_name.c_str(), x, y);
}

void NavBallCollectorNode::spawn_entity_callback(
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future)
{
    try
    {
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully spawned new ball");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to spawn ball");
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), "Spawn service exception: %s", e.what());
    }
}

std::string NavBallCollectorNode::generate_ball_sdf(
    const std::string & color, const std::string & entity_name)
{
    // Color RGB values
    std::string rgb;
    if (color == "red") rgb = "1 0 0 1";
    else if (color == "green") rgb = "0 1 0 1";
    else if (color == "blue") rgb = "0 0 1 1";
    else if (color == "yellow") rgb = "1 1 0 1";
    else rgb = "1 0 1 1";  // Magenta for unknown
    
    std::string sdf = R"(
<?xml version="1.0"?>
<sdf version="1.9">
  <model name=")" + entity_name + R"(">
    <static>false</static>
    <link name="link">
      <pose>0 0 0.075 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000225</ixx>
          <iyy>0.000225</iyy>
          <izz>0.000225</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>0.075</radius></sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere><radius>0.075</radius></sphere>
        </geometry>
        <material>
          <ambient>)" + rgb + R"(</ambient>
          <diffuse>)" + rgb + R"(</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
)";
    return sdf;
}

void NavBallCollectorNode::get_random_spawn_position(double & x, double & y)
{
    // Random position in arena - avoid robot spawn areas
    // Robots spawn at: ballvac1 (0, -3), ballvac2 (-3, 0)
    std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
    std::uniform_real_distribution<double> y_dist(-5.0, 5.0);
    
    // Get robot position to avoid spawning too close
    double robot_x = 0.0, robot_y = 0.0;
    if (latest_odom_)
    {
        robot_x = latest_odom_->pose.pose.position.x;
        robot_y = latest_odom_->pose.pose.position.y;
    }
    
    // Try up to 10 times to find a position away from robot
    const double min_dist_from_robot = 3.0;  // At least 3m from robot
    for (int attempts = 0; attempts < 10; attempts++)
    {
        x = x_dist(rng_);
        y = y_dist(rng_);
        
        double dist = std::hypot(x - robot_x, y - robot_y);
        if (dist >= min_dist_from_robot)
        {
            return;  // Good position found
        }
    }
    // If all attempts failed, just use last generated position
}

std::string NavBallCollectorNode::get_entity_name(const std::string & color)
{
    // Entity names match the SDF model names (simple format: ball_color)
    return "ball_" + color;
}

bool NavBallCollectorNode::is_ball_near_wall(const geometry_msgs::msg::PoseStamped & ball_pose, double threshold)
{
    double bx = ball_pose.pose.position.x;
    double by = ball_pose.pose.position.y;
    
    // Check distance to arena boundaries (assumes 20m x 20m arena centered at origin)
    double dist_to_min_x = bx - exploration_min_x_;
    double dist_to_max_x = exploration_max_x_ - bx;
    double dist_to_min_y = by - exploration_min_y_;
    double dist_to_max_y = exploration_max_y_ - by;
    
    return (dist_to_min_x < threshold || dist_to_max_x < threshold ||
            dist_to_min_y < threshold || dist_to_max_y < threshold);
}

geometry_msgs::msg::PoseStamped NavBallCollectorNode::calculate_safe_approach_pose(
    const geometry_msgs::msg::PoseStamped & ball_pose)
{
    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header = ball_pose.header;
    approach_pose.header.frame_id = map_frame_;
    
    double bx = ball_pose.pose.position.x;
    double by = ball_pose.pose.position.y;
    
    // Check which walls are nearby
    double dist_to_min_x = bx - exploration_min_x_;
    double dist_to_max_x = exploration_max_x_ - bx;
    double dist_to_min_y = by - exploration_min_y_;
    double dist_to_max_y = exploration_max_y_ - by;
    
    double approach_offset = 1.5;  // How far to stand back from ball
    double offset_x = 0.0;
    double offset_y = 0.0;
    
    // Determine approach direction based on which wall is closest
    // Approach from the OPPOSITE direction of the wall
    double min_dist = std::min({dist_to_min_x, dist_to_max_x, dist_to_min_y, dist_to_max_y});
    
    if (min_dist == dist_to_min_x)
    {
        // Ball near -X wall, approach from +X direction
        offset_x = approach_offset;
        RCLCPP_DEBUG(this->get_logger(), "Ball near -X wall, approaching from +X");
    }
    else if (min_dist == dist_to_max_x)
    {
        // Ball near +X wall, approach from -X direction
        offset_x = -approach_offset;
        RCLCPP_DEBUG(this->get_logger(), "Ball near +X wall, approaching from -X");
    }
    else if (min_dist == dist_to_min_y)
    {
        // Ball near -Y wall, approach from +Y direction
        offset_y = approach_offset;
        RCLCPP_DEBUG(this->get_logger(), "Ball near -Y wall, approaching from +Y");
    }
    else if (min_dist == dist_to_max_y)
    {
        // Ball near +Y wall, approach from -Y direction
        offset_y = -approach_offset;
        RCLCPP_DEBUG(this->get_logger(), "Ball near +Y wall, approaching from -Y");
    }
    
    // If near a corner (two walls close), adjust approach
    int walls_close = 0;
    if (dist_to_min_x < 2.0) walls_close++;
    if (dist_to_max_x < 2.0) walls_close++;
    if (dist_to_min_y < 2.0) walls_close++;
    if (dist_to_max_y < 2.0) walls_close++;
    
    if (walls_close >= 2)
    {
        // Corner situation - approach diagonally from the open quadrant
        offset_x = (dist_to_min_x < dist_to_max_x) ? approach_offset : -approach_offset;
        offset_y = (dist_to_min_y < dist_to_max_y) ? approach_offset : -approach_offset;
        // Normalize to maintain distance
        double len = std::sqrt(offset_x*offset_x + offset_y*offset_y);
        offset_x = offset_x / len * approach_offset;
        offset_y = offset_y / len * approach_offset;
        RCLCPP_DEBUG(this->get_logger(), "Ball in corner, approaching diagonally");
    }
    
    approach_pose.pose.position.x = bx + offset_x;
    approach_pose.pose.position.y = by + offset_y;
    approach_pose.pose.position.z = 0.0;
    
    // Clamp to safe bounds (stay away from walls)
    double safety_margin = 1.0;
    approach_pose.pose.position.x = std::clamp(approach_pose.pose.position.x, 
        exploration_min_x_ + safety_margin, exploration_max_x_ - safety_margin);
    approach_pose.pose.position.y = std::clamp(approach_pose.pose.position.y,
        exploration_min_y_ + safety_margin, exploration_max_y_ - safety_margin);
    
    // Face towards the ball
    double dx = bx - approach_pose.pose.position.x;
    double dy = by - approach_pose.pose.position.y;
    double yaw = std::atan2(dy, dx);
    approach_pose.pose.orientation.z = std::sin(yaw / 2.0);
    approach_pose.pose.orientation.w = std::cos(yaw / 2.0);
    
    return approach_pose;
}

bool NavBallCollectorNode::is_approach_path_blocked()
{
    if (!latest_scan_)
    {
        return false;
    }
    
    // Check if there's a wall directly in front in the direction we want to go
    float min_front, min_left, min_right;
    check_obstacle_sectors(min_front, min_left, min_right);
    
    // If wall is close in front, approach is blocked
    return min_front < obstacle_stop_m_ * 1.5;
}

}  // namespace ballvac_ball_collector

// =============================================================================
// Main function
// =============================================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ballvac_ball_collector::NavBallCollectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
