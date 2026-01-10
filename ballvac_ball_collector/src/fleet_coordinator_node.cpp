/**
 * @file fleet_coordinator_node.cpp
 * @brief Implementation of fleet coordinator for multi-robot ball collection
 * 
 * Central coordinator that:
 * - Aggregates ball detections from all robots
 * - Maintains a registry with ball states (UNCLAIMED/CLAIMED/COLLECTED)
 * - Assigns balls based on COLOR PRIORITY then path cost
 * - Handles claim timeouts and robot failures
 */

#include "ballvac_ball_collector/fleet_coordinator_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace ballvac_ball_collector
{

FleetCoordinatorNode::FleetCoordinatorNode(const rclcpp::NodeOptions & options)
: Node("fleet_coordinator", options)
{
    // -------------------------------------------------------------------------
    // Declare parameters
    // -------------------------------------------------------------------------
    
    // Color priority (index 0 = highest priority)
    this->declare_parameter<std::vector<std::string>>(
        "color_priority", 
        {"red", "blue", "green", "yellow", "orange", "purple", "cyan"});
    color_priority_ = this->get_parameter("color_priority").as_string_array();
    
    // Robot names to coordinate
    this->declare_parameter<std::vector<std::string>>(
        "robot_names", {"ballvac1", "ballvac2", "ballvac3"});
    auto robot_names = this->get_parameter("robot_names").as_string_array();
    
    // Timeouts
    this->declare_parameter<double>("claim_timeout_sec", 20.0);  // Reduced for faster reassignment
    claim_timeout_sec_ = this->get_parameter("claim_timeout_sec").as_double();
    
    this->declare_parameter<double>("heartbeat_timeout_sec", 8.0);
    heartbeat_timeout_sec_ = this->get_parameter("heartbeat_timeout_sec").as_double();
    
    // Assignment rate - increased for faster response
    this->declare_parameter<double>("assignment_rate_hz", 4.0);
    assignment_rate_hz_ = this->get_parameter("assignment_rate_hz").as_double();
    
    // Standoff distance
    this->declare_parameter<double>("standoff_distance", 0.5);
    standoff_distance_ = this->get_parameter("standoff_distance").as_double();
    
    // Use planner for cost estimation
    this->declare_parameter<bool>("use_planner_for_cost", false);
    use_planner_for_cost_ = this->get_parameter("use_planner_for_cost").as_bool();
    
    // Map bounds (20m x 20m centered at origin)
    this->declare_parameter<double>("map_bounds_min_x", -10.0);
    map_bounds_min_x_ = this->get_parameter("map_bounds_min_x").as_double();
    
    this->declare_parameter<double>("map_bounds_max_x", 10.0);
    map_bounds_max_x_ = this->get_parameter("map_bounds_max_x").as_double();
    
    this->declare_parameter<double>("map_bounds_min_y", -10.0);
    map_bounds_min_y_ = this->get_parameter("map_bounds_min_y").as_double();
    
    this->declare_parameter<double>("map_bounds_max_y", 10.0);
    map_bounds_max_y_ = this->get_parameter("map_bounds_max_y").as_double();
    
    // Wall safety parameters
    this->declare_parameter<double>("wall_safety_margin", 1.5);  // Stay 1.5m from walls
    wall_safety_margin_ = this->get_parameter("wall_safety_margin").as_double();
    
    // Robot conflict avoidance
    this->declare_parameter<double>("robot_conflict_radius", 2.0);  // Robots shouldn't target same area
    robot_conflict_radius_ = this->get_parameter("robot_conflict_radius").as_double();

    // Initial balls from world file (format: "ball_red_2,x,y")
    this->declare_parameter<std::vector<std::string>>(
        "initial_balls", std::vector<std::string>{});
    auto initial_balls = this->get_parameter("initial_balls").as_string_array();

    // -------------------------------------------------------------------------
    // Create publishers with reliable QoS for coordination
    // -------------------------------------------------------------------------
    auto reliable_qos = rclcpp::QoS(10).reliable().transient_local();
    
    registry_pub_ = this->create_publisher<ballvac_msgs::msg::BallRegistry>(
        "/fleet/ball_registry", reliable_qos);
    
    assignments_pub_ = this->create_publisher<ballvac_msgs::msg::FleetAssignments>(
        "/fleet/assignments", reliable_qos);

    // -------------------------------------------------------------------------
    // Create subscriber for robot status updates (from all robots)
    // -------------------------------------------------------------------------
    robot_status_sub_ = this->create_subscription<ballvac_msgs::msg::RobotStatus>(
        "/fleet/robot_status",
        rclcpp::QoS(50).reliable(),
        std::bind(&FleetCoordinatorNode::robot_status_callback, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Setup each robot
    // -------------------------------------------------------------------------
    for (const auto & robot_id : robot_names)
    {
        setup_robot(robot_id);
    }

    // -------------------------------------------------------------------------
    // Load initial ball registry entries from the world file
    // -------------------------------------------------------------------------
    load_initial_balls(initial_balls);

    // -------------------------------------------------------------------------
    // Subscribe to ball_launcher's ground truth positions
    // -------------------------------------------------------------------------
    ball_position_sub_ = this->create_subscription<ballvac_msgs::msg::BallDetectionArray>(
        "/fleet/ball_positions",
        rclcpp::SensorDataQoS(),
        std::bind(&FleetCoordinatorNode::ball_position_callback, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Create timers
    // -------------------------------------------------------------------------
    auto assignment_period = std::chrono::duration<double>(1.0 / assignment_rate_hz_);
    assignment_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(assignment_period),
        std::bind(&FleetCoordinatorNode::assignment_timer_callback, this));
    
    // Cleanup timer - check for stale claims every 2 seconds
    cleanup_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&FleetCoordinatorNode::cleanup_timer_callback, this));

    // -------------------------------------------------------------------------
    // Log startup
    // -------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Fleet Coordinator Started - MULTI ROBOT MODE");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Coordinating %zu robots", robot_names.size());
    RCLCPP_INFO(this->get_logger(), "Color priority:");
    for (size_t i = 0; i < color_priority_.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "  %zu: %s", i, color_priority_[i].c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Map bounds: [%.1f, %.1f] x [%.1f, %.1f]",
        map_bounds_min_x_, map_bounds_max_x_, map_bounds_min_y_, map_bounds_max_y_);
    RCLCPP_INFO(this->get_logger(), "Wall safety margin: %.1f m", wall_safety_margin_);
    RCLCPP_INFO(this->get_logger(), "Robot conflict radius: %.1f m", robot_conflict_radius_);
    RCLCPP_INFO(this->get_logger(), "Claim timeout: %.1f sec", claim_timeout_sec_);
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void FleetCoordinatorNode::setup_robot(const std::string & robot_id)
{
    RCLCPP_INFO(this->get_logger(), "Setting up robot: %s", robot_id.c_str());
    
    // Initialize robot info
    RobotInfo robot_info;
    robot_info.robot_id = robot_id;
    robot_info.state = ballvac_msgs::msg::RobotStatus::IDLE;
    robot_info.last_heartbeat = this->now();
    robot_info.is_operational = false;
    robot_info.navigation_ready = false;
    robot_registry_[robot_id] = robot_info;
    
    // Subscribe to robot's detection topic
    std::string detection_topic = "/" + robot_id + "/ball_detections";
    detection_subs_[robot_id] = this->create_subscription<ballvac_msgs::msg::BallDetectionArray>(
        detection_topic,
        rclcpp::SensorDataQoS(),
        [this, robot_id](const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg) {
            this->detection_callback(msg, robot_id);
        });
    
    // Create per-robot assignment publisher
    std::string assignment_topic = "/" + robot_id + "/assignment";
    auto reliable_qos = rclcpp::QoS(10).reliable().transient_local();
    robot_assignment_pubs_[robot_id] = this->create_publisher<ballvac_msgs::msg::RobotAssignment>(
        assignment_topic, reliable_qos);
    
    // Create planner client if using planner for cost
    if (use_planner_for_cost_)
    {
        std::string planner_action = "/" + robot_id + "/compute_path_to_pose";
        planner_clients_[robot_id] = rclcpp_action::create_client<ComputePathToPose>(
            this, planner_action);
    }
}

void FleetCoordinatorNode::ball_position_callback(
    const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(registry_mutex_);
    
    // Process ground-truth ball positions from ball_launcher
    for (const auto & det : msg->detections)
    {
        if (det.name.empty())
        {
            continue;
        }
        
        // Convert mm back to meters (ball_launcher encodes position in center_x/center_y)
        double ball_x = static_cast<double>(det.center_x) / 1000.0;
        double ball_y = static_cast<double>(det.center_y) / 1000.0;
        
        // Create pose
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.header.frame_id = "map";
        pose.pose.position.x = ball_x;
        pose.pose.position.y = ball_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        
        // Update or add ball
        if (ball_registry_.count(det.name) == 0)
        {
            // Try to match by color and proximity before registering a new id.
            const double max_match_distance_m = 1.5;
            std::string matched_id = match_ball_by_color_and_distance(
                det.color, pose, max_match_distance_m);

            if (!matched_id.empty())
            {
                ball_registry_[matched_id].pose = pose;
                ball_registry_[matched_id].last_seen_time = this->now();
                continue;
            }

            // New ball - add to registry
            BallInfo ball;
            ball.ball_id = det.name;
            ball.color = det.color;
            ball.pose = pose;
            ball.state = ballvac_msgs::msg::BallState::UNCLAIMED;
            ball.claimed_by_robot = "";
            ball.last_seen_time = this->now();
            ball.priority = get_color_priority(det.color);
            ball_registry_[det.name] = ball;
            
            RCLCPP_INFO(this->get_logger(), 
                "New ball detected: %s (%s) at (%.2f, %.2f) - priority %d",
                det.name.c_str(), det.color.c_str(), ball_x, ball_y, ball.priority);
        }
        else
        {
            // Update existing ball position and last seen time
            ball_registry_[det.name].pose = pose;
            ball_registry_[det.name].last_seen_time = this->now();
        }
    }
}

void FleetCoordinatorNode::detection_callback(
    const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg,
    const std::string & robot_id)
{
    std::lock_guard<std::mutex> lock(registry_mutex_);
    
    // Update robot's last seen time
    if (robot_registry_.count(robot_id) > 0)
    {
        robot_registry_[robot_id].last_heartbeat = this->now();
        robot_registry_[robot_id].is_operational = true;
    }
    
    // Process each detection - just update last seen time
    // Ball positions come from ball_launcher via ball_position_callback
    for (const auto & det : msg->detections)
    {
        if (det.name.empty())
        {
            continue;
        }
        
        // Update last seen time if ball exists
        if (ball_registry_.count(det.name) > 0)
        {
            ball_registry_[det.name].last_seen_time = this->now();
        }
    }
}

void FleetCoordinatorNode::robot_status_callback(
    const ballvac_msgs::msg::RobotStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(registry_mutex_);
    
    const std::string & robot_id = msg->robot_id;
    
    // Update robot registry
    if (robot_registry_.count(robot_id) == 0)
    {
        RCLCPP_WARN(this->get_logger(), 
            "Received status from unknown robot: %s", robot_id.c_str());
        return;
    }
    
    auto & robot = robot_registry_[robot_id];
    robot.pose = msg->pose;
    robot.state = msg->state;
    robot.assigned_ball_id = msg->assigned_ball_id;
    robot.last_heartbeat = this->now();
    robot.is_operational = msg->is_operational;
    robot.navigation_ready = msg->navigation_ready;
    
    // Handle specific actions
    switch (msg->action)
    {
        case ballvac_msgs::msg::RobotStatus::ACTION_CLAIM:
            // Robot is claiming a ball
            if (!msg->action_ball_id.empty() && 
                ball_registry_.count(msg->action_ball_id) > 0)
            {
                auto & ball = ball_registry_[msg->action_ball_id];
                if (ball.state == ballvac_msgs::msg::BallState::UNCLAIMED ||
                    ball.claimed_by_robot == robot_id)
                {
                    ball.state = ballvac_msgs::msg::BallState::CLAIMED;
                    ball.claimed_by_robot = robot_id;
                    ball.claim_time = this->now();
                    RCLCPP_INFO(this->get_logger(), 
                        "Ball %s claimed by %s", 
                        msg->action_ball_id.c_str(), robot_id.c_str());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), 
                        "Ball %s already claimed by %s, %s cannot claim",
                        msg->action_ball_id.c_str(), 
                        ball.claimed_by_robot.c_str(),
                        robot_id.c_str());
                }
            }
            break;
            
        case ballvac_msgs::msg::RobotStatus::ACTION_COLLECTED:
            // Robot collected a ball
            if (!msg->action_ball_id.empty())
            {
                mark_collected(msg->action_ball_id);
                RCLCPP_INFO(this->get_logger(), 
                    "Ball %s COLLECTED by %s", 
                    msg->action_ball_id.c_str(), robot_id.c_str());
            }
            break;
            
        case ballvac_msgs::msg::RobotStatus::ACTION_LOST:
            // Robot lost sight of the ball
            if (!msg->action_ball_id.empty())
            {
                release_claim(msg->action_ball_id);
                RCLCPP_WARN(this->get_logger(), 
                    "Ball %s LOST by %s - released for reassignment",
                    msg->action_ball_id.c_str(), robot_id.c_str());
            }
            break;
            
        case ballvac_msgs::msg::RobotStatus::ACTION_HEARTBEAT:
            // Just a heartbeat, already updated above
            break;
            
        default:
            break;
    }
}

void FleetCoordinatorNode::assignment_timer_callback()
{
    std::lock_guard<std::mutex> lock(registry_mutex_);
    
    // For each robot that's operational and doesn't have an assignment
    for (auto & [robot_id, robot] : robot_registry_)
    {
        // Skip non-operational robots
        if (!robot.is_operational)
        {
            continue;
        }
        
        // Skip robots that already have an active assignment
        if (!robot.assigned_ball_id.empty())
        {
            // Verify the ball is still claimed by this robot
            if (ball_registry_.count(robot.assigned_ball_id) > 0)
            {
                auto & ball = ball_registry_[robot.assigned_ball_id];
                if (ball.state == ballvac_msgs::msg::BallState::CLAIMED &&
                    ball.claimed_by_robot == robot_id)
                {
                    // Still valid - publish the assignment
                    continue;
                }
            }
            // Assignment is stale - clear it
            robot.assigned_ball_id = "";
        }
        
        // Find best ball for this robot
        std::string best_ball = select_best_ball_for_robot(robot_id);
        
        if (!best_ball.empty())
        {
            assign_ball(robot_id, best_ball);
        }
    }
    
    // Publish all assignments
    publish_assignments();
    publish_registry();
}

void FleetCoordinatorNode::cleanup_timer_callback()
{
    std::lock_guard<std::mutex> lock(registry_mutex_);
    
    auto now = this->now();
    
    // Check for stale claims
    for (auto & [ball_id, ball] : ball_registry_)
    {
        if (ball.state == ballvac_msgs::msg::BallState::CLAIMED)
        {
            double claim_age = (now - ball.claim_time).seconds();
            
            if (claim_age > claim_timeout_sec_)
            {
                RCLCPP_WARN(this->get_logger(), 
                    "Claim timeout: ball %s (claimed by %s for %.1fs)",
                    ball_id.c_str(), ball.claimed_by_robot.c_str(), claim_age);
                release_claim(ball_id);
            }
        }
    }
    
    // Check for robot heartbeat timeouts
    for (auto & [robot_id, robot] : robot_registry_)
    {
        double time_since_heartbeat = (now - robot.last_heartbeat).seconds();
        
        if (robot.is_operational && time_since_heartbeat > heartbeat_timeout_sec_)
        {
            RCLCPP_WARN(this->get_logger(), 
                "Robot %s heartbeat timeout (%.1fs) - marking non-operational",
                robot_id.c_str(), time_since_heartbeat);
            
            robot.is_operational = false;
            
            // Release any balls claimed by this robot
            for (auto & [ball_id, ball] : ball_registry_)
            {
                if (ball.claimed_by_robot == robot_id)
                {
                    release_claim(ball_id);
                }
            }
        }
    }
    
    // Remove old collected balls from registry (after 30 seconds)
    std::vector<std::string> to_remove;
    for (auto & [ball_id, ball] : ball_registry_)
    {
        if (ball.state == ballvac_msgs::msg::BallState::COLLECTED)
        {
            double time_since_seen = (now - ball.last_seen_time).seconds();
            if (time_since_seen > 30.0)
            {
                to_remove.push_back(ball_id);
            }
        }
    }
    for (const auto & ball_id : to_remove)
    {
        ball_registry_.erase(ball_id);
    }
}

void FleetCoordinatorNode::load_initial_balls(const std::vector<std::string> & entries)
{
    if (entries.empty())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(registry_mutex_);

    for (const auto & entry : entries)
    {
        std::stringstream ss(entry);
        std::string ball_id;
        std::string x_str;
        std::string y_str;
        if (!std::getline(ss, ball_id, ',') ||
            !std::getline(ss, x_str, ',') ||
            !std::getline(ss, y_str, ','))
        {
            RCLCPP_WARN(this->get_logger(),
                "Invalid initial_balls entry: '%s' (expected ball_id,x,y)",
                entry.c_str());
            continue;
        }

        try
        {
            double ball_x = std::stod(x_str);
            double ball_y = std::stod(y_str);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = ball_x;
            pose.pose.position.y = ball_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;

            BallInfo ball;
            ball.ball_id = ball_id;
            ball.color = get_color_from_ball_id(ball_id);
            ball.pose = pose;
            ball.state = ballvac_msgs::msg::BallState::UNCLAIMED;
            ball.claimed_by_robot = "";
            ball.last_seen_time = this->now();
            ball.priority = get_color_priority(ball.color);
            ball_registry_[ball_id] = ball;

            RCLCPP_INFO(this->get_logger(),
                "Loaded initial ball: %s (%s) at (%.2f, %.2f)",
                ball_id.c_str(), ball.color.c_str(), ball_x, ball_y);
        }
        catch (const std::exception &)
        {
            RCLCPP_WARN(this->get_logger(),
                "Invalid initial_balls entry: '%s' (x/y parse failed)",
                entry.c_str());
        }
    }
}

std::string FleetCoordinatorNode::get_color_from_ball_id(const std::string & ball_id) const
{
    const std::string prefix = "ball_";
    std::string color = ball_id;
    if (color.rfind(prefix, 0) == 0)
    {
        color = color.substr(prefix.size());
    }
    auto underscore = color.find('_');
    if (underscore != std::string::npos)
    {
        color = color.substr(0, underscore);
    }
    return color;
}

std::string FleetCoordinatorNode::match_ball_by_color_and_distance(
    const std::string & color,
    const geometry_msgs::msg::PoseStamped & pose,
    double max_distance_m) const
{
    std::string best_id;
    double best_dist = std::numeric_limits<double>::max();

    for (const auto & [ball_id, ball] : ball_registry_)
    {
        if (ball.color != color || ball.state == ballvac_msgs::msg::BallState::COLLECTED)
        {
            continue;
        }

        double dx = ball.pose.pose.position.x - pose.pose.position.x;
        double dy = ball.pose.pose.position.y - pose.pose.position.y;
        double dist = std::hypot(dx, dy);
        if (dist < best_dist)
        {
            best_dist = dist;
            best_id = ball_id;
        }
    }

    if (!best_id.empty() && best_dist <= max_distance_m)
    {
        return best_id;
    }
    return "";
}

uint8_t FleetCoordinatorNode::get_color_priority(const std::string & color)
{
    for (size_t i = 0; i < color_priority_.size(); i++)
    {
        if (color_priority_[i] == color)
        {
            return static_cast<uint8_t>(i);
        }
    }
    // Unknown color - lowest priority
    return static_cast<uint8_t>(color_priority_.size());
}

std::string FleetCoordinatorNode::select_best_ball_for_robot(const std::string & robot_id)
{
    if (robot_registry_.count(robot_id) == 0)
    {
        return "";
    }
    
    const auto & robot = robot_registry_[robot_id];
    
    // Find all unclaimed balls
    std::vector<std::pair<std::string, double>> candidates;  // (ball_id, cost)
    
    uint8_t best_priority = 255;
    
    // First pass: find the highest priority (lowest number) among unclaimed balls
    for (const auto & [ball_id, ball] : ball_registry_)
    {
        if (ball.state != ballvac_msgs::msg::BallState::UNCLAIMED)
        {
            continue;
        }
        
        if (!is_within_bounds(ball.pose))
        {
            continue;
        }
        
        if (ball.priority < best_priority)
        {
            best_priority = ball.priority;
        }
    }
    
    if (best_priority == 255)
    {
        return "";  // No unclaimed balls
    }
    
    // Second pass: collect all balls with the best priority and compute costs
    for (const auto & [ball_id, ball] : ball_registry_)
    {
        if (ball.state != ballvac_msgs::msg::BallState::UNCLAIMED)
        {
            continue;
        }
        
        if (ball.priority != best_priority)
        {
            continue;  // Only consider highest priority color
        }
        
        if (!is_within_bounds(ball.pose))
        {
            continue;
        }
        
        // Check if another robot is already targeting this ball or nearby
        bool conflicting = false;
        for (const auto & [other_id, other_robot] : robot_registry_)
        {
            if (other_id == robot_id)
            {
                continue;
            }
            
            // Check if other robot is navigating to a nearby target
            if (!other_robot.assigned_ball_id.empty() && other_robot.is_operational)
            {
                if (ball_registry_.count(other_robot.assigned_ball_id) > 0)
                {
                    const auto & other_ball = ball_registry_.at(other_robot.assigned_ball_id);
                    double dx = ball.pose.pose.position.x - other_ball.pose.pose.position.x;
                    double dy = ball.pose.pose.position.y - other_ball.pose.pose.position.y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    // If balls are very close, skip to avoid robot collision
                    if (dist < robot_conflict_radius_)
                    {
                        conflicting = true;
                        RCLCPP_DEBUG(this->get_logger(), 
                            "Ball %s too close to %s's target - skipping for %s",
                            ball_id.c_str(), other_id.c_str(), robot_id.c_str());
                        break;
                    }
                }
            }
            
            // Check if paths would cross (robot positions)
            double other_dx = ball.pose.pose.position.x - other_robot.pose.pose.position.x;
            double other_dy = ball.pose.pose.position.y - other_robot.pose.pose.position.y;
            double other_dist = std::sqrt(other_dx*other_dx + other_dy*other_dy);
            
            // If other robot is closer to this ball, let them have it
            double our_dx = ball.pose.pose.position.x - robot.pose.pose.position.x;
            double our_dy = ball.pose.pose.position.y - robot.pose.pose.position.y;
            double our_dist = std::sqrt(our_dx*our_dx + our_dy*our_dy);
            
            if (other_dist < our_dist * 0.7 && other_robot.is_operational && other_robot.assigned_ball_id.empty())
            {
                // Other robot is significantly closer - skip
                conflicting = true;
                RCLCPP_DEBUG(this->get_logger(), 
                    "Ball %s - %s is closer (%.2f vs %.2f) - skipping for %s",
                    ball_id.c_str(), other_id.c_str(), other_dist, our_dist, robot_id.c_str());
                break;
            }
        }
        
        if (conflicting)
        {
            continue;
        }
        
        // Compute base cost (distance)
        double cost = compute_path_cost(robot_id, ball.pose);
        
        // Add penalty for balls near walls (harder to reach safely)
        if (is_near_wall(ball.pose, wall_safety_margin_))
        {
            cost += 2.0;  // Penalty for wall-adjacent balls
            RCLCPP_DEBUG(this->get_logger(), 
                "Ball %s near wall - adding penalty for %s", ball_id.c_str(), robot_id.c_str());
        }
        
        candidates.push_back({ball_id, cost});
    }
    
    if (candidates.empty())
    {
        return "";
    }
    
    // Sort by cost (ascending)
    std::sort(candidates.begin(), candidates.end(),
        [](const auto & a, const auto & b) { return a.second < b.second; });
    
    // Return the best (lowest cost) candidate
    return candidates.front().first;
}

double FleetCoordinatorNode::compute_path_cost(
    const std::string & robot_id,
    const geometry_msgs::msg::PoseStamped & target)
{
    if (robot_registry_.count(robot_id) == 0)
    {
        return std::numeric_limits<double>::max();
    }
    
    const auto & robot = robot_registry_[robot_id];
    
    // Use Euclidean distance as the cost
    // TODO: Optionally use Nav2 planner for actual path length
    double dx = target.pose.position.x - robot.pose.pose.position.x;
    double dy = target.pose.position.y - robot.pose.pose.position.y;
    
    return std::sqrt(dx * dx + dy * dy);
}

void FleetCoordinatorNode::assign_ball(const std::string & robot_id, const std::string & ball_id)
{
    if (ball_registry_.count(ball_id) == 0 || robot_registry_.count(robot_id) == 0)
    {
        return;
    }
    
    auto & ball = ball_registry_[ball_id];
    auto & robot = robot_registry_[robot_id];
    
    // Claim the ball
    ball.state = ballvac_msgs::msg::BallState::CLAIMED;
    ball.claimed_by_robot = robot_id;
    ball.claim_time = this->now();
    
    // Update robot's assignment
    robot.assigned_ball_id = ball_id;
    
    RCLCPP_INFO(this->get_logger(), 
        "Assigned ball %s (%s, priority=%d) to robot %s",
        ball_id.c_str(), ball.color.c_str(), ball.priority, robot_id.c_str());
    
    // Publish individual assignment
    if (robot_assignment_pubs_.count(robot_id) > 0)
    {
        auto assignment_msg = ballvac_msgs::msg::RobotAssignment();
        assignment_msg.header.stamp = this->now();
        assignment_msg.header.frame_id = "map";
        assignment_msg.robot_id = robot_id;
        assignment_msg.ball_id = ball_id;
        assignment_msg.ball_color = ball.color;
        assignment_msg.goal_pose = ball.pose;
        assignment_msg.standoff_distance = standoff_distance_;
        assignment_msg.has_assignment = true;
        assignment_msg.estimated_cost = compute_path_cost(robot_id, ball.pose);
        
        robot_assignment_pubs_[robot_id]->publish(assignment_msg);
    }
}

void FleetCoordinatorNode::release_claim(const std::string & ball_id)
{
    if (ball_registry_.count(ball_id) == 0)
    {
        return;
    }
    
    auto & ball = ball_registry_[ball_id];
    
    // Clear claim from robot
    if (!ball.claimed_by_robot.empty() && 
        robot_registry_.count(ball.claimed_by_robot) > 0)
    {
        auto & robot = robot_registry_[ball.claimed_by_robot];
        if (robot.assigned_ball_id == ball_id)
        {
            robot.assigned_ball_id = "";
        }
    }
    
    // Release the ball
    ball.state = ballvac_msgs::msg::BallState::UNCLAIMED;
    ball.claimed_by_robot = "";
}

void FleetCoordinatorNode::mark_collected(const std::string & ball_id)
{
    if (ball_registry_.count(ball_id) == 0)
    {
        return;
    }
    
    auto & ball = ball_registry_[ball_id];
    
    // Clear assignment from robot
    if (!ball.claimed_by_robot.empty() && 
        robot_registry_.count(ball.claimed_by_robot) > 0)
    {
        auto & robot = robot_registry_[ball.claimed_by_robot];
        robot.assigned_ball_id = "";
    }
    
    ball.state = ballvac_msgs::msg::BallState::COLLECTED;
    ball.last_seen_time = this->now();
}

bool FleetCoordinatorNode::is_within_bounds(const geometry_msgs::msg::PoseStamped & pose)
{
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    
    return (x >= map_bounds_min_x_ && x <= map_bounds_max_x_ &&
            y >= map_bounds_min_y_ && y <= map_bounds_max_y_);
}

bool FleetCoordinatorNode::is_near_wall(const geometry_msgs::msg::PoseStamped & pose, double threshold)
{
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    
    // Check distance to each wall
    double dist_to_min_x = x - map_bounds_min_x_;
    double dist_to_max_x = map_bounds_max_x_ - x;
    double dist_to_min_y = y - map_bounds_min_y_;
    double dist_to_max_y = map_bounds_max_y_ - y;
    
    return (dist_to_min_x < threshold || dist_to_max_x < threshold ||
            dist_to_min_y < threshold || dist_to_max_y < threshold);
}

bool FleetCoordinatorNode::paths_conflict(
    const std::string & robot1, const std::string & robot2,
    const geometry_msgs::msg::PoseStamped & target)
{
    if (robot_registry_.count(robot1) == 0 || robot_registry_.count(robot2) == 0)
    {
        return false;
    }
    
    const auto & r1 = robot_registry_[robot1];
    const auto & r2 = robot_registry_[robot2];
    
    // Get positions
    double r1_x = r1.pose.pose.position.x;
    double r1_y = r1.pose.pose.position.y;
    double r2_x = r2.pose.pose.position.x;
    double r2_y = r2.pose.pose.position.y;
    double t_x = target.pose.position.x;
    double t_y = target.pose.position.y;
    
    // Vector from r1 to target
    double v1_x = t_x - r1_x;
    double v1_y = t_y - r1_y;
    double len1 = std::sqrt(v1_x*v1_x + v1_y*v1_y);
    
    if (len1 < 0.1) return false;
    
    // Normalize
    v1_x /= len1;
    v1_y /= len1;
    
    // Check if r2 is along the path from r1 to target
    double r2_rel_x = r2_x - r1_x;
    double r2_rel_y = r2_y - r1_y;
    
    // Project r2 onto the line r1->target
    double proj = r2_rel_x * v1_x + r2_rel_y * v1_y;
    
    if (proj < 0 || proj > len1) return false;  // r2 not between r1 and target
    
    // Distance from r2 to the line
    double closest_x = r1_x + proj * v1_x;
    double closest_y = r1_y + proj * v1_y;
    double dist = std::sqrt((r2_x - closest_x)*(r2_x - closest_x) + 
                           (r2_y - closest_y)*(r2_y - closest_y));
    
    return dist < robot_conflict_radius_;
}

double FleetCoordinatorNode::get_safe_approach_angle(
    const geometry_msgs::msg::PoseStamped & ball_pose,
    const geometry_msgs::msg::PoseStamped & robot_pose)
{
    double bx = ball_pose.pose.position.x;
    double by = ball_pose.pose.position.y;
    double rx = robot_pose.pose.position.x;
    double ry = robot_pose.pose.position.y;
    
    // Direct angle from robot to ball
    double direct_angle = std::atan2(by - ry, bx - rx);
    
    // Check which walls are nearby the ball
    double dist_to_min_x = bx - map_bounds_min_x_;
    double dist_to_max_x = map_bounds_max_x_ - bx;
    double dist_to_min_y = by - map_bounds_min_y_;
    double dist_to_max_y = map_bounds_max_y_ - by;
    
    // If ball is near a wall, approach from the opposite direction
    double offset = 0.0;
    if (dist_to_min_x < wall_safety_margin_)
    {
        // Ball near -X wall, approach from +X
        offset = (direct_angle > 0) ? -M_PI/4 : M_PI/4;  // Steer away from wall
    }
    else if (dist_to_max_x < wall_safety_margin_)
    {
        // Ball near +X wall, approach from -X
        offset = (direct_angle > 0) ? M_PI/4 : -M_PI/4;
    }
    else if (dist_to_min_y < wall_safety_margin_)
    {
        // Ball near -Y wall, approach from +Y
        offset = (std::abs(direct_angle) < M_PI/2) ? M_PI/4 : -M_PI/4;
    }
    else if (dist_to_max_y < wall_safety_margin_)
    {
        // Ball near +Y wall, approach from -Y
        offset = (std::abs(direct_angle) < M_PI/2) ? -M_PI/4 : M_PI/4;
    }
    
    return direct_angle + offset;
}

void FleetCoordinatorNode::publish_assignments()
{
    auto msg = ballvac_msgs::msg::FleetAssignments();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.color_priority = color_priority_;
    
    for (const auto & [robot_id, robot] : robot_registry_)
    {
        auto assignment = ballvac_msgs::msg::RobotAssignment();
        assignment.header.stamp = this->now();
        assignment.header.frame_id = "map";
        assignment.robot_id = robot_id;
        
        if (!robot.assigned_ball_id.empty() && 
            ball_registry_.count(robot.assigned_ball_id) > 0)
        {
            const auto & ball = ball_registry_[robot.assigned_ball_id];
            assignment.ball_id = ball.ball_id;
            assignment.ball_color = ball.color;
            assignment.goal_pose = ball.pose;
            assignment.standoff_distance = standoff_distance_;
            assignment.has_assignment = true;
            assignment.estimated_cost = compute_path_cost(robot_id, ball.pose);
        }
        else
        {
            assignment.has_assignment = false;
        }
        
        msg.assignments.push_back(assignment);
    }
    
    assignments_pub_->publish(msg);
}

void FleetCoordinatorNode::publish_registry()
{
    auto msg = ballvac_msgs::msg::BallRegistry();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    
    uint32_t unclaimed = 0, claimed = 0, collected = 0;
    
    for (const auto & [ball_id, ball] : ball_registry_)
    {
        auto ball_state = ballvac_msgs::msg::BallState();
        ball_state.header.stamp = this->now();
        ball_state.header.frame_id = "map";
        ball_state.ball_id = ball.ball_id;
        ball_state.color = ball.color;
        ball_state.pose = ball.pose;
        ball_state.state = ball.state;
        ball_state.claimed_by_robot = ball.claimed_by_robot;
        ball_state.claim_time = ball.claim_time;
        ball_state.last_seen_time = ball.last_seen_time;
        ball_state.priority = ball.priority;
        
        msg.balls.push_back(ball_state);
        
        switch (ball.state)
        {
            case ballvac_msgs::msg::BallState::UNCLAIMED:
                unclaimed++;
                break;
            case ballvac_msgs::msg::BallState::CLAIMED:
                claimed++;
                break;
            case ballvac_msgs::msg::BallState::COLLECTED:
                collected++;
                break;
            default:
                break;
        }
    }
    
    msg.total_balls = static_cast<uint32_t>(ball_registry_.size());
    msg.unclaimed_balls = unclaimed;
    msg.claimed_balls = claimed;
    msg.collected_balls = collected;
    
    registry_pub_->publish(msg);
}

}  // namespace ballvac_ball_collector

// Main function
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ballvac_ball_collector::FleetCoordinatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
