/**
 * @file ball_launcher_node.cpp
 * @brief Implementation of ball launcher node
 * 
 * Launches balls into the arena with random velocities, simulating
 * a person throwing balls for the robot to collect.
 */

#include "ballvac_ball_collector/ball_launcher_node.hpp"
#include <cmath>
#include <algorithm>

namespace ballvac_ball_collector
{

// =============================================================================
// Constructor
// =============================================================================

BallLauncherNode::BallLauncherNode(const rclcpp::NodeOptions & options)
: Node("ball_launcher_node", options),
  balls_launched_(0),
  ball_counter_(0)
{
    // -------------------------------------------------------------------------
    // Declare parameters
    // -------------------------------------------------------------------------
    
    // Spawn service
    this->declare_parameter<std::string>("spawn_service", "/world/ball_arena/create");
    spawn_service_ = this->get_parameter("spawn_service").as_string();
    
    // Launcher position (west wall, elevated)
    this->declare_parameter<double>("launcher_x", -4.3);
    this->declare_parameter<double>("launcher_y", 0.0);
    this->declare_parameter<double>("launcher_z", 1.2);
    launcher_x_ = this->get_parameter("launcher_x").as_double();
    launcher_y_ = this->get_parameter("launcher_y").as_double();
    launcher_z_ = this->get_parameter("launcher_z").as_double();
    
    // Launch speed range (m/s)
    this->declare_parameter<double>("min_launch_speed", 3.0);
    this->declare_parameter<double>("max_launch_speed", 6.0);
    min_launch_speed_ = this->get_parameter("min_launch_speed").as_double();
    max_launch_speed_ = this->get_parameter("max_launch_speed").as_double();
    
    // Horizontal angle range (radians, 0 = straight into room)
    this->declare_parameter<double>("min_launch_angle", -0.6);  // ~-35 degrees
    this->declare_parameter<double>("max_launch_angle", 0.6);   // ~+35 degrees
    min_launch_angle_ = this->get_parameter("min_launch_angle").as_double();
    max_launch_angle_ = this->get_parameter("max_launch_angle").as_double();
    
    // Elevation angle range (radians, 0 = horizontal)
    this->declare_parameter<double>("min_elevation", 0.1);   // ~6 degrees up
    this->declare_parameter<double>("max_elevation", 0.5);   // ~29 degrees up
    min_elevation_ = this->get_parameter("min_elevation").as_double();
    max_elevation_ = this->get_parameter("max_elevation").as_double();
    
    // Timing
    this->declare_parameter<double>("launch_interval", 8.0);  // seconds
    this->declare_parameter<int>("max_balls", 20);
    this->declare_parameter<bool>("auto_launch", true);
    this->declare_parameter<bool>("respawn_on_delete", true);
    launch_interval_ = this->get_parameter("launch_interval").as_double();
    max_balls_ = this->get_parameter("max_balls").as_int();
    auto_launch_ = this->get_parameter("auto_launch").as_bool();
    respawn_on_delete_ = this->get_parameter("respawn_on_delete").as_bool();
    
    // -------------------------------------------------------------------------
    // Initialize colors
    // -------------------------------------------------------------------------
    colors_ = {"red", "green", "blue", "yellow", "cyan", "purple"};
    
    // -------------------------------------------------------------------------
    // Initialize random number generator
    // -------------------------------------------------------------------------
    std::random_device rd;
    rng_ = std::mt19937(rd());
    
    // -------------------------------------------------------------------------
    // Create ROS 2 interfaces
    // -------------------------------------------------------------------------
    
    // Spawn service client
    spawn_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(
        spawn_service_);
    
    // Manual launch service
    manual_launch_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/launch_ball",
        std::bind(&BallLauncherNode::manual_launch_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Launch info publisher
    launch_info_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/launch_info", 10);
    
    // Ball detection publisher for fleet coordinator
    // This publishes the ground-truth positions of spawned balls
    ball_detection_pub_ = this->create_publisher<ballvac_msgs::msg::BallDetectionArray>(
        "/fleet/ball_positions", rclcpp::SensorDataQoS());
    
    // Subscribe to ball deletion events to respawn balls
    ball_deleted_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/fleet/ball_deleted", 10,
        std::bind(&BallLauncherNode::ball_deleted_callback, this, std::placeholders::_1));
    
    // Auto launch timer
    if (auto_launch_)
    {
        launch_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(launch_interval_),
            std::bind(&BallLauncherNode::launch_timer_callback, this));
    }
    
    // Periodic ball position publisher (for fleet coordinator)
    auto ball_position_timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            if (spawned_balls_.empty()) return;
            
            ballvac_msgs::msg::BallDetectionArray msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            
            for (const auto & ball : spawned_balls_) {
                ballvac_msgs::msg::BallDetection det;
                det.header = msg.header;
                det.name = ball.name;
                det.color = ball.color;
                det.confidence = 1.0;  // Ground truth
                // Use center_x/center_y as a hack to send world position
                // (fleet coordinator will need to read these)
                det.center_x = static_cast<int>(ball.x * 1000);  // mm precision
                det.center_y = static_cast<int>(ball.y * 1000);
                msg.detections.push_back(det);
            }
            
            ball_detection_pub_->publish(msg);
        });
    
    // -------------------------------------------------------------------------
    // Log startup
    // -------------------------------------------------------------------------
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Ball Launcher Node Started");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "  Launcher position: (%.1f, %.1f, %.1f)",
        launcher_x_, launcher_y_, launcher_z_);
    RCLCPP_INFO(this->get_logger(), "  Launch speed: %.1f - %.1f m/s",
        min_launch_speed_, max_launch_speed_);
    RCLCPP_INFO(this->get_logger(), "  Launch interval: %.1f s", launch_interval_);
    RCLCPP_INFO(this->get_logger(), "  Max balls: %d", max_balls_);
    RCLCPP_INFO(this->get_logger(), "  Auto launch: %s", auto_launch_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "  Respawn on delete: %s", respawn_on_delete_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // Launch first ball after a short delay
    if (auto_launch_)
    {
        auto initial_timer = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                this->launch_ball();
                // This is a one-shot timer, cancel after first execution
            });
    }
}

// =============================================================================
// Timer callback for automatic launching
// =============================================================================

void BallLauncherNode::launch_timer_callback()
{
    if (balls_launched_ < max_balls_)
    {
        launch_ball();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), 
            "Maximum balls (%d) reached, stopping auto-launch", max_balls_);
        launch_timer_->cancel();
    }
}

// =============================================================================
// Manual launch service callback
// =============================================================================

void BallLauncherNode::manual_launch_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    launch_ball();
    response->success = true;
    response->message = "Ball launched! Total: " + std::to_string(balls_launched_);
}

// =============================================================================
// Launch a ball with random parameters
// =============================================================================

void BallLauncherNode::launch_ball()
{
    if (!spawn_client_->service_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), 
            "Spawn service '%s' not ready, cannot launch ball", spawn_service_.c_str());
        return;
    }
    
    // Generate random spawn position within arena (avoiding walls and center obstacles)
    std::uniform_real_distribution<double> x_dist(-3.5, 3.5);
    std::uniform_real_distribution<double> y_dist(-3.5, 3.5);
    
    double spawn_x, spawn_y;
    bool valid_position = false;
    int attempts = 0;
    
    // Find a valid spawn position (avoid center and obstacles)
    while (!valid_position && attempts < 20)
    {
        spawn_x = x_dist(rng_);
        spawn_y = y_dist(rng_);
        
        // Avoid center area where robot might be
        double dist_from_center = std::sqrt(spawn_x * spawn_x + spawn_y * spawn_y);
        
        // Avoid robot spawn area (0, -3)
        double dist_from_robot_spawn = std::sqrt(spawn_x * spawn_x + (spawn_y + 3.0) * (spawn_y + 3.0));
        
        if (dist_from_center > 1.5 && dist_from_robot_spawn > 1.0)
        {
            valid_position = true;
        }
        attempts++;
    }
    
    // Get random color
    std::string color = get_random_color();
    
    // Create spawn request
    auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    
    ball_counter_++;
    // Use simple naming: ball_color (only one of each color at a time)
    std::string entity_name = "ball_" + color;
    
    request->entity_factory.name = entity_name;
    request->entity_factory.allow_renaming = false;  // Don't allow renaming - will fail if exists
    request->entity_factory.sdf = generate_ball_sdf(color, 0.0, 0.0, 0.0);  // No velocity
    request->entity_factory.pose.position.x = spawn_x;
    request->entity_factory.pose.position.y = spawn_y;
    request->entity_factory.pose.position.z = 0.15;  // Slightly above ground
    
    RCLCPP_INFO(this->get_logger(), 
        "Spawning %s ball #%d at position (%.1f, %.1f)",
        color.c_str(), ball_counter_, spawn_x, spawn_y);
    
    // Send spawn request
    auto future = spawn_client_->async_send_request(
        request,
        std::bind(&BallLauncherNode::spawn_callback, this, std::placeholders::_1));
    
    balls_launched_++;
    
    // Track spawned ball for fleet coordinator
    // Remove old ball of same color if exists
    spawned_balls_.erase(
        std::remove_if(spawned_balls_.begin(), spawned_balls_.end(),
            [&entity_name](const SpawnedBall& b) { return b.name == entity_name; }),
        spawned_balls_.end());
    
    // Add new ball
    SpawnedBall ball_info;
    ball_info.name = entity_name;
    ball_info.color = color;
    ball_info.x = spawn_x;
    ball_info.y = spawn_y;
    spawned_balls_.push_back(ball_info);
    
    // Publish launch info
    auto msg = std_msgs::msg::String();
    msg.data = "Spawned " + color + " ball #" + std::to_string(ball_counter_) + 
               " at (" + std::to_string(spawn_x).substr(0,4) + ", " + 
               std::to_string(spawn_y).substr(0,4) + ")";
    launch_info_pub_->publish(msg);
}

// =============================================================================
// Spawn callback
// =============================================================================

void BallLauncherNode::spawn_callback(
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future)
{
    try
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_DEBUG(this->get_logger(), "Ball spawned successfully");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to spawn ball");
        }
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), "Spawn exception: %s", e.what());
    }
}

// =============================================================================
// Generate ball SDF with initial velocity
// =============================================================================

std::string BallLauncherNode::generate_ball_sdf(const std::string & color,
                                                  double /*vx*/, double /*vy*/, double /*vz*/)
{
    std::string ambient, diffuse, emissive;
    get_color_rgb(color, ambient, diffuse, emissive);
    
    // SDF for a colored ball (no velocity - Gazebo Harmonic doesn't support it in SDF)
    std::string sdf = R"(<?xml version="1.0"?>
<sdf version="1.8">
  <model name="ball">
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
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.5</restitution_coefficient>
          </bounce>
        </surface>
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
// Get random color
// =============================================================================

std::string BallLauncherNode::get_random_color()
{
    std::uniform_int_distribution<size_t> dist(0, colors_.size() - 1);
    return colors_[dist(rng_)];
}

// =============================================================================
// Get RGB values for color
// =============================================================================

void BallLauncherNode::get_color_rgb(const std::string & color,
                                      std::string & ambient,
                                      std::string & diffuse,
                                      std::string & emissive)
{
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
}

// =============================================================================
// Ball deleted callback - respawn a new ball when one is collected
// =============================================================================

void BallLauncherNode::ball_deleted_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!respawn_on_delete_)
    {
        return;
    }

    std::string deleted_ball = msg->data;
    
    RCLCPP_INFO(this->get_logger(), "Ball deleted notification: %s - spawning replacement", 
        deleted_ball.c_str());
    
    // Remove from tracking
    spawned_balls_.erase(
        std::remove_if(spawned_balls_.begin(), spawned_balls_.end(),
            [&deleted_ball](const SpawnedBall& b) { return b.name == deleted_ball; }),
        spawned_balls_.end());
    
    // Wait a moment, then spawn a new ball
    auto respawn_timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            this->launch_ball();
        });
    // Cancel after one shot - create_wall_timer doesn't have one_shot option,
    // so we'll just let it fire once and it will be cleaned up
}

}  // namespace ballvac_ball_collector

// =============================================================================
// Main entry point
// =============================================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ballvac_ball_collector::BallLauncherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
