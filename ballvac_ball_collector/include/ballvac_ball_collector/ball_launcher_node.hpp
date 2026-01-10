/**
 * @file ball_launcher_node.hpp
 * @brief Node that launches balls into the arena simulating a person throwing balls
 * 
 * This node spawns balls with initial velocity from a launcher position,
 * simulating random throws into the arena for the robot to collect.
 */

#ifndef BALLVAC_BALL_COLLECTOR__BALL_LAUNCHER_NODE_HPP_
#define BALLVAC_BALL_COLLECTOR__BALL_LAUNCHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "ballvac_msgs/msg/ball_detection.hpp"
#include "ballvac_msgs/msg/ball_detection_array.hpp"

#include <random>
#include <string>
#include <vector>

namespace ballvac_ball_collector
{

class BallLauncherNode : public rclcpp::Node
{
public:
    explicit BallLauncherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // =========================================================================
    // Callbacks
    // =========================================================================
    
    /**
     * @brief Timer callback to periodically launch balls
     */
    void launch_timer_callback();
    
    /**
     * @brief Service callback for manual ball launch
     */
    void manual_launch_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief Callback for spawn entity response
     */
    void spawn_callback(
        rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future);
    
    /**
     * @brief Callback when a ball is deleted/collected - respawn a new one
     */
    void ball_deleted_callback(const std_msgs::msg::String::SharedPtr msg);
    
    // =========================================================================
    // Helper functions
    // =========================================================================
    
    /**
     * @brief Launch a ball with random parameters
     */
    void launch_ball();
    
    /**
     * @brief Spawn initial balls at random positions across the arena
     */
    void spawn_initial_balls();
    
    /**
     * @brief Generate SDF for a ball with initial velocity
     * @param color Ball color name
     * @param vx Initial X velocity
     * @param vy Initial Y velocity
     * @param vz Initial Z velocity
     * @return SDF XML string
     */
    std::string generate_ball_sdf(const std::string & color, 
                                   double vx, double vy, double vz);
    
    /**
     * @brief Get random color from available colors
     */
    std::string get_random_color();
    
    /**
     * @brief Get RGB values for a color
     */
    void get_color_rgb(const std::string & color, 
                       std::string & ambient, 
                       std::string & diffuse,
                       std::string & emissive);

    // =========================================================================
    // ROS 2 interfaces
    // =========================================================================
    
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_launch_srv_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr launch_info_pub_;
    rclcpp::Publisher<ballvac_msgs::msg::BallDetectionArray>::SharedPtr ball_detection_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ball_deleted_sub_;
    rclcpp::TimerBase::SharedPtr launch_timer_;
    rclcpp::TimerBase::SharedPtr initial_spawn_timer_;  // Timer for initial ball spawning
    
    // =========================================================================
    // Parameters
    // =========================================================================
    
    std::string spawn_service_;
    
    // Launcher position (where balls are thrown from)
    double launcher_x_;
    double launcher_y_;
    double launcher_z_;
    
    // Launch parameters
    double min_launch_speed_;
    double max_launch_speed_;
    double min_launch_angle_;   // Horizontal angle (radians)
    double max_launch_angle_;
    double min_elevation_;      // Vertical angle (radians)
    double max_elevation_;
    
    // Timing
    double launch_interval_;    // Seconds between automatic launches
    int max_balls_;            // Maximum balls to launch
    bool auto_launch_;         // Whether to launch automatically
    bool respawn_on_delete_;   // Whether to respawn on delete events
    
    // =========================================================================
    // State
    // =========================================================================
    
    std::mt19937 rng_;
    int balls_launched_;
    int ball_counter_;  // For unique naming
    std::vector<std::string> colors_;
    
    // Track spawned balls for publishing detections
    struct SpawnedBall {
        std::string name;
        std::string color;
        double x;
        double y;
    };
    std::vector<SpawnedBall> spawned_balls_;
};

}  // namespace ballvac_ball_collector

#endif  // BALLVAC_BALL_COLLECTOR__BALL_LAUNCHER_NODE_HPP_
