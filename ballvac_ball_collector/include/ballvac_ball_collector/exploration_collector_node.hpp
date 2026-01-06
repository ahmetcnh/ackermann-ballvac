/**
 * @file exploration_collector_node.hpp
 * @brief Node that explores the room to build a map, then collects balls
 * 
 * This node implements a two-phase behavior:
 * 1. EXPLORATION: Navigate the room to build a complete map using SLAM
 * 2. COLLECTION: Use the map to efficiently collect detected balls
 */

#ifndef BALLVAC_BALL_COLLECTOR__EXPLORATION_COLLECTOR_NODE_HPP_
#define BALLVAC_BALL_COLLECTOR__EXPLORATION_COLLECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>

#include "ballvac_msgs/msg/ball_detection_array.hpp"

#include <vector>
#include <queue>
#include <set>
#include <string>

namespace ballvac_ball_collector
{

/**
 * @brief State machine states for exploration and collection
 */
enum class ExplorationState
{
    WAITING,      // Waiting for SLAM to initialize
    EXPLORING,    // Exploring the room to build map
    RETURNING,    // Returning to a frontier
    MAP_COMPLETE, // Map is complete, transitioning to collection
    COLLECTING,   // Collecting balls
    APPROACHING,  // Approaching a detected ball
    RECOVERING    // Recovery behavior when stuck
};

/**
 * @brief Frontier cell for exploration
 */
struct Frontier
{
    int x;
    int y;
    double world_x;
    double world_y;
    int size;  // Number of frontier cells in this cluster
};

/**
 * @brief Target ball info
 */
struct BallTarget
{
    std::string name;
    std::string color;
    float bearing;
    float size;
    bool valid;
};

class ExplorationCollectorNode : public rclcpp::Node
{
public:
    explicit ExplorationCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // =========================================================================
    // Callbacks
    // =========================================================================
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void detection_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg);
    void control_loop();
    
    // =========================================================================
    // State machine functions
    // =========================================================================
    
    void execute_waiting();
    void execute_exploring();
    void execute_returning();
    void execute_map_complete();
    void execute_collecting();
    void execute_approaching();
    void execute_recovering();
    
    void transition_to(ExplorationState new_state);
    std::string state_to_string(ExplorationState state);
    
    // =========================================================================
    // Exploration functions
    // =========================================================================
    
    /**
     * @brief Find frontier cells (boundary between known and unknown)
     */
    std::vector<Frontier> find_frontiers();
    
    /**
     * @brief Select the best frontier to explore
     */
    bool select_frontier(Frontier & best);
    
    /**
     * @brief Check if exploration is complete (no more frontiers)
     */
    bool is_exploration_complete();
    
    /**
     * @brief Calculate exploration progress (0-100%)
     */
    double calculate_exploration_progress();
    
    // =========================================================================
    // Navigation functions
    // =========================================================================
    
    /**
     * @brief Navigate towards a goal position
     */
    void navigate_to_goal(double goal_x, double goal_y);
    
    /**
     * @brief Wall following behavior for exploration
     */
    void wall_follow();
    
    /**
     * @brief Check for obstacles and compute avoidance steering
     */
    bool check_obstacles(float & min_range);
    float compute_avoidance_steering();
    
    /**
     * @brief Publish velocity command
     */
    void publish_cmd_vel(float linear, float angular);
    
    // =========================================================================
    // Collection functions
    // =========================================================================
    
    bool select_ball_target();
    void approach_ball();
    void collect_ball();
    void delete_entity(const std::string & name);
    void delete_callback(
        rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future);
    std::string get_entity_name(const std::string & color);
    
    // =========================================================================
    // ROS 2 interfaces
    // =========================================================================
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ballvac_msgs::msg::BallDetectionArray>::SharedPtr detection_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // =========================================================================
    // Parameters
    // =========================================================================
    
    std::string scan_topic_;
    std::string map_topic_;
    std::string odom_topic_;
    std::string cmd_topic_;
    std::string detection_topic_;
    std::string delete_service_;
    
    double exploration_speed_;
    double collection_speed_;
    double max_steer_;
    double obstacle_stop_distance_;
    double obstacle_slow_distance_;
    
    double frontier_min_size_;      // Minimum frontier cluster size
    double exploration_threshold_;   // % of map explored to consider complete
    
    // =========================================================================
    // State
    // =========================================================================
    
    ExplorationState current_state_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    
    // Robot pose
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool have_pose_;
    
    // Exploration state
    Frontier current_frontier_;
    bool have_frontier_;
    double exploration_progress_;
    rclcpp::Time exploration_start_time_;
    
    // Collection state
    BallTarget current_target_;
    bool delete_pending_;
    int balls_collected_;
    
    // Recovery
    rclcpp::Time recovery_start_time_;
    int recovery_phase_;
    float recovery_turn_dir_;
    
    // Stuck detection
    rclcpp::Time last_progress_time_;
    double last_x_;
    double last_y_;
};

}  // namespace ballvac_ball_collector

#endif  // BALLVAC_BALL_COLLECTOR__EXPLORATION_COLLECTOR_NODE_HPP_
