/**
 * @file nav_ball_collector_node.hpp
 * @brief Header for navigation-enabled ball collector node
 * 
 * This node uses Nav2 for path planning to navigate to balls
 * while avoiding obstacles using the global costmap.
 * 
 * Key Features:
 * - Uses Nav2 NavigateToPose action for path planning
 * - Transforms ball detections to map coordinates
 * - Plans optimal paths avoiding obstacles
 * - Falls back to reactive control when needed
 * - Multi-robot support with fleet coordinator integration
 */

#ifndef BALLVAC_BALL_COLLECTOR__NAV_BALL_COLLECTOR_NODE_HPP_
#define BALLVAC_BALL_COLLECTOR__NAV_BALL_COLLECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <ballvac_msgs/msg/ball_detection.hpp>
#include <ballvac_msgs/msg/ball_detection_array.hpp>
#include <ballvac_msgs/msg/robot_assignment.hpp>
#include <ballvac_msgs/msg/robot_status.hpp>
#include <ballvac_msgs/msg/ball_registry.hpp>
#include <std_msgs/msg/string.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <chrono>
#include <random>
#include <mutex>

namespace ballvac_ball_collector
{

// Action type aliases
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/**
 * @brief Enum representing the FSM states for navigation-based collection
 */
enum class NavCollectorState
{
    IDLE,           // Waiting for system to be ready
    EXPLORING,      // Exploring the environment using Nav2
    NAVIGATING,     // Navigating to a ball using Nav2
    APPROACHING,    // Fine approach using reactive control
    COLLECTING,     // Close enough to collect (delete) the ball
    RECOVERING      // Recovery behavior after failure
};

/**
 * @brief Convert state enum to string for logging
 */
inline std::string nav_state_to_string(NavCollectorState state)
{
    switch (state)
    {
        case NavCollectorState::IDLE:        return "IDLE";
        case NavCollectorState::EXPLORING:   return "EXPLORING";
        case NavCollectorState::NAVIGATING:  return "NAVIGATING";
        case NavCollectorState::APPROACHING: return "APPROACHING";
        case NavCollectorState::COLLECTING:  return "COLLECTING";
        case NavCollectorState::RECOVERING:  return "RECOVERING";
        default:                             return "UNKNOWN";
    }
}

/**
 * @brief Structure to hold detected ball information with world position
 */
struct NavTargetBall
{
    std::string name;                   // Entity name in Gazebo
    std::string color;                  // Ball color
    float bearing;                      // Angle from robot center (camera frame)
    float apparent_size;                // Size in pixels (larger = closer)
    geometry_msgs::msg::PoseStamped world_pose;  // Estimated position in map frame
    rclcpp::Time last_seen;             // When we last saw this ball
    bool valid;                         // Is target valid?
    bool position_known;                // Do we have world position?
    double estimated_distance;          // Estimated distance from apparent size
};

/**
 * @class NavBallCollectorNode
 * @brief ROS 2 node using Nav2 for path-planned ball collection
 */
class NavBallCollectorNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    explicit NavBallCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    ~NavBallCollectorNode() = default;

private:
    // =========================================================================
    // Callback functions
    // =========================================================================
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void detection_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void assignment_callback(const ballvac_msgs::msg::RobotAssignment::SharedPtr msg);
    void ball_registry_callback(const ballvac_msgs::msg::BallRegistry::SharedPtr msg);
    void heartbeat_timer_callback();
    void deleted_ball_callback(const std_msgs::msg::String::SharedPtr msg);
    void ball_claimed_callback(const std_msgs::msg::String::SharedPtr msg);  // Handle other robots' claims
    void pose_log_timer_callback();
    void control_loop();
    
    // =========================================================================
    // Fleet coordination functions
    // =========================================================================
    
    void publish_robot_status(uint8_t action, const std::string & action_ball_id = "");
    void publish_claim(const std::string & ball_id);
    void publish_collected(const std::string & ball_id);
    void publish_lost(const std::string & ball_id);
    bool is_ball_claimed_by_other(const std::string & ball_id);

    // =========================================================================
    // Navigation Action Callbacks
    // =========================================================================
    
    void navigate_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
    void navigate_feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void navigate_result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

    // =========================================================================
    // State machine functions
    // =========================================================================

    void execute_idle();
    void execute_exploring();
    void execute_navigating();
    void execute_approaching();
    void execute_collecting();
    void execute_recovering();
    void transition_to(NavCollectorState new_state);

    // =========================================================================
    // Navigation functions
    // =========================================================================

    /**
     * @brief Send a navigation goal to Nav2
     * @param goal_pose Target pose in map frame
     * @return True if goal was sent successfully
     */
    bool send_navigation_goal(const geometry_msgs::msg::PoseStamped & goal_pose);

    /**
     * @brief Cancel current navigation goal
     */
    void cancel_navigation();

    /**
     * @brief Generate next exploration waypoint
     * @return Pose for next exploration target
     */
    geometry_msgs::msg::PoseStamped generate_exploration_goal();

    /**
     * @brief Estimate ball world position from detection
     * @param detection Ball detection message
     * @return Estimated pose in map frame
     */
    geometry_msgs::msg::PoseStamped estimate_ball_world_pose(
        const ballvac_msgs::msg::BallDetection & detection);

    /**
     * @brief Transform pose between frames
     */
    bool transform_pose(
        const geometry_msgs::msg::PoseStamped & input,
        geometry_msgs::msg::PoseStamped & output,
        const std::string & target_frame);

    // =========================================================================
    // Helper functions
    // =========================================================================

    bool check_obstacle_front(float & min_range);
    void check_obstacle_sectors(float & min_front, float & min_left, float & min_right);
    void check_diagonal_sectors(float & min_front_left, float & min_front_right);
    float compute_obstacle_avoidance_steering();
    void publish_cmd_vel(float linear, float angular);
    
    /**
     * @brief Check if ball position is near a wall
     */
    bool is_ball_near_wall(const geometry_msgs::msg::PoseStamped & ball_pose, double threshold = 1.5);
    
    /**
     * @brief Calculate safe approach offset for wall-adjacent balls
     * Returns an offset pose that allows approaching the ball from a safe direction
     */
    geometry_msgs::msg::PoseStamped calculate_safe_approach_pose(
        const geometry_msgs::msg::PoseStamped & ball_pose);
    
    /**
     * @brief Check if current approach path is blocked by wall
     */
    bool is_approach_path_blocked();
    
    // =========================================================================
    // Fast LiDAR-based corner escape functions
    // =========================================================================
    
    /**
     * @brief Detect if robot is stuck in a corner (two walls meeting)
     * @return True if robot appears to be in a corner situation
     */
    bool detect_corner_situation();
    
    /**
     * @brief Find the best escape direction using LiDAR gap finding
     * @param best_angle Output: Best angle to escape (in radians, 0 = front)
     * @param best_distance Output: Distance to obstacles in that direction
     * @return True if a valid escape direction was found
     */
    bool find_escape_direction(float & best_angle, float & best_distance);
    
    /**
     * @brief Execute fast LiDAR-based escape maneuver
     * @return True if escape is complete, false if still escaping
     */
    bool execute_lidar_escape();
    
    /**
     * @brief Check if stuck (no progress being made)
     * @return True if robot is stuck
     */
    bool is_stuck();
    void delete_entity(const std::string & entity_name);
    void delete_entity_callback(
        rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future);
    void spawn_ball(const std::string & color);
    void spawn_ball_with_name(const std::string & color, const std::string & entity_name);
    void spawn_entity_callback(
        rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future);
    std::string generate_ball_sdf(const std::string & color, const std::string & entity_name);
    void get_random_spawn_position(double & x, double & y);
    std::string get_entity_name(const std::string & color);
    double estimate_distance_from_size(double apparent_size);
    
    // =========================================================================
    // NEW: Coverage-based exploration helpers
    // =========================================================================
    
    /**
     * @brief Update visited cells grid based on current position
     */
    void update_visited_cells();
    
    /**
     * @brief Get color priority value (lower = higher priority)
     */
    int get_color_priority(const std::string & color);
    
    /**
     * @brief Check if detected ball has higher priority than current target
     */
    bool is_higher_priority(const std::string & new_color, const std::string & current_color);
    
    /**
     * @brief Callback for other robot odometry (for avoidance)
     */
    void other_robot_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string & robot_name);
    
    /**
     * @brief Check distance to other robots, return repulsive steering if needed
     */
    float compute_robot_avoidance_steering();
    
    /**
     * @brief Execute escape maneuver when stuck
     */
    void execute_escape_maneuver();

    // =========================================================================
    // ROS 2 Communication
    // =========================================================================
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ballvac_msgs::msg::BallDetectionArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ballvac_msgs::msg::RobotAssignment>::SharedPtr assignment_sub_;
    rclcpp::Subscription<ballvac_msgs::msg::BallRegistry>::SharedPtr ball_registry_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr deleted_ball_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<ballvac_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ball_deleted_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ball_claimed_pub_;  // Publish when claiming a ball
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ball_claimed_sub_;  // Track other robots' claims
    rclcpp::Publisher<ballvac_msgs::msg::BallDetectionArray>::SharedPtr fleet_ball_pos_pub_;
    
    // Action client for Nav2
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    
    // Service clients
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr pose_log_timer_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // =========================================================================
    // Parameters
    // =========================================================================
    
    // Topics
    std::string scan_topic_;
    std::string detection_topic_;
    std::string cmd_topic_;
    std::string odom_topic_;
    std::string delete_service_;
    std::string spawn_service_;
    
    // Fleet coordination
    std::string robot_id_;
    bool use_fleet_coordinator_;
    std::string assignment_topic_;
    std::string robot_status_topic_;
    
    // Exploration bounds (20m x 20m)
    double exploration_min_x_;
    double exploration_max_x_;
    double exploration_min_y_;
    double exploration_max_y_;
    
    // Frame IDs
    std::string map_frame_;
    std::string robot_frame_;
    std::string camera_frame_;
    
    // Control parameters
    double collect_distance_m_;
    double obstacle_stop_m_;
    double obstacle_slow_m_;
    double obstacle_avoid_m_;  // Early steering distance
    double approach_speed_;
    double max_steer_;
    double control_rate_;
    bool respawn_balls_;
    double pose_log_interval_;
    
    // Approach parameters
    double steering_gain_;
    double approach_radius_threshold_;
    double nav_to_approach_distance_;  // Switch from nav to reactive at this distance
    
    // Recovery parameters
    double recover_duration_;
    double recover_speed_;
    
    // Ball detection parameters
    double min_ball_radius_;
    double max_ball_radius_;
    double collection_cooldown_;
    double target_lost_timeout_;
    double ball_visible_timeout_;
    
    // Camera parameters for distance estimation
    double camera_fov_horizontal_;      // Horizontal FOV in radians
    double camera_resolution_width_;    // Camera width in pixels
    double ball_actual_diameter_;       // Actual ball diameter in meters
    double camera_focal_length_;        // Estimated focal length

    // Exploration parameters
    double exploration_waypoint_distance_;
    double exploration_timeout_;
    bool explore_with_nav2_;
    double nav_goal_retry_cooldown_;
    double wander_bias_interval_;
    double wander_bias_max_;

    // =========================================================================
    // State variables
    // =========================================================================
    
    NavCollectorState current_state_;
    NavTargetBall target_ball_;
    
    // Current robot pose
    geometry_msgs::msg::PoseStamped current_pose_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    bool pose_received_;
    
    // LiDAR data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    
    // Navigation state
    bool navigation_in_progress_;
    bool navigation_succeeded_;
    bool nav2_ready_;  // Flag to track if Nav2 is ready
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    geometry_msgs::msg::PoseStamped current_nav_goal_;
    rclcpp::Time last_goal_rejection_time_;  // Cooldown for rejected goals
    int consecutive_rejections_;              // Count consecutive rejections
    rclcpp::Time last_nav_result_time_;
    
    // Exploration state
    std::vector<geometry_msgs::msg::PoseStamped> exploration_history_;
    int exploration_waypoint_index_;
    rclcpp::Time exploration_start_time_;
    rclcpp::Time last_wander_bias_time_;
    double wander_bias_;
    
    // Ball collection tracking
    std::set<std::string> collected_balls_;
    std::map<std::string, int> ball_collect_count_;
    rclcpp::Time last_collection_time_;
    
    // Fleet coordination state
    ballvac_msgs::msg::RobotAssignment current_assignment_;
    bool has_active_assignment_;
    rclcpp::Time last_heartbeat_time_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    
    // Fleet coordination - claimed balls tracking
    std::map<std::string, std::string> claimed_balls_;  // ball_id -> robot_id that claimed it
    
    // Recovery state
    rclcpp::Time recover_start_time_;
    int recover_phase_;
    float recover_turn_direction_;
    
    // Stuck detection
    rclcpp::Time last_progress_time_;
    float last_min_front_range_;
    int stuck_count_;
    
    // Corner escape state
    bool in_corner_escape_;
    rclcpp::Time corner_escape_start_time_;
    float escape_target_angle_;
    int escape_phase_;  // 0=detect, 1=backup, 2=turn, 3=forward
    double last_escape_x_;
    double last_escape_y_;
    int consecutive_stuck_count_;
    int same_direction_attempts_;    // How many times tried same direction
    float last_escape_direction_;    // Track last escape direction tried
    std::vector<float> blocked_directions_;  // Directions that are blocked
    rclcpp::Time blocked_directions_clear_time_;  // When to clear blocked directions
    
    // Random number generator
    std::mt19937 rng_;
    
    // Service pending flags
    bool delete_pending_;
    int delete_attempt_;  // Track which variant we're trying to delete (0=base, 1=_2 suffix)
    std::string current_delete_name_;  // Store the name being deleted
    
    // Thread safety
    std::mutex state_mutex_;
    
    // =========================================================================
    // NEW: Coverage-based exploration
    // =========================================================================
    static constexpr double COVERAGE_CELL_SIZE = 1.0;  // 1m cells
    std::map<std::pair<int,int>, int> visited_cells_;  // (x_cell, y_cell) -> visit_count
    
    // =========================================================================
    // NEW: Color priority for target selection (lower = higher priority)
    // =========================================================================
    std::map<std::string, int> color_priority_;
    
    // =========================================================================
    // NEW: Robot-robot avoidance
    // =========================================================================
    std::map<std::string, geometry_msgs::msg::Pose> other_robot_poses_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> other_robot_odom_subs_;
    double robot_avoid_radius_;
    std::vector<std::string> other_robot_names_;
    
    // =========================================================================
    // NEW: Never-stuck watchdog
    // =========================================================================
    rclcpp::Time last_movement_time_;
    double stall_timeout_;
    double velocity_threshold_;
};

}  // namespace ballvac_ball_collector

#endif  // BALLVAC_BALL_COLLECTOR__NAV_BALL_COLLECTOR_NODE_HPP_
