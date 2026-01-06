/**
 * @file ball_collector_node.hpp
 * @brief Header for ball collector node - FSM for autonomous ball collection
 * 
 * This node implements a finite state machine for autonomous ball collection:
 * - SEARCH: Wander while looking for balls, avoid obstacles
 * - APPROACH: Navigate towards a detected ball
 * - COLLECT: Verify ball is within range and delete it from simulation
 * - RECOVER: Back up and turn when stuck or collection fails
 */

#ifndef BALLVAC_BALL_COLLECTOR__BALL_COLLECTOR_NODE_HPP_
#define BALLVAC_BALL_COLLECTOR__BALL_COLLECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ballvac_msgs/msg/ball_detection.hpp>
#include <ballvac_msgs/msg/ball_detection_array.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>

#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>
#include <chrono>
#include <random>

namespace ballvac_ball_collector
{

/**
 * @brief Enum representing the FSM states
 */
enum class CollectorState
{
    EXPLORE,    // Exploring/mapping the room (wall-following)
    APPROACH,   // Approaching a ball that's in our way
    COLLECT,    // Close enough to collect (delete) the ball
    RECOVER     // Recovery behavior after failure or getting stuck
};

/**
 * @brief Convert state enum to string for logging
 */
inline std::string state_to_string(CollectorState state)
{
    switch (state)
    {
        case CollectorState::EXPLORE:  return "EXPLORE";
        case CollectorState::APPROACH: return "APPROACH";
        case CollectorState::COLLECT:  return "COLLECT";
        case CollectorState::RECOVER:  return "RECOVER";
        default:                       return "UNKNOWN";
    }
}

/**
 * @brief Structure to hold information about the current target ball
 */
struct TargetBall
{
    std::string name;           // Entity name in Gazebo
    std::string color;          // Ball color
    float bearing;              // Angle from robot center
    float apparent_size;        // Size in pixels (larger = closer)
    rclcpp::Time last_seen;     // When we last saw this ball
    bool valid;                 // Is target valid?
};

/**
 * @class BallCollectorNode
 * @brief ROS 2 node implementing FSM for autonomous ball collection
 */
class BallCollectorNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes node with parameters and communication
     */
    explicit BallCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    ~BallCollectorNode() = default;

private:
    // =========================================================================
    // Callback functions
    // =========================================================================
    
    /**
     * @brief Callback for LiDAR scan data
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Callback for ball detection messages
     */
    void detection_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg);

    /**
     * @brief Main control loop timer callback
     */
    void control_loop();

    // =========================================================================
    // State machine functions
    // =========================================================================

    /**
     * @brief Execute EXPLORE state behavior (wall-following exploration)
     */
    void execute_explore();

    /**
     * @brief Execute APPROACH state behavior
     */
    void execute_approach();

    /**
     * @brief Execute COLLECT state behavior
     */
    void execute_collect();

    /**
     * @brief Execute RECOVER state behavior
     */
    void execute_recover();

    /**
     * @brief Transition to a new state
     */
    void transition_to(CollectorState new_state);

    // =========================================================================
    // Helper functions
    // =========================================================================

    /**
     * @brief Select the best target ball from detections
     * @param detections Array of ball detections
     * @return True if a valid target was selected
     */
    bool select_target(const ballvac_msgs::msg::BallDetectionArray & detections);

    /**
     * @brief Check if there's an obstacle in front using LiDAR
     * @param min_range Output parameter for minimum range in front sector
     * @return True if obstacle is too close
     */
    bool check_obstacle_front(float & min_range);

    /**
     * @brief Compute steering to avoid obstacles
     * @return Steering command to avoid obstacles
     */
    float compute_obstacle_avoidance_steering();

    /**
     * @brief Compute velocity using Potential Field method
     * @param target_bearing Target direction (0 = straight ahead)
     * @param linear_vel Output linear velocity
     * @param angular_vel Output angular velocity
     * @return True if path is safe, false if blocked
     */
    bool compute_potential_field_velocity(float target_bearing, float & linear_vel, float & angular_vel);

    /**
     * @brief Find the best gap/opening in the LiDAR scan
     * @param preferred_direction Preferred direction (-1 to 1, 0 = front)
     * @return Best direction to go (angle in radians)
     */
    float find_best_gap(float preferred_direction);

    /**
     * @brief Check if a direction is safe to travel
     * @param angle Direction to check (radians, 0 = front)
     * @param safe_distance Minimum required distance
     * @return True if direction is safe
     */
    bool is_direction_safe(float angle, float safe_distance);

    /**
     * @brief Publish velocity command
     */
    void publish_cmd_vel(float linear, float angular);

    /**
     * @brief Call the DeleteEntity service to remove a ball
     * @param entity_name Name of the entity to delete
     */
    void delete_entity(const std::string & entity_name);

    /**
     * @brief Callback for delete entity service response
     */
    void delete_entity_callback(
        rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future);

    /**
     * @brief Spawn a new ball of the given color at a random position
     * @param color The color of the ball to spawn
     */
    void spawn_ball(const std::string & color);

    /**
     * @brief Callback for spawn entity service response
     */
    void spawn_entity_callback(
        rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future);

    /**
     * @brief Generate SDF string for a colored ball
     * @param color The color of the ball
     * @param entity_name The unique entity name for the ball
     * @return SDF XML string
     */
    std::string generate_ball_sdf(const std::string & color, const std::string & entity_name);

    /**
     * @brief Get a random spawn position for a ball
     * @param x Output X coordinate
     * @param y Output Y coordinate
     */
    void get_random_spawn_position(double & x, double & y);

    /**
     * @brief Get the Gazebo entity name for a ball based on color
     */
    std::string get_entity_name(const std::string & color);

    // =========================================================================
    // ROS 2 Communication
    // =========================================================================
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ballvac_msgs::msg::BallDetectionArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // =========================================================================
    // Parameters
    // =========================================================================
    
    // Topics
    std::string scan_topic_;
    std::string detection_topic_;
    std::string cmd_topic_;
    std::string delete_service_;
    std::string spawn_service_;
    
    // Control parameters
    double collect_distance_m_;     // Distance at which to collect ball
    double obstacle_stop_m_;        // Distance at which to stop for obstacles
    double obstacle_slow_m_;        // Distance at which to slow down
    double search_speed_;           // Linear speed during search
    double approach_speed_;         // Linear speed during approach
    double max_steer_;              // Maximum steering angle
    double control_rate_;           // Control loop rate in Hz
    
    // Approach parameters
    double steering_gain_;          // Proportional gain for steering towards ball
    double approach_radius_threshold_;  // Ball radius threshold for collection
    
    // Recovery parameters
    double recover_duration_;       // How long to back up during recovery
    double recover_speed_;          // Speed to back up during recovery
    
    // Stuck detection parameters
    double stuck_timeout_;          // Time without progress to consider stuck
    double stuck_distance_threshold_; // Minimum LiDAR range change to consider moving
    
    // Ball radius filtering parameters
    double min_ball_radius_;        // Minimum radius to consider valid detection
    double max_ball_radius_;        // Maximum radius (filter out large false positives)
    double collection_cooldown_;    // Cooldown after collection attempt

    // Potential Field parameters
    double repulsive_gain_;         // Gain for repulsive force from obstacles
    double attractive_gain_;        // Gain for attractive force to target
    double influence_distance_;     // Distance at which obstacles start affecting robot
    double critical_distance_;      // Distance at which robot must stop

    // =========================================================================
    // State variables
    // =========================================================================
    
    CollectorState current_state_;
    TargetBall target_ball_;
    
    // LiDAR data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    
    // Set of collected ball colors (to avoid targeting same ball)
    std::set<std::string> collected_balls_;
    
    // Ball collection counter per color
    std::map<std::string, int> ball_collect_count_;
    
    // Random number generator for spawn positions
    std::mt19937 rng_;
    
    // Recovery state tracking
    rclcpp::Time recover_start_time_;
    int recover_phase_;  // 0 = back up, 1 = turn, 2 = turn more
    float recover_turn_direction_;  // Direction to turn during recovery
    
    // Stuck detection
    rclcpp::Time last_progress_time_;
    float last_min_front_range_;
    int stuck_count_;  // How many times we've been stuck recently
    
    // Search behavior
    double search_steering_direction_;
    rclcpp::Time search_direction_change_time_;
    
    // Delete service pending
    bool delete_pending_;

    // Exploration behavior (wall-following)
    bool following_wall_;           // Are we currently following a wall?
    double wall_follow_distance_;   // Target distance from wall
    double exploration_start_time_; // When exploration started
    int wall_follow_side_;          // 1 = right wall, -1 = left wall
    
    // Timing
    rclcpp::Time last_detection_time_;
    double target_lost_timeout_;  // Seconds before considering target lost
    rclcpp::Time last_collection_time_;  // When last collection was attempted
};

}  // namespace ballvac_ball_collector

#endif  // BALLVAC_BALL_COLLECTOR__BALL_COLLECTOR_NODE_HPP_
