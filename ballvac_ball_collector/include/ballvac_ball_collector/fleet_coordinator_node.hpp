/**
 * @file fleet_coordinator_node.hpp
 * @brief Header for fleet coordinator node for multi-robot ball collection
 * 
 * This node manages coordination between multiple robots:
 * - Maintains a registry of all detected balls
 * - Assigns balls to robots based on color priority and path cost
 * - Handles claims, collections, and lost ball reports
 * - Prevents duplicate ball targeting
 */

#ifndef BALLVAC_BALL_COLLECTOR__FLEET_COORDINATOR_NODE_HPP_
#define BALLVAC_BALL_COLLECTOR__FLEET_COORDINATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ballvac_msgs/msg/ball_detection_array.hpp>
#include <ballvac_msgs/msg/ball_state.hpp>
#include <ballvac_msgs/msg/ball_registry.hpp>
#include <ballvac_msgs/msg/robot_assignment.hpp>
#include <ballvac_msgs/msg/robot_status.hpp>
#include <ballvac_msgs/msg/fleet_assignments.hpp>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

namespace ballvac_ball_collector
{

/**
 * @brief Structure to track a robot's state
 */
struct RobotInfo
{
    std::string robot_id;
    geometry_msgs::msg::PoseStamped pose;
    uint8_t state;
    std::string assigned_ball_id;
    rclcpp::Time last_heartbeat;
    bool is_operational;
    bool navigation_ready;
    geometry_msgs::msg::PoseStamped target_pose;  // Current navigation target
    double velocity_x;  // Current velocity for collision prediction
    double velocity_y;
};

/**
 * @brief Structure to track a ball's state
 */
struct BallInfo
{
    std::string ball_id;
    std::string color;
    geometry_msgs::msg::PoseStamped pose;
    uint8_t state;  // UNCLAIMED, CLAIMED, COLLECTED, LOST
    std::string claimed_by_robot;
    rclcpp::Time claim_time;
    rclcpp::Time last_seen_time;
    uint8_t priority;  // 0 = highest priority
};

/**
 * @class FleetCoordinatorNode
 * @brief Central coordinator for multi-robot ball collection
 */
class FleetCoordinatorNode : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

    explicit FleetCoordinatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~FleetCoordinatorNode() = default;

private:
    // =========================================================================
    // Parameters
    // =========================================================================
    std::vector<std::string> color_priority_;  // e.g., ["red", "blue", "green", ...]
    double claim_timeout_sec_;
    double heartbeat_timeout_sec_;
    double assignment_rate_hz_;
    double standoff_distance_;
    bool use_planner_for_cost_;
    double map_bounds_min_x_;
    double map_bounds_max_x_;
    double map_bounds_min_y_;
    double map_bounds_max_y_;
    double wall_safety_margin_;      // Safety distance from walls
    double robot_conflict_radius_;   // Distance to consider robot paths conflicting

    // =========================================================================
    // State
    // =========================================================================
    std::map<std::string, BallInfo> ball_registry_;
    std::map<std::string, RobotInfo> robot_registry_;
    std::mutex registry_mutex_;

    // =========================================================================
    // ROS Communication
    // =========================================================================
    
    // Subscribers - one for each robot's detection topic
    std::map<std::string, rclcpp::Subscription<ballvac_msgs::msg::BallDetectionArray>::SharedPtr> detection_subs_;
    
    // Subscriber for robot status updates
    rclcpp::Subscription<ballvac_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    
    // Subscriber for ball positions from ball_launcher
    rclcpp::Subscription<ballvac_msgs::msg::BallDetectionArray>::SharedPtr ball_position_sub_;
    
    // Publishers
    rclcpp::Publisher<ballvac_msgs::msg::BallRegistry>::SharedPtr registry_pub_;
    rclcpp::Publisher<ballvac_msgs::msg::FleetAssignments>::SharedPtr assignments_pub_;
    
    // Per-robot assignment publishers
    std::map<std::string, rclcpp::Publisher<ballvac_msgs::msg::RobotAssignment>::SharedPtr> robot_assignment_pubs_;
    
    // Action clients for path planning (one per robot namespace)
    std::map<std::string, rclcpp_action::Client<ComputePathToPose>::SharedPtr> planner_clients_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr assignment_timer_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;

    // =========================================================================
    // Callbacks
    // =========================================================================
    void detection_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg,
                           const std::string & robot_id);
    void ball_position_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg);
    void robot_status_callback(const ballvac_msgs::msg::RobotStatus::SharedPtr msg);
    void assignment_timer_callback();
    void cleanup_timer_callback();

    // =========================================================================
    // Core logic
    // =========================================================================
    
    /**
     * @brief Get priority index for a color (lower = higher priority)
     */
    uint8_t get_color_priority(const std::string & color);
    
    /**
     * @brief Update or add a ball to the registry
     */
    void update_ball(const std::string & ball_id, const std::string & color,
                     const geometry_msgs::msg::PoseStamped & pose,
                     const std::string & seen_by_robot);
    
    /**
     * @brief Select the best ball for assignment (highest priority color, then closest)
     */
    std::string select_best_ball_for_robot(const std::string & robot_id);
    
    /**
     * @brief Compute path cost from robot to ball (Euclidean or planner-based)
     */
    double compute_path_cost(const std::string & robot_id, 
                            const geometry_msgs::msg::PoseStamped & target);
    
    /**
     * @brief Assign a ball to a robot
     */
    void assign_ball(const std::string & robot_id, const std::string & ball_id);
    
    /**
     * @brief Release a ball claim
     */
    void release_claim(const std::string & ball_id);
    
    /**
     * @brief Mark a ball as collected
     */
    void mark_collected(const std::string & ball_id);
    
    /**
     * @brief Check if pose is within map bounds
     */
    bool is_within_bounds(const geometry_msgs::msg::PoseStamped & pose);
    
    /**
     * @brief Check if ball is near a wall (requires careful approach)
     */
    bool is_near_wall(const geometry_msgs::msg::PoseStamped & pose, double threshold = 1.5);
    
    /**
     * @brief Check if two robots' paths would conflict
     */
    bool paths_conflict(const std::string & robot1, const std::string & robot2,
                       const geometry_msgs::msg::PoseStamped & target);
    
    /**
     * @brief Get safe approach angle to avoid walls
     */
    double get_safe_approach_angle(const geometry_msgs::msg::PoseStamped & ball_pose,
                                   const geometry_msgs::msg::PoseStamped & robot_pose);

    /**
     * @brief Load initial ball registry entries from launch-provided strings.
     */
    void load_initial_balls(const std::vector<std::string> & entries);

    /**
     * @brief Extract color name from a ball model name like "ball_red_2".
     */
    std::string get_color_from_ball_id(const std::string & ball_id) const;

    /**
     * @brief Match a detection by color and proximity to an existing ball id.
     */
    std::string match_ball_by_color_and_distance(
        const std::string & color,
        const geometry_msgs::msg::PoseStamped & pose,
        double max_distance_m) const;
    
    /**
     * @brief Publish current assignments to all robots
     */
    void publish_assignments();
    
    /**
     * @brief Publish ball registry snapshot
     */
    void publish_registry();
    
    /**
     * @brief Initialize subscriptions for a robot
     */
    void setup_robot(const std::string & robot_id);
};

}  // namespace ballvac_ball_collector

#endif  // BALLVAC_BALL_COLLECTOR__FLEET_COORDINATOR_NODE_HPP_
