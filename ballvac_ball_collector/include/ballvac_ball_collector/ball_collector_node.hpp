#ifndef BALLVAC_BALL_COLLECTOR__BALL_COLLECTOR_NODE_HPP_
#define BALLVAC_BALL_COLLECTOR__BALL_COLLECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ballvac_msgs/msg/ball_detection_array.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>

#include <string>
#include <vector>
#include <random>
#include <map>
#include <set>

namespace ballvac_ball_collector
{

enum class CollectorState
{
  EXPLORE,
  APPROACH,
  COLLECT,
  RECOVER
};

struct TargetBall
{
  bool valid;
  std::string name;
  std::string color;
  float bearing;
  float apparent_size;
  rclcpp::Time last_seen;
};

class BallCollectorNode : public rclcpp::Node
{
public:
  explicit BallCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void detection_callback(const ballvac_msgs::msg::BallDetectionArray::SharedPtr msg);
  void control_loop();
  
  // Service callbacks
  void delete_entity_callback(rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future);
  void spawn_entity_callback(rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future);

  // State execution functions
  void execute_explore();
  void execute_approach();
  void execute_collect();
  void execute_recover();

  // Helper functions
  void transition_to(CollectorState new_state);
  bool select_target(const ballvac_msgs::msg::BallDetectionArray & detections);
  bool check_obstacle_front(float & min_range);
  float compute_obstacle_avoidance_steering();
  void publish_cmd_vel(float linear, float angular);
  std::string get_entity_name(const std::string & color);
  void delete_entity(const std::string & entity_name);
  void spawn_ball(const std::string & color);
  std::string generate_ball_sdf(const std::string & color, const std::string & entity_name);
  void get_random_spawn_position(double & x, double & y);
  bool compute_potential_field_velocity(float target_bearing, float & linear_vel, float & angular_vel);
  float find_best_gap(float preferred_direction);
  bool is_direction_safe(float angle, float safe_distance);
  
  std::string state_to_string(CollectorState state) {
    switch(state) {
      case CollectorState::EXPLORE: return "EXPLORE";
      case CollectorState::APPROACH: return "APPROACH";
      case CollectorState::COLLECT: return "COLLECT";
      case CollectorState::RECOVER: return "RECOVER";
      default: return "UNKNOWN";
    }
  }

  // Member variables
  CollectorState current_state_;
  TargetBall target_ball_;
  
  // State variables
  int recover_phase_;
  float recover_turn_direction_;
  float last_min_front_range_;
  int stuck_count_;
  float search_steering_direction_;
  bool delete_pending_;
  bool following_wall_;
  double wall_follow_distance_;
  int wall_follow_side_;
  
  // Timing
  rclcpp::Time last_collection_time_;
  rclcpp::Time last_detection_time_;
  rclcpp::Time search_direction_change_time_;
  rclcpp::Time recover_start_time_;
  rclcpp::Time last_progress_time_;
  
  // Data
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::map<std::string, int> ball_collect_count_;
  std::set<std::string> collected_balls_;
  std::mt19937 rng_;

  // Parameters
  std::string scan_topic_;
  std::string detection_topic_;
  std::string cmd_topic_;
  std::string delete_service_;
  std::string spawn_service_;
  
  double collect_distance_m_;
  double obstacle_stop_m_;
  double obstacle_slow_m_;
  double search_speed_;
  double approach_speed_;
  double max_steer_;
  double control_rate_;
  double steering_gain_;
  double approach_radius_threshold_;
  double recover_duration_;
  double recover_speed_;
  double stuck_timeout_;
  double stuck_distance_threshold_;
  double target_lost_timeout_;
  double min_ball_radius_;
  double max_ball_radius_;
  double collection_cooldown_;
  double repulsive_gain_;
  double attractive_gain_;
  double influence_distance_;
  double critical_distance_;

  // ROS Interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<ballvac_msgs::msg::BallDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace ballvac_ball_collector

#endif  // BALLVAC_BALL_COLLECTOR__BALL_COLLECTOR_NODE_HPP_