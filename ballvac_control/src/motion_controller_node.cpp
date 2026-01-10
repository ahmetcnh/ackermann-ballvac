#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MotionControllerNode : public rclcpp::Node
{
public:
    MotionControllerNode()
    : Node("motion_controller_node"),
      timeout_sec_(0.0),
      had_command_(false)
    {
        this->declare_parameter<std::string>("input_topic", "cmd_vel_in");
        this->declare_parameter<std::string>("output_topic", "cmd_vel");
        this->declare_parameter<double>("command_timeout_sec", 0.0);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        timeout_sec_ = this->get_parameter("command_timeout_sec").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_topic_,
            10,
            std::bind(&MotionControllerNode::cmd_callback, this, std::placeholders::_1));

        if (timeout_sec_ > 0.0)
        {
            auto period = std::chrono::duration<double>(std::max(0.05, timeout_sec_ / 2.0));
            watchdog_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&MotionControllerNode::watchdog_timer_callback, this));
        }

        RCLCPP_INFO(this->get_logger(),
            "Motion controller ready: input=%s output=%s timeout=%.2f",
            input_topic_.c_str(), output_topic_.c_str(), timeout_sec_);
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();
        had_command_ = true;
        cmd_pub_->publish(*msg);
    }

    void watchdog_timer_callback()
    {
        if (!had_command_)
        {
            return;
        }

        if ((this->now() - last_cmd_time_).seconds() <= timeout_sec_)
        {
            return;
        }

        geometry_msgs::msg::Twist stop;
        cmd_pub_->publish(stop);
        had_command_ = false;
    }

    std::string input_topic_;
    std::string output_topic_;
    double timeout_sec_;
    bool had_command_;
    rclcpp::Time last_cmd_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControllerNode>());
    rclcpp::shutdown();
    return 0;
}
