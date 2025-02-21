#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class CmdVelToRPM : public rclcpp::Node
{
public:
    CmdVelToRPM() : Node("cmd_vel_to_rpm"), wheelbase_(0.287), wheel_radius_(0.033), max_rpm_(140.0)
    {
        // Declare dynamically adjustable parameters
        this->declare_parameter("wheelbase", wheelbase_);
        this->declare_parameter("wheel_radius", wheel_radius_);
        this->declare_parameter("max_rpm", max_rpm_);

        // Create subscribers and publishers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToRPM::cmdVelCallback, this, std::placeholders::_1));

        left_wheel_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

        // Parameter callback to dynamically update values
        param_sub_ = this->add_on_set_parameters_callback(
            std::bind(&CmdVelToRPM::parameterCallback, this, std::placeholders::_1));
    }

private:
    double wheelbase_;
    double wheel_radius_;
    double max_rpm_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_rpm_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_sub_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;

        // Compute wheel speeds in m/s
        double left_wheel_speed = linear_vel - (angular_vel * wheelbase_ / 2.0);
        double right_wheel_speed = linear_vel + (angular_vel * wheelbase_ / 2.0);

        // Convert m/s to RPM
        double left_wheel_rpm = (left_wheel_speed / (2 * M_PI * wheel_radius_)) * 60.0;
        double right_wheel_rpm = (right_wheel_speed / (2 * M_PI * wheel_radius_)) * 60.0;

        // Clamp RPM values within max RPM limits
        left_wheel_rpm = std::clamp(left_wheel_rpm, -max_rpm_, max_rpm_);
        right_wheel_rpm = std::clamp(right_wheel_rpm, -max_rpm_, max_rpm_);

        // Publish RPM values
        std_msgs::msg::Float64 left_rpm_msg, right_rpm_msg;
        left_rpm_msg.data = left_wheel_rpm;
        right_rpm_msg.data = right_wheel_rpm;

        left_wheel_rpm_pub_->publish(left_rpm_msg);
        right_wheel_rpm_pub_->publish(right_rpm_msg);
    }

    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            if (param.get_name() == "wheelbase")
            {
                wheelbase_ = param.as_double();
            }
            else if (param.get_name() == "wheel_radius")
            {
                wheel_radius_ = param.as_double();
            }
            else if (param.get_name() == "max_rpm")
            {
                max_rpm_ = param.as_double();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Updated parameters: wheelbase=%.3f, wheel_radius=%.3f, max_rpm=%.1f",
                    wheelbase_, wheel_radius_, max_rpm_);

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToRPM>());
    rclcpp::shutdown();
    return 0;
}
