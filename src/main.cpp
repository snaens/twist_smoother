#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This node subscribes to a Twist (cmd_vel) message,
 * applies a PT1 regulator to it,
 * and publishes the result */

class TwistLimiter : public rclcpp::Node {

public:
    TwistLimiter() : Node("twist_limiter") {
        this->declare_parameter("acceleration_limit_mps2", 1.5);
        this->declare_parameter("publisher_hz", 20);

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "twist_limiter/in", 10, std::bind(&TwistLimiter::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("twist_limiter/out", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / static_cast<double>(this->get_parameter("publisher_hz").as_int()))), std::bind(&TwistLimiter::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "twist limiter online!");
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist& msg) {
        this->in_value = msg;
        this->last_sub_msg = std::chrono::system_clock::now();
    }

    bool is_twist_zeroed (const geometry_msgs::msg::Twist msg) {
        return !(msg.linear.x || msg.linear.y || msg.linear.z || msg.angular.x || msg.angular.y || msg.angular.z );
    }

    void timer_callback() {
        if ((this->last_sub_msg + 2s) < std::chrono::system_clock::now() && !is_twist_zeroed(in_value)) {
            // timeout detected + values aren't 0 (previous zeroing or controller untouched)
            RCLCPP_WARN(this->get_logger(), "Last message was 2s ago - Lag detected! Zeroing output.");
            this->in_value.linear.x = this->in_value.linear.y = this->in_value.linear.z = this->in_value.angular.x = this->in_value.angular.y = this->in_value.angular.z = 0.0;
        }

        const double max_velocity = this->get_parameter("acceleration_limit_mps2").as_double() / static_cast<double>(this->get_parameter("publisher_hz").as_int()); // m/s^2 / 1/s = m/s

        auto controller = [max_velocity](double *out, const double in) -> void {
            (*out) += std::max(-max_velocity, std::min(max_velocity, in - (*out))); // limit max accel, min/max
        };
        controller(&this->out_value.linear.x, this->in_value.linear.x);
        controller(&this->out_value.linear.y, this->in_value.linear.y);
        controller(&this->out_value.linear.z, this->in_value.linear.z);
        controller(&this->out_value.angular.x, this->in_value.angular.x);
        controller(&this->out_value.angular.y, this->in_value.angular.y);
        controller(&this->out_value.angular.z, this->in_value.angular.z);

        this->publisher_->publish(this->out_value);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    geometry_msgs::msg::Twist in_value;
    geometry_msgs::msg::Twist out_value;
    std::chrono::time_point<std::chrono::system_clock> last_sub_msg;

};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistLimiter>());
    rclcpp::shutdown();
    return 0;
}
