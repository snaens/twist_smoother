#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This node subscribes to a Twist (cmd_vel) message,
 * applies a PT1 regulator to it,
 * and publishes the result */

class TwistSmoother : public rclcpp::Node {

public:
    TwistSmoother() : Node("twist_smoother") {
        this->declare_parameter("acceleration_limit_mps2", 1.5);
        this->declare_parameter("cutoff_threshold", 0.05);
        this->declare_parameter("publisher_hz", 20);

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "twist_smoother/in", 10, std::bind(&TwistSmoother::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("twist_smoother/out", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / static_cast<double>(this->get_parameter("publisher_hz").as_int()))), std::bind(&TwistSmoother::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "twist smoother online!");
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
        const double cutoff_threshold = this->get_parameter("cutoff_threshold").as_double();

        auto controller = [max_velocity, cutoff_threshold](double *out, const double in) -> void {
            if (abs(in - (*out)) < cutoff_threshold) // hard cut off → we are close to limit (limit is reached in infinity time)
                (*out) = (in < cutoff_threshold) ? 0.0 : in; // zero if approaching zero, else upper limit
            else
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
    rclcpp::spin(std::make_shared<TwistSmoother>());
    rclcpp::shutdown();
    return 0;
}
