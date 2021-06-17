// Periodically publish a setpoint message.

#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/float64.hpp"

class Setpoint : public rclcpp::Node
{
public:
  explicit Setpoint()
      : Node("setpoint")
  {
    pub_ = this->create_publisher<std_msgs::msg::Float64>("setpoint", 10);

    this->declare_parameter<double>("setpoint_value", 1.0);

    std::chrono::milliseconds ms{static_cast<long int>(5000)};
    timer_ = this->create_wall_timer(ms, std::bind(&Setpoint::timer_callback, this));
  }

private:
  void timer_callback()
  {
    double setpoint_value;
    this->get_parameter("setpoint_value", setpoint_value);

    std_msgs::msg::Float64 msg;
    msg.data = setpoint_value;

    pub_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Setpoint>());
  rclcpp::shutdown();
  return 0;
}
