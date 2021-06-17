#include <functional>
#include <memory>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Imu2rpy : public rclcpp::Node
{
public:
  Imu2rpy()
      : Node("imu_to_rpy")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&Imu2rpy::imu_callback, this, _1));

    roll_publisher_ = this->create_publisher<std_msgs::msg::Float64>("roll", 10);
    pitch_publisher_ = this->create_publisher<std_msgs::msg::Float64>("pitch", 10);
    yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("yaw", 10);

    roll_rate_publisher_ = this->create_publisher<std_msgs::msg::Float64>("roll_rate", 10);
    pitch_rate_publisher_ = this->create_publisher<std_msgs::msg::Float64>("pitch_rate", 10);
    yaw_rate_publisher_ = this->create_publisher<std_msgs::msg::Float64>("yaw_rate", 10);
  }

private:

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    auto roll_msg = std_msgs::msg::Float64();
    roll_msg.data = euler[0];

    auto pitch_msg = std_msgs::msg::Float64();
    pitch_msg.data = euler[1];

    auto yaw_msg = std_msgs::msg::Float64();
    yaw_msg.data = euler[2];

    auto roll_rate_msg = std_msgs::msg::Float64();
    roll_rate_msg.data = msg->angular_velocity.x;

    auto pitch_rate_msg = std_msgs::msg::Float64();
    pitch_rate_msg.data = msg->angular_velocity.y;

    auto yaw_rate_msg = std_msgs::msg::Float64();
    yaw_rate_msg.data = msg->angular_velocity.z;

    roll_publisher_->publish(roll_msg);
    pitch_publisher_->publish(pitch_msg);
    yaw_publisher_->publish(yaw_msg);
    roll_rate_publisher_->publish(roll_rate_msg);
    pitch_rate_publisher_->publish(pitch_rate_msg);
    yaw_rate_publisher_->publish(yaw_rate_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_rate_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_rate_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Imu2rpy>());
  rclcpp::shutdown();
  return 0;
}