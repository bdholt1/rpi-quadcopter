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

    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    //std::string message1 = "w=" + std::to_string(q.w()) + ", x= " + std::to_string(q.x()) +
    //  ", y= " + std::to_string(q.y()) + ", z= " + std::to_string(q.z());
    //RCLCPP_INFO(this->get_logger(), "Incoming quaternion '%s'", message1.c_str());

    //std::string message2 = "roll=" + std::to_string(roll) + ", pitch= " + std::to_string(pitch) + ", yaw= " + std::to_string(yaw);
    //RCLCPP_INFO(this->get_logger(), "Converted euler angles '%s'", message2.c_str());

    //Eigen::Quaterniond q2 = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    //  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    //std::string message3 = "w=" + std::to_string(q2.w()) + ", x= " + std::to_string(q2.x()) +
    //  ", y= " + std::to_string(q2.y()) + ", z= " + std::to_string(q2.z());
    //RCLCPP_INFO(this->get_logger(), "Converted quaternion '%s'\n", message3.c_str());

    auto roll_msg = std_msgs::msg::Float64();
    roll_msg.data = roll;

    auto pitch_msg = std_msgs::msg::Float64();
    pitch_msg.data = pitch;

    auto yaw_msg = std_msgs::msg::Float64();
    yaw_msg.data = yaw;

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
