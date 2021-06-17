#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class AltitudeController : public rclcpp::Node
{
public:
  AltitudeController()
      : Node("altitude_controller")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("output", 10);

    control_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "input", 10, std::bind(&AltitudeController::control_callback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&AltitudeController::imu_callback, this, _1));

    this->declare_parameter<double>("freq", 1000.0);
    this->declare_parameter<double>("mass", 1.0);

    this->get_parameter("freq", freq_);
    std::chrono::milliseconds ms{static_cast<long int>(1000 / freq_)};
    timer_ = this->create_wall_timer(ms, std::bind(&AltitudeController::timer_callback, this));
  }

private:

  void timer_callback()
  {
    this->get_parameter("mass", mass_);
    const double kGravity = 9.81; // m/s^2

    double thrust; // total thrust to apply through all 4 motors (Newtons)
    {
      std::lock_guard<std::mutex> guard(mutex_);
      thrust = 1./ (cos(roll_) * cos(pitch_)) * (control_ + mass_ * kGravity);
    }
    auto message = std_msgs::msg::Float64();
    message.data = thrust;
    publisher_->publish(message);
  }

  void control_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    control_ = msg->data;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

    std::lock_guard<std::mutex> guard(mutex_);
    roll_ = euler[0];
    pitch_ = euler[1];
  }

  double control_, roll_, pitch_;
  double freq_;
  double mass_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AltitudeController>());
  rclcpp::shutdown();
  return 0;
}