#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include <mutex>

#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class QuadcopterMotorController : public rclcpp::Node
{
public:
  QuadcopterMotorController()
      : Node("quadcopter_motor_controller")
  {
    thrust_sub_ = this->create_subscription<std_msgs::msg::Float64>("thrust", 1, std::bind(&QuadcopterMotorController::thrust_callback, this, _1));
    roll_sub_ = this->create_subscription<std_msgs::msg::Float64>("roll", 1, std::bind(&QuadcopterMotorController::roll_callback, this, _1));
    pitch_sub_ = this->create_subscription<std_msgs::msg::Float64>("pitch", 1, std::bind(&QuadcopterMotorController::pitch_callback, this, _1));
    yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("yaw", 1, std::bind(&QuadcopterMotorController::yaw_callback, this, _1));

    this->declare_parameter<double>("freq", 1000.0);
    this->declare_parameter<unsigned short>("motor_front_gpio", 5);
    this->declare_parameter<unsigned short>("motor_right_gpio", 6);
    this->declare_parameter<unsigned short>("motor_back_gpio", 12);
    this->declare_parameter<unsigned short>("motor_left_gpio", 13);

    this->get_parameter("freq", freq_);
    std::chrono::milliseconds ms{static_cast<long int>(1000 / freq_)};
    timer_ = this->create_wall_timer(ms, std::bind(&QuadcopterMotorController::timer_callback, this));

    this->get_parameter("motor_front_gpio", motor_front_gpio_);
    this->get_parameter("motor_right_gpio", motor_right_gpio_);
    this->get_parameter("motor_back_gpio", motor_back_gpio_);
    this->get_parameter("motor_left_gpio", motor_left_gpio_);

    long kPWMFrequency = 50;
    //pi.set_PWM_frequency(motor_front_gpio_, kPWMFrequency)
    //pi.set_PWM_frequency(motor_right_gpio_, kPWMFrequency);
    //pi.set_PWM_frequency(motor_back_gpio_, kPWMFrequency);
    //pi.set_PWM_frequency(motor_left_gpio_, kPWMFrequency);

    long kPWMRange = 40000;
    //pi.set_PWM_range(motor_front_gpio_, kPWMRange)
    //pi.set_PWM_range(motor_right_gpio_, kPWMRange);
    //pi.set_PWM_range(motor_back_gpio_, kPWMRange);
    //pi.set_PWM_range(motor_left_gpio_, kPWMRange);
  }

private:
  void timer_callback()
  {
    double motor_front, motor_right, motor_back, motor_left;

    // calculate the amount of control to apply to each motor
    {
      std::lock_guard<std::mutex> guard(control_mutex_);
      motor_front = thrust_control_/4 + pitch_control_ + yaw_control_;
      motor_right = thrust_control_/4 - roll_control_  - yaw_control_;
      motor_back  = thrust_control_/4 - pitch_control_ + yaw_control_;
      motor_left  = thrust_control_/4 + roll_control_  - yaw_control_;
    }

    // apply the control to the motor by adjusting the PWM dutycycle
    // the ESC accepts a PWM signal at 50Hz (20ms)
    // the motor is at 0% throttle at a 5% dutycycle (1ms over 20ms)
    // the motor is at 100% throttle at a 10% dutycycle (2ms over 20ms)
    long kPWMLow = 2000;
    long kPWMHigh = 4000;
    //pi.set_PWM_dutycycle(motor_front_gpio_, map(motor_front, 0, 2000, kPWMLow, kPWMHigh));
    //pi.set_PWM_dutycycle(motor_right_gpio_, map(motor_right, 0, 2000, kPWMLow, kPWMHigh));
    //pi.set_PWM_dutycycle(motor_back_gpio_, map(motor_back, 0, 2000, kPWMLow, kPWMHigh));
    //pi.set_PWM_dutycycle(motor_left_gpio_, map(motor_left, 0, 2000, kPWMLow, kPWMHigh));
  }

  void thrust_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(control_mutex_);
    thrust_control_ = msg->data;
  }

  void roll_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(control_mutex_);
    roll_control_ = msg->data;
  }

  void pitch_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(control_mutex_);
    pitch_control_ = msg->data;
  }

  void yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(control_mutex_);
    yaw_control_ = msg->data;
  }

  double thrust_control_, roll_control_,  pitch_control_, yaw_control_;
  unsigned short motor_front_gpio_, motor_right_gpio_, motor_back_gpio_, motor_left_gpio_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr thrust_sub_, roll_sub_, pitch_sub_, yaw_sub_;
  std::mutex control_mutex_;
  double freq_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadcopterMotorController>());
  rclcpp::shutdown();
  return 0;
}