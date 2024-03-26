/// \file turtle_control.cpp
/// \brief This node publishes wheel commands and joint states
/// \author Megan Black (blackm159)
/// PARAMETERS:
/// wheel_radius (double): The radius of the wheels
/// track_width (double): The distance between the wheels
/// motor_cmd_max (double): The maximum motor command
/// motor_cmd_per_rad_sec (double): The motor command per radian per second
/// encoder_ticks_per_rad (double): The encoder ticks per radian
/// collision_radius (double): The radius of the robot
/// PUBLISHERS:
/// /wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Publishes the wheel commands
/// /joint_states (sensor_msgs::msg::JointState): Publishes the joint states
/// SUBSCRIBERS:
/// /cmd_vel (geometry_msgs::msg::Twist): The velocity command
/// /sensor_data (nuturtlebot_msgs::msg::SensorData): The sensor data
/// SERVICES:
/// none
/// CLIENTS:
/// none

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"


using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Parameters
    declare_parameter("wheel_radius", -1.0);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    if (wheel_radius_ < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting wheel_radius parameter");
      rclcpp::shutdown();
    }

    declare_parameter("track_width", -1.0);
    track_width_ = get_parameter("track_width").as_double();
    if (track_width_ < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting track_width parameter");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_max", -1.0);
    motor_cmd_max_ = get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_max_ < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting motor_cmd_max parameter");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting motor_cmd_per_rad_sec parameter");
      rclcpp::shutdown();
    }

    declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    if (encoder_ticks_per_rad_ < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting encoder_ticks_per_rad parameter");
      rclcpp::shutdown();
    }

    declare_parameter("collision_radius", -1.0);
    collision_radius_ = get_parameter("collision_radius").as_double();
    if (collision_radius_ < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting collision_radius parameter");
      rclcpp::shutdown();
    }


    // Publishers
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd",
      10);

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states",
      10);

    // Subscribers
    cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    sensor_subscriber_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data",
      10,
      std::bind(&TurtleControl::sensor_callback, this, std::placeholders::_1));


    diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);

  }

private:
  // cmd_vel Callback
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    turtlelib::Twist2D twist;
    twist.x = msg->linear.x;
    twist.y = msg->linear.y;
    twist.omega = msg->angular.z;

    turtlelib::WheelPositions wheel_velocities = diff_drive_.twist_to_wheel_velocities(twist);

    // Publish Wheel Commands
    nuturtlebot_msgs::msg::WheelCommands new_msg;
    new_msg.left_velocity =
      std::min(
      std::max(
        wheel_velocities.phi_left_ / motor_cmd_per_rad_sec_,
        -motor_cmd_max_), motor_cmd_max_);
    new_msg.right_velocity =
      std::min(
      std::max(
        wheel_velocities.phi_right_ / motor_cmd_per_rad_sec_,
        -motor_cmd_max_), motor_cmd_max_);

    wheel_cmd_publisher_->publish(new_msg);
  }

  // sensor_data Callback
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    turtlelib::WheelPositions wheel_encoder;
    wheel_encoder.phi_left_ = double(msg->left_encoder) / encoder_ticks_per_rad_;
    wheel_encoder.phi_right_ = double(msg->right_encoder) / encoder_ticks_per_rad_;

    // Publish Joint States
    sensor_msgs::msg::JointState new_msg;
    new_msg.name = {"wheel_left_joint", "wheel_right_joint"};
    new_msg.position = {wheel_encoder.phi_left_, wheel_encoder.phi_right_};
    new_msg.header.stamp = get_clock()->now();


    joint_state_publisher_->publish(new_msg);
  }


  // Shared Pointers for ROS2
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;


  // Private Variables
  double wheel_radius_;
  double track_width_;
  double motor_cmd_max_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
  turtlelib::DiffDrive diff_drive_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
