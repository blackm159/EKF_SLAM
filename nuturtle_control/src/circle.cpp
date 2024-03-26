/// \file circle.cpp
/// \brief This node publishes twist messages to make the robot move in a circle
/// \author Megan Black (blackm159)
/// PARAMETERS:
/// frequency (double): The frequency at which to publish twist messages
/// PUBLISHERS:
/// cmd_vel (geometry_msgs::msg::Twist): Publishes twist messages to make the robot move in a circle
/// SUBSCRIBERS:
/// none
/// SERVICES:
/// control (nuturtle_control::srv::Control): Service to set the velocity and radius of the circle
/// reverse (std_srvs::srv::Empty): Service to reverse the direction of the circle
/// stop (std_srvs::srv::Empty): Service to stop the circle
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
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include "nuturtle_control/srv/control.hpp"


using namespace std::chrono_literals;
using namespace turtlelib;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {

    // Parameters
    declare_parameter("frequency", 100.0);
    frequency_ = get_parameter("frequency").as_double();


    // Publishers
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Services
    control_service = create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));

    reverse_service = create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));

    stop_service = create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main Timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency_), std::bind(&Circle::timer_callback, this));


  }

private:
  // Main Timer Callback
  void timer_callback()
  {

    if (almost_equal(velocity_, 0.0, 1.0e-12)) {
    } else {
      update_twist();
      cmd_vel_publisher_->publish(cmd_vel_msg_);
    }

  }

  // Control Service Callback
  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    velocity_ = request->velocity;
    radius_ = request->radius;
  }

  // Reverse Service Callback
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    velocity_ = -velocity_;
  }

  // Stop Service Callback
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    velocity_ = 0.0;

    update_twist();
    cmd_vel_publisher_->publish(cmd_vel_msg_);

  }


  // Shared Pointers for ROS2
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service;
  rclcpp::TimerBase::SharedPtr timer_;


  // Private Variables
  double frequency_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  double velocity_; // angular velocity
  double radius_;

  // Private Functions
  void update_twist()
  {
    cmd_vel_msg_.linear.x = velocity_;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.linear.z = 0.0;
    cmd_vel_msg_.angular.x = 0.0;
    cmd_vel_msg_.angular.y = 0.0;
    cmd_vel_msg_.angular.z = velocity_ / radius_;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
