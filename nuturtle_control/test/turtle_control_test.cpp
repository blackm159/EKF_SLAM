#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;

nuturtlebot_msgs::msg::WheelCommands wheel_commands;
sensor_msgs::msg::JointState js_msg;

TEST_CASE("Test turtle_control node", "[turtle_control]") {

  auto node = rclcpp::Node::make_shared("turtle_control_test");


  double encoder_ticks_per_rad_ = 651.898646904;

  auto cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_subscriber_ = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, [](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      // save wheel commands
      wheel_commands = *msg;
    });

  auto sensor_publisher_ = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
    "sensor_data",
    10);
  auto joint_state_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // save joint states
      js_msg = *msg;
    });

  // Pure Translation
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;

  nuturtlebot_msgs::msg::SensorData sensor_data;
  sensor_data.left_encoder = 100;
  sensor_data.right_encoder = 100;

  DiffDrive diff_drive(0.160, 0.033);

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {

    // Publish a message to the topic
    cmd_vel_publisher_->publish(twist);

    sensor_publisher_->publish(sensor_data);


    rclcpp::spin_some(node);
  }

  Twist2D twist2d;
  twist2d.x = twist.linear.x;
  twist2d.y = twist.linear.y;
  twist2d.omega = twist.angular.z;

  turtlelib::WheelPositions wheel_velocities = diff_drive.twist_to_wheel_velocities(twist2d);

  RCLCPP_INFO(
    node->get_logger(), "Wheel velocities: %f, %f", wheel_velocities.phi_left_,
    wheel_velocities.phi_right_);
  RCLCPP_INFO(
    node->get_logger(), "Wheel commands: %d, %d", wheel_commands.left_velocity,
    wheel_commands.right_velocity);

  // find the wheel commands
  REQUIRE_THAT(
    wheel_commands.left_velocity,
    Catch::Matchers::WithinAbs(
      int(std::min(
        std::max(wheel_velocities.phi_left_ / 0.024, -265.0),
        265.0)), 1e-6));
  REQUIRE_THAT(
    wheel_commands.right_velocity,
    Catch::Matchers::WithinAbs(
      int(std::min(
        std::max(wheel_velocities.phi_right_ / 0.024, -265.0),
        265.0)), 1e-6));


  REQUIRE_THAT(
    js_msg.position.at(0),
    Catch::Matchers::WithinAbs(sensor_data.left_encoder / encoder_ticks_per_rad_, 1e-6));
  REQUIRE_THAT(
    js_msg.position.at(1),
    Catch::Matchers::WithinAbs(sensor_data.right_encoder / encoder_ticks_per_rad_, 1e-6));

  // Pure Rotation
  twist.linear.x = 0.0;
  twist.angular.z = 1.0;

  start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_publisher_->publish(twist);

    rclcpp::spin_some(node);
  }

  // Twist2D twist2d;
  twist2d.x = twist.linear.x;
  twist2d.y = twist.linear.y;
  twist2d.omega = twist.angular.z;

  wheel_velocities = diff_drive.twist_to_wheel_velocities(twist2d);


  RCLCPP_INFO(
    node->get_logger(), "Wheel velocities: %f, %f", wheel_velocities.phi_left_,
    wheel_velocities.phi_right_);
  RCLCPP_INFO(
    node->get_logger(), "Wheel commands: %d, %d", wheel_commands.left_velocity,
    wheel_commands.right_velocity);

  // find the wheel commands
  REQUIRE_THAT(
    wheel_commands.left_velocity,
    Catch::Matchers::WithinAbs(
      int(std::min(
        std::max(wheel_velocities.phi_left_ / 0.024, -265.0),
        265.0)), 1e-6));
  REQUIRE_THAT(
    wheel_commands.right_velocity,
    Catch::Matchers::WithinAbs(
      int(std::min(
        std::max(wheel_velocities.phi_right_ / 0.024, -265.0),
        265.0)), 1e-6));


  // REQUIRE(0 == 1);

}
