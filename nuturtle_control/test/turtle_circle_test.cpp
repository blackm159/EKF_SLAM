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

#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;

double counter = 0.0;

TEST_CASE("Test turtle_circle node", "[turtle_circle]") {

  auto node = rclcpp::Node::make_shared("turtle_circle_test");


  auto cmd_vel_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](const geometry_msgs::msg::Twist::SharedPtr) {
      // increment counter
      counter = counter + 1.0;
    });

  auto control_service_client = node->create_client<nuturtle_control::srv::Control>("control");
  // wait for service to become available
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    if (control_service_client->wait_for_service(0s)) {
      break;
    }

    rclcpp::spin_some(node);
  }

  auto request = std::make_shared<nuturtle_control::srv::Control::Request>();
  request->velocity = 1.0;
  request->radius = 0.1;

  auto result = control_service_client->async_send_request(request);

  while (rclcpp::spin_until_future_complete(node, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
  }


  start_time = rclcpp::Clock().now();
  rclcpp::Time end_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    end_time = rclcpp::Clock().now();
    // if (counter > 100) {
    //   break;
    // }

    rclcpp::spin_some(node);
  }

  RCLCPP_INFO(node->get_logger(), "Frequency: %f", counter / (end_time - start_time).seconds());

  REQUIRE_THAT(counter / (end_time - start_time).seconds(), Catch::Matchers::WithinAbs(100.0, 5.0));


  // REQUIRE(0 == 1);

}
