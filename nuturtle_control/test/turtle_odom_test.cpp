#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;


TEST_CASE("Test odometry_node", "[odometry]") {

  auto node = rclcpp::Node::make_shared("odometry_test");

  auto init_pose_client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");
  bool service_found = false;

  auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  geometry_msgs::msg::TransformStamped tf;

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    if (init_pose_client->wait_for_service(0s)) {
      service_found = true;
      // break;
    }
    try {
      tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
    }

    rclcpp::spin_some(node);
  }

  CHECK(service_found);

  REQUIRE_THAT(tf.transform.translation.x, Catch::Matchers::WithinAbs(0.0, 0.01));
  REQUIRE_THAT(tf.transform.translation.y, Catch::Matchers::WithinAbs(0.0, 0.01));
  REQUIRE_THAT(tf.transform.translation.z, Catch::Matchers::WithinAbs(0.0, 0.01));
  REQUIRE_THAT(tf.transform.rotation.x, Catch::Matchers::WithinAbs(0.0, 0.01));
  REQUIRE_THAT(tf.transform.rotation.y, Catch::Matchers::WithinAbs(0.0, 0.01));
  REQUIRE_THAT(tf.transform.rotation.z, Catch::Matchers::WithinAbs(0.0, 0.01));
  REQUIRE_THAT(tf.transform.rotation.w, Catch::Matchers::WithinAbs(1.0, 0.01));

  // REQUIRE(0 == 1);

}
