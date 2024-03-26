/// \file odometry.cpp
/// \brief This node publishes odometry and path messages
/// \author Megan Black (blackm159)
/// PARAMETERS:
/// body_id (string): The frame id of the robot
/// odom_id (string): The frame id of the odometry
/// wheel_left (string): The name of the left wheel joint
/// wheel_right (string): The name of the right wheel joint
/// wheel_radius (double): The radius of the wheels
/// track_width (double): The distance between the wheels
/// PUBLISHERS:
/// /odom (nav_msgs::msg::Odometry): Publishes the odometry of the robot
/// /path (nav_msgs::msg::Path): Publishes the path of the robot
/// SUBSCRIBERS:
/// /joint_states (sensor_msgs::msg::JointState): The joint states of the robot
/// SERVICES:
/// /initial_pose (nuturtle_control::srv::InitialPose): Service to set the initial pose of the robot
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
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include "nuturtle_control/srv/initial_pose.hpp"


using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // Parameters
    declare_parameter("body_id", " ");
    body_id_ = get_parameter("body_id").as_string();
    if (body_id_ == " ") {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting body_id parameter");
      rclcpp::shutdown();
    }

    declare_parameter("odom_id", " ");
    odom_id_ = get_parameter("odom_id").as_string();
    if (odom_id_ == " ") {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting odom_id parameter");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_left", " ");
    wheel_left_ = get_parameter("wheel_left").as_string();
    if (wheel_left_ == " ") {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting wheel_left parameter");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_right", " ");
    wheel_right_ = get_parameter("wheel_right").as_string();
    if (wheel_right_ == " ") {
      RCLCPP_ERROR_STREAM(get_logger(), "Error getting wheel_right parameter");
      rclcpp::shutdown();
    }

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


    // Subscribers
    joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_states_callback, this, std::placeholders::_1));

    // Publishers
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", 10);

    // Services
    init_pose_service_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(&Odometry::init_pose_callback, this, std::placeholders::_1, std::placeholders::_2));

    // TF Broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Main Timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 200.0), std::bind(&Odometry::timer_callback, this));


    diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);

    odom_msg.header.frame_id = odom_id_;
    odom_msg.child_frame_id = body_id_;

    odom_tf.header.frame_id = odom_id_;
    odom_tf.child_frame_id = body_id_;

  }

private:
  // Main Timer Callback
  void timer_callback()
  {
    update_odom_msgs();
    odom_publisher_->publish(odom_msg);
    tf_broadcaster_->sendTransform(odom_tf);

    update_path_msgs();
    path_publisher_->publish(path_msg_);

  }

  // joint_states Callback
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    turtlelib::WheelPositions wheel_positions;
    wheel_positions.phi_left_ = msg->position.at(0);
    wheel_positions.phi_right_ = msg->position.at(1);

    diff_drive_.update_configuration(wheel_positions);
    turtlelib::Configuration configuration = diff_drive_.get_configuration();

    turtlelib::Twist2D twist = diff_drive_.get_twist();

    update_odom_msgs(configuration, twist);

  }

  // init_pose Callback
  void init_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    turtlelib::Configuration configuration;
    configuration.x_ = request->x;
    configuration.y_ = request->y;
    configuration.theta_ = request->theta;

    diff_drive_.set_configuration(configuration);

    turtlelib::Twist2D twist;
    twist.x = 0.0;
    twist.y = 0.0;
    twist.omega = 0.0;

    update_odom_msgs(configuration, twist);

    odom_publisher_->publish(odom_msg);
    tf_broadcaster_->sendTransform(odom_tf);

  }


  // Shared Pointers for ROS2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr init_pose_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;


  // Private Variables
  std::string body_id_;
  std::string odom_id_;
  std::string wheel_left_;
  std::string wheel_right_;
  double wheel_radius_;
  double track_width_;

  turtlelib::DiffDrive diff_drive_;

  nav_msgs::msg::Odometry odom_msg;

  geometry_msgs::msg::PoseWithCovariance pose_cov;
  geometry_msgs::msg::Pose pose;

  tf2::Quaternion q;

  geometry_msgs::msg::TransformStamped odom_tf;

  nav_msgs::msg::Path path_msg_;

  void update_odom_msgs(turtlelib::Configuration configuration, turtlelib::Twist2D twist)
  {

    odom_msg.header.stamp = get_clock()->now();

    pose.position.x = configuration.x_;
    pose.position.y = configuration.y_;
    pose.position.z = 0.0;

    q.setRPY(0.0, 0.0, configuration.theta_);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    pose_cov.pose = pose;
    odom_msg.pose = pose_cov;

    odom_msg.twist.twist.linear.x = twist.x;
    odom_msg.twist.twist.linear.y = twist.y;
    odom_msg.twist.twist.angular.z = twist.omega;


    odom_tf.header.stamp = get_clock()->now();

    odom_tf.transform.translation.x = pose.position.x;
    odom_tf.transform.translation.y = pose.position.y;
    odom_tf.transform.translation.z = pose.position.z;
    odom_tf.transform.rotation.x = pose.orientation.x;
    odom_tf.transform.rotation.y = pose.orientation.y;
    odom_tf.transform.rotation.z = pose.orientation.z;
    odom_tf.transform.rotation.w = pose.orientation.w;

  }

  void update_odom_msgs()
  {

    odom_msg.header.stamp = get_clock()->now();

    odom_tf.header.stamp = get_clock()->now();

  }

  void update_path_msgs()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = odom_id_;
    pose.pose.position.x = odom_tf.transform.translation.x;
    pose.pose.position.y = odom_tf.transform.translation.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = odom_tf.transform.rotation.x;
    pose.pose.orientation.y = odom_tf.transform.rotation.y;
    pose.pose.orientation.z = odom_tf.transform.rotation.z;
    pose.pose.orientation.w = odom_tf.transform.rotation.w;

    path_msg_.header.stamp = get_clock()->now();
    path_msg_.header.frame_id = odom_id_;
    path_msg_.poses.push_back(pose);
  }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
