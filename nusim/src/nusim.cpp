/// \file nusim.cpp
/// \brief This is a ROS2 node that simulates a differential drive robot with a LIDAR sensor
/// \author Megan Black (blackm159)
/// PARAMETERS:
///   draw_only - Whether to only draw the arena or to simulate the robot
///   arena_x_length - The length of the arena in the x direction
///   arena_y_length - The length of the arena in the y direction
///   obstacles/x - The x position of the obstacles
///   obstacles/y - The y position of the obstacles
///   obstacles/r - The radius of the obstacles
///   rate - The rate at which to run the simulation
///   x0 - The initial x position of the robot
///   y0 - The initial y position of the robot
///   theta0 - The initial theta of the robot
///   wheel_radius - The radius of the wheels
///   track_width - The track width of the robot
///   motor_cmd_per_rad_sec - The motor command per radian per second
///   encoder_ticks_per_rad - The encoder ticks per radian
///   collision_radius - The collision radius of the robot
///   input_noise - The noise to add to the input
///   slip_fraction - The fraction of slip to add to the input
///   basic_sensor_variance - The basic sensor variance
///   max_range - The maximum range of the LIDAR sensor
///   min_range - The minimum range of the LIDAR sensor
///   angle_increment - The angle increment of the LIDAR sensor
///   number_samples - The number of samples of the LIDAR sensor
///   resolution - The resolution of the LIDAR sensor
///   noise_level - The noise level of the LIDAR sensor
/// PUBLISHERS:
///   ~/timestep - Publishes the current timestep
///   sensor_data - Publishes the sensor data
///   joint_states - Publishes the joint states
///   path - Publishes the path of the robot
///   /fake_sensor - Publishes the fake sensor data
///   laser - Publishes the laser scan
/// SUBSCRIBERS:
///   wheel_cmd - Subscribes to the wheel commands
/// SERVICES:
///   ~/reset - Resets the simulation
///   ~/teleport - Teleports the robot to a new location
/// CLIENTS:
///   None

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

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nusim/srv/teleport.hpp"

#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include <random>


using namespace std::chrono_literals;

class NUSIM : public rclcpp::Node
{
public:
  NUSIM()
  : Node("nusim"), count_(0)
  {
    // Parameters
    declare_parameter("draw_only", "false");
    draw_only = get_parameter("draw_only").as_string();

    declare_parameter("arena_x_length", 5.0);
    arena_x_length = get_parameter("arena_x_length").as_double();
    declare_parameter("arena_y_length", 5.0);
    arena_y_length = get_parameter("arena_y_length").as_double();

    declare_parameter("obstacles/x", std::vector<double>());
    obstacles_x = get_parameter("obstacles/x").as_double_array();
    declare_parameter("obstacles/y", std::vector<double>());
    obstacles_y = get_parameter("obstacles/y").as_double_array();
    if (obstacles_x.size() != obstacles_y.size()) {
      RCLCPP_ERROR(get_logger(), "Obstacles x and y must be the same size");
      rclcpp::shutdown();
    }
    declare_parameter("obstacles/r", 0.25);
    obstacles_r = get_parameter("obstacles/r").as_double();

    // Some Publishers
    rclcpp::QoS markerQoS(10);
    markerQoS.transient_local();
    wall_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      markerQoS);
    obstacle_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      markerQoS);

    // Initialize the walls
    for (int i = 0; i < 4; i++) {
      wall.header.frame_id = "nusim/world";
      wall.header.stamp = get_clock()->now();
      wall.ns = "nusim";
      wall.id = i;
      wall.type = visualization_msgs::msg::Marker::CUBE;
      wall.action = visualization_msgs::msg::Marker::ADD;
      wall.pose.position.z = wall_height / 2.0;
      wall.pose.orientation.x = 0.0;
      wall.pose.orientation.y = 0.0;
      wall.pose.orientation.z = 0.0;
      wall.pose.orientation.w = 1.0;
      wall.scale.z = wall_height;
      wall.color.a = 1.0;
      wall.color.r = 1.0;
      wall.color.g = 0.0;
      wall.color.b = 0.5;
      if (i == 0) {
        wall.pose.position.x = arena_x_length / 2.0 + wall_thickness / 2.0;
        wall.pose.position.y = 0;
        wall.scale.x = wall_thickness;
        wall.scale.y = arena_y_length;
      } else if (i == 1) {
        wall.pose.position.x = -arena_x_length / 2.0 - wall_thickness / 2.0;
        wall.pose.position.y = 0;
        wall.scale.x = wall_thickness;
        wall.scale.y = arena_y_length;
      } else if (i == 2) {
        wall.pose.position.x = 0;
        wall.pose.position.y = arena_y_length / 2.0 + wall_thickness / 2.0;
        wall.scale.x = arena_x_length;
        wall.scale.y = wall_thickness;
      } else if (i == 3) {
        wall.pose.position.x = 0;
        wall.pose.position.y = -arena_y_length / 2.0 - wall_thickness / 2.0;
        wall.scale.x = arena_x_length;
        wall.scale.y = wall_thickness;
      }
      walls.markers.push_back(wall);
    }
    // Publish the walls
    wall_publisher_->publish(walls);

    // Initialize the obstacles
    for (int i = 0; i < int(obstacles_x.size()); i++) {
      obstacle.header.frame_id = "nusim/world";
      obstacle.header.stamp = get_clock()->now();
      obstacle.ns = "nusim";
      obstacle.id = i;
      obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle.action = visualization_msgs::msg::Marker::ADD;
      obstacle.pose.position.x = obstacles_x.at(i);
      obstacle.pose.position.y = obstacles_y.at(i);
      obstacle.pose.position.z = obstacles_height / 2.0;
      obstacle.pose.orientation.x = 0.0;
      obstacle.pose.orientation.y = 0.0;
      obstacle.pose.orientation.z = 0.0;
      obstacle.pose.orientation.w = 1.0;
      obstacle.scale.z = obstacles_height;
      obstacle.scale.x = obstacles_r * 2.0;
      obstacle.scale.y = obstacles_r * 2.0;
      obstacle.color.a = 1.0;
      obstacle.color.r = 1.0;
      obstacle.color.g = 0.0;
      obstacle.color.b = 0.5;

      obstacles.markers.push_back(obstacle);
    }
    // Publish the obstacles
    obstacle_publisher_->publish(obstacles);


    if (draw_only == "false") {

      declare_parameter("rate", 200.0);
      rate_ = get_parameter("rate").as_double();

      declare_parameter("x0", 0.0);
      x0_ = get_parameter("x0").as_double();
      declare_parameter("y0", 0.0);
      y0_ = get_parameter("y0").as_double();
      declare_parameter("theta0", 0.0);
      theta0_ = get_parameter("theta0").as_double();

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

      declare_parameter("input_noise", 5.0);
      input_noise_ = get_parameter("input_noise").as_double();

      declare_parameter("slip_fraction", 0.5);
      slip_fraction_ = get_parameter("slip_fraction").as_double();

      declare_parameter("basic_sensor_variance", 0.01);
      basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();

      declare_parameter("max_range", 3.5);
      max_range_ = get_parameter("max_range").as_double();

      declare_parameter("min_range", 0.12);
      min_range_ = get_parameter("min_range").as_double();

      declare_parameter("angle_increment", 0.01745);
      angle_increment_ = get_parameter("angle_increment").as_double();

      declare_parameter("number_samples", 360);
      number_samples_ = get_parameter("number_samples").as_int();

      declare_parameter("resolution", 0.01745);
      resolution_ = get_parameter("resolution").as_double();

      declare_parameter("noise_level", 0.005);
      noise_level_ = get_parameter("noise_level").as_double();


      // Publishers
      publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
        "sensor_data",
        10);
      joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states",
        10);
      path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "path",
        10);
      fake_obstacle_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor",
        10);
      laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "laser",
        10);

      // Subscribers
      wheel_cmd_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "wheel_cmd",
        10,
        std::bind(&NUSIM::wheel_cmd_callback, this, std::placeholders::_1));

      // Services
      reset_service_ = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&NUSIM::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
      teleport_service_ = create_service<nusim::srv::Teleport>(
        "~/teleport",
        std::bind(&NUSIM::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

      // Transform Broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Main Timer
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_), std::bind(&NUSIM::timer_callback, this));


      // Initialize the transform at robot starting location
      updateTransform(x0_, y0_, theta0_);
      tf_broadcaster_->sendTransform(t_);

      diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);

      configuration0_.x_ = x0_;
      configuration0_.y_ = y0_;
      configuration0_.theta_ = theta0_;
      diff_drive_.set_configuration(configuration0_);

      sensor_data_msg_.left_encoder = 0;
      sensor_data_msg_.right_encoder = 0;

      // wheel_velocities.phi_left_ = 0.0;
      // wheel_velocities.phi_right_ = 0.0;

      wheel_encoders.phi_left_ = 0.0;
      wheel_encoders.phi_right_ = 0.0;

      wheel_positions_.phi_left_ = 0.0;
      wheel_positions_.phi_right_ = 0.0;

    }

  }

private:
  // Main Timer Callback
  void timer_callback()
  {
    // Publish the timestep
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++ *1.0 / rate_ * 10e3;
    publisher_->publish(message);

    updateTransform();
    tf_broadcaster_->sendTransform(t_);

    updateJointState();
    joint_state_publisher_->publish(joint_state_msg_);

    sensor_data_publisher_->publish(sensor_data_msg_);

    updatePath();
    path_publisher_->publish(path_msg_);

    if (timestep_ - old_timestep_ > 1.0 / 5.0 * rate_) {
      old_timestep_ = timestep_;

      std::normal_distribution<double> obstacle_noise_(0.0, basic_sensor_variance_);
      // std::normal_distribution<double> obstacle_noise_(0.0, 0.0);

      fake_obstacles_.markers.clear();

      for (int i = 0; i < int(obstacles_x.size()); i++) {
        fake_obstacle_.header.frame_id = "red/base_footprint";
        fake_obstacle_.header.stamp = get_clock()->now();
        fake_obstacle_.ns = "fake_sensor";
        fake_obstacle_.id = i;
        fake_obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
        // fake_obstacle_.action = visualization_msgs::msg::Marker::ADD;
        double x_diff = obstacles_x.at(i) - diff_drive_.get_configuration().x_;
        double y_diff = obstacles_y.at(i) - diff_drive_.get_configuration().y_;
        double theta = diff_drive_.get_configuration().theta_;
        fake_obstacle_.pose.position.x = x_diff * cos(theta) + y_diff * sin(theta) +
          obstacle_noise_(generator_);
        fake_obstacle_.pose.position.y = y_diff * cos(theta) - x_diff * sin(theta) +
          obstacle_noise_(generator_);
        fake_obstacle_.pose.position.z = obstacles_height / 2.0;
        fake_obstacle_.pose.orientation.x = 0.0;
        fake_obstacle_.pose.orientation.y = 0.0;
        fake_obstacle_.pose.orientation.z = 0.0;
        fake_obstacle_.pose.orientation.w = 1.0;
        fake_obstacle_.scale.z = obstacles_height;
        fake_obstacle_.scale.x = obstacles_r * 2.0;
        fake_obstacle_.scale.y = obstacles_r * 2.0;
        fake_obstacle_.color.a = 1.0;
        fake_obstacle_.color.r = 1.0;
        fake_obstacle_.color.g = 1.0;
        fake_obstacle_.color.b = 0.0;

        auto dist_ = std::sqrt(
          std::pow(fake_obstacle_.pose.position.x, 2) +
          std::pow(fake_obstacle_.pose.position.y, 2));

        if (dist_ < max_range_) {
          fake_obstacle_.action = visualization_msgs::msg::Marker::ADD;
        } else {
          fake_obstacle_.action = visualization_msgs::msg::Marker::DELETE;
        }

        fake_obstacles_.markers.push_back(fake_obstacle_);
      }

      fake_obstacle_publisher_->publish(fake_obstacles_);

      updateLaserScan();
      laser_scan_publisher_->publish(scan);


    }

  }

  // Wheel Command Callback
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    turtlelib::WheelPositions wheel_velocities;
    wheel_velocities.phi_left_ = double(msg->left_velocity) * motor_cmd_per_rad_sec_;
    wheel_velocities.phi_right_ = double(msg->right_velocity) * motor_cmd_per_rad_sec_;

    std::normal_distribution<double> noise_{0.0, input_noise_};

    // Add noise to the wheel velocities
    if (turtlelib::almost_equal(wheel_velocities.phi_left_, 0.0, 1e-3)) {
      wheel_velocities.phi_left_ = 0.0;
    } else {
      wheel_velocities.phi_left_ = wheel_velocities.phi_left_ + noise_(generator_);
    }

    if (turtlelib::almost_equal(wheel_velocities.phi_right_, 0.0, 1e-3)) {
      wheel_velocities.phi_right_ = 0.0;
    } else {
      wheel_velocities.phi_right_ = wheel_velocities.phi_right_ + noise_(generator_);
    }


    wheel_encoders.phi_left_ = wheel_encoders.phi_left_ + wheel_velocities.phi_left_ *
      encoder_ticks_per_rad_ / rate_;
    wheel_encoders.phi_right_ = wheel_encoders.phi_right_ + wheel_velocities.phi_right_ *
      encoder_ticks_per_rad_ / rate_;

    updateSensorData(wheel_encoders);


    std::uniform_real_distribution<double> slip_(-slip_fraction_, slip_fraction_);

    // Add slip to the wheel velocities
    wheel_velocities.phi_left_ = wheel_velocities.phi_left_ * (1.0 + slip_(generator_));
    wheel_velocities.phi_right_ = wheel_velocities.phi_right_ * (1.0 + slip_(generator_));

    wheel_positions_.phi_left_ = wheel_positions_.phi_left_ + wheel_velocities.phi_left_ / rate_;
    wheel_positions_.phi_right_ = wheel_positions_.phi_right_ + wheel_velocities.phi_right_ / rate_;

    diff_drive_.update_configuration(wheel_positions_);

    updateJointState(wheel_positions_);

    turtlelib::Configuration configuration = diff_drive_.get_configuration();

    for (int i = 0; i < int(obstacles_x.size()); i++) {
      // auto fake_obstacle_x = fake_obstacles_.markers[i].pose.position.x;
      // auto fake_obstacle_y = fake_obstacles_.markers[i].pose.position.y;

      auto fake_obstacle_x = obstacles_x.at(i);
      auto fake_obstacle_y = obstacles_y.at(i);

      if (std::sqrt(
          std::pow(fake_obstacle_x - configuration.x_, 2) +
          std::pow(fake_obstacle_y - configuration.y_, 2)) < collision_radius_ + obstacles_r)
      {

        // RCLCPP_INFO(get_logger(), "Collision detected");

        turtlelib::Vector2D collision_vector;
        collision_vector.x = fake_obstacle_x - configuration.x_;
        collision_vector.y = fake_obstacle_y - configuration.y_;

        turtlelib::Vector2D collision_vector_unit = turtlelib::normalize(collision_vector);

        configuration.x_ = fake_obstacle_x - collision_vector_unit.x *
          (collision_radius_ + obstacles_r);
        configuration.y_ = fake_obstacle_y - collision_vector_unit.y *
          (collision_radius_ + obstacles_r);

        diff_drive_.set_configuration(configuration);

      }

    }

    updateTransform(configuration.x_, configuration.y_, configuration.theta_);

  }

  // Reset Service Callback
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    updateTransform(x0_, y0_, theta0_);
    RCLCPP_INFO(get_logger(), "Resetting timestep");
  }

  // Teleport Service Callback
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    updateTransform(request->x, request->y, request->theta);
    tf_broadcaster_->sendTransform(t_);
  }

  // Update the transform, given a new x, y, and theta (in radians)
  void updateTransform(double x, double y, double theta)
  {
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
    t_.header.stamp = get_clock()->now();

    t_.transform.translation.x = x;
    t_.transform.translation.y = y;
    t_.transform.translation.z = 0.0;

    q_.setRPY(0, 0, theta);

    t_.transform.rotation.x = q_.x();
    t_.transform.rotation.y = q_.y();
    t_.transform.rotation.z = q_.z();
    t_.transform.rotation.w = q_.w();

  }

  // Update the transform, keep same x, y, and theta (in radians)
  void updateTransform()
  {
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
    t_.header.stamp = get_clock()->now();

  }

  // Update the joint state
  void updateJointState(turtlelib::WheelPositions wheel_velocities)
  {
    joint_state_msg_.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state_msg_.position = {wheel_velocities.phi_left_, wheel_velocities.phi_right_};
    joint_state_msg_.header.stamp = get_clock()->now();
  }

  // Update the joint state
  void updateJointState()
  {
    joint_state_msg_.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state_msg_.header.stamp = get_clock()->now();

    try {
      joint_state_msg_.position =
      {joint_state_msg_.position.at(0), joint_state_msg_.position.at(1)};
    } catch (const std::out_of_range & e) {
      joint_state_msg_.position = {0.0, 0.0};
      RCLCPP_INFO(get_logger(), "Joint state position not initialized");
    }
  }

  // Update the sensor data
  void updateSensorData(turtlelib::WheelPositions wheel_velocities)
  {
    sensor_data_msg_.left_encoder = wheel_velocities.phi_left_;
    sensor_data_msg_.right_encoder = wheel_velocities.phi_right_;
  }

  void updatePath()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = t_.transform.translation.x;
    pose.pose.position.y = t_.transform.translation.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = t_.transform.rotation.x;
    pose.pose.orientation.y = t_.transform.rotation.y;
    pose.pose.orientation.z = t_.transform.rotation.z;
    pose.pose.orientation.w = t_.transform.rotation.w;

    path_msg_.header.stamp = get_clock()->now();
    path_msg_.header.frame_id = "nusim/world";
    path_msg_.poses.push_back(pose);
  }

  void updateLaserScan()
  {

    scan.ranges.clear();

    scan.header.frame_id = "red/base_scan";
    scan.header.stamp = get_clock()->now();
    scan.angle_min = 0.0;
    scan.angle_max = 2 * turtlelib::PI;
    scan.angle_increment = angle_increment_;
    scan.range_max = max_range_;
    scan.range_min = min_range_;


    for (int i = 0; i < number_samples_; i++) {
      double gamma = i * resolution_;
      double scan_val_ = 2.0 * max_range_;


      turtlelib::Vector2D robot_location;
      robot_location.x = diff_drive_.get_configuration().x_;
      robot_location.y = diff_drive_.get_configuration().y_;
      double robot_theta = diff_drive_.get_configuration().theta_;
      turtlelib::Transform2D T_wr(robot_location, robot_theta);

      turtlelib::Transform2D T_rl(gamma);

      turtlelib::Vector2D laser_end_location;
      laser_end_location.x = max_range_;
      laser_end_location.y = 0.0;
      turtlelib::Transform2D T_lb(laser_end_location);

      turtlelib::Transform2D T_lr = T_rl.inv();
      turtlelib::Transform2D T_rw = T_wr.inv();


      turtlelib::Transform2D T_lw = T_lr * T_rw;

      // Top right
      turtlelib::Point2D wall_point1;
      wall_point1.x = arena_x_length / 2.0;
      wall_point1.y = arena_y_length / 2.0;

      // Bottom Right
      turtlelib::Point2D wall_point2;
      wall_point2.x = arena_x_length / 2.0;
      wall_point2.y = -arena_y_length / 2.0;

      // Transform the wall points to lidar frame
      turtlelib::Point2D wall_point1_lidar = T_lw(wall_point1);
      turtlelib::Point2D wall_point2_lidar = T_lw(wall_point2);


      double m = (wall_point2_lidar.y - wall_point1_lidar.y) /
        (wall_point2_lidar.x - wall_point1_lidar.x);
      double b = wall_point1_lidar.y - m * wall_point1_lidar.x;

      // Check for intersection
      double x = (0 - b) / m;


      if (x >= 0) {
        scan_val_ = x;
      }


      // Top left
      wall_point1.x = -arena_x_length / 2.0;
      wall_point1.y = arena_y_length / 2.0;

      // Bottom left
      wall_point2.x = -arena_x_length / 2.0;
      wall_point2.y = -arena_y_length / 2.0;

      // Transform the wall points to lidar frame
      wall_point1_lidar = T_lw(wall_point1);
      wall_point2_lidar = T_lw(wall_point2);


      m = (wall_point2_lidar.y - wall_point1_lidar.y) / (wall_point2_lidar.x - wall_point1_lidar.x);
      b = wall_point1_lidar.y - m * wall_point1_lidar.x;

      // Check for intersection
      x = (0 - b) / m;

      if (x >= 0) {
        if (x < scan_val_) {
          scan_val_ = x;
        }
      }


      // Top right
      wall_point1.x = arena_x_length / 2.0;
      wall_point1.y = arena_y_length / 2.0;

      // Top left
      wall_point2.x = -arena_x_length / 2.0;
      wall_point2.y = arena_y_length / 2.0;

      // Transform the wall points to lidar frame
      wall_point1_lidar = T_lw(wall_point1);
      wall_point2_lidar = T_lw(wall_point2);


      m = (wall_point2_lidar.y - wall_point1_lidar.y) / (wall_point2_lidar.x - wall_point1_lidar.x);
      b = wall_point1_lidar.y - m * wall_point1_lidar.x;

      // Check for intersection
      x = (0 - b) / m;


      if (x >= 0) {
        if (x < scan_val_) {
          scan_val_ = x;
        }
      }


      // Bottom right
      wall_point1.x = arena_x_length / 2.0;
      wall_point1.y = -arena_y_length / 2.0;

      // Bottom left
      wall_point2.x = -arena_x_length / 2.0;
      wall_point2.y = -arena_y_length / 2.0;

      // Transform the wall points to lidar frame
      wall_point1_lidar = T_lw(wall_point1);
      wall_point2_lidar = T_lw(wall_point2);


      m = (wall_point2_lidar.y - wall_point1_lidar.y) / (wall_point2_lidar.x - wall_point1_lidar.x);
      b = wall_point1_lidar.y - m * wall_point1_lidar.x;

      // Check for intersection
      x = (0 - b) / m;


      if (x >= 0) {
        if (x < scan_val_) {
          scan_val_ = x;
        }
      }

      for (int i = 0; i < int(obstacles_x.size()); i++) {
        turtlelib::Point2D obstacle_point;
        obstacle_point.x = obstacles_x.at(i);
        obstacle_point.y = obstacles_y.at(i);

        turtlelib::Point2D obstacle_point_lidar = T_lw(obstacle_point);

        double obstacle_radius = obstacles_r;

        double x1 = std::sqrt(
          std::pow(
            obstacle_radius,
            2) - std::pow(-obstacle_point_lidar.y, 2)) + obstacle_point_lidar.x;

        double x2 = -std::sqrt(
          std::pow(
            obstacle_radius,
            2) - std::pow(-obstacle_point_lidar.y, 2)) + obstacle_point_lidar.x;

        if (x1 >= 0.0) {
          if (x1 < scan_val_) {
            scan_val_ = x1;
          }
        }

        if (x2 >= 0.0) {
          if (x2 < scan_val_) {
            scan_val_ = x2;
          }
        }

      }
      // turtlelib::Point2D obstacle_point;
      // obstacle_point.x = obstacles_x.at(0);
      // obstacle_point.y = obstacles_y.at(0);

      // turtlelib::Point2D obstacle_point_lidar = T_lw(obstacle_point);

      // double obstacle_radius = obstacles_r;

      // double x1 = std::sqrt(
      //   std::pow(obstacle_radius, 2) - std::pow(-obstacle_point_lidar.y, 2)) + obstacle_point_lidar.x;

      // double x2 = -std::sqrt(
      //   std::pow(obstacle_radius, 2) - std::pow(-obstacle_point_lidar.y, 2)) + obstacle_point_lidar.x;

      // if (x1 >= 0.0) {
      //   if (x1 < scan_val_){
      //     scan_val_ = x1;
      //   }
      // }

      // if (x2 >= 0.0) {
      //   if (x2 < scan_val_){
      //     scan_val_ = x2;
      //   }
      // }

      //Add noise to the scan
      std::normal_distribution<double> lidar_noise_(0.0, noise_level_);
      // std::normal_distribution<double> lidar_noise_(0.0, 0.0);
      scan_val_ = scan_val_ + lidar_noise_(generator_);


      if (scan_val_ > max_range_) {
        scan_val_ = max_range_;
      } else if (scan_val_ < min_range_) {
        scan_val_ = min_range_;
      }


      scan.ranges.push_back(scan_val_);
    }


  }

  // Shared Pointers for ROS2
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Private Variables
  std::string draw_only;
  size_t count_;
  double rate_;
  long unsigned timestep_ = 0;
  double x0_;
  double y0_;
  double theta0_;
  double arena_x_length;
  double arena_y_length;
  double wall_height = 0.25;
  double wall_thickness = 0.1;
  double wheel_radius_;
  double track_width_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double input_noise_;
  double slip_fraction_;
  double basic_sensor_variance_;
  double max_range_;
  double collision_radius_;
  double min_range_;
  double angle_increment_;
  int number_samples_;
  double resolution_;
  double noise_level_;

  turtlelib::Configuration configuration0_;

  std::vector<double> obstacles_x;
  std::vector<double> obstacles_y;
  double obstacles_r;
  double obstacles_height = 0.25;

  geometry_msgs::msg::TransformStamped t_;
  tf2::Quaternion q_;

  visualization_msgs::msg::MarkerArray walls;
  visualization_msgs::msg::Marker wall;

  visualization_msgs::msg::MarkerArray obstacles;
  visualization_msgs::msg::Marker obstacle;

  turtlelib::DiffDrive diff_drive_;

  sensor_msgs::msg::JointState joint_state_msg_;
  nuturtlebot_msgs::msg::SensorData sensor_data_msg_;

  turtlelib::WheelPositions wheel_encoders;
  turtlelib::WheelPositions wheel_positions_;

  nav_msgs::msg::Path path_msg_;

  // std::normal_distribution<double> noise_{0.0, input_noise_};

  std::default_random_engine generator_;

  double old_timestep_ = 0.0;

  visualization_msgs::msg::MarkerArray fake_obstacles_;
  visualization_msgs::msg::Marker fake_obstacle_;

  sensor_msgs::msg::LaserScan scan;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NUSIM>());
  rclcpp::shutdown();
  return 0;
}
