/// \file slam.cpp
/// \brief This node performs SLAM using the EKF algorithm
/// \author Megan Black (blackm159)
/// PARAMETERS:
/// none
/// PUBLISHERS:
/// /green/path (nav_msgs::msg::Path): Publishes the path of the robot
/// /green/obstacles (visualization_msgs::msg::MarkerArray): Publishes the landmarks found in the laser scan data
/// SUBSCRIBERS:
/// /odom (nav_msgs::msg::Odometry): The odometry data
/// /landmarks (visualization_msgs::msg::MarkerArray): The landmarks found in the laser scan data
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
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include <armadillo>


using namespace std::chrono_literals;

class SLAM : public rclcpp::Node
{
public:
  SLAM()
  : Node("slam")
  {

    // Subscribers
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SLAM::odom_callback, this, std::placeholders::_1));
    // sensor_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    //   "/fake_sensor", 10, std::bind(&SLAM::sensor_callback_fake_sensors, this, std::placeholders::_1));
    sensor_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/landmarks", 10, std::bind(&SLAM::sensor_callback, this, std::placeholders::_1));


    // Publishers
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("green/path", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "green/obstacles", 10);

    // Broadcasters
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer
    timer_ =
      this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 5.0),
      std::bind(&SLAM::timer_callback, this));

  }

private:
  // Main Timer Callback
  void timer_callback()
  {

    // mu = mu_bar;
    // sigma = sigma_bar;

    turtlelib::Vector2D r;
    r.x = mu.at(1);
    r.y = mu.at(2);
    T_mr = turtlelib::Transform2D(r, mu.at(0));

    T_mo = T_mr * T_or.inv();

    update_map_msg();
    update_odom_msgs();

    tf_broadcaster_->sendTransform(T_mo_msg);
    tf_broadcaster_->sendTransform(T_or_msg);

    path_publisher_->publish(path_msg);

    update_green_markers();
    marker_publisher_->publish(green_marker_msg);

  }

  // Odometry Callback
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    turtlelib::Vector2D p_or;
    p_or.x = msg->pose.pose.position.x;
    p_or.y = msg->pose.pose.position.y;
    T_or = turtlelib::Transform2D(p_or, get_yaw(msg->pose.pose.orientation));

    // get the change in position
    double dx = msg->pose.pose.position.x - prev_odom_pose_.position.x;
    double dy = msg->pose.pose.position.y - prev_odom_pose_.position.y;
    double dtheta = turtlelib::normalize_angle(
      get_yaw(msg->pose.pose.orientation) -
      get_yaw(prev_odom_pose_.orientation));

    // save current pose as previous pose
    prev_odom_pose_ = msg->pose.pose;

    // update the motion model
    predict(dx, dy, dtheta);

    // mu = mu_bar;
    // sigma = sigma_bar;

  }

  // Sensor Callback
  void sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {

    int num_updates = 0;


    for (int i = 0; i < int(msg->markers.size()); i++) {

      // Find ID of measurement
      auto temp_neighbors = neighbors;
      temp_neighbors.push_back(neighbors.size());
      arma::vec new_loc = {msg->markers.at(i).pose.position.x, msg->markers.at(i).pose.position.y};
      arma::vec z_i = convert_r_phi(new_loc.at(0), new_loc.at(1));
      double min_dist = distance_threshold_;
      int min_id = int(neighbors.size());

      for (int n = 0; n < int(neighbors.size()); n++) {
        arma::vec obstacle_loc =
        {mu_bar.at(get_mu_index(neighbors.at(n))) - mu_bar.at(1),
          mu_bar.at(get_mu_index(neighbors.at(n)) + 1) - mu_bar.at(2)};

        double sigma_x = obstacle_loc.at(0); // - mu_bar.at(1);
        double sigma_y = obstacle_loc.at(1); // - mu_bar.at(2);
        double d = sigma_x * sigma_x + sigma_y * sigma_y;

        arma::mat H = arma::zeros(2, N);

        H.at(0, 1) = -sigma_x / sqrt(d);
        H.at(0, 2) = -sigma_y / sqrt(d);
        H.at(1, 0) = -1.0;
        H.at(1, 1) = sigma_y / d;
        H.at(1, 2) = -sigma_x / d;

        H.at(0, get_mu_index(n)) = sigma_x / sqrt(d);
        H.at(0, get_mu_index(n) + 1) = sigma_y / sqrt(d);
        H.at(1, get_mu_index(n)) = -sigma_y / d;
        H.at(1, get_mu_index(n) + 1) = sigma_x / d;

        arma::mat psi = H * sigma_bar * H.t() + R;

        arma::vec z_hat = convert_r_phi(sigma_x, sigma_y);
        z_hat.at(1) = turtlelib::normalize_angle(z_hat.at(1) - mu_bar.at(0));

        arma::vec z_diff = arma::zeros(2);
        z_diff.at(0) = z_i.at(0) - z_hat.at(0);
        z_diff.at(1) = turtlelib::normalize_angle(z_i.at(1) - z_hat.at(1));

        arma::vec mahalanobis = z_diff.t() * psi.i() * z_diff;


        if (mahalanobis.at(0) < min_dist) {
          min_dist = mahalanobis.at(0);
          min_id = neighbors.at(n);
        }

      }

      if (is_in_neighbors(min_id)) {
        update(min_id, new_loc.at(0), new_loc.at(1));
        num_updates++;
        // } else if (add_landmark_counter_ < 5) {
        //   add_landmark_counter_++;

      } else {
        neighbors.push_back(min_id);

        // RCLCPP_INFO_STREAM(get_logger(), "Adding: " << min_id << " x: " << new_loc.at(0) << " y: " << new_loc.at(1));
        // RCLCPP_INFO_STREAM(get_logger(), "Neighbors: " << neighbors.size());

        // sort neighbors
        // std::sort(neighbors.begin(), neighbors.end());

        mu_bar.at(get_mu_index(min_id)) = mu_bar.at(1) + new_loc.at(0);
        mu_bar.at(get_mu_index(min_id) + 1) = mu_bar.at(2) + new_loc.at(1);

        mu.at(get_mu_index(min_id)) = mu_bar.at(get_mu_index(min_id));
        mu.at(get_mu_index(min_id) + 1) = mu_bar.at(get_mu_index(min_id));


        // add_landmark_counter_ = 0;
      }

    }

    if (num_updates == 0) {
      mu = mu_bar;
      sigma = sigma_bar;
    }

  }


  // void sensor_callback_fake_sensors(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  // {

  //   int num_updates = 0;

  //   for (int i = 0; i < int(msg->markers.size()); i++) {
  //     if (msg->markers[i].action == visualization_msgs::msg::Marker::ADD) {
  //       if (is_in_neighbors(msg->markers[i].id)) {
  //         update(
  //           msg->markers[i].id, msg->markers[i].pose.position.x,
  //           msg->markers[i].pose.position.y);
  //         num_updates++;
  //       } else {
  //         neighbors.push_back(msg->markers[i].id);

  //         // sort neighbors
  //         std::sort(neighbors.begin(), neighbors.end());

  //         mu_bar.at(get_mu_index(msg->markers[i].id)) = mu_bar.at(1) +
  //           msg->markers[i].pose.position.x;
  //         mu_bar.at(get_mu_index(msg->markers[i].id) + 1) = mu_bar.at(2) +
  //           msg->markers[i].pose.position.y;

  //         mu.at(get_mu_index(msg->markers[i].id)) = mu_bar.at(get_mu_index(msg->markers[i].id));
  //         mu.at(get_mu_index(msg->markers[i].id) + 1) = mu_bar.at(get_mu_index(msg->markers[i].id));
  //       }
  //     }
  //   }

  //   if (num_updates == 0) {
  //     mu = mu_bar;
  //     sigma = sigma_bar;
  //   }

  // }


  void predict(double dx, double dy, double dtheta)
  {

    arma::vec odom = arma::zeros(N);
    odom.at(0) = dtheta;
    odom.at(1) = dx;
    odom.at(2) = dy;

    // Predict the new mean for the robot location
    mu_bar = mu_bar + odom;

    // Predict the new covariance for the robot location
    arma::mat At = arma::zeros(N, N);
    At.at(1, 0) = -dy;
    At.at(2, 0) = dx;

    arma::mat A = arma::eye(N, N) + At;

    arma::mat Q_bar = arma::zeros(N, N);
    Q_bar.at(0, 0) = Q.at(0, 0);
    Q_bar.at(1, 1) = Q.at(1, 1);
    Q_bar.at(2, 2) = Q.at(2, 2);

    sigma_bar = A * sigma_bar * A.t() + Q_bar;

  }

  void update(int id, double x, double y)
  {

    arma::vec z = convert_r_phi(x, y);

    double sigma_x = mu_bar.at(get_mu_index(id)) - mu_bar.at(1);
    double sigma_y = mu_bar.at(get_mu_index(id) + 1) - mu_bar.at(2);
    double d = sigma_x * sigma_x + sigma_y * sigma_y;

    arma::mat H = arma::zeros(2, N);

    H.at(0, 1) = -sigma_x / sqrt(d);
    H.at(0, 2) = -sigma_y / sqrt(d);
    H.at(1, 0) = -1.0;
    H.at(1, 1) = sigma_y / d;
    H.at(1, 2) = -sigma_x / d;

    H.at(0, get_mu_index(id)) = sigma_x / sqrt(d);
    H.at(0, get_mu_index(id) + 1) = sigma_y / sqrt(d);
    H.at(1, get_mu_index(id)) = -sigma_y / d;
    H.at(1, get_mu_index(id) + 1) = sigma_x / d;

    arma::mat K = arma::zeros(N, 2);

    K = sigma_bar * H.t() * arma::inv(H * sigma_bar * H.t() + R);

    auto z_hat =
      convert_r_phi(
      mu_bar.at(get_mu_index(id)) - mu_bar.at(1), mu_bar.at(
        get_mu_index(
          id) + 1) - mu_bar.at(2));
    z_hat.at(1) = turtlelib::normalize_angle(z_hat.at(1) - mu_bar.at(0));
    arma::vec z_diff = arma::zeros(2);
    z_diff.at(0) = z.at(0) - z_hat.at(0);
    z_diff.at(1) = turtlelib::normalize_angle(z.at(1) - z_hat.at(1));
    mu = mu_bar + K * (z_diff);
    // mu = mu_bar + K * (z - H * mu_bar);

    sigma = (arma::eye(N, N) - K * H) * sigma_bar;

    mu_bar = mu;
    sigma_bar = sigma;

  }

  arma::vec convert_r_phi(double x, double y)
  {
    double r = sqrt(x * x + y * y);
    double phi = atan2(y, x);
    arma::vec z = {r, phi};
    return z;
  }


  int get_mu_index(int id)
  {
    int ind = 0;

    ind = 3 + 2 * id;

    return ind;
  }

  bool is_in_neighbors(int id)
  {
    for (int i = 0; i < int(neighbors.size()); i++) {
      if (neighbors.at(i) == id) {
        return true;
      }
    }
    return false;
  }


  double get_yaw(geometry_msgs::msg::Quaternion q)
  {
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return yaw;
  }


  void update_map_msg()
  {

    T_mo_msg.header.stamp = this->now();
    T_mo_msg.header.frame_id = "map";
    T_mo_msg.child_frame_id = "green/odom";
    T_mo_msg.transform.translation.x = T_mo.translation().x;
    T_mo_msg.transform.translation.y = T_mo.translation().y;
    T_mo_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, T_mo.rotation());
    T_mo_msg.transform.rotation.x = q.x();
    T_mo_msg.transform.rotation.y = q.y();
    T_mo_msg.transform.rotation.z = q.z();
    T_mo_msg.transform.rotation.w = q.w();

  }

  void update_odom_msgs()
  {

    T_or_msg.header.stamp = this->now();
    T_or_msg.header.frame_id = "green/odom";
    T_or_msg.child_frame_id = "green/base_footprint";
    T_or_msg.transform.translation.x = T_or.translation().x;
    T_or_msg.transform.translation.y = T_or.translation().y;
    T_or_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, T_or.rotation());
    T_or_msg.transform.rotation.x = q.x();
    T_or_msg.transform.rotation.y = q.y();
    T_or_msg.transform.rotation.z = q.z();
    T_or_msg.transform.rotation.w = q.w();

    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = T_mr.translation().x;
    pose.pose.position.y = T_mr.translation().y;
    pose.pose.position.z = 0.0;
    // pose.pose.orientation = T_or_msg.transform.rotation;

    path_msg.poses.push_back(pose);

  }

  void update_green_markers()
  {

    green_marker_msg.markers.clear();

    visualization_msgs::msg::Marker green_marker;

    for (int i = 0; i < int(neighbors.size()); i++) {
      green_marker.header.frame_id = "map";
      green_marker.header.stamp = this->now();
      green_marker.ns = "green";
      green_marker.id = neighbors.at(i);
      green_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      green_marker.action = visualization_msgs::msg::Marker::ADD;
      green_marker.pose.position.x = mu.at(get_mu_index(neighbors.at(i)));
      green_marker.pose.position.y = mu.at(get_mu_index(neighbors.at(i)) + 1);
      green_marker.pose.position.z = 0.1 / 2.0;
      green_marker.pose.orientation.x = 0.0;
      green_marker.pose.orientation.y = 0.0;
      green_marker.pose.orientation.z = 0.0;
      green_marker.pose.orientation.w = 1.0;
      green_marker.scale.x = 0.1;
      green_marker.scale.y = 0.1;
      green_marker.scale.z = 0.1;
      green_marker.color.r = 0.0;
      green_marker.color.g = 1.0;
      green_marker.color.b = 0.0;
      green_marker.color.a = 1.0;
      green_marker_msg.markers.push_back(green_marker);
    }

  }


  // Shared Pointers for ROS2
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;


  // Private Variables
  turtlelib::Transform2D T_mr;
  turtlelib::Transform2D T_or;
  turtlelib::Transform2D T_mo;

  int N = 10;

  geometry_msgs::msg::Pose prev_odom_pose_;

  // arma::vec mu_bar = arma::zeros(9);
  // arma::vec mu = arma::zeros(9);

  arma::vec mu_bar = arma::zeros(N);
  arma::vec mu = arma::zeros(N);

  // arma::mat sigma_bar = arma::eye(9, 9);
  // arma::mat sigma = arma::eye(9, 9);

  arma::mat sigma_bar = arma::eye(N, N);
  arma::mat sigma = arma::eye(N, N);

  // arma::mat A = arma::zeros(9, 9);

  arma::mat Q = arma::eye(3, 3) * 0.05;

  arma::mat R = arma::eye(2, 2) * 1.0;

  geometry_msgs::msg::TransformStamped T_mo_msg;
  geometry_msgs::msg::TransformStamped T_or_msg;

  nav_msgs::msg::Path path_msg;

  visualization_msgs::msg::MarkerArray green_marker_msg;

  std::default_random_engine generator_;

  double w = 0.1;
  double v = 0.1;

  std::vector<int> neighbors = {};

  double distance_threshold_ = 0.5;

  int add_landmark_counter_ = 0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAM>());
  rclcpp::shutdown();
  return 0;
}
