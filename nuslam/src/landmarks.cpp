/// \file landmarks.cpp
/// \brief This node finds clusters of points in the laser scan data and fits circles to them
/// \author Megan Black (blackm159)
/// PARAMETERS:
/// none
/// PUBLISHERS:
/// /landmarks (visualization_msgs::msg::MarkerArray): Publishes the circles found in the laser scan data
/// SUBSCRIBERS:
/// /scan (sensor_msgs::msg::LaserScan): The laser scan data
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
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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
#include "sensor_msgs/msg/laser_scan.hpp"

#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include <armadillo>


using namespace std::chrono_literals;

class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {

    rclcpp::QoS scanQOS(10);
    scanQOS.best_effort();

    // Subscribers
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", scanQOS, std::bind(&Landmarks::laser_callback, this, std::placeholders::_1));


    rclcpp::QoS markerQoS(10);
    markerQoS.transient_local();

    // Publishers
    landmarks_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/landmarks", markerQoS);

  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser_msg_ = *msg;

    find_clusters();

    circle_regression();

    visualize_clusters();

  }

  void find_clusters()
  {

    std::vector<std::vector<arma::vec>> temp_clusters;
    arma::vec loc_0 = convert_x_y(laser_msg_.ranges.at(0), laser_msg_.angle_min);
    arma::vec loc_1 = {0.0, 0.0};
    // int cluster_ind = 0;
    std::vector<arma::vec> temp_cluster;
    temp_cluster.push_back(loc_0);


    // check if the 2 points are within a certain distance
    for (int i = 1; i < int(laser_msg_.ranges.size()); i++) {
      loc_1 = convert_x_y(
        laser_msg_.ranges.at(
          i), laser_msg_.angle_min + i * laser_msg_.angle_increment);

      if (arma::norm(loc_1 - loc_0) < distance_threshold_) {
        temp_cluster.push_back(loc_1);
      } else {
        temp_clusters.push_back(temp_cluster);
        temp_cluster.clear();
        temp_cluster.push_back(loc_1);
      }

      loc_0 = loc_1;
    }

    loc_1 = convert_x_y(laser_msg_.ranges.at(0), laser_msg_.angle_min);
    if (arma::norm(loc_1 - loc_0) < distance_threshold_) {
      temp_clusters.at(0).insert(
        temp_clusters.at(0).end(), temp_cluster.begin(),
        temp_cluster.end());
    }

    clusters_.clear();

    for (int i = 0; i < int(temp_clusters.size()); i++) {
      if (temp_clusters.at(i).size() > 3 && temp_clusters.at(i).size() < 25) {
        clusters_.push_back(temp_clusters.at(i));
      }
    }

  }


  void circle_regression()
  {

    // // create fake data
    // std::vector<std::vector<arma::vec>> temp_clusters;
    // std::vector<arma::vec> temp_cluster;
    // arma::vec temp_pt = {0.0, 0.0};

    // temp_pt = {1.0, 7.0};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {2.0, 6.0};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {5.0, 8.0};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {7.0, 7.0};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {9.0, 5.0};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {3.0, 7.0};
    // temp_cluster.push_back(temp_pt);

    // temp_clusters.push_back(temp_cluster);

    // temp_cluster.clear();

    // temp_pt = {-1.0, 0.0};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {-0.3, -0.06};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {0.3, 0.1};
    // temp_cluster.push_back(temp_pt);
    // temp_pt = {1.0, 0.0};
    // temp_cluster.push_back(temp_pt);

    // temp_clusters.push_back(temp_cluster);

    // clusters_ = temp_clusters;

    circle_params_.clear();

    for (int i = 0; i < int(clusters_.size()); i++) {
      arma::vec centroid = {0.0, 0.0};

      for (int j = 0; j < int(clusters_.at(i).size()); j++) {
        centroid.at(0) = centroid.at(0) + clusters_.at(i).at(j).at(0);
        centroid.at(1) = centroid.at(1) + clusters_.at(i).at(j).at(1);
      }

      centroid.at(0) = centroid.at(0) / clusters_.at(i).size();
      centroid.at(1) = centroid.at(1) / clusters_.at(i).size();

      double z_bar = 0.0;

      arma::mat Z = arma::ones(clusters_.at(i).size(), 4);

      for (int j = 0; j < int(clusters_.at(i).size()); j++) {
        clusters_.at(i).at(j).at(0) = clusters_.at(i).at(j).at(0) - centroid.at(0);
        clusters_.at(i).at(j).at(1) = clusters_.at(i).at(j).at(1) - centroid.at(1);

        z_bar = z_bar + pow(clusters_.at(i).at(j).at(0), 2) + pow(clusters_.at(i).at(j).at(1), 2);

        Z(j, 0) = pow(clusters_.at(i).at(j).at(0), 2) + pow(clusters_.at(i).at(j).at(1), 2);
        Z(j, 1) = clusters_.at(i).at(j).at(0);
        Z(j, 2) = clusters_.at(i).at(j).at(1);
        Z(j, 3) = 1.0;

      }

      z_bar = z_bar / clusters_.at(i).size();

      arma::mat M = (1.0 / clusters_.at(i).size()) * Z.t() * Z;

      arma::mat H = arma::eye(4, 4);
      H(0, 0) = 8.0 * z_bar;
      H(0, 3) = 2.0;
      H(3, 0) = 2.0;
      H(3, 3) = 0.0;

      arma::mat H_inv = arma::eye(4, 4);
      H_inv(0, 0) = 0.0;
      H_inv(0, 3) = 0.5;
      H_inv(3, 0) = 0.5;
      H_inv(3, 3) = -2.0 * z_bar;

      arma::mat U;
      arma::vec Sigma;
      arma::mat V;

      arma::svd(U, Sigma, V, Z);

      arma::vec A = arma::zeros(4);
      if (Sigma.at(3) < 1e-12) {

        A.at(0) = V.at(0, 3);
        A.at(1) = V.at(1, 3);
        A.at(2) = V.at(2, 3);
        A.at(3) = V.at(3, 3);
      } else {

        arma::mat temp_sigma = arma::zeros(4, 4);
        temp_sigma(0, 0) = Sigma.at(0);
        temp_sigma(1, 1) = Sigma.at(1);
        temp_sigma(2, 2) = Sigma.at(2);
        temp_sigma(3, 3) = Sigma.at(3);

        arma::mat Y = V * temp_sigma * V.t();
        arma::mat Q = Y * H_inv * Y;

        arma::vec eig_val;
        arma::mat eig_vec;
        eig_sym(eig_val, eig_vec, Q);

        int min_ind = 0;
        double min_val = 1000000.0;
        for (int ind = 0; ind < 4; ind++) {
          if (eig_val.at(ind) < min_val && eig_val.at(ind) > 0.0) {
            min_val = eig_val.at(ind);
            min_ind = ind;
          }
        }

        arma::vec A_temp = arma::zeros(4);
        A_temp.at(0) = eig_vec.at(0, min_ind);
        A_temp.at(1) = eig_vec.at(1, min_ind);
        A_temp.at(2) = eig_vec.at(2, min_ind);
        A_temp.at(3) = eig_vec.at(3, min_ind);

        A = Y.i() * A_temp;

      }

      arma::vec temp_param = arma::zeros(3);

      temp_param.at(0) = -A.at(1) / (2.0 * A.at(0)) + centroid.at(0);
      temp_param.at(1) = -A.at(2) / (2.0 * A.at(0)) + centroid.at(1);
      temp_param.at(2) =
        sqrt(
        (pow(A.at(1), 2) + pow(A.at(2), 2) - 4.0 * A.at(0) * A.at(3)) / (4.0 * pow(
          A.at(
            0), 2)) );


      if (temp_param.at(2) > min_radius_ && temp_param.at(2) < max_radius_) {

        // remove circles > 1m away
        if ( sqrt( pow(temp_param.at(0), 2) + pow(temp_param.at(1), 2) ) < 1.0) {
          circle_params_.push_back(temp_param);
        }
        // circle_params_.push_back(temp_param);
      }

    }

  }


  arma::vec convert_x_y(double r, double phi)
  {
    double x = r * cos(phi);
    double y = r * sin(phi);
    arma::vec z = {x, y};
    return z;
  }

  void visualize_clusters()
  {
    visualization_msgs::msg::MarkerArray landmarks_marker_msg;

    for (int i = 0; i < int(circle_params_.size()); i++) {
      double x = circle_params_.at(i).at(0);
      double y = circle_params_.at(i).at(1);
      double r = circle_params_.at(i).at(2);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "red/base_footprint";
      marker.header.stamp = this->now();
      marker.ns = "landmarks";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = r;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = r * 2.0;
      marker.scale.y = r * 2.0;
      marker.scale.z = r * 2.0;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      landmarks_marker_msg.markers.push_back(marker);
    }
    landmarks_marker_publisher_->publish(landmarks_marker_msg);
  }


  // Shared Pointers for ROS2
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarks_marker_publisher_;


  // Private Variables
  sensor_msgs::msg::LaserScan laser_msg_;

  std::vector<std::vector<arma::vec>> clusters_;
  std::vector<arma::vec> circle_params_;

  visualization_msgs::msg::MarkerArray landmarks_marker_msg_;

  double distance_threshold_ = 0.2;

  double min_radius_ = 0.026;
  double max_radius_ = 0.05;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
