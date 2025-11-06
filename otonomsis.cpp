#include <cmath>
#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <Eigen/Dense>
#include <tf2/utils.h>

#define PI M_PI
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode()
  : rclcpp::Node("controller_node"),
    e3(0,0,1),
    F2W(4,4),
    hz(200.0)
  {
    // === Parameters ===
    this->declare_parameter<double>("radius", 3.0);
    this->declare_parameter<double>("altitude", 3.0);
    this->declare_parameter<double>("omega", 0.6);
    this->declare_parameter<double>("hz", hz);

    this->declare_parameter<double>("kx", 4.0);
    this->declare_parameter<double>("kv", 2.5);
    this->declare_parameter<double>("kr", 1.0);
    this->declare_parameter<double>("komega", 0.2);

    this->get_parameter("radius", circle_radius);
    this->get_parameter("altitude", circle_altitude);
    this->get_parameter("omega", circle_omega);
    this->get_parameter("hz", hz);
    this->get_parameter("kx", kx);
    this->get_parameter("kv", kv);
    this->get_parameter("kr", kr);
    this->get_parameter("komega", komega);

    // === Physical constants ===
    m = 1.0;
    cd = 1e-5;
    cf = 1e-3;
    g = 9.81;
    d = 0.3;
    J << 1.0,0.0,0.0,
         0.0,1.0,0.0,
         0.0,0.0,1.0;

    x.setZero(); v.setZero(); R.setIdentity(); omega.setZero();
    xd.setZero(); vd.setZero(); ad.setZero(); yawd = 0.0;

    // === F2W matrix (example, adjust if needed) ===
    double ct = cd / cf;
    F2W.setZero();
    F2W(0,0)=cf; F2W(0,1)=cf; F2W(0,2)=cf; F2W(0,3)=cf;
    F2W(1,0)= d*cf; F2W(1,1)=-d*cf; F2W(1,2)=-d*cf; F2W(1,3)= d*cf;
    F2W(2,0)=-d*cf; F2W(2,1)=-d*cf; F2W(2,2)= d*cf; F2W(2,3)= d*cf;
    F2W(3,0)=-cd; F2W(3,1)= cd; F2W(3,2)=-cd; F2W(3,3)= cd;

    // === ROS interfaces ===
    auto qos = rclcpp::SystemDefaultsQoS();

    desired_sub_ = this->create_subscription<
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
        "desired_state", qos,
        std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));

    current_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state", qos,
        std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<mav_msgs::msg::Actuators>("rotor_speed_cmds", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0/hz),
      std::bind(&ControllerNode::controlLoop, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "ControllerNode active | radius=%.2f alt=%.2f omega=%.2f",
      circle_radius, circle_altitude, circle_omega);
  }

private:
  // ROS entities
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Controller params
  double kx, kv, kr, komega;
  double m,g,d,cf,cd;
  Eigen::Matrix3d J;
  Eigen::Vector3d e3;
  Eigen::MatrixXd F2W;

  // States
  Eigen::Vector3d x,v,omega;
  Eigen::Matrix3d R;

  // Desired states
  Eigen:
