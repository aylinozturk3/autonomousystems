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
    this->declare_parameter<double>("radius", 2.0);
    this->declare_parameter<double>("altitude", 2.0);
    this->declare_parameter<double>("omega", 0.5);
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

    // === F2W matrix (example, may adjust to your sim) ===
    double ct = cd / cf;
    F2W.setZero();
    F2W(0,0)=cf; F2W(0,1)=cf; F2W(0,2)=cf; F2W(0,3)=cf;
    F2W(1,0)=
