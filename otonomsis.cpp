#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <Eigen/Dense>
#include <tf2/utils.h>
#include <cmath>

class GeoHoverController : public rclcpp::Node {
public:
  GeoHoverController() : Node("geo_hover_controller") {
    // Subscribers and publisher
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "current_state", 10,
      std::bind(&GeoHoverController::odomCallback, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<mav_msgs::msg::Actuators>(
      "rotor_speed_cmds", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&GeoHoverController::controlLoop, this));

    // === Parameters ===
    m_ = 1.0;
    g_ = 9.81;
    cf_ = 1e-3;
    cd_ = 1e-5;
    d_ = 0.3;

    // === Controller gains ===
    kx_ = 6.0;
    kv_ = 3.0;
    kr_ = 1.5;
    komega_ = 0.15;

    // === Desired hover state ===
    xd_ << 0.0, 0.0, 30.0; // 30 metre yükseklik
    vd_.setZero();
    ad_.setZero();
    yawd_ = 0.0;

    e3_ << 0, 0, 1;

    RCLCPP_INFO(this->get_logger(), "Geometric hover controller initialized (z=30m).");
  }

private:
  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params
  double m_, g_, cf_, cd_, d_;
  double kx_, kv_, kr_, komega_;
  Eigen::Vector3d e3_;

  // States
  Eigen::Vector3d x_, v_, omega_;
  Eigen::Matrix3d R_;

  // Desired
  Eigen::Vector3d xd_, vd_, ad_;
  double yawd_;

  // Utils
  static Eigen::Vector3d vee(const Eigen::Matrix3d &M) {
    return Eigen::Vector3d(M(2,1), M(0,2), M(1,0));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ << msg->pose.pose.position.x,
          msg->pose.pose.position.y,
          msg->pose.pose.position.z;
    v_ << msg->twist.twist.linear.x,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.z;

    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    R_ = q.toRotationMatrix();

    Eigen::Vector3d w_world(
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z);
    omega_ = R_.transpose() * w_world;
  }

  void controlLoop() {
    // === Position/velocity errors ===
    Eigen::Vector3d ex = x_ - xd_;
    Eigen::Vector3d ev = v_ - vd_;

    // === Desired direction of body z-axis (b3d) ===
    Eigen::Vector3d a_des = -kx_ * ex - kv_ * ev - g_ * e3_ + ad_;
    Eigen::Vector3d b3d = a_des.normalized();

    // === Desired yaw direction ===
    Eigen::Vector3d b1d(std::cos(yawd_), std::sin(yawd_), 0);
    Eigen::Vector3d b2d = b3d.cross(b1d).normalized();
    Eigen::Vector3d b1_new = b2d.cross(b3d);
    Eigen::Matrix3d Rd;
    Rd << b1_new, b2d, b3d;

    // === Orientation and angular velocity errors ===
    Eigen::Matrix3d errM = 0.5 * (Rd.transpose() * R_ - R_.transpose() * Rd);
    Eigen::Vector3d eR = vee(errM);
    Eigen::Vector3d eOmega = omega_;

    // === Control law ===
    double f = m_ * a_des.dot(R_ * e3_);
    Eigen::Vector3d M = -kr_ * eR - komega_ * eOmega;

    // === Map desired wrench -> rotor thrusts ===
    double ctf = cd_ / cf_;
    Eigen::Matrix4d F2W;
    F2W << 1, 1, 1, 1,
           0, d_, 0, -d_,
           -d_, 0, d_, 0,
           -ctf, ctf, -ctf, ctf;

    Eigen::Vector4d W;
    W << f, M(0), M(1), M(2);
    Eigen::Vector4d fi = F2W.inverse() * W;

    // === Convert thrusts to angular velocities ===
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);
    for (int i = 0; i < 4; i++) {
      double thrust = std::max(fi(i), 0.0);
      double w = std::sqrt(thrust / cf_);
      cmd.angular_velocities[i] = w;
    }

    motor_pub_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeoHoverController>());
  rclcpp::shutdown();
  return 0;
}
