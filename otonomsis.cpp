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
      "Controller ready: radius=%.2f alt=%.2f omega=%.2f Hz=%.1f",
      circle_radius, circle_altitude, circle_omega, hz);
  }

private:
  // === ROS interfaces ===
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // === Controller parameters ===
  double kx, kv, kr, komega;
  double m, g, d, cf, cd;
  Eigen::Matrix3d J;
  Eigen::Vector3d e3;
  Eigen::MatrixXd F2W;

  // === States ===
  Eigen::Vector3d x, v;
  Eigen::Matrix3d R;
  Eigen::Vector3d omega;

  // === Desired states ===
  Eigen::Vector3d xd, vd, ad;
  double yawd;

  // === Loop frequency & trajectory ===
  double hz;
  double circle_radius, circle_altitude, circle_omega;
  rclcpp::Time start_time_;

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    if (val == 0.0) return 0.0;
    return val > 0.0 ? std::sqrt(val) : -std::sqrt(-val);
  }

  void onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr des_state_msg){
    if (!des_state_msg) return;
    xd << des_state_msg->transforms[0].translation.x,
          des_state_msg->transforms[0].translation.y,
          des_state_msg->transforms[0].translation.z;
    vd << des_state_msg->velocities[0].linear.x,
          des_state_msg->velocities[0].linear.y,
          des_state_msg->velocities[0].linear.z;
    ad << des_state_msg->accelerations[0].linear.x,
          des_state_msg->accelerations[0].linear.y,
          des_state_msg->accelerations[0].linear.z;
    yawd = tf2::getYaw(des_state_msg->transforms[0].rotation);
  }

  void onCurrentState(const nav_msgs::msg::Odometry::SharedPtr cur_state_msg){
    if (!cur_state_msg) return;
    x << cur_state_msg->pose.pose.position.x,
         cur_state_msg->pose.pose.position.y,
         cur_state_msg->pose.pose.position.z;
    v << cur_state_msg->twist.twist.linear.x,
         cur_state_msg->twist.twist.linear.y,
         cur_state_msg->twist.twist.linear.z;
    auto q = cur_state_msg->pose.pose.orientation;
    Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
    R = Q.toRotationMatrix();
    Eigen::Vector3d omega_world;
    omega_world << cur_state_msg->twist.twist.angular.x,
                   cur_state_msg->twist.twist.angular.y,
                   cur_state_msg->twist.twist.angular.z;
    omega = R.transpose() * omega_world;
  }

  void controlLoop(){
    rclcpp::Time now = this->now();
    double t = (now - start_time_).seconds();

    // --- reference trajectory ---
    double takeoff_time = 3.0;
    if (t < takeoff_time) {
      double z = (t / takeoff_time) * circle_altitude;
      double zd = circle_altitude / takeoff_time;
      xd << 0.0, 0.0, z;
      vd << 0.0, 0.0, zd;
      ad << 0.0, 0.0, 0.0;
      yawd = 0.0;
    } else {
      double t2 = t - takeoff_time;
      xd << circle_radius * std::cos(circle_omega * t2),
            circle_radius * std::sin(circle_omega * t2),
            circle_altitude;
      vd << -circle_radius * circle_omega * std::sin(circle_omega * t2),
             circle_radius * circle_omega * std::cos(circle_omega * t2),
             0.0;
      ad << -circle_radius * circle_omega * circle_omega * std::cos(circle_omega * t2),
            -circle_radius * circle_omega * circle_omega * std::sin(circle_omega * t2),
             0.0;
      yawd = std::atan2(vd(1), vd(0));
    }

    // --- geometric controller ---
    Eigen::Vector3d ex = x - xd;
    Eigen::Vector3d ev = v - vd;

    Eigen::Vector3d u = -kx * ex - kv * ev + m * (ad + g * e3);
    Eigen::Vector3d b3d = u.normalized();

    Eigen::Vector3d b1d(std::cos(yawd), std::sin(yawd), 0.0);
    Eigen::Vector3d b2d = b3d.cross(b1d);
    if (b2d.norm() < 1e-6) b2d = b3d.cross(Eigen::Vector3d(1,0,0));
    b2d.normalize();
    Eigen::Vector3d b1d_proj = b2d.cross(b3d);
    b1d_proj.normalize();

    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d_proj;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;

    Eigen::Matrix3d errMat = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    Eigen::Vector3d er = Vee(errMat);
    Eigen::Vector3d eomega = omega;

    double f_des = u.dot(R.col(2));
    Eigen::Vector3d M = -kr * er - komega * eomega + omega.cross(J * omega);

    Eigen::Vector4d wrench;
    wrench << f_des, M(0), M(1), M(2);
    Eigen::Vector4d fi = F2W.inverse() * wrench;

    Eigen::Vector4d rotor_speeds;
    for (int i=0;i<4;i++)
      rotor_speeds(i) = signed_sqrt(fi(i) / cf);

    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.assign(rotor_speeds.data(), rotor_speeds.data()+4);
    motor_pub_->publish(cmd);

    static int counter = 0;
    if (++counter % int(hz/5.0) == 0)
      RCLCPP_INFO(this->get_logger(),
        "t=%.2f pos(%.2f,%.2f,%.2f) xd(%.2f,%.2f,%.2f)",
        t, x(0),x(1),x(2), xd(0),xd(1),xd(2));
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
