#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <Eigen/Dense>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI M_PI

class ControllerNode : public rclcpp::Node {

  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr des_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cur_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double kx, kv, kr, komega;
  double m, g, d, cf, cd;
  Eigen::Matrix3d J;
  Eigen::Vector3d e3;
  Eigen::MatrixXd F2W;

  Eigen::Vector3d x, v, omega;
  Eigen::Matrix3d R;
  Eigen::Vector3d xd, vd, ad;
  double yawd;
  double hz;

  static Eigen::Vector3d Vee(const Eigen::Matrix3d &in) {
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  ControllerNode()
  : Node("controller_node"), e3(0,0,1), F2W(4,4), hz(1000.0)
  {
    // --- Subscribers & publisher ---
    des_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "desired_state", 10,
      std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));

    cur_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "current_state", 10,
      std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<mav_msgs::msg::Actuators>("rotor_speed_cmds", 10);

    timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6/hz)),
      std::bind(&ControllerNode::controlLoop, this));

    // Gains
    kx = 6.0;
    kv = 4.0;
    kr = 2.0;
    komega = 0.1;

    // Physical parameters
    m = 1.0;
    cd = 1e-5;
    cf = 1e-3;
    g = 9.81;
    d = 0.3;
    J << 0.01,0,0,
         0,0.01,0,
         0,0,0.02;

    // Allocation matrix (45° rotated x-axis convention)
    double ct = cf;
    double cq = cd;
    F2W <<
       ct,  ct,  ct,  ct,
       0,  d*ct, 0, -d*ct,
      -d*ct, 0, d*ct,  0,
       cq, -cq,  cq, -cq;

    R = Eigen::Matrix3d::Identity();
    x.setZero(); v.setZero(); omega.setZero();
    xd.setZero(); vd.setZero(); ad.setZero(); yawd = 0.0;

    RCLCPP_INFO(this->get_logger(), "controller_node ready (hz=%.1f)", hz);
  }

  void onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr des_state_msg) {
    // desired pos, vel, acc
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

  void onCurrentState(const nav_msgs::msg::Odometry::SharedPtr cur_state_msg) {
    x << cur_state_msg->pose.pose.position.x,
         cur_state_msg->pose.pose.position.y,
         cur_state_msg->pose.pose.position.z;
    v << cur_state_msg->twist.twist.linear.x,
         cur_state_msg->twist.twist.linear.y,
         cur_state_msg->twist.twist.linear.z;

    tf2::Quaternion q;
    tf2::fromMsg(cur_state_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 m_rot(q);
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
        R(i,j) = m_rot[i][j];

    Eigen::Vector3d omega_world(
      cur_state_msg->twist.twist.angular.x,
      cur_state_msg->twist.twist.angular.y,
      cur_state_msg->twist.twist.angular.z);
    omega = R.transpose() * omega_world;
  }

  void controlLoop(){
    // Position & velocity error
    Eigen::Vector3d ex = x - xd;
    Eigen::Vector3d ev = v - vd;

    // Desired body z-axis (b3d)
    Eigen::Vector3d A = -kx*ex - kv*ev + m*g*e3 + m*ad;
    Eigen::Vector3d b3d = A.normalized();

    // Desired yaw direction b1d
    Eigen::Vector3d b1d(cos(yawd), sin(yawd), 0);
    Eigen::Vector3d b2d = b3d.cross(b1d);
    b2d.normalize();
    b1d = b2d.cross(b3d);

    Eigen::Matrix3d Rd;
    Rd << b1d, b2d, b3d;

    // Orientation & angular velocity errors
    Eigen::Matrix3d Re = Rd.transpose() * R;
    Eigen::Vector3d er = 0.5 * Vee(Re - Re.transpose());
    Eigen::Vector3d eomega = omega;

    // Force and torque
    double f = m * A.dot(R * e3);
    Eigen::Vector3d M = -kr*er - komega*eomega + omega.cross(J*omega);

    // Compute rotor speeds
    Eigen::Vector4d Wrench;
    Wrench << f, M(0), M(1), M(2);
    Eigen::Vector4d omega_sq = F2W.colPivHouseholderQr().solve(Wrench);
    Eigen::Vector4d w;
    for (int i=0;i<4;i++)
      w(i) = signed_sqrt(std::abs(omega_sq(i))) * ((omega_sq(i) >= 0) ? 1.0 : -1.0);

    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);
    for (int i=0;i<4;i++)
      cmd.angular_velocities[i] = w(i);
    motor_pub_->publish(cmd);
  }
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}

