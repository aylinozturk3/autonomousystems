// controller_node.cpp
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
    hz(200.0) // default loop freq
  {
    // --- ROS params (you can override via CLI or params file) ---
    this->declare_parameter<double>("radius", 2.0);
    this->declare_parameter<double>("altitude", 2.0);
    this->declare_parameter<double>("omega", 0.5); // rad/s, circular speed
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

    // Physical constants (change if you like)
    m = 1.0;
    cd = 1e-5;
    cf = 1e-3;
    g = 9.81;
    d = 0.3;
    J << 1.0,0.0,0.0,
         0.0,1.0,0.0,
         0.0,0.0,1.0;

    // initial states (zero)
    x.setZero();
    v.setZero();
    R.setIdentity();
    omega.setZero();

    xd.setZero();
    vd.setZero();
    ad.setZero();
    yawd = 0.0;

    // set a reasonable F2W mapping (may need to be adapted to your sim)
    // Rows: [ total_thrust ; tau_x ; tau_y ; tau_z ]
    // Columns: rotor 1..4 contributions (units include cf and cd)
    // This is a common template; check your assignment PDF and simulator rotor ordering.
    // We'll use a mapping where thrust = cf * w^2
    // and roll/pitch contributions proportional to d*cf with signs based on rotor layout.
    double ct = cd / cf; // torque coefficient ratio
    F2W.setZero();
    // thrust contributions (cf factor included)
    F2W(0,0) = cf; F2W(0,1) = cf; F2W(0,2) = cf; F2W(0,3) = cf;
    // roll (tau_x) contributions (example signs)
    F2W(1,0) =  d*cf; F2W(1,1) = -d*cf; F2W(1,2) = -d*cf; F2W(1,3) =  d*cf;
    // pitch (tau_y) contributions (example signs)
    F2W(2,0) = -d*cf; F2W(2,1) = -d*cf; F2W(2,2) =  d*cf; F2W(2,3) =  d*cf;
    // yaw (tau_z) contributions from drag
    F2W(3,0) = -cd; F2W(3,1) =  cd; F2W(3,2) = -cd; F2W(3,3) =  cd;

    // --- ROS interfaces ---
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

    RCLCPP_INFO(this->get_logger(), "controller_node ready (hz=%.1f) circle r=%.2f alt=%.2f omega=%.2f",
                hz, circle_radius, circle_altitude, circle_omega);
  }

private:
  // ROS handles
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Controller parameters
  double kx = 4.0, kv = 2.5, kr = 1.0, komega = 0.2;

  // Physical constants
  double m;
  double g;
  double d;
  double cf, cd;
  Eigen::Matrix3d J;
  Eigen::Vector3d e3;
  Eigen::MatrixXd F2W;

  // state
  Eigen::Vector3d x, v;
  Eigen::Matrix3d R;
  Eigen::Vector3d omega;

  // desired state
  Eigen::Vector3d xd, vd, ad;
  double yawd;

  // control loop freq
  double hz;

  // trajectory params
  double circle_radius = 2.0;
  double circle_altitude = 2.0;
  double circle_omega = 0.5; // rad/s

  // time
  rclcpp::Time start_time_;

  // helper
  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    if (val == 0.0) return 0.0;
    return val > 0.0 ? std::sqrt(val) : -std::sqrt(-val);
  }

  // --- callbacks ---
  void onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr des_state_msg){
    // If you publish an external desired_state, it will override internal circle generator
    if (!des_state_msg) return;

    // position
    if (des_state_msg->transforms.size() > 0){
      xd << des_state_msg->transforms[0].translation.x,
            des_state_msg->transforms[0].translation.y,
            des_state_msg->transforms[0].translation.z;
      yawd = tf2::getYaw(des_state_msg->transforms[0].rotation);
    }
    // velocity
    if (des_state_msg->velocities.size() > 0){
      vd << des_state_msg->velocities[0].linear.x,
            des_state_msg->velocities[0].linear.y,
            des_state_msg->velocities[0].linear.z;
    }
    // acceleration
    if (des_state_msg->accelerations.size() > 0){
      ad << des_state_msg->accelerations[0].linear.x,
            des_state_msg->accelerations[0].linear.y,
            des_state_msg->accelerations[0].linear.z;
    }
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

    // convert to body frame
    omega = R.transpose() * omega_world;
  }

  void controlLoop(){
    // If user didn't publish desired_state, generate circular trajectory
    rclcpp::Time now = this->now();
    double t = (now - start_time_).seconds();

    // internal desired trajectory (overwritten if external message arrived recently)
    xd << circle_radius * std::cos(circle_omega * t),
          circle_radius * std::sin(circle_omega * t),
          circle_altitude;

    vd << -circle_radius * circle_omega * std::sin(circle_omega * t),
           circle_radius * circle_omega * std::cos(circle_omega * t),
           0.0;

    ad << -circle_radius * circle_omega * circle_omega * std::cos(circle_omega * t),
          -circle_radius * circle_omega * circle_omega * std::sin(circle_omega * t),
          0.0;

    yawd = std::atan2(vd(1), vd(0)); // point approximately along velocity

    // 5.1 position and velocity errors
    Eigen::Vector3d ex = x - xd;
    Eigen::Vector3d ev = v - vd;

    // 5.2 compute desired b3d
    // u = -kx ex - kv ev + m*(ad + g*e3)
    Eigen::Vector3d u = -kx * ex - kv * ev + m * (ad + g * e3);
    Eigen::Vector3d b3d;
    double u_norm = u.norm();
    if (u_norm < 1e-6){
      b3d = R.col(2); // fallback to current body z
    } else {
      b3d = u / u_norm;
    }

    // desired heading vector (b1d) from yawd
    Eigen::Vector3d b1d;
    b1d << std::cos(yawd), std::sin(yawd), 0.0;

    // compute b2d = b3d x b1d
    Eigen::Vector3d b2d = b3d.cross(b1d);
    if (b2d.norm() < 1e-6) {
      // if nearly parallel, pick some orthogonal
      b2d = b3d.cross(Eigen::Vector3d(1,0,0));
      if (b2d.norm() < 1e-6) b2d = b3d.cross(Eigen::Vector3d(0,1,0));
    }
    b2d.normalize();

    Eigen::Vector3d b1d_proj = b2d.cross(b3d);
    b1d_proj.normalize();

    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d_proj;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;

    // 5.3 orientation error
    Eigen::Matrix3d errMat = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    Eigen::Vector3d er = Vee(errMat);

    // 5.3 rotation-rate error (neglect omega_d)
    Eigen::Vector3d eomega = omega; // since omega_d is approx 0

    // 5.4 desired thrust (f)
    double f_des = u.dot(R.col(2)); // u · (R * e3)  since R*e3 = current body z in world
    // ensure nonzero reasonable
    if (f_des < -1e3) f_des = -1e3;
    if (f_des > 1e4) f_des = 1e4;

    // 5.4 desired moments (M)
    Eigen::Vector3d M = -kr * er - komega * eomega + omega.cross(J * omega);

    // 5.5 recover rotor thrusts (fi) from wrench = [f_des; M]
    Eigen::Vector4d wrench;
    wrench << f_des, M(0), M(1), M(2);

    // Solve F2W * fi = wrench  -> fi = F2W^{-1} * wrench
    // Use a robust solver
    Eigen::Vector4d fi;
    Eigen::FullPivLU<Eigen::Matrix4d> lu(F2W);
    if (lu.isInvertible()){
      fi = F2W.inverse() * wrench;
    } else {
      // fallback: least squares
      fi = F2W.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(wrench);
    }

    // convert thrusts to rotor speeds (signed)
    Eigen::Vector4d rotor_speeds;
    for (int i=0;i<4;i++){
      double thrust_i = fi(i);
      // make sure division safe
      double val = thrust_i / cf;
      rotor_speeds(i) = signed_sqrt(val);
      if (!std::isfinite(rotor_speeds(i))) rotor_speeds(i) = 0.0;
    }

    // 5.6 publish actuator commands
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);
    cmd.angular_velocities[0] = rotor_speeds(0);
    cmd.angular_velocities[1] = rotor_speeds(1);
    cmd.angular_velocities[2] = rotor_speeds(2);
    cmd.angular_velocities[3] = rotor_speeds(3);
    motor_pub_->publish(cmd);

    // optional debug print occasionally
    static int counter = 0;
    if ((counter++ % int(std::max(1.0, hz/5.0))) == 0) {
      RCLCPP_INFO(this->get_logger(),
        "t=%.2f pos=(%.2f,%.2f,%.2f) xd=(%.2f,%.2f,%.2f) f=%.3f",
        t, x(0),x(1),x(2), xd(0),xd(1),xd(2), f_des);
    }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
