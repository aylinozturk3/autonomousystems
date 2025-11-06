void controlLoop(){
  rclcpp::Time now = this->now();
  double t = (now - start_time_).seconds();

  // --- generate reference trajectory ---
  // phase 1: takeoff (0–3 s)
  double takeoff_time = 3.0;
  if (t < takeoff_time) {
    double z = (t / takeoff_time) * circle_altitude;
    double zd = circle_altitude / takeoff_time;
    double zdd = 0.0;

    xd << 0.0, 0.0, z;
    vd << 0.0, 0.0, zd;
    ad << 0.0, 0.0, zdd;
    yawd = 0.0;  // no yaw change yet
  }
  // phase 2: circular motion at altitude
  else {
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

    yawd = std::atan2(vd(1), vd(0));  // face tangent to the circle
  }

  // --- geometric controller (same as before) ---
  Eigen::Vector3d ex = x - xd;
  Eigen::Vector3d ev = v - vd;

  Eigen::Vector3d u = -kx * ex - kv * ev + m * (ad + g * e3);
  Eigen::Vector3d b3d = u.normalized();

  Eigen::Vector3d b1d;
  b1d << std::cos(yawd), std::sin(yawd), 0.0;
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
  for (int i=0; i<4; ++i)
    rotor_speeds(i) = signed_sqrt(fi(i) / cf);

  mav_msgs::msg::Actuators cmd;
  cmd.angular_velocities.assign(rotor_speeds.data(), rotor_speeds.data()+4);
  motor_pub_->publish(cmd);
}
