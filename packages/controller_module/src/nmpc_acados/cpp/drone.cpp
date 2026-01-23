#include <yaml-cpp/yaml.h>

#include <common.hpp>
#include <drone.hpp>

/* drone constructors //{ */

Drone::Drone() {
  m_ = 0.85;      // kg
  l_ = 0.15;      // m
  kappa_ = 0.001; // set something better
  J_ = Vector<3>(0.001, 0.001, 0.0014).asDiagonal();
  J_inv_ = J_.inverse();

  motor_allocation_ << Vector<4>::Ones().transpose(), (l_ / sqrt(2.0)) * Vector<4>(-1.0, 1.0, -1.0, 1.0).transpose(),
      (l_ / sqrt(2.0)) * Vector<4>(-1.0, 1.0, 1.0, -1.0).transpose(), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose();
  motor_allocation_inv_ = motor_allocation_.inverse();
}

/* drone from yaml file constructor //{ */

Drone::Drone(const std::string yaml_file_location) {
  m_ = 0.85;      // kg
  l_ = 0.15;      // m
  kappa_ = 0.001; // set something better
  Vector<3> inertia_diag = Vector<3>(0.001, 0.001, 0.0014);
  Vector<3> pid_params_p = Vector<3>(0.001, 0.001, 0.0014);
  Vector<3> pid_params_i = Vector<3>(0.001, 0.001, 0.0014);
  Vector<3> pid_params_d = Vector<3>(0.001, 0.001, 0.0014);
  Vector<3> pid_params_max_i = Vector<3>(0.001, 0.001, 0.0014);

  YAML::Node config = YAML::LoadFile(yaml_file_location);
  if (config["mass"]) {
    m_ = config["mass"].as<Scalar>();
  } else {
    std::cout << "MASS COULD NOT BE LOADED"
              << "\n";
    exit(-1);
  }

  if (config["arm_length"]) {
    l_ = config["arm_length"].as<Scalar>();
  } else {
    std::cout << "ARM LENGTH COULD NOT BE LOADED"
              << "\n";
    exit(-1);
  }

  if (config["inertia"]) {
    inertia_diag[0] = config["inertia"][0].as<Scalar>();
    inertia_diag[1] = config["inertia"][1].as<Scalar>();
    inertia_diag[2] = config["inertia"][2].as<Scalar>();
  } else {
    std::cout << "INERTIA COULD NOT BE LOADED"
              << "\n";
    exit(-1);
  }

  if (config["ctau"]) {
    kappa_ = config["ctau"].as<Scalar>();
  } else {
    std::cout << "KAPPA/CTAU COULD NOT BE LOADED"
              << "\n";
    exit(-1);
  }

  if (config["kp_z"] && config["kd_z"] && config["ki_z"] && config["kp_xy"] && config["kd_xy"] && config["ki_xy"] && config["max_integral_xy"] &&
      config["max_integral_z"]) {
    pid_params_p[0] = config["kp_xy"].as<Scalar>();
    pid_params_p[1] = config["kp_xy"].as<Scalar>();
    pid_params_p[2] = config["kp_z"].as<Scalar>();
    pid_params_d[0] = config["kd_xy"].as<Scalar>();
    pid_params_d[1] = config["kd_xy"].as<Scalar>();
    pid_params_d[2] = config["kd_z"].as<Scalar>();
    pid_params_i[0] = config["ki_xy"].as<Scalar>();
    pid_params_i[1] = config["ki_xy"].as<Scalar>();
    pid_params_i[2] = config["ki_z"].as<Scalar>();
    kp_pid_mat_ = pid_params_p.asDiagonal();
    ki_pid_mat_ = pid_params_i.asDiagonal();
    kd_pid_mat_ = pid_params_d.asDiagonal();
    max_integral_error_xy_ = config["max_integral_xy"].as<Scalar>();
    max_integral_error_z_ = config["max_integral_z"].as<Scalar>();
  } else {
    std::cout << "PID GAINS COULD NOT BE LOADED"
              << "\n";
    exit(-1);
  }

  J_ = inertia_diag.asDiagonal();
  J_inv_ = J_.inverse();

  motor_allocation_ << Vector<4>::Ones().transpose(), (l_ / sqrt(2.0)) * Vector<4>(-1.0, 1.0, -1.0, 1.0).transpose(),
      (l_ / sqrt(2.0)) * Vector<4>(-1.0, 1.0, 1.0, -1.0).transpose(), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose();
  motor_allocation_inv_ = motor_allocation_.inverse();
}

//}

/* drone from params constructor //{ */

Drone::Drone(double m, double l, double kappa, Vector<3> inertia_diag) {
  m_ = m;         // kg
  l_ = l;         // m
  kappa_ = kappa; // set something better

  J_ = inertia_diag.asDiagonal();
  J_inv_ = J_.inverse();

  motor_allocation_ << Vector<4>::Ones().transpose(), (l_ / sqrt(2.0)) * Vector<4>(-1.0, 1.0, -1.0, 1.0).transpose(),
      (l_ / sqrt(2.0)) * Vector<4>(-1.0, 1.0, 1.0, -1.0).transpose(), kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose();
  motor_allocation_inv_ = motor_allocation_.inverse();
}

//}

//}

/* get_derivative() //{ */

void Drone::get_derivative(const Ref<const Vector<DroneState::SIZE>> state, const Ref<const Vector<4>> T, Ref<Vector<DroneState::SIZE>> derivative) {
  //                ^
  //     *3   *0    |
  //       \ /      |
  //        .       |
  //       / \      |x
  //     *1   *2    |
  //                |
  //  <_____________|
  //         y

  // Scalar weight_norm_t_thrust = (T(0) + T(1) + T(2) + T(3)) / this->m_;

  // Vector<3> tau = this->get_tau(T);

  const Vector<3> omega(state(DroneState::OMEX), state(DroneState::OMEY), state(DroneState::OMEZ));
  const Quaternion q_omega(0, state(DroneState::OMEX), state(DroneState::OMEY), state(DroneState::OMEZ));

  // p_dot = v
  derivative.segment<DroneState::NPOS>(DroneState::POS) = state.segment<DroneState::NVEL>(DroneState::VEL);

  // q_dot = 0.5 * q_right(q_omega) * [0, w]
  derivative.segment<DroneState::NATT>(DroneState::ATT) = 0.5 * Q_right(q_omega) * state.segment<DroneState::NATT>(DroneState::ATT);

  //[total_thrust;tau_x;tau_y;tau_z]
  // std::cout << "T" << T << std::endl;
  const Vector<4> force_torques = motor_allocation_ * T;
  // std::cout << "motor_allocation_" << motor_allocation_ << std::endl;

  // w_dot = Jinv * (tau -  omega x (quad_.J_ * omega))
  derivative.segment<DroneState::NOME>(DroneState::OME) = J_inv_ * (force_torques.segment<3>(1) - omega.cross(J_ * omega));

  // std::cout << derivative.segment<DroneState::NOME>(DroneState::OME)
  //          << std::endl;

  const Matrix<3, 3> R = Quaternion(state(DroneState::ATTW), state(DroneState::ATTX), state(DroneState::ATTY), state(DroneState::ATTZ)).toRotationMatrix();
  // v_dot = rotate_quat(q, vertcat(0, 0, (T[0]+T[1]+T[2]+T[3])/self.m)) + g
  //(- v * self.cd)

  const Vector<3> force(0.0, 0.0, force_torques[0]);
  derivative.segment<DroneState::NVEL>(DroneState::VEL) = R * force / m_ + GVEC;
}

//}

Quaternion DroneState::q() const { return Quaternion(x(ATTW), x(ATTX), x(ATTY), x(ATTZ)); }

std::ostream &operator<<(std::ostream &o, const DroneState &s) {
  o << "[t:" << s.t << ";  p:" << s.p(0) << "," << s.p(1) << ","
    << s.p(2)
    //   << ";q:" << s.qx(0) << "," << s.qx(1) << "," << s.qx(2) << "," <<
    //   s.qx(3) << ";" //<< std::endl
    << ";  v:" << s.v(0) << "," << s.v(1) << "," << s.v(2) << ";  w:" << s.w(0) << "," << s.w(1) << ","
    << s.w(2)
    //<< ";a:" << s.a(0) << "," << s.a(1) << "," << s.a(2)
    << "]";
  return o;
}

Matrix<3, 3> skew(const Vector<3> &v) { return (Matrix<3, 3>() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0).finished(); }

Matrix<4, 4> Q_left(const Quaternion &q) {
  return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(), q.w(), -q.x(), q.z(), -q.y(), q.x(), q.w()).finished();
}

Matrix<4, 4> Q_right(const Quaternion &q) {
  return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), q.z(), -q.y(), q.y(), -q.z(), q.w(), q.x(), q.z(), q.y(), -q.x(), q.w()).finished();
}

/* rk4() //{ */
void Drone::rk4(const Ref<const Vector<>> initial_state, const Scalar dt, const Ref<const Vector<4>> T, Ref<Vector<>> final_state) {
  static const Vector<4> rk4_sum_vec{1.0 / 6.0, 2.0 / 6.0, 2.0 / 6.0, 1.0 / 6.0};
  Matrix<> k = Matrix<>::Zero(initial_state.rows(), 4);

  final_state = initial_state;

  // k_1
  this->get_derivative(final_state, T, k.col(0));

  // k_2
  final_state = initial_state + 0.5 * dt * k.col(0);
  this->get_derivative(final_state, T, k.col(1));

  // k_3
  final_state = initial_state + 0.5 * dt * k.col(1);
  this->get_derivative(final_state, T, k.col(2));

  // k_4
  final_state = initial_state + dt * k.col(2);
  this->get_derivative(final_state, T, k.col(3));

  final_state = initial_state + dt * (k * rk4_sum_vec);
}

//}
