/*This is copied from https://github.com/uzh-rpg/sb_min_time_quadrotor_planning*/
#pragma once

#include <Eigen/Eigen>
#include <iostream>

using Scalar = double;

template <int rows = Eigen::Dynamic, int cols = rows> using Matrix = Eigen::Matrix<Scalar, rows, cols>;

template <int rows = Eigen::Dynamic> using Vector = Eigen::Matrix<Scalar, rows, 1>;

// Using `Quaternion` with type.
using Quaternion = Eigen::Quaternion<Scalar>;

template <class Derived> using Ref = Eigen::Ref<Derived>;

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

struct DroneParams {
  double mass;
  std::vector<double> mot0_pos;
  std::vector<double> mot1_pos;
  std::vector<double> mot2_pos;
  std::vector<double> mot3_pos;
  std::vector<double> inertia_diag;
  double ctau;
  double tau_mot;
  std::vector<double> drag;
  // constraints
  double thrust_collective_max;
  double thrust_collective_min;
  double thrust_actuator_max;
  double thrust_actuator_min;
  std::vector<double> max_rot_speed;
  std::vector<double> max_input_rot_speed;
  // pid
  std::vector<double> kp;
  std::vector<double> ki;
  std::vector<double> kd;
  std::vector<double> max_integral;
};

struct AcadosParams {
  int ac_num_iter = 40;
  double ac_horizon = 2; // [s]
  std::vector<double> ac_time_steps;

  // cost function weights
  double Q_pos_xy = 0;
  double Q_pos_z = 0;
  double Q_rot_xy = 0;
  double Q_rot_z = 0;
  double Q_vel = 0;
  double Q_ome = 0;
  double Q_int = 0; // PID integral
  double Q_thr = 0;

  double R_col_thrust = 0;
  double R_ome_in = 0;
  double integral_relaxation_index = 20;
  bool enable_integral_relaxation = false;
};
