#pragma once

#include <Eigen/Dense>
#include <drone.hpp>

#include "../c_generated_code/acados_solver_quadrotor_ode.h"

enum class AcadosParamsIDX : int {
  MASS = 0,
  MOTOR0_POSX = 1,
  MOTOR0_POSY = 2,
  MOTOR1_POSX = 3,
  MOTOR1_POSY = 4,
  MOTOR2_POSX = 5,
  MOTOR2_POSY = 6,
  MOTOR3_POSX = 7,
  MOTOR3_POSY = 8,
  INERTIA_XX = 9,
  INERTIA_YY = 10,
  INERTIA_ZZ = 11,
  C_TAU = 12,
  DRAG_X = 13,
  DRAG_Y = 14,
  DRAG_Z = 15,
  Q_REF_W = 16,
  Q_REF_X = 17,
  Q_REF_Y = 18,
  Q_REF_Z = 19,
  K_D_X = 20,
  K_D_Y = 21,
  K_D_Z = 22,
  K_P_X = 23,
  K_P_Y = 24,
  K_P_Z = 25,
  K_I_X = 26,
  K_I_Y = 27,
  K_I_Z = 28,
  TAU_MOT = 29,
  THRUST_ACTUATOR_MAX = 30,
  SIZE = 30,
};

enum class CostRefIDX : int {
  POSX = 0,
  POSY = 1,
  POSZ = 2,
  QUAT_ERR_X = 3,
  QUAT_ERR_Y = 4,
  QUAT_ERR_Z = 5,
  VELX = 6,
  VELY = 7,
  VELZ = 8,
  OMEX = 9,
  OMEY = 10,
  OMEZ = 11,
  INTX = 12,
  INTY = 13,
  INTZ = 14,
  THR1 = 15,
  THR2 = 16,
  THR3 = 17,
  THR4 = 18,
  COL_THRUST = 19,
  CMD_OME_X = 20,
  CMD_OME_Y = 21,
  CMD_OME_Z = 22,
  SIZE = 23,
};

class acados_drone {
private:
  quadrotor_ode_solver_capsule *acados_ocp_capsule;
  ocp_nlp_config *nlp_config;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_in *nlp_in;
  ocp_nlp_out *nlp_out;
  ocp_nlp_solver *nlp_solver;
  void *nlp_opts;
  double ref_thrust;
  double parameters[QUADROTOR_ODE_NP];
  double loaded_parameters[QUADROTOR_ODE_NP];
  bool enable_integral_relaxation;
  int integral_relaxation_index;

public:
  int N;      // number of shooting intervals
  double *dt; // time interval
  acados_drone();
  acados_drone(int _N, double *_time_steps);
  acados_drone(DroneParams uavParams, AcadosParams acParams);
  acados_drone(acados_drone &&other);
  ~acados_drone();
  acados_drone &operator=(acados_drone &&other);

  void set_init_state(DroneState _x0);
  // void set_init_solution(DroneState _x0);
  void set_all_init_solutions(double xtraj[], double utraj[]);
  void get_all_init_solutions(double xtraj[], double utraj[]);
  void set_ref(DroneState y_ref);
  void set_ref(std::vector<DroneState> ref_traj);
  bool compute_control();
  double get_elapsed_time();
  Vector<QUADROTOR_ODE_NU> get_first_control_action();
  std::vector<Vector<QUADROTOR_ODE_NU>> get_all_control_actions();
  std::vector<DroneState> get_all_computed_states();
  void print_stats();
  void setNewCosts(const Eigen::VectorXd &new_costs);
  void setNewConstants(const Eigen::VectorXd &new_constants);
  void setNewThrustMax(double first_thrust_max, double next_thrust_max = NAN);
};
