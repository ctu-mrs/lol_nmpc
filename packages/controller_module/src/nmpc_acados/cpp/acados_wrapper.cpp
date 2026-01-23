
// standard
#include <cfloat>
#include <stdio.h>
#include <stdlib.h>
#include <yaml-cpp/yaml.h>

#include <acados_wrapper.hpp>
#include <common.hpp>

/* define //{ */
#define NX QUADROTOR_ODE_NX
#define NZ QUADROTOR_ODE_NZ
#define NU QUADROTOR_ODE_NU
#define NP QUADROTOR_ODE_NP
#define NBX QUADROTOR_ODE_NBX
#define NBX0 QUADROTOR_ODE_NBX0
#define NBU QUADROTOR_ODE_NBU
#define NSBX QUADROTOR_ODE_NSBX
#define NSBU QUADROTOR_ODE_NSBU
#define NSH QUADROTOR_ODE_NSH
#define NSG QUADROTOR_ODE_NSG
#define NSPHI QUADROTOR_ODE_NSPHI
#define NSHN QUADROTOR_ODE_NSHN
#define NSGN QUADROTOR_ODE_NSGN
#define NSPHIN QUADROTOR_ODE_NSPHIN
#define NSBXN QUADROTOR_ODE_NSBXN
#define NS QUADROTOR_ODE_NS
#define NSN QUADROTOR_ODE_NSN
#define NG QUADROTOR_ODE_NG
#define NBXN QUADROTOR_ODE_NBXN
#define NGN QUADROTOR_ODE_NGN
#define NY0 QUADROTOR_ODE_NY0
#define NY QUADROTOR_ODE_NY
#define NYN QUADROTOR_ODE_NYN
#define NH QUADROTOR_ODE_NH
#define NPHI QUADROTOR_ODE_NPHI
#define NHN QUADROTOR_ODE_NHN
#define NPHIN QUADROTOR_ODE_NPHIN
#define NR QUADROTOR_ODE_NR
//}

acados_drone::acados_drone() : acados_drone(QUADROTOR_ODE_N, NULL) {}

/* acados_drone constructor //{ */

acados_drone::acados_drone(int _N, double *new_time_steps) : N(_N) {
  std::cout << "creating acados_drone" << std::endl;
  acados_ocp_capsule = quadrotor_ode_acados_create_capsule();
  // there is an opportunity to change the number of shooting intervals in C
  // without new code generation
  int status = quadrotor_ode_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

  if (status) {
    printf("quadrotor_ode_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  if (new_time_steps == NULL) {
    dt = new double[N];
    std::fill_n(dt, N, 2.0 / 30.0);
  } else {
    dt = new double[N];
    for (int i = 0; i < N; i++) {
      dt[i] = new_time_steps[i];
    }
  }

  // initialize parameters to nominal value
  parameters[int(AcadosParamsIDX::MASS)] = 0.85;         // mass
  parameters[int(AcadosParamsIDX::MOTOR0_POSX)] = 0.15;  // mot0_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR0_POSY)] = -0.15; // mot0_pos[1]
  parameters[int(AcadosParamsIDX::MOTOR1_POSX)] = -0.15; // mot1_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR1_POSY)] = 0.15;  // mot1_pos[1]
  parameters[int(AcadosParamsIDX::MOTOR2_POSX)] = -0.15; // mot2_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR2_POSY)] = -0.15; // mot2_pos[1]
  parameters[int(AcadosParamsIDX::MOTOR3_POSX)] = 0.15;  // mot3_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR3_POSY)] = 0.15;  // mot3_pos[1]
  parameters[int(AcadosParamsIDX::INERTIA_XX)] = 0.001;  // inertia x
  parameters[int(AcadosParamsIDX::INERTIA_YY)] = 0.001;  // inertia y
  parameters[int(AcadosParamsIDX::INERTIA_ZZ)] = 0.0014; // inertia z
  parameters[int(AcadosParamsIDX::C_TAU)] = 0.05;        // ctau
  parameters[int(AcadosParamsIDX::DRAG_X)] = 0;          // dragx
  parameters[int(AcadosParamsIDX::DRAG_Y)] = 0;          // dragy
  parameters[int(AcadosParamsIDX::DRAG_Z)] = 0;          // dragz
  parameters[int(AcadosParamsIDX::Q_REF_W)] = 1.0;       // q_ref_w
  parameters[int(AcadosParamsIDX::Q_REF_X)] = 0.0;       // q_ref_x
  parameters[int(AcadosParamsIDX::Q_REF_Y)] = 0.0;       // q_ref_y
  parameters[int(AcadosParamsIDX::Q_REF_Z)] = 0.0;       // q_ref_z

  nlp_config = quadrotor_ode_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = quadrotor_ode_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = quadrotor_ode_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = quadrotor_ode_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = quadrotor_ode_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = quadrotor_ode_acados_get_nlp_opts(acados_ocp_capsule);
}

//}

/* acados_drone from yaml constructor //{ */

/* acados_drone from yaml constructor //{ */

acados_drone::acados_drone(DroneParams uavParams, AcadosParams acParams) {
  std::cout << "creating acados_drone from DroneParams and AcadosParams structs" << std::endl;
  acados_ocp_capsule = quadrotor_ode_acados_create_capsule();

  // set default
  N = QUADROTOR_ODE_N;
  dt = new double[N];
  std::fill_n(dt, N, 2.0 / 30.0);
  double *new_time_steps = NULL;

  N = acParams.ac_num_iter;
  std::vector<double> time_steps = acParams.ac_time_steps;
  if (time_steps.size() != N) {
    printf("ac_time_steps not same size as ac_num_iter\n");
    exit(1);
  }
  const Scalar ac_horizon = acParams.ac_horizon;
  // dt = acados_tf / N;

  double xtraj[DroneState::SIZE * (N + 1)];
  double utraj[DroneState::SIZE * N];

  double time_sum = 0;
  double time_min = DBL_MAX;
  new_time_steps = new double[N];
  dt = new double[N];
  for (int i = 0; i < N; i++) {
    new_time_steps[i] = time_steps.at(i);
    dt[i] = time_steps.at(i);
    time_sum += time_steps.at(i);
    time_min = time_steps.at(i) < time_min ? time_steps.at(i) : time_min;
  }
  if (fabs(time_sum - ac_horizon) > time_min) {
    printf("ac_time_steps sum %f is not equal to ac_horizon %f\n", time_sum, ac_horizon);
    exit(1);
  }

  // there is an opportunity to change the number of shooting intervals in C
  // without new code generation
  int status = quadrotor_ode_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

  if (status) {
    printf("quadrotor_ode_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  if (new_time_steps != NULL) {
    delete (new_time_steps);
  }

  nlp_config = quadrotor_ode_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = quadrotor_ode_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = quadrotor_ode_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = quadrotor_ode_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = quadrotor_ode_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = quadrotor_ode_acados_get_nlp_opts(acados_ocp_capsule);

  // parse drone configuration
  // initialize parameters to nominal value
  parameters[int(AcadosParamsIDX::MASS)] = uavParams.mass;                               // mass
  parameters[int(AcadosParamsIDX::MOTOR0_POSX)] = uavParams.mot0_pos.at(0);              // mot0_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR0_POSY)] = uavParams.mot0_pos.at(1);              // mot0_pos[1]
  parameters[int(AcadosParamsIDX::MOTOR1_POSX)] = uavParams.mot1_pos.at(0);              // mot1_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR1_POSY)] = uavParams.mot1_pos.at(1);              // mot1_pos[1]
  parameters[int(AcadosParamsIDX::MOTOR2_POSX)] = uavParams.mot2_pos.at(0);              // mot2_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR2_POSY)] = uavParams.mot2_pos.at(1);              // mot2_pos[1]
  parameters[int(AcadosParamsIDX::MOTOR3_POSX)] = uavParams.mot3_pos.at(0);              // mot3_pos[0]
  parameters[int(AcadosParamsIDX::MOTOR3_POSY)] = uavParams.mot3_pos.at(1);              // mot3_pos[1]
  parameters[int(AcadosParamsIDX::INERTIA_XX)] = uavParams.inertia_diag.at(0);           // inertia x
  parameters[int(AcadosParamsIDX::INERTIA_YY)] = uavParams.inertia_diag.at(1);           // inertia y
  parameters[int(AcadosParamsIDX::INERTIA_ZZ)] = uavParams.inertia_diag.at(2);           // inertia z
  parameters[int(AcadosParamsIDX::C_TAU)] = uavParams.ctau;                              // ctau
  parameters[int(AcadosParamsIDX::DRAG_X)] = uavParams.drag.at(0);                       // dragx
  parameters[int(AcadosParamsIDX::DRAG_Y)] = uavParams.drag.at(1);                       // dragy
  parameters[int(AcadosParamsIDX::DRAG_Z)] = uavParams.drag.at(2);                       // dragz
  parameters[int(AcadosParamsIDX::Q_REF_W)] = 1.0;                                       // q_ref_w
  parameters[int(AcadosParamsIDX::Q_REF_X)] = 0.0;                                       // q_ref_x
  parameters[int(AcadosParamsIDX::Q_REF_Y)] = 0.0;                                       // q_ref_y
  parameters[int(AcadosParamsIDX::Q_REF_Z)] = 0.0;                                       // q_ref_z
  parameters[int(AcadosParamsIDX::K_D_X)] = uavParams.kd.at(0);                          // kd_xy
  parameters[int(AcadosParamsIDX::K_D_Y)] = uavParams.kd.at(1);                          // kd_xy
  parameters[int(AcadosParamsIDX::K_D_Z)] = uavParams.kd.at(2);                          // kd_z
  parameters[int(AcadosParamsIDX::K_P_X)] = uavParams.kp.at(0);                          // kp_xy
  parameters[int(AcadosParamsIDX::K_P_Y)] = uavParams.kp.at(1);                          // kp_xy
  parameters[int(AcadosParamsIDX::K_P_Z)] = uavParams.kp.at(2);                          // kp_z
  parameters[int(AcadosParamsIDX::K_I_X)] = uavParams.ki.at(0);                          // ki_xy
  parameters[int(AcadosParamsIDX::K_I_Y)] = uavParams.ki.at(1);                          // ki_xy
  parameters[int(AcadosParamsIDX::K_I_Z)] = uavParams.ki.at(2);                          // ki_z
  parameters[int(AcadosParamsIDX::TAU_MOT)] = uavParams.tau_mot;                         // tau_mot
  parameters[int(AcadosParamsIDX::THRUST_ACTUATOR_MAX)] = uavParams.thrust_actuator_max; // thrust_actuator_max

  // PID part

  std::copy(std::begin(parameters), std::end(parameters), std::begin(loaded_parameters));

  enable_integral_relaxation = acParams.enable_integral_relaxation;
  integral_relaxation_index = acParams.integral_relaxation_index;
  for (int i = 0; i <= N; i++) {
    if (enable_integral_relaxation and i > integral_relaxation_index) {
      parameters[int(AcadosParamsIDX::K_I_X)] = 0; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = 0; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = 0; // kiz
    } else {
      parameters[int(AcadosParamsIDX::K_I_X)] = loaded_parameters[int(AcadosParamsIDX::K_I_X)]; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = loaded_parameters[int(AcadosParamsIDX::K_I_Y)]; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = loaded_parameters[int(AcadosParamsIDX::K_I_Z)]; // kiz
    }
    quadrotor_ode_acados_update_params(acados_ocp_capsule, i, parameters, NP);
  }

  // parse optimizer matrices
  double Q_pos_xy = acParams.Q_pos_xy;
  double Q_pos_z = acParams.Q_pos_z;
  double Q_rot_xy = acParams.Q_rot_xy;
  double Q_rot_z = acParams.Q_rot_z;
  double Q_vel = acParams.Q_vel;
  double Q_ome = acParams.Q_ome;
  double Q_int = acParams.Q_int;
  double Q_thr = acParams.Q_thr;
  double R_col_thrust = acParams.R_col_thrust;
  double R_ome_in = acParams.R_ome_in;

  double *W = new double[NY * NY]();
  // change only the non-zero elements:
  W[int(CostRefIDX::POSX) + (NY) * int(CostRefIDX::POSX)] = Q_pos_xy;
  W[int(CostRefIDX::POSY) + (NY) * int(CostRefIDX::POSY)] = Q_pos_xy;
  W[int(CostRefIDX::POSZ) + (NY) * int(CostRefIDX::POSZ)] = Q_pos_z;
  W[int(CostRefIDX::QUAT_ERR_X) + (NY) * int(CostRefIDX::QUAT_ERR_X)] = Q_rot_xy;
  W[int(CostRefIDX::QUAT_ERR_Y) + (NY) * int(CostRefIDX::QUAT_ERR_Y)] = Q_rot_xy;
  W[int(CostRefIDX::QUAT_ERR_Z) + (NY) * int(CostRefIDX::QUAT_ERR_Z)] = Q_rot_z;
  W[int(CostRefIDX::VELX) + (NY) * int(CostRefIDX::VELX)] = Q_vel;
  W[int(CostRefIDX::VELY) + (NY) * int(CostRefIDX::VELY)] = Q_vel;
  W[int(CostRefIDX::VELZ) + (NY) * int(CostRefIDX::VELZ)] = Q_vel;
  W[int(CostRefIDX::OMEX) + (NY) * int(CostRefIDX::OMEX)] = Q_ome;
  W[int(CostRefIDX::OMEY) + (NY) * int(CostRefIDX::OMEY)] = Q_ome;
  W[int(CostRefIDX::OMEZ) + (NY) * int(CostRefIDX::OMEZ)] = Q_ome;
  W[int(CostRefIDX::INTX) + (NY) * int(CostRefIDX::INTX)] = Q_int;
  W[int(CostRefIDX::INTY) + (NY) * int(CostRefIDX::INTY)] = Q_int;
  W[int(CostRefIDX::INTZ) + (NY) * int(CostRefIDX::INTZ)] = Q_int;
  W[int(CostRefIDX::THR1) + (NY) * int(CostRefIDX::THR1)] = Q_thr;
  W[int(CostRefIDX::THR2) + (NY) * int(CostRefIDX::THR2)] = Q_thr;
  W[int(CostRefIDX::THR3) + (NY) * int(CostRefIDX::THR3)] = Q_thr;
  W[int(CostRefIDX::THR4) + (NY) * int(CostRefIDX::THR4)] = Q_thr;
  W[int(CostRefIDX::COL_THRUST) + (NY) * int(CostRefIDX::COL_THRUST)] = R_col_thrust;
  W[int(CostRefIDX::CMD_OME_X) + (NY) * int(CostRefIDX::CMD_OME_X)] = R_ome_in;
  W[int(CostRefIDX::CMD_OME_Y) + (NY) * int(CostRefIDX::CMD_OME_Y)] = R_ome_in;
  W[int(CostRefIDX::CMD_OME_Z) + (NY) * int(CostRefIDX::CMD_OME_Z)] = R_ome_in;

  printf("Costs: Q_pos_xy:%f, Q_pos_z:%f, Q_rot_xy:%f, Q_rot_z:%f, "
         "Q_vel:%f,Q_rot:%f,Q_int:%f,Q_thr:%f,R_col_thrust:%f,R_ome_in:%f\n",
         Q_pos_xy, Q_pos_z, Q_rot_xy, Q_rot_z, Q_vel, Q_ome, Q_int, Q_thr, R_col_thrust, R_ome_in);
  for (int i = 0; i < N; i++) {
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
  }
  delete (W);

  double *W_e = new double[NYN * NYN]();
  // change only the non-zero elements:
  W_e[int(CostRefIDX::POSX) + (NYN) * int(CostRefIDX::POSX)] = Q_pos_xy;
  W_e[int(CostRefIDX::POSY) + (NYN) * int(CostRefIDX::POSY)] = Q_pos_xy;
  W_e[int(CostRefIDX::POSZ) + (NYN) * int(CostRefIDX::POSZ)] = Q_pos_z;
  W_e[int(CostRefIDX::QUAT_ERR_X) + (NYN) * int(CostRefIDX::QUAT_ERR_X)] = Q_rot_xy;
  W_e[int(CostRefIDX::QUAT_ERR_Y) + (NYN) * int(CostRefIDX::QUAT_ERR_Y)] = Q_rot_xy;
  W_e[int(CostRefIDX::QUAT_ERR_Z) + (NYN) * int(CostRefIDX::QUAT_ERR_Z)] = Q_rot_z;
  W_e[int(CostRefIDX::VELX) + (NYN) * int(CostRefIDX::VELX)] = Q_vel;
  W_e[int(CostRefIDX::VELY) + (NYN) * int(CostRefIDX::VELY)] = Q_vel;
  W_e[int(CostRefIDX::VELZ) + (NYN) * int(CostRefIDX::VELZ)] = Q_vel;
  W_e[int(CostRefIDX::OMEX) + (NYN) * int(CostRefIDX::OMEX)] = Q_ome;
  W_e[int(CostRefIDX::OMEY) + (NYN) * int(CostRefIDX::OMEY)] = Q_ome;
  W_e[int(CostRefIDX::OMEZ) + (NYN) * int(CostRefIDX::OMEZ)] = Q_ome;
  W_e[int(CostRefIDX::INTX) + (NYN) * int(CostRefIDX::INTX)] = Q_int;
  W_e[int(CostRefIDX::INTY) + (NYN) * int(CostRefIDX::INTY)] = Q_int;
  W_e[int(CostRefIDX::INTZ) + (NYN) * int(CostRefIDX::INTZ)] = Q_int;
  W_e[int(CostRefIDX::THR1) + (NYN) * int(CostRefIDX::THR1)] = Q_thr;
  W_e[int(CostRefIDX::THR2) + (NYN) * int(CostRefIDX::THR2)] = Q_thr;
  W_e[int(CostRefIDX::THR3) + (NYN) * int(CostRefIDX::THR3)] = Q_thr;
  W_e[int(CostRefIDX::THR4) + (NYN) * int(CostRefIDX::THR4)] = Q_thr;

  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);

  delete (W_e);

  // parse constrains:
  double thrust_max_collective = uavParams.thrust_collective_max;
  double thrust_min_collective = uavParams.thrust_collective_min;
  double thrust_actuator_min = uavParams.thrust_actuator_min;
  double thrust_actuator_max = uavParams.thrust_actuator_max;

  // thrust = thrust_actuator_max * throttle*throttle  - assuming 1.0 is max throttle
  // throttle = sqrt(thrust/thrust_actuator_max)
  // this uses the fact that single-rotor throttle is the same as collective
  double throttle_max_collective = sqrt((thrust_max_collective / 4.0) / thrust_actuator_max);
  double throttle_min_collective = sqrt((thrust_min_collective / 4.0) / thrust_actuator_max);
  std::cout << "throttle_min_collective " << throttle_min_collective << std::endl;
  std::cout << "throttle_max_collective " << throttle_max_collective << std::endl;
  int *idxbu = new int[NBU];

  idxbu[0] = 0;
  idxbu[1] = 1;
  idxbu[2] = 2;
  idxbu[3] = 3;
  double *lubu = new double[2 * NBU]();
  double *lbu = lubu;
  double *ubu = lubu + NBU;

  lbu[0] = throttle_min_collective;
  lbu[1] = -uavParams.max_input_rot_speed.at(0);
  lbu[2] = -uavParams.max_input_rot_speed.at(1);
  lbu[3] = -uavParams.max_input_rot_speed.at(2);
  ubu[0] = throttle_max_collective;
  ubu[1] = uavParams.max_input_rot_speed.at(0);
  ubu[2] = uavParams.max_input_rot_speed.at(1);
  ubu[3] = uavParams.max_input_rot_speed.at(2);

  for (int i = 0; i < N; i++) {
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
  }
  delete (idxbu);
  delete (lubu);

  int *idxbx = new int[NBX];

  // omega
  idxbx[0] = 10;
  idxbx[1] = 11;
  idxbx[2] = 12;
  // PID integral limit
  idxbx[3] = 13;
  idxbx[4] = 14;
  idxbx[5] = 15;
  // thrust
  // idxbx[6] = 16;
  // idxbx[7] = 17;
  // idxbx[8] = 18;
  // idxbx[9] = 19;

  double *lubx = new double[2 * NBX];
  double *lbx = lubx;
  double *ubx = lubx + NBX;

  lbx[0] = -uavParams.max_rot_speed.at(0);
  ubx[0] = uavParams.max_rot_speed.at(0);
  lbx[1] = -uavParams.max_rot_speed.at(1);
  ubx[1] = uavParams.max_rot_speed.at(1);
  lbx[2] = -uavParams.max_rot_speed.at(2);
  ubx[2] = uavParams.max_rot_speed.at(2);
  lbx[3] = -uavParams.max_integral.at(0);
  ubx[3] = uavParams.max_integral.at(0);
  lbx[4] = -uavParams.max_integral.at(1);
  ubx[4] = uavParams.max_integral.at(1);
  lbx[5] = -uavParams.max_integral.at(2);
  ubx[5] = uavParams.max_integral.at(2);
  // lbx[6] = thrust_min;
  // ubx[6] = thrust_max;
  // lbx[7] = thrust_min;
  // ubx[7] = thrust_max;
  // lbx[8] = thrust_min;
  // ubx[8] = thrust_max;
  // lbx[9] = thrust_min;
  // ubx[9] = thrust_max;

  for (int i = 1; i < N + 1; i++) {
    if (enable_integral_relaxation and i > integral_relaxation_index) {
      lbx[3] = -99999;
      ubx[3] = 99999;
      lbx[4] = -99999;
      ubx[4] = 99999;
      lbx[5] = -99999;
      ubx[5] = 99999;
    } else {
      lbx[3] = -uavParams.max_integral.at(0);
      ubx[3] = uavParams.max_integral.at(0);
      lbx[4] = -uavParams.max_integral.at(1);
      ubx[4] = uavParams.max_integral.at(1);
      lbx[5] = -uavParams.max_integral.at(2);
      ubx[5] = uavParams.max_integral.at(2);
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
  }
  delete (idxbx);
  delete (lubx);

  double ctau = uavParams.ctau;
  double kp_xy = uavParams.kp[0];
  double kp_z = uavParams.kp[2];
  double *C = new double[NG * NX]();
  double *D = new double[NG * NU]();
  double *lug = new double[2 * NG]();
  double *lg = lug;
  double *ug = lug + NG;

  // thrust = thrust_actuator_max * throttle*throttle - assuming 1.0 is max throttle
  // throttle = sqrt(thrust/thrust_actuator_max)
  double throttle_actuator_max = 1.0;
  double throttle_actuator_min = sqrt(thrust_actuator_min / thrust_actuator_max);

  // printf("TODO: need to implement correct min-max throttle with allocation of actuators of PID not allocation matrix\n");
  // exit(1);
  Eigen::MatrixXd Alloc = Eigen::MatrixXd::Ones(4, 4);
  Alloc(0, 1) = -0.7071;
  Alloc(1, 1) = 0.7071;
  Alloc(2, 1) = -0.7071;
  Alloc(3, 1) = 0.7071;
  Alloc(0, 2) = -0.7071;
  Alloc(1, 2) = 0.7071;
  Alloc(2, 2) = 0.7071;
  Alloc(3, 2) = -0.7071;
  Alloc(0, 3) = -1;
  Alloc(1, 3) = -1;
  Alloc(2, 3) = 1;
  Alloc(3, 3) = 1;
  // M_mat = M_mat.inverse().eval();

  // std::cout << Alloc<<std::endl;
  D[0 + NG * 0] = Alloc(0, 0);
  D[0 + NG * 1] = Alloc(0, 1) * kp_xy;
  D[0 + NG * 2] = Alloc(0, 2) * kp_xy;
  D[0 + NG * 3] = Alloc(0, 3) * kp_z;
  D[1 + NG * 0] = Alloc(1, 0);
  D[1 + NG * 1] = Alloc(1, 1) * kp_xy;
  D[1 + NG * 2] = Alloc(1, 2) * kp_xy;
  D[1 + NG * 3] = Alloc(1, 3) * kp_z;
  D[2 + NG * 0] = Alloc(2, 0);
  D[2 + NG * 1] = Alloc(2, 1) * kp_xy;
  D[2 + NG * 2] = Alloc(2, 2) * kp_xy;
  D[2 + NG * 3] = Alloc(2, 3) * kp_z;
  D[3 + NG * 0] = Alloc(3, 0);
  D[3 + NG * 1] = Alloc(3, 1) * kp_xy;
  D[3 + NG * 2] = Alloc(3, 2) * kp_xy;
  D[3 + NG * 3] = Alloc(3, 3) * kp_z;
  // std::cout << "D:" << std::endl;
  // for (int ii = 0; ii < 4; ii++) {
  //   for (int jj = 0; jj < 4; jj++) {
  //     std::cout << ii << " " << jj << " " << D[ii + NG * jj] << std::endl;
  //   }
  // }

  C[0 + NG * 10] = -Alloc(0, 1) * kp_xy;
  C[0 + NG * 11] = -Alloc(0, 2) * kp_xy;
  C[0 + NG * 12] = -Alloc(0, 3) * kp_z;
  C[1 + NG * 10] = -Alloc(1, 1) * kp_xy;
  C[1 + NG * 11] = -Alloc(1, 2) * kp_xy;
  C[1 + NG * 12] = -Alloc(1, 3) * kp_z;
  C[2 + NG * 10] = -Alloc(2, 1) * kp_xy;
  C[2 + NG * 11] = -Alloc(2, 2) * kp_xy;
  C[2 + NG * 12] = -Alloc(2, 3) * kp_z;
  C[3 + NG * 10] = -Alloc(3, 1) * kp_xy;
  C[3 + NG * 11] = -Alloc(3, 2) * kp_xy;
  C[3 + NG * 12] = -Alloc(3, 3) * kp_z;

  // std::cout << "C:" << std::endl;
  // for (int ii = 0; ii < 4; ii++) {
  //   for (int jj = 10; jj < 13; jj++) {
  //     std::cout << ii << " " << jj << " " << C[ii + NG * jj] << std::endl;
  //   }
  // }

  lg[0] = throttle_actuator_min;
  lg[1] = throttle_actuator_min;
  lg[2] = throttle_actuator_min;
  lg[3] = throttle_actuator_min;

  ug[0] = throttle_actuator_max;
  ug[1] = throttle_actuator_max;
  ug[2] = throttle_actuator_max;
  ug[3] = throttle_actuator_max;

  for (int i = 0; i < N; i++) {
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "D", D);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "C", C);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lg", lg);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ug", ug);
  }
  free(D);
  free(C);
  free(lug);
  // exit(1);
}

//}

/* acados_drone by move constructor //{ */

acados_drone::acados_drone(acados_drone &&other) {
  std::cout << "creating acados_drone by move constructor" << std::endl;
  exit(1);
  if (this != &other) {
    acados_ocp_capsule = other.acados_ocp_capsule;
    nlp_config = other.nlp_config;
    nlp_dims = other.nlp_dims;
    nlp_in = other.nlp_in;
    nlp_out = other.nlp_out;
    nlp_solver = other.nlp_solver;
    nlp_opts = other.nlp_opts;

    N = other.N;
    dt = other.dt;

    std::copy(other.parameters, other.parameters + QUADROTOR_ODE_NP, parameters);
    std::copy(other.loaded_parameters, other.loaded_parameters + QUADROTOR_ODE_NP, loaded_parameters);
    enable_integral_relaxation = other.enable_integral_relaxation;
    integral_relaxation_index = other.integral_relaxation_index;
    ref_thrust = other.ref_thrust;

    other.acados_ocp_capsule = nullptr;
  }
}

//}

/* acados_drone destructor //{ */

acados_drone::~acados_drone() {
  if (acados_ocp_capsule == nullptr) {
    std::cout << "freeing deleted acados_drone" << std::endl;
    return;
  }

  std::cout << "freeing acados_drone" << std::endl;
  // free solver
  int status = quadrotor_ode_acados_free(acados_ocp_capsule);
  if (status) {
    printf("quadrotor_ode_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = quadrotor_ode_acados_free_capsule(acados_ocp_capsule);
  if (status) {
    printf("quadrotor_ode_acados_free_capsule() returned status %d. \n", status);
  }
  acados_ocp_capsule = nullptr;
}

//}

/* acados_drone by move operator  //{ */

acados_drone &acados_drone::operator=(acados_drone &&other) {
  std::cout << "creating acados_drone by move operator" << std::endl;

  if (this != &other) {
    this->~acados_drone();

    acados_ocp_capsule = other.acados_ocp_capsule;
    nlp_config = other.nlp_config;
    nlp_dims = other.nlp_dims;
    nlp_in = other.nlp_in;
    nlp_out = other.nlp_out;
    nlp_solver = other.nlp_solver;
    nlp_opts = other.nlp_opts;

    N = other.N;
    dt = other.dt;

    std::copy(other.parameters, other.parameters + QUADROTOR_ODE_NP, parameters);
    std::copy(other.loaded_parameters, other.loaded_parameters + QUADROTOR_ODE_NP, loaded_parameters);
    enable_integral_relaxation = other.enable_integral_relaxation;
    integral_relaxation_index = other.integral_relaxation_index;
    ref_thrust = other.ref_thrust;

    other.acados_ocp_capsule = nullptr;
  }

  return *this;
}

//}

/* compute_control() //{ */

bool acados_drone::compute_control() {
  // solve ocp in loop
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
  int status = quadrotor_ode_acados_solve(acados_ocp_capsule);

  if (status != ACADOS_SUCCESS) {
    printf("quadrotor_ode_acados_solve() failed with status %d.\n", status);
    return false;
  }
  // quadrotor_ode_acados_print_stats(acados_ocp_capsule);
  return true;
}

//}

/* print_stats() //{ */

void acados_drone::print_stats() {
  int sqp_iter, stat_m, stat_n, tmp_int;
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "sqp_iter", &sqp_iter);
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "stat_n", &stat_n);
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "stat_m", &stat_m);

  double stat[1200];
  ocp_nlp_get(acados_ocp_capsule->nlp_config, acados_ocp_capsule->nlp_solver, "statistics", stat);

  int qp_iter = (int)stat[2 + 1 * 3];
  double elapsed_time = get_elapsed_time();
  std::cout << "qp iterations: " << qp_iter << ", time elapsed: " << elapsed_time * 1000 << " ms." << std::endl;
}

//}

// --------------------------------------------------------------
// |                        set functions                       |
// --------------------------------------------------------------

/* set_init_state() //{ */

void acados_drone::set_init_state(DroneState _x0) {
  // initial condition
  int idxbx0[NBX0];
  double lbx0[NBX0];
  double ubx0[NBX0];

  for (int i = 0; i < NBX0; i++) {
    idxbx0[i] = i;
    lbx0[i] = _x0.x(i);
    ubx0[i] = _x0.x(i);
  }

  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

  // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, 0, "x", lbx0);
}

//}

/* set_init_solution() //{ */

// void acados_drone::set_init_solution(DroneState _x0) {
//   double x_init[QUADROTOR_ODE_NX];
//   double u0[QUADROTOR_ODE_NU];

//   // initialization for state values
//   for (int i = 0; i < NX; i++) {
//     x_init[i] = _x0.x(i);
//   }
//   // initial value for control input
//   u0[0] = 8.5;
//   u0[1] = 8.5;
//   u0[2] = 8.5;
//   u0[3] = 8.5;

//   // initialize solution
//   for (int i = 0; i < N; i++) {
//     ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
//     ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
//   }
//   ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
// }

//}

/* set_ref(DroneState) //{ */

void acados_drone::set_ref(DroneState y_ref) {
  // std::cout << "set_ref single point not use it now " <<std::endl;
  // exit(1);
  //   std::cout << "set_ref ref_thrust " << ref_thrust << " parameters "
  //             << parameters << std::endl;
  //   std::cout << "parameters: ";
  //   for (size_t i = 0; i <= 10; i++) {
  //     std::cout << parameters[i] << " ";
  //   }
  //   std::cout << std::endl;
  double yref[NY];
  for (int i = 0; i < 3; i++) // set reference position
    yref[i] = y_ref.p(i);
  for (int i = 3; i < 6; i++) // set reference attitude error
    yref[i] = 0;
  for (int i = 6; i < NYN; i++) // set reference velocity and angular velocity
    yref[i] = y_ref.x(i + 1);

  /* std::cout << "u: "; */
  for (int i = 0; i < NU; i++) {
    yref[NYN + i] = y_ref.u(i); // U ref
    /* std::cout << y_ref.u(i) << ", "; */
  }
  /* std::cout << std::endl; */
  /* std::cout << "[NMPC controller] yref: "; */
  for (size_t i = 0; i < NY; i++) {
    /* std::cout << yref[i] << ", "; */
  }
  /* std::cout << std::endl; */
  // exit(1);
  for (int i = 0; i <= N; i++) {
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
  }

  parameters[int(AcadosParamsIDX::Q_REF_W)] = y_ref.x(DroneState::ATTW); // q_ref_w
  parameters[int(AcadosParamsIDX::Q_REF_X)] = y_ref.x(DroneState::ATTX); // q_ref_x
  parameters[int(AcadosParamsIDX::Q_REF_Y)] = y_ref.x(DroneState::ATTY); // q_ref_y
  parameters[int(AcadosParamsIDX::Q_REF_Z)] = y_ref.x(DroneState::ATTZ); // q_ref_z
  for (int i = 0; i <= N; i++) {
    if (enable_integral_relaxation and i > integral_relaxation_index) {
      parameters[int(AcadosParamsIDX::K_I_X)] = 0; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = 0; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = 0; // kiz
    } else {
      parameters[int(AcadosParamsIDX::K_I_X)] = loaded_parameters[int(AcadosParamsIDX::K_I_X)]; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = loaded_parameters[int(AcadosParamsIDX::K_I_Y)]; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = loaded_parameters[int(AcadosParamsIDX::K_I_Z)]; // kiz
    }
    quadrotor_ode_acados_update_params(acados_ocp_capsule, i, parameters, NP);
  }
}

//}

/* set_ref(vector of DroneState) //{ */

void acados_drone::set_ref(std::vector<DroneState> ref_traj) {
  if (ref_traj.size() <= N) {
    std::cout << "ERROR: ref_traj has only " << std::to_string(ref_traj.size()) << " elements. Minimum is " << std::to_string(N + 1) << std::endl;
    exit(-1);
  }

  double yref[NY];
  for (int j = 0; j <= N; j++) {
    yref[int(CostRefIDX::POSX)] = ref_traj[j].x(DroneState::POSX); // set reference position
    yref[int(CostRefIDX::POSY)] = ref_traj[j].x(DroneState::POSY); // set reference position
    yref[int(CostRefIDX::POSZ)] = ref_traj[j].x(DroneState::POSZ); // set reference position
    yref[int(CostRefIDX::QUAT_ERR_X)] = 0.0; // set attitude error to zero which is calculated from quaternion in three axes in quaternion.py
    yref[int(CostRefIDX::QUAT_ERR_Y)] = 0.0; // set attitude error to zero which is calculated from quaternion in three axes in quaternion.py
    yref[int(CostRefIDX::QUAT_ERR_Z)] = 0.0; // set attitude error to zero which is calculated from quaternion in three axes in quaternion.py
    parameters[int(AcadosParamsIDX::Q_REF_W)] = ref_traj[j].x(DroneState::ATTW); // q_ref_w
    parameters[int(AcadosParamsIDX::Q_REF_X)] = ref_traj[j].x(DroneState::ATTX); // q_ref_x
    parameters[int(AcadosParamsIDX::Q_REF_Y)] = ref_traj[j].x(DroneState::ATTY); // q_ref_y
    parameters[int(AcadosParamsIDX::Q_REF_Z)] = ref_traj[j].x(DroneState::ATTZ); // q_ref_z
    yref[int(CostRefIDX::VELX)] = ref_traj[j].x(DroneState::VELX);               // set reference velocity
    yref[int(CostRefIDX::VELY)] = ref_traj[j].x(DroneState::VELY);               // set reference velocity
    yref[int(CostRefIDX::VELZ)] = ref_traj[j].x(DroneState::VELZ);               // set reference velocity
    yref[int(CostRefIDX::OMEX)] = ref_traj[j].x(DroneState::OMEX);               // set reference angular velocity
    yref[int(CostRefIDX::OMEY)] = ref_traj[j].x(DroneState::OMEY);               // set reference angular velocity
    yref[int(CostRefIDX::OMEZ)] = ref_traj[j].x(DroneState::OMEZ);               // set reference angular velocity
    yref[int(CostRefIDX::INTX)] = 0.0;                                           // PID integral-x
    yref[int(CostRefIDX::INTY)] = 0.0;                                           // PID integral-y
    yref[int(CostRefIDX::INTZ)] = 0.0;                                           // PID integral-z
    yref[int(CostRefIDX::THR1)] = ref_traj[j].x(DroneState::T1);                 // U ref, set to acceleration norm * mass / 4
    yref[int(CostRefIDX::THR2)] = ref_traj[j].x(DroneState::T2);                 // U ref, set to acceleration norm * mass / 4
    yref[int(CostRefIDX::THR3)] = ref_traj[j].x(DroneState::T3);                 // U ref, set to acceleration norm * mass / 4
    yref[int(CostRefIDX::THR4)] = ref_traj[j].x(DroneState::T4);                 // U ref, set to acceleration norm * mass / 4
    yref[int(CostRefIDX::COL_THRUST)] = ref_traj[j].u(0);                        // Total thrust reference
    yref[int(CostRefIDX::CMD_OME_X)] = ref_traj[j].u(1);                         // Control reference for commanded omega x
    yref[int(CostRefIDX::CMD_OME_Y)] = ref_traj[j].u(2);                         // Control reference for commanded omega y
    yref[int(CostRefIDX::CMD_OME_Z)] = ref_traj[j].u(3);                         // Control reference for commanded omega z

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref", yref);

    if (enable_integral_relaxation and j > integral_relaxation_index) {
      parameters[int(AcadosParamsIDX::K_I_X)] = 0; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = 0; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = 0; // kiz
    } else {
      parameters[int(AcadosParamsIDX::K_I_X)] = loaded_parameters[int(AcadosParamsIDX::K_I_X)]; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = loaded_parameters[int(AcadosParamsIDX::K_I_Y)]; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = loaded_parameters[int(AcadosParamsIDX::K_I_Z)]; // kiz
    }
    quadrotor_ode_acados_update_params(acados_ocp_capsule, j, parameters, NP);
  }
}

//}

/* setNewCosts() //{ */

void acados_drone::setNewCosts(const Eigen::VectorXd &new_costs) {
  std::cout << "setNewCosts" << std::endl;
  // 0 = params.Q_pos_xy , 1 = params.Q_pos_z , 2 = params.Q_quat_xy ,
  // 3 = params.Q_quat_z , 4 = params.Q_velocity , 5 = params.Q_omega ,
  // 6 = params.Q_int , 7 = params.Q_thrust , 8 = params.R_col_thrust,
  // 9 = params.R_ome_in;
  double *W = new double[NY * NY]();
  // change only the non-zero elements:
  W[int(CostRefIDX::POSX) + (NY) * int(CostRefIDX::POSX)] = new_costs(0);             // Position-x
  W[int(CostRefIDX::POSY) + (NY) * int(CostRefIDX::POSY)] = new_costs(0);             // Position-y
  W[int(CostRefIDX::POSZ) + (NY) * int(CostRefIDX::POSZ)] = new_costs(1);             // Position-z
  W[int(CostRefIDX::QUAT_ERR_X) + (NY) * int(CostRefIDX::QUAT_ERR_X)] = new_costs(2); // Quaternion
  W[int(CostRefIDX::QUAT_ERR_Y) + (NY) * int(CostRefIDX::QUAT_ERR_Y)] = new_costs(2); // Quaternion
  W[int(CostRefIDX::QUAT_ERR_Z) + (NY) * int(CostRefIDX::QUAT_ERR_Z)] = new_costs(3); // Quaternion
  W[int(CostRefIDX::VELX) + (NY) * int(CostRefIDX::VELX)] = new_costs(4);             // Velocity-x
  W[int(CostRefIDX::VELY) + (NY) * int(CostRefIDX::VELY)] = new_costs(4);             // Velocity-y
  W[int(CostRefIDX::VELZ) + (NY) * int(CostRefIDX::VELZ)] = new_costs(4);             // Velocity-z
  W[int(CostRefIDX::OMEX) + (NY) * int(CostRefIDX::OMEX)] = new_costs(5);             // Omega-x
  W[int(CostRefIDX::OMEY) + (NY) * int(CostRefIDX::OMEY)] = new_costs(5);             // Omega-y
  W[int(CostRefIDX::OMEZ) + (NY) * int(CostRefIDX::OMEZ)] = new_costs(5);             // Omega-z
  W[int(CostRefIDX::INTX) + (NY) * int(CostRefIDX::INTX)] = new_costs(6);             // PID integral-x
  W[int(CostRefIDX::INTY) + (NY) * int(CostRefIDX::INTY)] = new_costs(6);             // PID integral-y
  W[int(CostRefIDX::INTZ) + (NY) * int(CostRefIDX::INTZ)] = new_costs(6);             // PID integral-z
  W[int(CostRefIDX::THR1) + (NY) * int(CostRefIDX::THR1)] = new_costs(7);             // Thrust 1
  W[int(CostRefIDX::THR2) + (NY) * int(CostRefIDX::THR2)] = new_costs(7);             // Thrust 2
  W[int(CostRefIDX::THR3) + (NY) * int(CostRefIDX::THR3)] = new_costs(7);             // Thrust 3
  W[int(CostRefIDX::THR4) + (NY) * int(CostRefIDX::THR4)] = new_costs(7);             // Thrust 4
  W[int(CostRefIDX::COL_THRUST) + (NY) * int(CostRefIDX::COL_THRUST)] = new_costs(8); // R_col_thrust
  W[int(CostRefIDX::CMD_OME_X) + (NY) * int(CostRefIDX::CMD_OME_X)] = new_costs(9);   // R commanded omega x
  W[int(CostRefIDX::CMD_OME_Y) + (NY) * int(CostRefIDX::CMD_OME_Y)] = new_costs(9);   // R commanded omega y
  W[int(CostRefIDX::CMD_OME_Z) + (NY) * int(CostRefIDX::CMD_OME_Z)] = new_costs(9);   // R commanded omega z

  for (int i = 0; i < N; i++) {
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
  }
  delete (W);

  std::cout << "setNewCosts2" << std::endl;

  double *W_e = new double[NYN * NYN]();
  // change only the non-zero elements:
  W_e[int(CostRefIDX::POSX) + (NYN) * int(CostRefIDX::POSX)] = new_costs(0);             // Position-x
  W_e[int(CostRefIDX::POSY) + (NYN) * int(CostRefIDX::POSY)] = new_costs(0);             // Position-y
  W_e[int(CostRefIDX::POSZ) + (NYN) * int(CostRefIDX::POSZ)] = new_costs(1);             // Position-z
  W_e[int(CostRefIDX::QUAT_ERR_X) + (NYN) * int(CostRefIDX::QUAT_ERR_X)] = new_costs(2); // Quaternion
  W_e[int(CostRefIDX::QUAT_ERR_Y) + (NYN) * int(CostRefIDX::QUAT_ERR_Y)] = new_costs(2); // Quaternion
  W_e[int(CostRefIDX::QUAT_ERR_Z) + (NYN) * int(CostRefIDX::QUAT_ERR_Z)] = new_costs(3); // Quaternion
  W_e[int(CostRefIDX::VELX) + (NYN) * int(CostRefIDX::VELX)] = new_costs(4);             // Velocity-x
  W_e[int(CostRefIDX::VELY) + (NYN) * int(CostRefIDX::VELY)] = new_costs(4);             // Velocity-y
  W_e[int(CostRefIDX::VELZ) + (NYN) * int(CostRefIDX::VELZ)] = new_costs(4);             // Velocity-z
  W_e[int(CostRefIDX::OMEX) + (NYN) * int(CostRefIDX::OMEX)] = new_costs(5);             // Omega-x
  W_e[int(CostRefIDX::OMEY) + (NYN) * int(CostRefIDX::OMEY)] = new_costs(5);             // Omega-y
  W_e[int(CostRefIDX::OMEZ) + (NYN) * int(CostRefIDX::OMEZ)] = new_costs(5);             // Omega-z
  W_e[int(CostRefIDX::INTX) + (NYN) * int(CostRefIDX::INTX)] = new_costs(6);             // PID integral-x
  W_e[int(CostRefIDX::INTY) + (NYN) * int(CostRefIDX::INTY)] = new_costs(6);             // PID integral-y
  W_e[int(CostRefIDX::INTZ) + (NYN) * int(CostRefIDX::INTZ)] = new_costs(6);             // PID integral-z
  W_e[int(CostRefIDX::THR1) + (NYN) * int(CostRefIDX::THR1)] = new_costs(7);             // Thrust 1
  W_e[int(CostRefIDX::THR2) + (NYN) * int(CostRefIDX::THR2)] = new_costs(7);             // Thrust 2
  W_e[int(CostRefIDX::THR3) + (NYN) * int(CostRefIDX::THR3)] = new_costs(7);             // Thrust 3
  W_e[int(CostRefIDX::THR4) + (NYN) * int(CostRefIDX::THR4)] = new_costs(7);             // Thrust 4

  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);

  delete (W_e);
}

//}

/* setNewConstants() //{ */

void acados_drone::setNewConstants(const Eigen::VectorXd &new_constants) {
  parameters[int(AcadosParamsIDX::MASS)] = new_constants(0);       // mass
  parameters[int(AcadosParamsIDX::INERTIA_XX)] = new_constants(2); // inertia x
  parameters[int(AcadosParamsIDX::INERTIA_YY)] = new_constants(3); // inertia y
  parameters[int(AcadosParamsIDX::INERTIA_ZZ)] = new_constants(4); // inertia z
  parameters[int(AcadosParamsIDX::C_TAU)] = new_constants(5);      // ctau

  for (int i = 0; i <= N; i++) {
    if (enable_integral_relaxation and i > integral_relaxation_index) {
      parameters[int(AcadosParamsIDX::K_I_X)] = 0; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = 0; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = 0; // kiz
    } else {
      parameters[int(AcadosParamsIDX::K_I_X)] = loaded_parameters[int(AcadosParamsIDX::K_I_X)]; // kix
      parameters[int(AcadosParamsIDX::K_I_Y)] = loaded_parameters[int(AcadosParamsIDX::K_I_Y)]; // kiy
      parameters[int(AcadosParamsIDX::K_I_Z)] = loaded_parameters[int(AcadosParamsIDX::K_I_Z)]; // kiz
    }
    quadrotor_ode_acados_update_params(acados_ocp_capsule, i, parameters, NP);
  }
}

//}

/* setNewThrustMax() //{ */

void acados_drone::setNewThrustMax(double first_thrust_max, double next_thrust_max) {

  if (std::isnan(next_thrust_max)) {
    next_thrust_max = first_thrust_max;
  }

  double *ubu = new double[NBU]();

  ubu[0] = first_thrust_max;
  ubu[1] = first_thrust_max;
  ubu[2] = first_thrust_max;
  ubu[3] = first_thrust_max;
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubu", ubu);

  ubu[0] = next_thrust_max;
  ubu[1] = next_thrust_max;
  ubu[2] = next_thrust_max;
  ubu[3] = next_thrust_max;
  for (int i = 1; i < N; i++) {
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
  }

  delete (ubu);
}

//}

// --------------------------------------------------------------
// |                        get functions                       |
// --------------------------------------------------------------

/* get_all_computed_states() //{ */

std::vector<DroneState> acados_drone::get_all_computed_states() {
  std::vector<DroneState> output = std::vector<DroneState>();
  Vector<NX> x = Vector<NX>();
  for (int i = 0; i <= N; i++) {
    DroneState a = DroneState();
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x.data());
    a.x << x;
    output.push_back(a);
  }
  return output;
}

//}

/* get_first_control_action() //{ */

Vector<NU> acados_drone::get_first_control_action() {
  Vector<NU> utraj = Vector<NU>();
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &utraj[0]);
  return utraj;
}

//}

/* get_all_control_actions() //{ */

std::vector<Vector<NU>> acados_drone::get_all_control_actions() {
  std::vector<Vector<NU>> output = std::vector<Vector<NU>>();
  Vector<NU> utraj = Vector<NU>();
  for (int i = 0; i < N; i++) {
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", utraj.data());
    output.push_back(utraj);
  }
  return output;
}

//}

/* set_all_init_solutions() //{ */

void acados_drone::set_all_init_solutions(double xtraj[], double utraj[]) {
  // initialize solution
  for (int i = 0; i < N; i++) {
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i * NX]);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", &utraj[i * NX]);
  }
  ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", &xtraj[N * NX]);
}

//}

/* get_all_init_solutions() //{ */

void acados_drone::get_all_init_solutions(double xtraj[], double utraj[]) {
  for (int ii = 0; ii <= nlp_dims->N - 1; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii + 1, "x", &xtraj[ii * NX]);
  for (int ii = 0; ii < nlp_dims->N - 1; ii++)
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii + 1, "u", &utraj[ii * NU]);
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, nlp_dims->N, "x", &xtraj[(nlp_dims->N) * NX]);
  ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, nlp_dims->N - 1, "u", &utraj[(nlp_dims->N - 1) * NU]);
}

//}

/* get_elapsed_time() //{ */

double acados_drone::get_elapsed_time() {
  double elapsed_time;
  ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
  return elapsed_time;
}

//}
