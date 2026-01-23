/*This is copied from
 * https://github.com/uzh-rpg/sb_min_time_quadrotor_planning*/

#pragma once

// #include <limits>
#include <common.hpp>
#include <tuple>


#define EQUALITY_ERROR (0.0001)
#define PRECISION (1E-4)

static constexpr Scalar G = 9.8066;
const Vector<3> GVEC{0, 0, -G};

Matrix<3, 3> skew(const Vector<3> &v);
Matrix<4, 4> Q_left(const Quaternion &q);
Matrix<4, 4> Q_right(const Quaternion &q);

struct DroneState {
  enum IDX : int {
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    OME = 10,
    OMEX = 10,
    OMEY = 11,
    OMEZ = 12,
    NOME = 3,
    INT = 13,
    INTX = 13,
    INTY = 14,
    INTZ = 15,
    NINT = 3,
    T = 16,
    T1 = 16,
    T2 = 17,
    T3 = 18,
    T4 = 19,
    NT = 4,
    // SIZE = 16,
    SIZE = 20
  };

  DroneState() {};
  DroneState(const Vector<IDX::SIZE> &x, const Vector<4> &u, const Scalar t = NAN) : x(x), u(u), t(t) {}
  DroneState(const DroneState &state) : x(state.x), u(state.u), t(state.t) {}

  void setZero() {
    x.setZero();
    u.setZero();
    t = 0;
    x(ATTW) = 1.0;
  }
  Vector<IDX::SIZE> x = Vector<IDX::SIZE>::Constant(NAN);
  Vector<4> u = Vector<4>::Constant(NAN);

  Ref<Vector<3>> p{x.segment<IDX::NPOS>(IDX::POS)};
  Ref<Vector<4>> qx{x.segment<IDX::NATT>(IDX::ATT)};
  Ref<Vector<3>> v{x.segment<IDX::NVEL>(IDX::VEL)};
  Ref<Vector<3>> w{x.segment<IDX::NOME>(IDX::OME)};
  Ref<Vector<3>> errorIntegral{x.segment<IDX::NINT>(IDX::INT)};
  Ref<Vector<4>> thrusts{x.segment<IDX::NT>(IDX::T)};
  // Ref<Vector<3>> a{x.segment<IDX::NACC>(IDX::ACC)};
  Quaternion q() const;

  Scalar t{NAN};
};

std::ostream &operator<<(std::ostream &o, const DroneState &s);

class Drone {
public:
  Drone();
  Drone(const std::string yaml_file_location);
  Drone(double m, double l, double kappa, Vector<3> inertia_diag);
  Scalar l_;
  Scalar m_;
  Vector<2> mot0_pos_;
  Vector<2> mot1_pos_;
  Vector<2> mot2_pos_;
  Vector<2> mot3_pos_;
  Scalar kappa_;
  Scalar max_integral_error_xy_;
  Scalar max_integral_error_z_;
  Matrix<3, 3> J_;
  Matrix<3, 3> J_inv_;
  Matrix<3, 3> kp_pid_mat_;
  Matrix<3, 3> ki_pid_mat_;
  Matrix<3, 3> kd_pid_mat_;
  Vector<3> integrated_error_;
  Matrix<4, 4> motor_allocation_;
  Matrix<4, 4> motor_allocation_inv_;
  void rk4(const Ref<const Vector<>> initial_state, const Scalar dt, const Ref<const Vector<4>> T, Ref<Vector<>> final_state);
  void get_derivative(const Ref<const Vector<DroneState::SIZE>> state, const Ref<const Vector<4>> T, Ref<Vector<DroneState::SIZE>> derivative);
  DroneState simulate(DroneState state, const Scalar dt, Vector<4> input);
  Vector<4> runMotors(const Scalar sim_dt, const Vector<4> &motor_thrusts_des, const Vector<4> &motor_thrusts_current);
};
