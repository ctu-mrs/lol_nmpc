#pragma once
#include <eigen3/Eigen/Eigen>

union TrajectorySegment {
  Eigen::Matrix<double, 17, 1> vector;
  struct data {
    double time;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;
    Eigen::Vector3d omega;
    Eigen::Vector3d acceleration;
  } data;
  TrajectorySegment() { vector = Eigen::Matrix<double, 17, 1>(); }
  TrajectorySegment(const TrajectorySegment &new_seg) { vector = new_seg.vector; }
  //   enum IDX : int {
  //     POS = 0,
  //     VEL = 3,
  //     ACC = 6,
  //     JER = 9,
  //     HEADINGS = 12,
  //   };
};

struct PmmTrajectory {
  enum IDX : int {
    TIME = 0,
    POS = 1,
    POSX = 1,
    POSY = 2,
    POSZ = 3,
    VEL = 4,
    VELX = 4,
    VELY = 5,
    VELZ = 6,
    ACC = 7,
    ACCX = 7,
    ACCY = 8,
    ACCZ = 9,
    // Sizes
    NVEL = 3,
    NPOS = 3,
    NACC = 3,
    SIZE = 10, // you can set the size lower to read less data (last index + 1)
    // Unused placeholders
    ATTW = -1,
    ATTX = -1,
    ATTY = -1,
    ATTZ = -1,
    OMEGAX = -1,
    OMEGAY = -1,
    OMEGAZ = -1,
    HEADING = -1,
  };
  bool ignore_header;
  bool use_time;
  bool use_jerk;
  bool use_orientation;
  bool use_heading;
  bool use_omega;
  PmmTrajectory() {
    ignore_header = false;
    use_time = true;
    use_jerk = false;
    use_orientation = false;
    use_heading = false;
    use_omega = false;
  }; // default constructor
};

struct CpcTrajectory {
  enum IDX : int {
    TIME = 0,
    POS = 1,
    POSX = 1,
    POSY = 2,
    POSZ = 3,
    ATT = 4,
    ATTW = 4,
    ATTX = 5,
    ATTY = 6,
    ATTZ = 7,
    VEL = 8,
    VELX = 8,
    VELY = 9,
    VELZ = 10,
    OMEGA = 11,
    OMEGAX = 11,
    OMEGAY = 12,
    OMEGAZ = 13,
    ACC = 14,
    ACCX = 14,
    ACCY = 15,
    ACCZ = 16,
    // reading upto here by setting SIZE to 17
    ALPHA = 17,
    ALPHAX = 17,
    ALPHAY = 18,
    ALPHAZ = 19,
    U = 20,
    U1 = 20,
    U2 = 21,
    U3 = 22,
    U4 = 23,
    // Sizes
    NPOS = 3,
    NATT = 4,
    NVEL = 3,
    NOMEGA = 3,
    NACC = 3,
    NALPHA = 3,
    NU = 4,
    SIZE = 17, // you can set the size lower to read less data (last index + 1)
    // Unused placeholders
    HEADING = -1
  };
  bool ignore_header;
  bool use_time;
  bool use_jerk;
  bool use_orientation;
  bool use_heading;
  bool use_omega;
  CpcTrajectory() {
    ignore_header = true;
    use_time = true;
    use_jerk = false;
    use_orientation = true;
    use_heading = false;
    use_omega = true;
  }; // default constructor
};

struct PolyTrajectory {
  enum IDX : int {
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    VEL = 3,
    VELX = 3,
    VELY = 4,
    VELZ = 5,
    ACC = 6,
    ACCX = 6,
    ACCY = 7,
    ACCZ = 8,
    JER = 9,
    JERX = 9,
    JERY = 10,
    JERZ = 11,
    HEADING = 12,
    HEADINGRATE = 13,
    // reading upto here by setting SIZE to 14
    EMPTY = 14,
    // sizes
    NVEL = 3,
    NPOS = 3,
    NACC = 3,
    NJER = 3,
    SIZE = 14, // you can set the size lower to read less data
    // Unused placeholders
    ATTW = -1,
    ATTX = -1,
    ATTY = -1,
    ATTZ = -1,
    OMEGAX = -1,
    OMEGAY = -1,
    OMEGAZ = -1,
    TIME = -1
  };
  bool ignore_header;
  bool use_jerk;
  bool use_orientation;
  bool use_omega;
  bool use_heading;
  bool use_time;
  PolyTrajectory() {
    ignore_header = false;
    use_jerk = false;
    use_orientation = false;
    use_omega = false;
    use_heading = true;
    use_time = false;
  }; // default constructor
};

std::ofstream &operator<<(std::ofstream &os, const TrajectorySegment &ts) {
  os << ts.data.pos(0) << ", " << ts.data.pos(1) << ", " << ts.data.pos(2) << ", " << ts.data.vel(0) << ", " << ts.data.vel(1) << ", " << ts.data.vel(2) << ", "
     << ts.data.quat.w() << ", " << ts.data.quat.x() << ", " << ts.data.quat.y() << ", " << ts.data.quat.z() << ", " << ts.data.omega(0) << ", "
     << ts.data.omega(1) << ", " << ts.data.omega(2);
  return os;
}
// const int i = sizeof(TrajectorySegment) / sizeof(double);

// const int b = sizeof(Eigen::Vector<double, 16>) / sizeof(double);

struct MetadataSegment {
  double max_vel;          // maximum square of velocity
  double slow_coefficient; // from 0 to 1; 1 = normal speed
  double new_traj_idx;
};

Eigen::Vector3d clamp(Eigen::Vector3d value, double max_size) {
  double norm = value.norm();
  if (norm > max_size) {
    return value * (max_size / norm);
  }
  return value;
}

double interpolate(double decimal, double val1, double val2) {
  decimal = decimal - ((int)decimal);
  return decimal * val2 + (1 - decimal) * val1;
}

Eigen::Vector3d interpolate(double decimal, Eigen::Vector3d val1, Eigen::Vector3d val2) {
  decimal = decimal - ((int)decimal);
  return decimal * val2 + (1 - decimal) * val1;
}
inline double square_size(Eigen::Vector3d vec) { return vec(0) * vec(0) + vec(1) * vec(1) + vec(2) * vec(2); }

inline double square_size(Eigen::Vector2d vec) { return vec(0) * vec(0) + vec(1) * vec(1); }
