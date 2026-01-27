
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/String.h>
#include <mrs_uav_managers/controller.h>
#include <mrs_uav_managers/tracker.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "trajectory.hpp"
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <vector>

template <int rows = Eigen::Dynamic> using Vector = Eigen::Matrix<double, rows, 1>;

// Using `Quaternion` with type.
using Quaternion = Eigen::Quaternion<double>;

namespace mrs_uav_trackers {

/* class CsvTracker //{ */

class CsvTracker : public mrs_uav_managers::Tracker {
public:
  ~CsvTracker(){};

  bool initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);
  std::tuple<bool, std::string> activate(const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd);
  void deactivate(void);
  bool resetStatic(void);

  std::optional<mrs_msgs::TrackerCommand> update(const mrs_msgs::UavState &uav_state, const mrs_uav_managers::Controller::ControlOutput &last_control_output);
  const mrs_msgs::TrackerStatus getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

  const mrs_msgs::ReferenceSrvResponse::ConstPtr setReference(const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr setVelocityReference(const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd);
  const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  // bool setThrustConstrain(const double max_acc);

private:
  ros::NodeHandle nh_;
  std::atomic<bool> is_active = false;
  std::atomic<int> is_initialized = 0;
  bool callbacks_enabled = false;
  std::string csv_url;
  std::string _uav_name_;

  double trajectory_time_length_ = 0;
  std::string trajectory_file_;
  int trajectory_type_ = 2;
  bool _velocity_is_heading_ = false;
  bool _acceleration_is_heading_ = false;
  bool _add_initial_position_ = false;

  geometry_msgs::PoseArray full_trajectory_out;
  // mrs_msgs::MpcPredictionFullState full_trajectory;
  mrs_msgs::MpcPredictionFullState msg_ref;
  // ros::Publisher publish_trajectory_;
  ros::Publisher publish_full_trajectory_;

  ros::Publisher publish_diagnostics_;
  std::vector<TrajectorySegment> trajectory; // original trajectory from CSV
  ros::Time activation_time;
  double actual_time;

  ros::ServiceServer fly_to_start_service_;
  bool fly_to_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  ros::ServiceClient cm_reference_client_;
  ros::ServiceClient cm_switch_tracker_client_;

  std::string _pos_ref_frame_;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;
  Eigen::Vector3d _initial_position_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d first_trajectory_position = Eigen::Vector3d(0, 0, 0);

  std::tuple<bool, std::string> load_trajectory();
  template <typename T> void read_traj(T trajectory_struct, std::string url, double time_delta);
  void visualizeFullTrajectory();
  bool check_yaml_and_trajectory();
  std::mutex mutex_trajectory_;

  // helper functions to convert between Eigen and ROS msgs
  geometry_msgs::Point vec_to_point(Eigen::Vector3d vec);
  geometry_msgs::Vector3 vec_to_msg(Eigen::Vector3d vec);
  geometry_msgs::Quaternion vec_to_msg(Eigen::Quaterniond vec);
};

//}

// | ------------------- trackers interface ------------------- |

/* //{ initialize() */

bool CsvTracker::initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                            std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {
  this->common_handlers_ = common_handlers;
  this->private_handlers_ = private_handlers;

  _uav_name_ = common_handlers->uav_name;

  nh_ = nh;

  ros::Time::waitForValid();

  bool success = check_yaml_and_trajectory();

  fly_to_start_service_ = nh_.advertiseService("fly_to_start", &CsvTracker::fly_to_start, this);
  std::string reference_srv = std::string("/") + common_handlers->uav_name + std::string("/control_manager/reference");
  cm_reference_client_ = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>(reference_srv);
  std::string switch_tracker_srv = std::string("/") + common_handlers->uav_name + std::string("/control_manager/switch_tracker");
  cm_switch_tracker_client_ = nh_.serviceClient<mrs_msgs::String>(switch_tracker_srv);

  publish_full_trajectory_ = nh_.advertise<geometry_msgs::PoseArray>("FullTrajectory", 1);
  publish_diagnostics_ = nh_.advertise<std_msgs::String>("Diagnostics", 1, true);

  ROS_INFO("[CsvTracker]: initialized");
  is_initialized = 1;
  return true;
}

//}

/* //{ check_yaml_and_trajectory() */
bool CsvTracker::check_yaml_and_trajectory() {
  std::string config_url = ros::package::getPath("csv_tracker") + "/config/csv_tracker_config.yaml";
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_url);
  } catch (YAML::BadFile &e) {
    ROS_ERROR_STREAM("[CsvTracker]: yaml config file at url "
                     << config_url << " does not exist. Initialising tracker but will not be able to activate and fly unless you change it");
    return false;
  }
  if (!config["trajectory_file"] || !config["trajectory_type"] || !config["trajectory_time_length"] || !config["add_initial_position"] ||
      !config["velocity_is_heading"] || !config["acceleration_is_heading"]) {
    ROS_ERROR_STREAM("[CsvTracker]: config parameter missing, check that you have trajectory_file, trajectory_type, trajectory_time_length, add_initial_position, velocity_is_heading, acceleration_is_heading in the yaml");
    return false;
  }

  trajectory_file_ = config["trajectory_file"].as<std::string>();
  ROS_INFO_STREAM("[CsvTracker]: Trajectory file: " << trajectory_file_);

  trajectory_type_ = config["trajectory_type"].as<int>();
  ROS_INFO_STREAM("[CsvTracker]: Trajectory type " << trajectory_type_);

  trajectory_time_length_ = config["trajectory_time_length"].as<double>();
  ROS_INFO_STREAM("[CsvTracker]: Trajectory time length " << trajectory_time_length_);

  _add_initial_position_ = config["add_initial_position"].as<bool>();
  ROS_INFO_STREAM("[CsvTracker]: add initial position " << _add_initial_position_);

  _velocity_is_heading_ = config["velocity_is_heading"].as<bool>();
  ROS_INFO_STREAM("[CsvTracker]: velocity is heading" << _velocity_is_heading_);

  _acceleration_is_heading_ = config["acceleration_is_heading"].as<bool>();
  ROS_INFO_STREAM("[CsvTracker]: acceleration is heading" << _acceleration_is_heading_);

  csv_url = ros::package::getPath("csv_tracker") + "/trajectories/" + trajectory_file_;
  std::ifstream file(csv_url);
  if (file.fail()) {
    ROS_ERROR_STREAM("[CsvTracker]: trajectory file at url " << csv_url << " does not exist.\n Tracker will not activate if the file does not exist.");
    return false;
  }
  if (trajectory_type_ == 1) {
    ROS_INFO_STREAM("[CsvTracker]: Polynomial type of trajectory " << trajectory_type_);
  } else if (trajectory_type_ == 2) {
    ROS_INFO_STREAM("[CsvTracker]: CPC time optimal type of trajectory " << trajectory_type_);
  } else if (trajectory_type_ == 3) {
    ROS_INFO_STREAM("[CsvTracker]: PMM time optimal type of trajectory " << trajectory_type_);
  } else {
    ROS_ERROR_STREAM("[CsvTracker]: Trajectory type " << trajectory_type_ << "is unknown \n Tracker will initialize but not activate unless you change it.\n");
    return false;
  }
  return true;
}

//}

/* load_trajectory() //{ */

std::tuple<bool, std::string> CsvTracker::load_trajectory() {
  /* read_traj_poly(csv_url); */
  if (trajectory_type_ == 1) { // polynomial
    // read_traj_poly(csv_url);
    PolyTrajectory poly_trajectory;
    read_traj<PolyTrajectory>(poly_trajectory, csv_url, 0.01);
  } else if (trajectory_type_ == 2) { // CPC
    // read_traj_cpc(csv_url);
    CpcTrajectory cpc_trajectory;
    read_traj<CpcTrajectory>(cpc_trajectory, csv_url, 0.01);
  } else if (trajectory_type_ == 3) { // PMM
    // read_traj_pmm(csv_url);
    PmmTrajectory pmm_trajectory;
    read_traj<PmmTrajectory>(pmm_trajectory, csv_url, 0.01);
  } else {
    ROS_ERROR_STREAM("unknown type of trajectory " << trajectory_type_);
    return std::tuple(false, "error");
  }
  is_initialized = 1;
  return std::tuple(true, "success");
}

//}

// /* read_traj //{ */
template <typename T> void CsvTracker::read_traj(T trajectory_struct, std::string url, double time_delta) {

  // check if file exists
  std::ifstream file(url);
  if (file.fail()) {
    ROS_ERROR_STREAM("file at url " << url << " does not exist.");
    return;
  }
  std::scoped_lock lock(mutex_trajectory_);
  std_msgs::String diagnostics_msg;
  diagnostics_msg.data = trajectory_file_;
  publish_diagnostics_.publish(diagnostics_msg);
  if (!trajectory_struct.use_time) {
    ROS_INFO("[CsvTracker]: time not used in trajectory, using time_delta supplied");
  }
  ROS_INFO("[CsvTracker]: starting read file");
  ros::Time stamp = ros::Time::now();
  int line_number = 0;
  trajectory.clear();

  TrajectorySegment segment = TrajectorySegment();
  std::string line;
  int size_of_struct = T::SIZE;
  std::vector<double> numbers(size_of_struct);
  std::vector<std::vector<double>> loaded_csv;
  file.seekg(0);
  // move to line 1
  if (trajectory_struct.ignore_header) {
    getline(file, line);
  }
  // depending on the if, reading starts at line 1 or line 2
  // load everything now
  while (getline(file, line)) {
    for (int i = 0; i < size_of_struct; i++) {
      int end_idx = line.find(',');
      std::string number = line.substr(0, end_idx);
      numbers[i] = stod(number);
      line = line.substr(end_idx + 1, line.length() - end_idx - 1);
    }
    loaded_csv.push_back(numbers);
  }
  file.close();
  std::cout << "[CsvTracker]: num points without repeat " << loaded_csv.size() << std::endl;

  // artifitial delay 2 seconds on start by storing the first position and repeating it
  // {
  numbers = loaded_csv[0];
  segment.data.pos(0) = numbers[T::POSX];
  segment.data.pos(1) = numbers[T::POSY];
  segment.data.pos(2) = numbers[T::POSZ];
  first_trajectory_position = segment.data.pos;
  segment.data.quat.setIdentity();
  if (trajectory_struct.use_orientation) {
    segment.data.quat = Quaternion(numbers[T::ATTW], numbers[T::ATTX], numbers[T::ATTY], numbers[T::ATTZ]);
  } else {
    Vector<3> acceleration;
    acceleration(0) = numbers[T::ACCX];
    acceleration(1) = numbers[T::ACCY];
    acceleration(2) = numbers[T::ACCZ] + 9.806;
    double heading = 0;
    if (trajectory_struct.use_heading) {
      heading = numbers[T::HEADING];
    }
    double acc_norm = acceleration.norm();
    Vector<3> z_body = acceleration.normalized();
    Vector<3> x_C = Vector<3>(cos(heading), sin(heading), 0);
    if (_velocity_is_heading_) {
      x_C = Vector<3>(numbers[T::VELX], numbers[T::VELY], 0);
    }
    if (_acceleration_is_heading_) {
      x_C = Vector<3>(numbers[T::ACCX], numbers[T::ACCY], 0);
    }
    x_C.normalize();
    Vector<3> y_body = (z_body.cross(x_C)).normalized();
    Vector<3> x_body = y_body.cross(z_body);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << x_body, y_body, z_body;
    segment.data.quat = Eigen::Quaterniond(rotation_matrix);
  }
  segment.data.vel.setZero();
  segment.data.omega.setZero();
  segment.data.acceleration.setZero();

  for (int i = 0; i < 100; i++) {
    segment.data.time = i * 0.01;
    trajectory.push_back(segment);
  }
  // } holding position for 2 seconds

  double add_time_start = trajectory.back().data.time + time_delta;

  for (int idx_csv = 0; idx_csv < loaded_csv.size(); idx_csv++) {
    numbers = loaded_csv[idx_csv];
    if (trajectory_struct.use_time) {
      segment.data.time = numbers[T::TIME] + add_time_start;
    } else {
      segment.data.time = idx_csv * time_delta + add_time_start;
    }
    segment.data.pos(0) = numbers[T::POSX];
    segment.data.pos(1) = numbers[T::POSY];
    segment.data.pos(2) = numbers[T::POSZ];
    if (trajectory_struct.use_orientation) {
      segment.data.quat = Quaternion(numbers[T::ATTW], numbers[T::ATTX], numbers[T::ATTY], numbers[T::ATTZ]);
    } else {
      Vector<3> acceleration;
      acceleration(0) = numbers[T::ACCX];
      acceleration(1) = numbers[T::ACCY];
      acceleration(2) = numbers[T::ACCZ] + 9.806;
      double heading = 0;
      if (trajectory_struct.use_heading) {
        heading = numbers[T::HEADING];
      }
      double acc_norm = acceleration.norm();
      Vector<3> z_body = acceleration.normalized();
      Vector<3> x_C = Vector<3>(cos(heading), sin(heading), 0);
      if (_velocity_is_heading_) {
        x_C = Vector<3>(numbers[T::VELX], numbers[T::VELY], 0);
      }
      if (_acceleration_is_heading_) {
        x_C = Vector<3>(numbers[T::ACCX], numbers[T::ACCY], 0);
      }
      x_C.normalize();
      Vector<3> y_body = (z_body.cross(x_C)).normalized();
      Vector<3> x_body = y_body.cross(z_body);
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix << x_body, y_body, z_body;
      segment.data.quat = Eigen::Quaterniond(rotation_matrix);
    }

    /* ROS_ERROR_STREAM("QUAT: " << segment.data.quat.w() << " " << segment.data.quat.x() << " " << segment.data.quat.y() << " " << segment.data.quat.z()); */

    segment.data.vel(0) = numbers[T::VELX];
    segment.data.vel(1) = numbers[T::VELY];
    segment.data.vel(2) = numbers[T::VELZ];
    if (trajectory_struct.use_omega) {
      segment.data.omega(0) = numbers[T::OMEGAX];
      segment.data.omega(1) = numbers[T::OMEGAY];
      segment.data.omega(2) = numbers[T::OMEGAZ];
    } else {
      segment.data.omega.setZero();
    }
    segment.data.acceleration(0) = numbers[T::ACCX];
    segment.data.acceleration(1) = numbers[T::ACCY];
    segment.data.acceleration(2) = numbers[T::ACCZ];

    for (int i = 0; i < segment.vector.size(); i++) {
      if (isnan(segment.vector[i])) {
        ROS_ERROR_STREAM("NAN_ in i=" << trajectory.size() << ", segment: " << segment.vector);
      }
    }

    trajectory.push_back(segment);
    line_number++;
  }
  segment.vector = trajectory.back().vector;
  segment.data.time += time_delta;
  segment.data.vel.setZero();
  segment.data.omega.setZero();
  segment.data.quat.setIdentity();
  segment.data.acceleration.setZero();
  trajectory.push_back(segment);
  ROS_INFO_STREAM("[CsvTracker] line_number: " << line_number);

  activation_time = ros::Time::now();
}

//}

/* //{ fly_to_start() */

bool CsvTracker::fly_to_start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  mrs_msgs::String tracker_to_switch;
  tracker_to_switch.request.value = "MpcTracker";
  if (cm_switch_tracker_client_.call(tracker_to_switch)) {
    if (tracker_to_switch.response.success) {
      ROS_INFO("[CsvTracker]: switched to MpcTracker");
    } else {
      res.success = false;
      res.message = "can not switch to MpcTracker";
      ROS_ERROR_STREAM("[CsvTracker]: can not switch to MpcTracker " << tracker_to_switch.response.message);
    }
  } else {
    res.success = false;
    res.message = "can not switch to MpcTracker";
    ROS_ERROR("[CsvTracker]: can not switch to MpcTracker");
    return true;
  }

  if (trajectory.size() > 0) {
    mrs_msgs::ReferenceStampedSrv srv;
    srv.request.reference.position.x = trajectory[0].data.pos(0);
    srv.request.reference.position.y = trajectory[0].data.pos(1);
    srv.request.reference.position.z = trajectory[0].data.pos(2);
    Eigen::Vector3d x_body(1.0, 0.0, 0.0);
    Eigen::Vector3d x_world = trajectory[0].data.quat * x_body;
    srv.request.reference.heading = atan2(x_world.y(), x_world.x());
    if (cm_reference_client_.call(srv)) {
      ROS_INFO("[CsvTracker]: Reference for flying to start called successfully");
      res.success = true;
      res.message = "flying to start";
      ROS_INFO("[CsvTracker]: flying to trajectory start");
    } else {
      res.success = false;
      res.message = "could not call tracker reference";
      ROS_ERROR("[CsvTracker]: could not call tracker reference");
    }
  } else {
    res.success = false;
    res.message = "not trajectory to fly";
    ROS_ERROR("[CsvTracker]: no trajectory to fly");
  }

  return true;
}

//}

/* //{ activate() */

std::tuple<bool, std::string> CsvTracker::activate([[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand> &last_tracker_cmd) {
  ROS_INFO("[CsvTracker]: activated");

  if (!check_yaml_and_trajectory()) {
    return std::tuple(false, "yaml or trajectory file not found");
  }
  std::cout << "All specified things look correct, attempting to load trajectory\n";

  std::tuple<bool, std::string> load_status = load_trajectory();
  if (!std::get<0>(load_status)) {
    return load_status;
  }
  std::cout << "Trajectory loaded upon activation \n";

  is_active = true;
  // reset the starting position to be used for trajectory by setting is_initialized to 1
  is_initialized = 1;
  return std::tuple(true, "activated");
}

//}

/* //{ visualizeFullTrajectory() */
void CsvTracker::visualizeFullTrajectory() {
  full_trajectory_out.header.stamp = ros::Time::now();
  full_trajectory_out.header.frame_id = _pos_ref_frame_;
  full_trajectory_out.poses.clear();

  for (int i = 0; i < trajectory.size(); i++) {

    geometry_msgs::Pose newPose;
    if (_add_initial_position_) {
      newPose.position.x = trajectory[i].data.pos[0] + _initial_position_[0] - first_trajectory_position(0);
      newPose.position.y = trajectory[i].data.pos[1] + _initial_position_[1] - first_trajectory_position(1);
      newPose.position.z = trajectory[i].data.pos[2] + _initial_position_[2] - first_trajectory_position(2);
    } else {
      newPose.position.x = trajectory[i].data.pos[0];
      newPose.position.y = trajectory[i].data.pos[1];
      newPose.position.z = trajectory[i].data.pos[2];
    }
    newPose.orientation.w = trajectory[i].data.quat.w();
    newPose.orientation.x = trajectory[i].data.quat.x();
    newPose.orientation.y = trajectory[i].data.quat.y();
    newPose.orientation.z = trajectory[i].data.quat.z();

    full_trajectory_out.poses.push_back(newPose);
  }

  publish_full_trajectory_.publish(full_trajectory_out);
  ROS_INFO("Full reference trajectory visualized.");
}

//}

/* //{ deactivate() */

void CsvTracker::deactivate(void) {
  ROS_INFO("[CsvTracker]: deactivated");
  is_active = false;
}

//}

/* //{ resetStatic() */

bool CsvTracker::resetStatic(void) { return false; }

//}

/* switchOdometrySource() //{ */

const std_srvs::TriggerResponse::ConstPtr CsvTracker::switchOdometrySource(const mrs_msgs::UavState &new_uav_state) {
  ROS_INFO("[CsvTracker]: Odometry source switched");
  return std_srvs::TriggerResponse::Ptr();
}

//}

// --------------------------------------------------------------
// |                         main update                        |
// --------------------------------------------------------------

/* update //{ */
std::optional<mrs_msgs::TrackerCommand> CsvTracker::update(const mrs_msgs::UavState &uav_state,
                                                           [[maybe_unused]] const mrs_uav_managers::Controller::ControlOutput &last_control_output) {
  if (!is_active) {
    return {};
  }
  if (is_initialized == 1) {
    _initial_position_(0) = uav_state.pose.position.x;
    _initial_position_(1) = uav_state.pose.position.y;
    _initial_position_(2) = uav_state.pose.position.z;
    _pos_ref_frame_ = uav_state.header.frame_id;

    visualizeFullTrajectory();

    is_initialized = 2;
    ROS_INFO_STREAM("[CsvTracker]: initial position set");
  }

  std::scoped_lock lock(mutex_trajectory_);
  msg_ref = {};
  msg_ref.header.stamp = ros::Time::now();
  msg_ref.header.frame_id = _pos_ref_frame_;
  ros::Time filling_time = ros::Time::now();
  int start_idx = 0;
  int end_idx = 0;

  // We find the time index of the trajectory that is closest to the current time but behind it
  // This is the time index from which we will start filling the reference message
  while (filling_time.toSec() > activation_time.toSec() + trajectory[start_idx].data.time and start_idx < trajectory.size()) {
    start_idx++;
  }
  start_idx = (start_idx == 0) ? 0 : start_idx - 1; // if we are at the first index, we do not need to go back

  // We find the time index that is at least trajectory_time_length_ ahead of the current time
  while (filling_time.toSec() + trajectory_time_length_ > activation_time.toSec() + trajectory[end_idx].data.time and end_idx < trajectory.size()) {
    end_idx++;
  }

  bool trajectory_ended = false;
  if (end_idx == trajectory.size()) {
    trajectory_ended = true;
    ROS_INFO_STREAM_THROTTLE(1, "[Csv tracker] end of trajectory reached");
    ROS_INFO_STREAM_THROTTLE(1, "[Csv tracker] begin_time_index " << start_idx);
    ROS_INFO_STREAM_THROTTLE(1, "[Csv tracker] end_time_index " << end_idx);
    end_idx -= 1;
  }

  for (int j = start_idx; j < end_idx; j++) {
    msg_ref.stamps.push_back(activation_time + ros::Duration(trajectory[j].data.time));
    if (_add_initial_position_) {
      msg_ref.position.push_back(vec_to_point(trajectory[j].data.pos + _initial_position_ - first_trajectory_position));
    } else {
      msg_ref.position.push_back(vec_to_point(trajectory[j].data.pos));
    }
    msg_ref.orientation.push_back(vec_to_msg(trajectory[j].data.quat));
    msg_ref.attitude_rate.push_back(vec_to_msg(trajectory[j].data.omega));
    msg_ref.velocity.push_back(vec_to_msg(trajectory[j].data.vel));
    msg_ref.acceleration.push_back(vec_to_msg(trajectory[j].data.acceleration));
  }

  if (trajectory_ended) { // if we reached the end of the trajectory, we fill the rest of the message with the last point
    int num_count_end = 20;
    ros::Time now = ros::Time::now();
    double dt = trajectory_time_length_ / num_count_end;
    for (int j = 0; j < num_count_end; j++) {
      msg_ref.stamps.push_back(now + ros::Duration(j * dt));
      if (_add_initial_position_) {
        msg_ref.position.push_back(vec_to_point(trajectory.back().data.pos + _initial_position_ - first_trajectory_position));
      } else {
        msg_ref.position.push_back(vec_to_point(trajectory.back().data.pos));
      }
      msg_ref.orientation.push_back(vec_to_msg(trajectory.back().data.quat));
      msg_ref.attitude_rate.push_back(vec_to_msg(trajectory.back().data.omega));
      msg_ref.velocity.push_back(vec_to_msg(trajectory.back().data.vel));
      msg_ref.acceleration.push_back(vec_to_msg(trajectory.back().data.acceleration));
    }
  }

  msg_ref.use_orientation = 1;
  msg_ref.use_attitude_rate = 1;

  // you have to return a position command
  mrs_msgs::TrackerCommand position_cmd;
  // set the header
  position_cmd.header.stamp = uav_state.header.stamp;
  position_cmd.header.frame_id = _pos_ref_frame_;

  // set positions
  position_cmd.position.x = msg_ref.position[0].x;
  position_cmd.position.y = msg_ref.position[0].y;
  position_cmd.position.z = msg_ref.position[0].z;
  position_cmd.use_position_vertical = 1;
  position_cmd.use_position_horizontal = 1;

  // set velocities
  position_cmd.velocity.x = msg_ref.velocity[0].x;
  position_cmd.velocity.y = msg_ref.velocity[0].y;
  position_cmd.velocity.z = msg_ref.velocity[0].z;
  position_cmd.use_velocity_vertical = 1;
  position_cmd.use_velocity_horizontal = 1;

  position_cmd.orientation.w = msg_ref.orientation[0].w;
  position_cmd.orientation.x = msg_ref.orientation[0].x;
  position_cmd.orientation.y = msg_ref.orientation[0].y;
  position_cmd.orientation.z = msg_ref.orientation[0].z;
  position_cmd.use_orientation = 1;

  position_cmd.attitude_rate.x = msg_ref.attitude_rate[0].x;
  position_cmd.attitude_rate.y = msg_ref.attitude_rate[0].y;
  position_cmd.attitude_rate.z = msg_ref.attitude_rate[0].z;
  position_cmd.use_attitude_rate = 1;

  position_cmd.full_state_prediction = msg_ref;
  position_cmd.use_full_state_prediction = 1;

  return {position_cmd};
}

//}

/* //{ getStatus() */

const mrs_msgs::TrackerStatus CsvTracker::getStatus() {
  mrs_msgs::TrackerStatus tracker_status;

  tracker_status.active = is_active;
  tracker_status.callbacks_enabled = callbacks_enabled;
  // ROS_INFO("[CsvTracker]: getStatus");
  return tracker_status;
}

//}

/* //{ enableCallbacks() */

const std_srvs::SetBoolResponse::ConstPtr CsvTracker::enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd) {
  std_srvs::SetBoolResponse res;

  std::stringstream ss;

  if (cmd->data != callbacks_enabled) {
    callbacks_enabled = cmd->data;

    ss << "callbacks " << (callbacks_enabled ? "enabled" : "disabled");

    ROS_DEBUG_STREAM("[CsvTracker]: " << ss.str());

  } else {
    ss << "callbacks were already " << (callbacks_enabled ? "enabled" : "disabled");
  }

  res.message = ss.str();
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(std::make_unique<std_srvs::SetBoolResponse>(res));
}

//}

/* //{ setReference() */

const mrs_msgs::ReferenceSrvResponse::ConstPtr CsvTracker::setReference([[maybe_unused]] const mrs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: setReference");
  return mrs_msgs::ReferenceSrvResponse::Ptr();
}

//}

/* //{ setVelocityReference() */

const mrs_msgs::VelocityReferenceSrvResponse::ConstPtr CsvTracker::setVelocityReference([
    [maybe_unused]] const mrs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: setVelocityRef");
  return mrs_msgs::VelocityReferenceSrvResponse::Ptr();
}

//}

/* //{ setTrajectoryReference() */

const mrs_msgs::TrajectoryReferenceSrvResponse::ConstPtr CsvTracker::setTrajectoryReference([
    [maybe_unused]] const mrs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: setTrajectoryRef");
  return mrs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}

//}

// | --------------------- other services --------------------- |

/* //{ hover() */

const std_srvs::TriggerResponse::ConstPtr CsvTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: hover");
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ startTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr CsvTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: startTrajTrack");
  std_srvs::TriggerResponse response;

  if (!check_yaml_and_trajectory()) {
    response.success = false;
    response.message = "yaml or trajectory file not found";
    return std_srvs::TriggerResponse::Ptr(new std_srvs::TriggerResponse(response));
  }

  std::tuple<bool, std::string> load_status = load_trajectory();
  if (!std::get<0>(load_status)) {
    response.success = false;
    response.message = std::get<1>(load_status);
    return std_srvs::TriggerResponse::Ptr(new std_srvs::TriggerResponse(response));
  }
  response.success = true;
  response.message = "[CsvTracker]: trajectory reloaded";
  return std_srvs::TriggerResponse::Ptr(new std_srvs::TriggerResponse(response));
}

//}

/* //{ stopTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr CsvTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: stopTrajTrack");
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ resumeTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr CsvTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: resumeTrajTrack");
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr CsvTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: gotoTrajStart");
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr CsvTracker::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd) {
  ROS_INFO("[CsvTracker]: setConstrains");
  return mrs_msgs::DynamicsConstraintsSrvResponse::Ptr();
}

//}

// | -------------------- helper functions ------------------- |

/* //{ vec_to_point() */
geometry_msgs::Point CsvTracker::vec_to_point(Eigen::Vector3d vec) {
  geometry_msgs::Point out;
  out.x = vec(0);
  out.y = vec(1);
  out.z = vec(2);
  return out;
}
//}

/* //{ vec_to_msg() */
geometry_msgs::Vector3 CsvTracker::vec_to_msg(Eigen::Vector3d vec) {
  geometry_msgs::Vector3 out;
  out.x = vec(0);
  out.y = vec(1);
  out.z = vec(2);
  return out;
}
//}

/* //{ vec_to_msg() */
geometry_msgs::Quaternion CsvTracker::vec_to_msg(Eigen::Quaterniond vec) {
  geometry_msgs::Quaternion out;
  out.w = vec.w();
  out.x = vec.x();
  out.y = vec.y();
  out.z = vec.z();
  return out;
}
//}

} // namespace mrs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trackers::CsvTracker, mrs_uav_managers::Tracker)

