
/* includes //{ */

#include "std_msgs/String.h"
#include <controller_module/AcadosParamsMsg.h>
#include <controller_module/DroneStateMsg.h>
#include <controller_module/NmpcControlOutput.h>
#include <controller_module/Telemetry.h>
#include <controller_module/controller_paramsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/DebugValue.h>
#include <mavros_msgs/ESCStatus.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_uav_managers/controller.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <yaml-cpp/yaml.h>

#include <acados_wrapper.hpp>
#include <common.hpp>
#include <drone.hpp>
#include <eigen3/Eigen/Eigen>
#include <git_hash.hpp>
#include <random>
// #include <traj_optim_wrapper.hpp>

#include <chrono>

//}

namespace controller_module {

/* class ControllerModule //{ */

class ControllerModule : public mrs_uav_managers::Controller {
public:
  ~ControllerModule() {};

  bool initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput &last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState &uav_state, const std::optional<mrs_msgs::TrackerCommand> &tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  /// | ----------------------- publishers ----------------------- |

  void publishAcadosParams();

  /// | ------------------------ callbacks ----------------------- |

  void callbackBatteryVoltage(const sensor_msgs::BatteryState &msg);
  void callbackIMUdata(const sensor_msgs::Imu &msg);
  void callbackPX4DebugVector(const mavros_msgs::DebugValue &msg);
  void callbackMotorSpeed(const std_msgs::Float64ConstPtr &msg, const int id);
  void callbackMotorSpeedRealUAV(const mavros_msgs::ESCStatusConstPtr &msg);

  /// | -------------------- helper functions -------------------- |

  bool containNAN(DroneState ds);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_ = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  std::string _pos_ref_frame_;
  std::string _run_type_;

  // flags to check if data was received
  bool imu_data_received_ = false;
  Vector<3> omega_error_integral_received_;
  bool prediction_full_state_was_received_ = false;

  bool use_imu_data_ = false;
  bool verbose = true;
  bool hold_state_active = false;

  //
  int TELEMETRY_PUBLISH_RATE_ = 100;
  int INSPECTOR_PUBLISH_RATE_ = 1;
  std::vector<DroneState> inbound_trajectory;
  mrs_msgs::MpcPredictionFullState msg_last_ref_traj;

  // uav parameters
  double _uav_mass_;
  double hover_thrust_;
  double actual_voltage;

  Quaternion ref_attitude; // debug
  Quaternion imu_orientation_ = Quaternion(0, 0, 0, 1);
  Vector<3> imu_angular_velocity_ = Vector<3>::Zero();

  Vector<3> quatError(Quaternion q, Quaternion q_ref);

  DroneParams uavParams;
  AcadosParams acParams;

  DroneState state;
  DroneState newState;
  DroneState refState;
  DroneState holdState;
  Vector<3> omega_error_integral_;
  Vector<4> motor_thrusts_;
  std::string _uav_name_;
  acados_drone ac;
  Drone quadcopter;

  double callback_timeout_ = 0.3;
  const int MODIFIABLE_COSTS = 10;
  double DSHOT_MIN_THROTTLE = 0.05;
  double DSHOT_MAX_THROTTLE = 0.999;

  bool inputMemoryCalled = false;

  std::vector<DroneState> ac_states_prediction;
  double rotor_velocity_slowdown_sim_;
  double force_constant_;

  // | ------------------------- mutexes ------------------------ |
  boost::recursive_mutex mutex_drs_;
  std::mutex mutex_omega_error_integral_received_;
  std::mutex mutex_imu_data_;
  std::mutex mutex_params_;
  std::mutex mutex_acados_drone_;
  std::mutex mutex_battery_voltage_;
  std::mutex mutex_motor_speeds_;

  // | --------------------- performance vars ------------------- |
  double computation_time_s = 0.0;
  double max_computation_time_s = 0.0;
  int unsuccess_counter_ = 0;
  double actual_velocity = 0.0;

  // | --------------------- time trackers ---------------------- |
  ros::Time time_of_last_trajectory_resample_;
  ros::Time time_of_last_imu_data_;
  ros::Time time_of_last_battery_data_;
  ros::Time time_of_last_esc_data_;
  ros::Time time_of_last_omega_integral_error_data_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_inspector_;
  void TimerInspector(const ros::TimerEvent &te);

  // | ----------------------- subscribers ---------------------- |

  ros::Subscriber sub_battery_;
  ros::Subscriber sub_px4_imu_;
  ros::Subscriber sub_px4_debug_vector_;
  std::vector<ros::Subscriber> sub_motor_speeds_;
  ros::Subscriber sub_escs_;
  // ros::Subscriber sub_full_state_ref_trajectory_;
  // ros::Subscriber sub_full_state_ref_trajectory_csv_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publish_acados_params_;
  ros::Publisher publish_telemetry_;
  ros::Publisher publisher_mrs_uav_status_;

  // | --------------- dynamic reconfigure server --------------- |

  typedef controller_module::controller_paramsConfig DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t> drs_;
  void callbackDrs(controller_module::controller_paramsConfig &params, uint32_t level);
  std::vector<DroneState> getReSampledTrajectory(const mrs_msgs::MpcPredictionFullState &msg);
  std::vector<DroneState> getHoverStateTrajectory(const Eigen::Vector3d &reference_position);
  bool doesTrajectoryContainNAN(const std::vector<DroneState> &trajectory);
  void saveFullStateTrajectory(const mrs_msgs::MpcPredictionFullState &msg);
  double getInterpolation(const double &data1, const double &data2, const double &stamp1, const double &stamp2, const double &time_desired);
  controller_module::DroneStateMsg dronestatetoDroneStateMsg(const DroneState &ds);
  DrsParams_t drs_params_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

bool ControllerModule::initialize(const ros::NodeHandle &nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;
  _uav_name_ = common_handlers->uav_name;
  common_handlers_ = common_handlers;
  private_handlers_ = private_handlers;
  _uav_mass_ = common_handlers->getMass();
  uavParams.mass = _uav_mass_;
  omega_error_integral_ = Vector<3>::Zero();
  omega_error_integral_received_ = Vector<3>::Zero();
  motor_thrusts_ = Vector<4>::Zero();
  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "ControllerModule");
  mrs_lib::ParamLoader param_loader_parent(common_handlers->parent_nh, "ControlManager");
  param_loader_parent.loadParam("run_type", _run_type_);
  ROS_INFO("about to load params");

  private_handlers->param_loader->addYamlFile(ros::package::getPath("controller_module") + "/config/private/nmpc_controller.yaml");
  private_handlers->param_loader->addYamlFile(ros::package::getPath("controller_module") + "/config/public/nmpc_controller.yaml");

  const std::string yaml_namespace = "mrs_uav_controllers/" + private_handlers_->name_space + "/";

  // UAVParams
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/mot0_pos", uavParams.mot0_pos);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/mot1_pos", uavParams.mot1_pos);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/mot2_pos", uavParams.mot2_pos);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/mot3_pos", uavParams.mot3_pos);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/inertia", uavParams.inertia_diag);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/ctau", uavParams.ctau);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/tau_mot", uavParams.tau_mot);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/drag", uavParams.drag);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/force_constant", force_constant_);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/use_imu_data", use_imu_data_);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_configuration/rotor_velocity_slowdown_sim", rotor_velocity_slowdown_sim_);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_constraints/thrust_collective_max", uavParams.thrust_collective_max);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_constraints/thrust_collective_min", uavParams.thrust_collective_min);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_constraints/thrust_actuator_max", uavParams.thrust_actuator_max);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_constraints/thrust_actuator_min", uavParams.thrust_actuator_min);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/uav_constraints/max_rot_speed", uavParams.max_rot_speed);
  uavParams.max_input_rot_speed = uavParams.max_rot_speed;

  uavParams.kp.resize(3);
  uavParams.ki.resize(3);
  uavParams.kd.resize(3);
  uavParams.max_integral.resize(3);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/kp_xy", uavParams.kp.at(0));
  uavParams.kp.at(1) = uavParams.kp.at(0); // same for y
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/kp_z", uavParams.kp.at(2));
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/ki_xy", uavParams.ki.at(0));
  uavParams.ki.at(1) = uavParams.ki.at(0); // same for y
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/ki_z", uavParams.ki.at(2));
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/kd_xy", uavParams.kd.at(0));
  uavParams.kd.at(1) = uavParams.kd.at(0); // same for y
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/kd_z", uavParams.kd.at(2));
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/max_integral_xy", uavParams.max_integral.at(0));
  uavParams.max_integral.at(1) = uavParams.max_integral.at(0); // same for y
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/pid/max_integral_z", uavParams.max_integral.at(2));

  // Acados definition
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/solver/ac_num_iter", acParams.ac_num_iter);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/solver/ac_horizon", acParams.ac_horizon);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/solver/ac_time_steps", acParams.ac_time_steps);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_position_xy", acParams.Q_pos_xy);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_position_z", acParams.Q_pos_z);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_rotation_xy", acParams.Q_rot_xy);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_rotation_z", acParams.Q_rot_z);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_velocity", acParams.Q_vel);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_omega", acParams.Q_ome);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_int", acParams.Q_int);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/Q_thrust", acParams.Q_thr);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/R_col_thrust", acParams.R_col_thrust);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/costs/R_ome_in", acParams.R_ome_in);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/solver/integral_relaxation_index", acParams.integral_relaxation_index);
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/solver/enable_integral_relaxation", acParams.enable_integral_relaxation);

  // verbose
  private_handlers->param_loader->loadParam(yaml_namespace + "nmpc/optional/verbose", verbose);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControllerModule]: Could not load all parameters!");
    ros::shutdown();
  }
  ROS_INFO("loaded");
  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, _uav_mass_ * common_handlers_->g);

  // | --------------------------- drs -------------------------- |

  // Costs
  drs_params_.Q_pos_xy = acParams.Q_pos_xy;
  drs_params_.Q_pos_z = acParams.Q_pos_z;
  drs_params_.Q_quat_xy = acParams.Q_rot_xy;
  drs_params_.Q_quat_z = acParams.Q_rot_z;
  drs_params_.Q_velocity = acParams.Q_vel;
  drs_params_.Q_omega = acParams.Q_ome;
  drs_params_.Q_int = acParams.Q_int;
  drs_params_.Q_thrust = acParams.Q_thr;
  drs_params_.R_col_thrust = acParams.R_col_thrust;
  drs_params_.R_ome_in = acParams.R_ome_in;

  // Constants
  drs_params_.mass = uavParams.mass;
  drs_params_.inertia_x = uavParams.inertia_diag.at(0);
  drs_params_.inertia_y = uavParams.inertia_diag.at(1);
  drs_params_.inertia_z = uavParams.inertia_diag.at(2);
  drs_params_.ctau = uavParams.ctau;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&ControllerModule::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------- end drs ------------------------ |

  state = DroneState();
  newState = DroneState();
  refState = DroneState();
  state.setZero();
  refState.setZero();
  holdState.setZero();

  actual_voltage = NAN;

  ref_attitude = Quaternion(1, 0, 0, 0);

  ROS_INFO("[NmpcPidController]: will create acados_drone");
  ac = acados_drone(uavParams, acParams);
  ROS_INFO("[NmpcPidController]: acados drone created");
  // quadcopter = Drone(config_url);
  // ROS_INFO("[NmpcPidController]: quadrotor created");

  // inputMemory = Eigen::MatrixXd::Zero(4, 30);

  // | ------------------------- timers ------------------------- |

  timer_inspector_ = nh_.createTimer(ros::Rate(INSPECTOR_PUBLISH_RATE_), &ControllerModule::TimerInspector, this);

  // | ----------------------- publishers ----------------------- |

  publish_acados_params_ = nh_.advertise<controller_module::AcadosParamsMsg>("AcadosParams", 100, true);
  publish_telemetry_ = nh_.advertise<controller_module::Telemetry>("Telemetry", 100);
  publisher_mrs_uav_status_ = nh_.advertise<std_msgs::String>("/" + _uav_name_ + "/mrs_uav_status/display_string", 1);

  // | ----------------------- subscribers ---------------------- |

  // sub_full_state_ref_trajectory_csv_ =
  //     nh_.subscribe(std::string("/") + common_handlers->uav_name + std::string("/control_manager/csv_tracker/ReferenceTrajectory"), 10,
  //                   &ControllerModule::callbackFullStateTrajectory, this);
  sub_battery_ =
      nh_.subscribe(std::string("/") + common_handlers->uav_name + std::string("/mavros/battery"), 10, &ControllerModule::callbackBatteryVoltage, this);
  if (use_imu_data_) {
    sub_px4_imu_ = nh_.subscribe(std::string("/") + common_handlers->uav_name + std::string("/mavros/imu/data"), 10, &ControllerModule::callbackIMUdata, this);
  }
  sub_px4_debug_vector_ = nh_.subscribe(std::string("/") + common_handlers->uav_name + std::string("/mavros/debug_value/debug_vector"), 10,
                                        &ControllerModule::callbackPX4DebugVector, this);

  sub_motor_speeds_.resize(4);
  for (int i = 0; i < 4; i++) {
    std::stringstream topic_name;
    topic_name << "/" << common_handlers->uav_name << "/motor_speed/" << (i);
    // std_msgs/Float64
    sub_motor_speeds_[i] = nh_.subscribe<std_msgs::Float64>(topic_name.str(), 10, boost::bind(&ControllerModule::callbackMotorSpeed, this, _1, i));
  }
  std::string sub_escs__topic_name = std::string("/") + common_handlers->uav_name + std::string("/mavros/esc_status");
  sub_escs_ = nh_.subscribe<mavros_msgs::ESCStatus>(sub_escs__topic_name, 10, &ControllerModule::callbackMotorSpeedRealUAV, this);

  time_of_last_esc_data_ = ros::Time::now();
  time_of_last_battery_data_ = ros::Time::now();
  time_of_last_imu_data_ = ros::Time::now();
  time_of_last_omega_integral_error_data_ = ros::Time::now();

  publishAcadosParams();

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[NmpcPidController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* activate() //{ */

bool ControllerModule::activate(const ControlOutput &last_control_output) {
  activation_control_output_ = last_control_output;

  publishAcadosParams();

  // inputMemoryCalled = false;
  is_active_ = true;
  ROS_WARN("[ControllerModule]: was activated");

  return true;
}

//}

/* deactivate() //{ */

void ControllerModule::deactivate(void) {
  is_active_ = false;

  ROS_INFO("[ControllerModule]: deactivated");
}

//}

/* updateInactive() //{ */

void ControllerModule::updateInactive(const mrs_msgs::UavState &uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand> &tracker_command) {}

//}

/* update() //{ */

ControllerModule::ControlOutput ControllerModule::updateActive(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command) {
  last_control_output_.desired_heading_rate = {};
  last_control_output_.desired_orientation = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output = {};
  last_control_output_.diagnostics = {};

  if (!is_active_) {
    return last_control_output_;
  }
  if (_run_type_ == "realworld" && (ros::Time::now() - time_of_last_battery_data_ > ros::Duration(callback_timeout_))) {
    ROS_ERROR_STREAM_THROTTLE(0.5,
                              "[NmpcPidController] no battery data received for " << (ros::Time::now() - time_of_last_battery_data_).toSec() << " seconds");
    return last_control_output_;
  }
  if (use_imu_data_ && (ros::Time::now() - time_of_last_imu_data_ > ros::Duration(callback_timeout_))) {
    ROS_ERROR_STREAM_THROTTLE(0.5, "[NmpcPidController] no imu data received for " << (ros::Time::now() - time_of_last_imu_data_).toSec() << " seconds");
    return last_control_output_;
  }
  if (ros::Time::now() - time_of_last_esc_data_ > ros::Duration(callback_timeout_)) {
    ROS_ERROR_STREAM_THROTTLE(0.5, "[NmpcPidController] no esc data received for " << (ros::Time::now() - time_of_last_esc_data_).toSec() << " seconds");
    return last_control_output_;
  }
  if (ros::Time::now() - time_of_last_omega_integral_error_data_ > ros::Duration(callback_timeout_)) {
    ROS_ERROR_STREAM_THROTTLE(0.5, "[NmpcPidController] no omega integral error data received for "
                                       << (ros::Time::now() - time_of_last_omega_integral_error_data_).toSec());
    return last_control_output_;
  }

  auto params = mrs_lib::get_mutexed(mutex_params_, drs_params_);

  _pos_ref_frame_ = uav_state.header.frame_id;
  // current state
  Eigen::Vector3d cur_pos(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  Eigen::Vector3d cur_vel(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);
  actual_velocity = cur_vel.norm();

  Eigen::Vector3d cur_omg(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z);
  Eigen::Vector4d cur_pose(uav_state.pose.orientation.w, uav_state.pose.orientation.x, uav_state.pose.orientation.y, uav_state.pose.orientation.z);

  Eigen::Matrix3d cur_att = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

  if (use_imu_data_) {
    std::scoped_lock lock(mutex_imu_data_);
    if (imu_data_received_) {
      cur_pose[0] = imu_orientation_.w();
      cur_pose[1] = imu_orientation_.x();
      cur_pose[2] = imu_orientation_.y();
      cur_pose[3] = imu_orientation_.z();
      cur_omg[0] = imu_angular_velocity_[0];
      cur_omg[1] = imu_angular_velocity_[1];
      cur_omg[2] = imu_angular_velocity_[2];
    }
  }
  {
    std::scoped_lock lock(mutex_omega_error_integral_received_);
    omega_error_integral_ = omega_error_integral_received_;
  }
  // reference
  Eigen::Vector3d ref_pos(tracker_command.position.x, tracker_command.position.y, tracker_command.position.z);
  Eigen::Vector3d ref_vel(tracker_command.velocity.x, tracker_command.velocity.y, tracker_command.velocity.z);
  Eigen::Vector3d ref_acc(tracker_command.acceleration.x, tracker_command.acceleration.y, tracker_command.acceleration.z);
  if (tracker_command.use_full_state_prediction) {
    saveFullStateTrajectory(tracker_command.full_state_prediction);
  }

  geometry_msgs::Quaternion quat = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_command.heading);

  state.p = cur_pos;
  state.qx = cur_pose;
  state.v = cur_vel;
  state.w = cur_omg;
  state.errorIntegral = omega_error_integral_;
  {
    std::scoped_lock lock(mutex_motor_speeds_);
    // throttle = sqrt(thrust/thrust_actuator_max_)
    state.thrusts(0) = sqrt(motor_thrusts_[0] / uavParams.thrust_actuator_max);
    state.thrusts(1) = sqrt(motor_thrusts_[1] / uavParams.thrust_actuator_max);
    state.thrusts(2) = sqrt(motor_thrusts_[2] / uavParams.thrust_actuator_max);
    state.thrusts(3) = sqrt(motor_thrusts_[3] / uavParams.thrust_actuator_max);
  }
  // std::cout << "state thrusts "<< state.thrusts.transpose() <<"\n";

  {
    std::scoped_lock lock(mutex_acados_drone_);
    double time_now = ros::Time::now().toSec();

    if (prediction_full_state_was_received_) {
      inbound_trajectory = getReSampledTrajectory(msg_last_ref_traj);
      if (doesTrajectoryContainNAN(inbound_trajectory)) {
        inbound_trajectory = getHoverStateTrajectory(ref_pos);
      }
    } else {
      inbound_trajectory = getHoverStateTrajectory(ref_pos);
    }

    ac.set_ref(inbound_trajectory);
    ac.set_init_state(state);

    auto start = std::chrono::high_resolution_clock::now(); // TODO: does not use ros time... (if is not running in real time)
    ros::Time bef = ros::Time::now();

    bool success = ac.compute_control();

    if (!success) {
      unsuccess_counter_++;
      ROS_WARN_STREAM("error computing from state:");
      std::cout << "p:" << state.p[0] << "," << state.p[1] << "," << state.p[2] << ", "
                << "q:" << state.qx[0] << "," << state.qx[1] << "," << state.qx[2] << "," << state.qx[3] << ", "
                << "v:" << state.v[0] << "," << state.v[1] << "," << state.v[2] << ", "
                << "w:" << state.w[0] << "," << state.w[1] << "," << state.w[2] << ", "
                << "integralerror:" << state.errorIntegral[0] << "," << state.errorIntegral[1] << "," << state.errorIntegral[2] << ", "
                << "thrusts:" << state.thrusts[0] << "," << state.thrusts[1] << "," << state.thrusts[2] << "," << state.thrusts[3] << ", " << std::endl;
      if (unsuccess_counter_ > 10) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[NmpcPidController] acados failed to compute control for more than 10 times in a row, deactivating controller");
        return last_control_output_;
      }
    } else {
      unsuccess_counter_ = 0;
    }

    ros::Duration rosDurationAcadosComputeTime = ros::Time::now() - bef;
    auto stop = std::chrono::high_resolution_clock::now();
    auto chronoDurationAcadosComputeTime = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    computation_time_s = chronoDurationAcadosComputeTime.count() / 1e6;

    if (verbose) {
      ROS_INFO_STREAM("[NmpcPidController] time compute nmpc " << computation_time_s << " s");
    }
    // handle max computation time
    if (max_computation_time_s == 0.0) {
      max_computation_time_s = computation_time_s;
    } else {
      if (max_computation_time_s < computation_time_s) {
        max_computation_time_s = computation_time_s;
      }
    }

    Vector<4> control = ac.get_first_control_action();
    ac_states_prediction = ac.get_all_computed_states();
    std::vector<Vector<4>> all_control_actions = ac.get_all_control_actions();

    newState = ac_states_prediction[1]; // instead of using rk4 use acados prediction
    newState.t = 0;

    {
      // msg telemetry //{

      controller_module::Telemetry msg_telemetry = {};
      msg_telemetry.header.stamp = time_of_last_trajectory_resample_;
      msg_telemetry.header.frame_id = uav_state.header.frame_id;
      Vector<3> q_err = quatError(Quaternion(state.qx(0), state.qx(1), state.qx(2), state.qx(3)), ref_attitude);
      msg_telemetry.quat_error.x = q_err[0];
      msg_telemetry.quat_error.y = q_err[1];
      msg_telemetry.quat_error.z = q_err[2];
      msg_telemetry.computation_time_ns = rosDurationAcadosComputeTime.toNSec();
      msg_telemetry.computation_time_us = chronoDurationAcadosComputeTime.count();
      msg_telemetry.computation_success = success;
      msg_telemetry.git_hash = GITHASH;
      msg_telemetry.git_branch = GITBRANCH;
      ////////////////////// initial state ///////////////////////
      // position
      msg_telemetry.initial_state = dronestatetoDroneStateMsg(state);

      ros::Time temp_time = time_of_last_trajectory_resample_;
      controller_module::NmpcControlOutput temp_control_output;

      for (int i = 0; i <= ac.N; i++) {
        ///////// stamps //////////
        msg_telemetry.stamps.push_back(temp_time);
        if (i < ac.N) {
          temp_time += ros::Duration(ac.dt[i]);
        }

        ///////////// control actions /////////////////
        if (i < ac.N) {
          temp_control_output.commanded_thrust = all_control_actions[i][0];
          temp_control_output.commanded_omegas[0] = all_control_actions[i][1];
          temp_control_output.commanded_omegas[1] = all_control_actions[i][2];
          temp_control_output.commanded_omegas[2] = all_control_actions[i][3];
          msg_telemetry.control_outputs.push_back(temp_control_output);
        }

        ///////////// computed states /////////////////
        // position

        if (i < ac_states_prediction.size()) {
          msg_telemetry.predicted_drone_states.push_back(dronestatetoDroneStateMsg(ac_states_prediction[i]));
        }

        ////////////////// reference states /////////////////
        if (i < inbound_trajectory.size()) {
          msg_telemetry.reference_trajectory.push_back(dronestatetoDroneStateMsg(inbound_trajectory[i]));
        }
      }

      //}
      publish_telemetry_.publish(msg_telemetry);
    }

    if (verbose) {
      ROS_INFO_STREAM_THROTTLE(1.0, "[NmpcPidController] control action " << control.transpose());
    }

    double voltage = 0.0;
    {
      std::scoped_lock lock(mutex_battery_voltage_);
      voltage = actual_voltage;
    }
    double des_throttle;
    double des_collective_thrust;
    if (isnan(voltage)) {
      ROS_ERROR_STREAM("[NmpcPidController] NaN voltage, selecting some nominal one with 15.6V");
      voltage = 15.6;
    }
    double des_sr_thrust = uavParams.thrust_actuator_max * control(0) * control(0);
    des_collective_thrust = 4 * des_sr_thrust;
    des_throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, des_collective_thrust);
    if (_run_type_ == "realworld") {
      // ROS_WARN_STREAM_THROTTLE(1.0, "[NmpcPidController] using voltage-based thrustmap");
      // We reduce the thrust by 0.05 to counteract the increased in thrust applied by PX4 due to the minimum DShot throttle value
      des_throttle = (des_throttle - DSHOT_MIN_THROTTLE) / (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE);
    }
    if (des_throttle < 0) {
      des_throttle = 0;
      ROS_WARN_STREAM_THROTTLE(0.5, "[NmpcPidController] created throttle smaller than zero from thrusts " << control(0));
    } else if (des_throttle > 1.0) {
      des_throttle = 1.0;
      ROS_WARN_STREAM_THROTTLE(0.5, "[NmpcPidController] created throttle larger than one from thrusts " << control(0));
    }

    // | --------------- prepare the attitude output -------------- |

    mrs_msgs::HwApiAttitudeRateCmd output_rate_cmd;
    output_rate_cmd.stamp = ros::Time::now();

    output_rate_cmd.throttle = des_throttle;
    output_rate_cmd.body_rate.x = control(1);
    output_rate_cmd.body_rate.y = control(2);
    output_rate_cmd.body_rate.z = control(3);

    last_control_output_.control_output = output_rate_cmd;

    last_control_output_.diagnostics.mass_difference = 0;
    last_control_output_.diagnostics.total_mass = _uav_mass_;
    // output_command.attitude = mrs_lib::AttitudeConverter(Rd);
    last_control_output_.desired_orientation = Eigen::Quaternion(newState.qx(0), newState.qx(1), newState.qx(2), newState.qx(3));
    last_control_output_.diagnostics.controller_enforcing_constraints = false;

    last_control_output_.diagnostics.controller = "ControllerPIDModule";

    // | ------------ compensated desired acceleration ------------ |

    double desired_x_accel = 0;
    double desired_y_accel = 0;
    double desired_z_accel = 0;
    {
      Eigen::Matrix3d des_orientation = mrs_lib::AttitudeConverter(Eigen::Quaternion(newState.qx(0), newState.qx(1), newState.qx(2), newState.qx(3)));
      Eigen::Vector3d thrust_vector = (des_collective_thrust)*des_orientation.col(2);

      double world_accel_x = (thrust_vector[0] / _uav_mass_);
      double world_accel_y = (thrust_vector[1] / _uav_mass_);
      double world_accel_z = (thrust_vector[2] / _uav_mass_) - common_handlers_->g;

      geometry_msgs::Vector3Stamped world_accel;

      world_accel.header.stamp = ros::Time::now();
      world_accel.header.frame_id = uav_state.header.frame_id;
      world_accel.vector.x = world_accel_x;
      world_accel.vector.y = world_accel_y;
      world_accel.vector.z = world_accel_z;

      auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

      if (res) {

        desired_x_accel = res.value().vector.x;
        desired_y_accel = res.value().vector.y;
        desired_z_accel = res.value().vector.z;
      }
    }

    last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(desired_x_accel, desired_y_accel, desired_z_accel);

    // | ------------------ desired heading rate ------------------ |

    last_control_output_.desired_heading_rate = mrs_lib::AttitudeConverter(cur_att).getHeadingRate(output_rate_cmd.body_rate);
    // std::cout << "Desired   state is: " << newState << std::endl <<
    // std::endl;
    return last_control_output_;
  }
} // namespace controller_module

//}

/* quatError() //{ */

Vector<3> ControllerModule::quatError(Quaternion q, Quaternion q_ref) {
  Vector<4> q_aux(q.w() * q_ref.w() + q.x() * q_ref.x() + q.y() * q_ref.y() + q.z() * q_ref.z(),
                  -q.x() * q_ref.w() + q.w() * q_ref.x() + q.z() * q_ref.y() - q.y() * q_ref.z(),
                  -q.y() * q_ref.w() - q.z() * q_ref.x() + q.w() * q_ref.y() + q.x() * q_ref.z(),
                  -q.z() * q_ref.w() + q.y() * q_ref.x() - q.x() * q_ref.y() + q.w() * q_ref.z());
  // attitude errors. SQRT have small quantities added (1e-3) to alleviate
  // the derivative not being defined at zero, and also because it's in the
  // denominator
  double q_att_denom = sqrt(q_aux(0) * q_aux(0) + q_aux(3) * q_aux(3) + 1e-3);
  Vector<3> q_att = Vector<3>(q_aux(0) * q_aux(1) - q_aux(2) * q_aux(3), q_aux(0) * q_aux(2) + q_aux(1) * q_aux(3), q_aux(3));
  q_att = q_att / q_att_denom;

  return q_att;
}

//}

/* dronestatetoDroneStateMsg() //{ */

controller_module::DroneStateMsg ControllerModule::dronestatetoDroneStateMsg(const DroneState &ds) {
  // Convert DroneState to controller_module::DroneState for telemetry msg
  controller_module::DroneStateMsg msg;
  // position
  msg.pose.position.x = ds.p[0];
  msg.pose.position.y = ds.p[1];
  msg.pose.position.z = ds.p[2];
  // orientation
  msg.pose.orientation.w = ds.qx[0];
  msg.pose.orientation.x = ds.qx[1];
  msg.pose.orientation.y = ds.qx[2];
  msg.pose.orientation.z = ds.qx[3];
  // velocity
  msg.twist.linear.x = ds.v[0];
  msg.twist.linear.y = ds.v[1];
  msg.twist.linear.z = ds.v[2];
  // angular velocity
  msg.twist.angular.x = ds.w[0];
  msg.twist.angular.y = ds.w[1];
  msg.twist.angular.z = ds.w[2];
  // integral error
  msg.integral_error[0] = ds.errorIntegral[0];
  msg.integral_error[1] = ds.errorIntegral[1];
  msg.integral_error[2] = ds.errorIntegral[2];
  // thrusts
  msg.motor_thrusts[0] = ds.thrusts[0];
  msg.motor_thrusts[1] = ds.thrusts[1];
  msg.motor_thrusts[2] = ds.thrusts[2];
  msg.motor_thrusts[3] = ds.thrusts[3];

  return msg;
}

//}

/* getReSampledTrajectory() //{ */

std::vector<DroneState> ControllerModule::getReSampledTrajectory(const mrs_msgs::MpcPredictionFullState &msg) {
  std::vector<DroneState> resampled_trajectory = std::vector<DroneState>(ac.N + 1);
  // find starting timestamp
  size_t chosen_index = 1;
  size_t fill_index = 0;
  size_t size_of_trajectory_supplied = msg.stamps.size();

  time_of_last_trajectory_resample_ = ros::Time::now();
  double time_desired = time_of_last_trajectory_resample_.toSec();

  // if the trajectory's last time stamp is already lesser than the first
  // desired time step, we fill the trajectory with the current state instead
  // with zero linear and angular velocities, position from current state, and
  // zero quaternion(w=1).
  if (time_desired >= msg.stamps[size_of_trajectory_supplied - 1].toSec()) {
    ROS_WARN("TrajectoryReSample: Reference trajectory too old, filling with current state instead and holding position");
    ROS_WARN_STREAM("TrajectoryReSample: Time desired " << time_desired << " Last time in trajectory: " << msg.stamps[size_of_trajectory_supplied - 1].toSec());
    if (!hold_state_active) {
      // if hold state is not active/true, assign current drone position for hold
      ROS_WARN("TrajectoryReSample: Activating hold state");
      hold_state_active = true;
      holdState.x(DroneState::POSX) = state.x(DroneState::POSX);
      holdState.x(DroneState::POSY) = state.x(DroneState::POSY);
      holdState.x(DroneState::POSZ) = state.x(DroneState::POSZ);
    }
    for (int i = 0; i < ac.N + 1; i++) {
      resampled_trajectory[i] = holdState;
    }
    return resampled_trajectory;
  }

  // we have crossed the point where trajectory might have been too old so holding state is not necessary
  hold_state_active = false;

  while (fill_index < ac.N + 1 && chosen_index < size_of_trajectory_supplied) {
    // Let's find suitable index where the timestamp lies between the timestamps
    // of two indices of the trajectory
    while (time_desired >= msg.stamps[chosen_index].toSec() && chosen_index < size_of_trajectory_supplied) {
      chosen_index++;
    }
    if (chosen_index == size_of_trajectory_supplied) {
      break;
    }
    double curr_time = msg.stamps[chosen_index].toSec();
    double prev_time = msg.stamps[chosen_index - 1].toSec();
    if (time_desired > curr_time) {
      // Check that the trajectory is not too old/in-the-past, but not checking if trajectory starts in the future because that is okay to interpolate
      ROS_WARN("TrajectoryReSample: Trajectory interpolation has time outside the borders");
    }

    resampled_trajectory[fill_index].setZero();

    // used algorithm described in
    // https://ieeexplore.ieee.org/document/5980409 position
    resampled_trajectory[fill_index].x(DroneState::POSX) =
        getInterpolation(msg.position[chosen_index - 1].x, msg.position[chosen_index].x, prev_time, curr_time, time_desired);
    resampled_trajectory[fill_index].x(DroneState::POSY) =
        getInterpolation(msg.position[chosen_index - 1].y, msg.position[chosen_index].y, prev_time, curr_time, time_desired);
    resampled_trajectory[fill_index].x(DroneState::POSZ) =
        getInterpolation(msg.position[chosen_index - 1].z, msg.position[chosen_index].z, prev_time, curr_time, time_desired);

    // velocity
    resampled_trajectory[fill_index].x(DroneState::VELX) =
        getInterpolation(msg.velocity[chosen_index - 1].x, msg.velocity[chosen_index].x, prev_time, curr_time, time_desired);
    resampled_trajectory[fill_index].x(DroneState::VELY) =
        getInterpolation(msg.velocity[chosen_index - 1].y, msg.velocity[chosen_index].y, prev_time, curr_time, time_desired);
    resampled_trajectory[fill_index].x(DroneState::VELZ) =
        getInterpolation(msg.velocity[chosen_index - 1].z, msg.velocity[chosen_index].z, prev_time, curr_time, time_desired);

    // acceleration
    Vector<3> interpolated_acceleration;
    interpolated_acceleration[0] = getInterpolation(msg.acceleration[chosen_index - 1].x, msg.acceleration[chosen_index].x, prev_time, curr_time, time_desired);
    interpolated_acceleration[1] = getInterpolation(msg.acceleration[chosen_index - 1].y, msg.acceleration[chosen_index].y, prev_time, curr_time, time_desired);
    interpolated_acceleration[2] =
        getInterpolation(msg.acceleration[chosen_index - 1].z, msg.acceleration[chosen_index].z, prev_time, curr_time, time_desired) + 9.806;
    double thrust_acc_norm = interpolated_acceleration.norm();

    // Individual motor thrusts
    // throttle = sqrt(thrust/thrust_actuator_max_)
    resampled_trajectory[fill_index].x(DroneState::T1) = (thrust_acc_norm * _uav_mass_ / 4.0);
    resampled_trajectory[fill_index].x(DroneState::T2) = (thrust_acc_norm * _uav_mass_ / 4.0);
    resampled_trajectory[fill_index].x(DroneState::T3) = (thrust_acc_norm * _uav_mass_ / 4.0);
    resampled_trajectory[fill_index].x(DroneState::T4) = (thrust_acc_norm * _uav_mass_ / 4.0);

    if (msg.use_orientation and msg.use_attitude_rate) {
      // case the quaternion and angular rates are provided
      ROS_DEBUG("TrajectoryReSample: msg.use_orientation and msg.use_attitude_rate");
      // attitude (rotation quaternion)
      // interpolation = (data1 + (time_desired - stamp1) * (data2 - data1) / (stamp2 - stamp1));
      const double slerp_t = std::clamp((time_desired - prev_time) / (curr_time - prev_time), 0.0, 1.0);

      const Eigen::Quaterniond quat_from(msg.orientation[chosen_index - 1].w, msg.orientation[chosen_index - 1].x, msg.orientation[chosen_index - 1].y,
                                         msg.orientation[chosen_index - 1].z);
      const Eigen::Quaterniond quat_to(msg.orientation[chosen_index].w, msg.orientation[chosen_index].x, msg.orientation[chosen_index].y,
                                       msg.orientation[chosen_index].z);
      Eigen::Quaterniond interpolated_quat = quat_from.slerp(slerp_t, quat_to);

      resampled_trajectory[fill_index].qx[0] = interpolated_quat.w();
      resampled_trajectory[fill_index].qx[1] = interpolated_quat.x();
      resampled_trajectory[fill_index].qx[2] = interpolated_quat.y();
      resampled_trajectory[fill_index].qx[3] = interpolated_quat.z();

      ref_attitude = interpolated_quat; // for Telemetry and debug

      // attitude rate
      resampled_trajectory[fill_index].w[0] =
          getInterpolation(msg.attitude_rate[chosen_index - 1].x, msg.attitude_rate[chosen_index].x, prev_time, curr_time, time_desired);
      resampled_trajectory[fill_index].w[1] =
          getInterpolation(msg.attitude_rate[chosen_index - 1].y, msg.attitude_rate[chosen_index].y, prev_time, curr_time, time_desired);
      resampled_trajectory[fill_index].w[2] =
          getInterpolation(msg.attitude_rate[chosen_index - 1].z, msg.attitude_rate[chosen_index].z, prev_time, curr_time, time_desired);
    } else {
      // case the quaternion and angular rates are not provided

      // attitude (rotation quaternion)
      double interpolated_heading = getInterpolation(msg.heading[chosen_index - 1], msg.heading[chosen_index], prev_time, curr_time, time_desired);
      Vector<3> z_body = interpolated_acceleration.normalized();
      Vector<3> x_C = Vector<3>(cos(interpolated_heading), sin(interpolated_heading), 0);
      Vector<3> y_body = (z_body.cross(x_C)).normalized();
      Vector<3> x_body = y_body.cross(z_body);
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix << x_body, y_body, z_body;
      Quaternion rotation(rotation_matrix);
      resampled_trajectory[fill_index].qx[0] = rotation.w();
      resampled_trajectory[fill_index].qx[1] = rotation.x();
      resampled_trajectory[fill_index].qx[2] = rotation.y();
      resampled_trajectory[fill_index].qx[3] = rotation.z();
      ref_attitude = rotation; // for Telemetry and debug

      // attitude rate
      Vector<3> interpolated_jerk;
      interpolated_jerk[0] = getInterpolation(msg.jerk[chosen_index - 1].x, msg.jerk[chosen_index].x, prev_time, curr_time, time_desired);
      interpolated_jerk[1] = getInterpolation(msg.jerk[chosen_index - 1].y, msg.jerk[chosen_index].y, prev_time, curr_time, time_desired);
      interpolated_jerk[2] = getInterpolation(msg.jerk[chosen_index - 1].z, msg.jerk[chosen_index].z, prev_time, curr_time, time_desired);
      double interpolated_heading_rate =
          getInterpolation(msg.heading_rate[chosen_index - 1], msg.heading_rate[chosen_index], prev_time, curr_time, time_desired);
      Vector<3> h_omega = (interpolated_jerk - z_body.dot(interpolated_jerk) * z_body) / thrust_acc_norm;
      resampled_trajectory[fill_index].w[0] = -h_omega.dot(y_body);
      resampled_trajectory[fill_index].w[1] = h_omega.dot(x_body);
      resampled_trajectory[fill_index].w[2] = interpolated_heading_rate * z_body(2);
    }
    // throttle = sqrt(thrust/thrust_actuator_max_)
    resampled_trajectory[fill_index].u(0) = thrust_acc_norm * _uav_mass_;
    // resampled_trajectory[fill_index].u(1) = resampled_trajectory[fill_index].w[0];
    // resampled_trajectory[fill_index].u(2) = resampled_trajectory[fill_index].w[1];
    // resampled_trajectory[fill_index].u(3) = resampled_trajectory[fill_index].w[2];
    /* resampled_trajectory[fill_index].u(0) = 0.0; */
    resampled_trajectory[fill_index].u(1) = 0.0;
    resampled_trajectory[fill_index].u(2) = 0.0;
    resampled_trajectory[fill_index].u(3) = 0.0;

    time_desired += ac.dt[fill_index];
    fill_index++;
    ROS_DEBUG("TrajectoryReSample: Went to index of %ld out of %ld", chosen_index, size_of_trajectory_supplied);
  }

  // fill the reference[0] with current state but not the input[0]
  //  resampled_trajectory[0].x = state.x;
  //  we uncommented it so that it is easier to do RMSE for Ondra in the future

  // fill the rest of the trajectory with the last state if the supplied
  // trajectory is shorter than the prediction horizon
  // We have already moved to the next index in the while loop, so we need to
  // use the previous index to fill the rest of the trajectory
  if (chosen_index == size_of_trajectory_supplied) {
    for (size_t i = fill_index; i < ac.N + 1; i++) {
      resampled_trajectory[i] = resampled_trajectory[i - 1];
    }
    ROS_WARN("TrajectoryReSample: Reference trajectory too short, filling with "
             "last state instead");
  }

  return resampled_trajectory;
}

std::vector<DroneState> ControllerModule::getHoverStateTrajectory(const Eigen::Vector3d &reference_position) {
  std::vector<DroneState> hover_trajectory = std::vector<DroneState>(ac.N + 1);
  for (size_t i = 0; i < ac.N + 1; i++) {
    hover_trajectory[i].setZero();
    hover_trajectory[i].x(DroneState::POSX) = reference_position[0];
    hover_trajectory[i].x(DroneState::POSY) = reference_position[1];
    hover_trajectory[i].x(DroneState::POSZ) = reference_position[2];
    hover_trajectory[i].x(DroneState::ATTW) = 1;
    hover_trajectory[i].x(DroneState::ATTX) = 0;
    hover_trajectory[i].x(DroneState::ATTY) = 0;
    hover_trajectory[i].x(DroneState::ATTZ) = 0;
    hover_trajectory[i].x(DroneState::VELX) = 0;
    hover_trajectory[i].x(DroneState::VELY) = 0;
    hover_trajectory[i].x(DroneState::VELZ) = 0;
    hover_trajectory[i].x(DroneState::OMEX) = 0;
    hover_trajectory[i].x(DroneState::OMEY) = 0;
    hover_trajectory[i].x(DroneState::OMEZ) = 0;
    hover_trajectory[i].x(DroneState::INTX) = 0;
    hover_trajectory[i].x(DroneState::INTY) = 0;
    hover_trajectory[i].x(DroneState::INTZ) = 0;
    hover_trajectory[i].x(DroneState::T1) = _uav_mass_ * common_handlers_->g / 4;
    hover_trajectory[i].x(DroneState::T2) = _uav_mass_ * common_handlers_->g / 4;
    hover_trajectory[i].x(DroneState::T3) = _uav_mass_ * common_handlers_->g / 4;
    hover_trajectory[i].x(DroneState::T4) = _uav_mass_ * common_handlers_->g / 4;
    hover_trajectory[i].u(0) = _uav_mass_ * common_handlers_->g;
    hover_trajectory[i].u(1) = 0;
    hover_trajectory[i].u(2) = 0;
    hover_trajectory[i].u(3) = 0;
  }
  return hover_trajectory;
}
//}

/* getInterpolation() //{ */

double ControllerModule::getInterpolation(const double &data1, const double &data2, const double &stamp1, const double &stamp2, const double &time_desired) {
  return (data1 + (time_desired - stamp1) * (data2 - data1) / (stamp2 - stamp1));
}

//}

/* saveFullStateTrajectory() //{ */

void ControllerModule::saveFullStateTrajectory(const mrs_msgs::MpcPredictionFullState &msg) {

  std::scoped_lock lock(mutex_acados_drone_);
  if (msg.stamps.size() == 0) {
    ROS_WARN_THROTTLE(1.0, "[NmpcPidController]: Received empty full state trajectory");
    return;
  }

  msg_last_ref_traj = msg;
  prediction_full_state_was_received_ = true;
}

//}

/* getStatus() //{ */

const mrs_msgs::ControllerStatus ControllerModule::getStatus() {
  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void ControllerModule::switchOdometrySource(const mrs_msgs::UavState &new_uav_state) {}

//}

/* resetDisturbanceEstimators() //{ */

void ControllerModule::resetDisturbanceEstimators(void) {}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr
ControllerModule::setConstraints([[maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {
  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
}

//}

/* containNAN() //{ */

bool ControllerModule::containNAN(DroneState ds) {
  for (int i = 0; i < ds.SIZE; i++) {
    if (isnan(ds.x(i))) {
      return true;
    }
  }
  return false;
}

//}

// --------------------------------------------------------------
// |                         publishers                         |
// --------------------------------------------------------------

/* publishAcadosParams() //{ */

void ControllerModule::publishAcadosParams() {

  controller_module::AcadosParamsMsg msg_acados_params = {};

  msg_acados_params.header.stamp = ros::Time::now();
  msg_acados_params.header.frame_id = "pid_throttle branch";
  msg_acados_params.Q_pos_xy = drs_params_.Q_pos_xy;
  msg_acados_params.Q_pos_z = drs_params_.Q_pos_z;
  msg_acados_params.Q_rot_xy = drs_params_.Q_quat_xy;
  msg_acados_params.Q_rot_z = drs_params_.Q_quat_z;
  msg_acados_params.Q_vel = drs_params_.Q_velocity;
  msg_acados_params.Q_omega = drs_params_.Q_omega;
  msg_acados_params.Q_thrust = drs_params_.Q_thrust;

  msg_acados_params.Q_int = drs_params_.Q_int;
  msg_acados_params.R_col_thrust = drs_params_.R_col_thrust;
  msg_acados_params.R_ome_in = drs_params_.R_ome_in;

  msg_acados_params.ac_horizon = acParams.ac_horizon;
  msg_acados_params.ac_num_iter = acParams.ac_num_iter;
  msg_acados_params.ac_time_steps = acParams.ac_time_steps;

  msg_acados_params.thrust_collective_max = uavParams.thrust_collective_max;
  msg_acados_params.thrust_collective_min = uavParams.thrust_collective_min;
  msg_acados_params.thrust_actuator_max = uavParams.thrust_actuator_max;
  msg_acados_params.thrust_actuator_min = uavParams.thrust_actuator_min;
  msg_acados_params.max_rot_speed = uavParams.max_rot_speed;
  msg_acados_params.max_input_rot_speed = uavParams.max_input_rot_speed;

  msg_acados_params.tau_mot = uavParams.tau_mot;

  publish_acados_params_.publish(msg_acados_params);
  ROS_INFO("[NmpcController]: Acados parameters published");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackBatteryVoltage() //{ */

void ControllerModule::callbackBatteryVoltage(const sensor_msgs::BatteryState &msg) {
  if (!is_initialized_) {
    return;
  }
  {
    std::scoped_lock lock(mutex_battery_voltage_);
    actual_voltage = msg.cell_voltage[0];
  }
  time_of_last_battery_data_ = ros::Time::now();
  // ROS_WARN_STREAM_THROTTLE(1.0, "[NmpcPidController] getting voltage");
}

//}

/* callbackIMUdata() //{ */

void ControllerModule::callbackIMUdata(const sensor_msgs::Imu &msg) {

  if (!is_initialized_) {
    return;
  }
  // ROS_WARN_STREAM_THROTTLE(1.0, "[NmpcPidController] getting imu data");
  {
    std::scoped_lock lock(mutex_imu_data_);
    imu_data_received_ = true;
    imu_orientation_.w() = msg.orientation.w;
    imu_orientation_.x() = msg.orientation.x;
    imu_orientation_.y() = msg.orientation.y;
    imu_orientation_.z() = msg.orientation.z;
    imu_angular_velocity_[0] = msg.angular_velocity.x;
    imu_angular_velocity_[1] = msg.angular_velocity.y;
    imu_angular_velocity_[2] = msg.angular_velocity.z;
  }
  time_of_last_imu_data_ = ros::Time::now();
}

//}

/* callbackPX4DebugVector() //{ */

void ControllerModule::callbackPX4DebugVector(const mavros_msgs::DebugValue &msg) {
  if (!is_initialized_) {
    return;
  }
  static std::string debug_name("forRobert");
  if (!msg.name.compare(debug_name)) {
    // ROS_WARN_STREAM_THROTTLE(1.0, "[NmpcPidController] getting PID integral part vector");
    {
      std::scoped_lock lock(mutex_omega_error_integral_received_);
      omega_error_integral_received_[0] = msg.data[0];
      omega_error_integral_received_[1] = msg.data[1];
      omega_error_integral_received_[2] = msg.data[2];
    }
    time_of_last_omega_integral_error_data_ = ros::Time::now();
  }
}

//}

/* callbackMotorSpeed() //{ */

void ControllerModule::callbackMotorSpeed(const std_msgs::Float64ConstPtr &msg, const int id) {
  if (!is_initialized_) {
    return;
  }
  {
    std::scoped_lock lock(mutex_motor_speeds_);

    // double rotor_velocity_slowdown_sim_ = 0.0159236;
    // double force_constant_ = 20.21;
    // ROS_INFO_STREAM_THROTTLE(1.0, "get motor " << id << " speed " << msg->data);
    // ROS_ERROR_STREAM_THROTTLE(1.0,"[NmpcPidController] implement getting thrust
    // from gazebo/real drone speeds");
    double real_motor_velocity = msg->data * rotor_velocity_slowdown_sim_;
    double force = std::fabs(real_motor_velocity * real_motor_velocity) * force_constant_;
    // ROS_INFO_STREAM_THROTTLE(1.0, "real_motor_velocity " << real_motor_velocity);
    // ROS_INFO_STREAM_THROTTLE(1.0, "force " << force);
    switch (id) {
    case 0:
      motor_thrusts_(0) = force;
      break;
    case 1:
      motor_thrusts_(1) = force;
      break;
    case 2:
      motor_thrusts_(3) = force;
      break;
    case 3:
      motor_thrusts_(2) = force;
      break;
    default:
      break;
    }
    // ROS_WARN_STREAM_THROTTLE(1.0, "[NmpcPidController] getting motor speeds");
  }
  time_of_last_esc_data_ = ros::Time::now();
  // in simulation == id param
  //  0 == +x -y
  //  1 == -x +y
  //  2 == +x +y
  //  3 == -x -y

  // in NMPC drone == index inside motor_thrusts_
  //                ^
  //     *3   *0    |
  //       \ /      |
  //        .       |
  //       / \      |x
  //     *1   *2    |
  //                |
  //  <_____________|
  //         y
}

//}

/* callbackMotorSpeedRealUAV() //{ */

void ControllerModule::callbackMotorSpeedRealUAV(const mavros_msgs::ESCStatusConstPtr &msg) {
  if (!is_initialized_) {
    return;
  }
  {
    std::scoped_lock lock(mutex_motor_speeds_);

    // ROS_WARN_STREAM_THROTTLE(1.0, "get ESC feedback");

    for (int id = 0; id < 4; id++) {
      double rpm = msg->esc_status[id].rpm;
      double force = 5.36454314e-08 * rpm * rpm - 1.60381704e-04 * rpm + 2.83895003e-01;
      // ROS_INFO_STREAM_THROTTLE(1.0, "motor " << id << " has rpm " << rpm << " calc force " << force);
      switch (id) {
      case 0:
        motor_thrusts_(0) = force;
        break;
      case 1:
        motor_thrusts_(1) = force;
        break;
      case 2:
        motor_thrusts_(3) = force;
        break;
      case 3:
        motor_thrusts_(2) = force;
        break;
      default:
        break;
      }
    }
  }
  time_of_last_esc_data_ = ros::Time::now();
  // in simulation == id param
  //  0 == +x -y
  //  1 == -x +y
  //  2 == +x +y
  //  3 == -x -y

  // in NMPC drone == index inside motor_thrusts_
  //                ^
  //     *3   *0    |
  //       \ /      |
  //        .       |
  //       / \      |x
  //     *1   *2    |
  //                |
  //  <_____________|
  //         y
}

bool ControllerModule::doesTrajectoryContainNAN(const std::vector<DroneState> &trajectory) {
  bool containsNAN = false;
  for (int i = 0; i < inbound_trajectory.size(); i++) {
    if (containNAN(inbound_trajectory[i])) {
      ROS_ERROR_STREAM("[NmpcPidController]: drone state at " << i << " index of reference trajectory contains NaN !\n " << inbound_trajectory[i]);
      containsNAN = true;
    }
  }
  return containsNAN;
}

//}

/* //{ callbackDrs() */

void ControllerModule::callbackDrs(controller_module::controller_paramsConfig &params, [[maybe_unused]] uint32_t level) {
  if (!is_initialized_) {
    return;
  }
  Eigen::VectorXd new_costs = Eigen::VectorXd::Zero(MODIFIABLE_COSTS);
  new_costs(0) = params.Q_pos_xy;
  new_costs(1) = params.Q_pos_z;
  new_costs(2) = params.Q_quat_xy;
  new_costs(3) = params.Q_quat_z;
  new_costs(4) = params.Q_velocity;
  new_costs(5) = params.Q_omega;
  new_costs(6) = params.Q_int;
  new_costs(7) = params.Q_thrust;
  new_costs(8) = params.R_col_thrust;
  new_costs(9) = params.R_ome_in;
  Eigen::VectorXd new_constants = Eigen::VectorXd::Zero(MODIFIABLE_COSTS);
  new_constants(0) = params.mass;
  new_constants(1) = params.arm_length;
  new_constants(2) = params.inertia_x;
  new_constants(3) = params.inertia_y;
  new_constants(4) = params.inertia_z;
  new_constants(5) = params.ctau;
  {
    std::scoped_lock lock(mutex_acados_drone_);
    ac.setNewCosts(new_costs);
    // ac.setNewConstants(new_constants);
    // quadcopter = Drone(new_constants(0), new_constants(1), new_constants(5), Vector<3>(new_constants(2), new_constants(3), new_constants(4)));
  }

  verbose = params.verbose;

  publishAcadosParams();

  ROS_INFO("[ControllerModule]: DRS updated");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* TimerInspector() //{ */

void ControllerModule::TimerInspector(const ros::TimerEvent &te) {
  if (!is_active_) {
    return;
  }
  if (verbose == false) {
    ROS_INFO("------------- NMPC Controller status ------------------");
    ROS_INFO_STREAM("max computation time: " << max_computation_time_s << " s");
    ROS_INFO_STREAM("current velocity: " << actual_velocity << " m/s");
    ROS_INFO_STREAM(" - actual voltage: " << actual_voltage << " V");
    ROS_INFO("-------------------------------------------------------");
  }

  // MRS UAV status
  std::string string_to_show = "-g";
  // set system name
  string_to_show += " NMPC T: ";
  std::stringstream stream;
  stream << std::fixed << std::setprecision(3) << max_computation_time_s;
  std::stringstream stream2;
  stream2 << std::fixed << std::setprecision(2) << actual_velocity;
  string_to_show += (" " + stream.str());
  string_to_show += " s";
  string_to_show += ",";
  string_to_show += "v: ";
  string_to_show += (stream2.str());
  string_to_show += " m/s";

  std_msgs::String msg;
  msg.data = string_to_show;

  // publish string message
  try {
    publisher_mrs_uav_status_.publish(msg);
  } catch (...) {
    ROS_ERROR("[NmpcController]: Exception caught during publishing topic %s.", publisher_mrs_uav_status_.getTopic().c_str());
  }

  // zero max computation time
  max_computation_time_s = 0.0;
}

//}

} // namespace controller_module

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(controller_module::ControllerModule, mrs_uav_managers::Controller)
