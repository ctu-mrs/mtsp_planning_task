/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mavros_msgs/State.h>

#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/ValidateReference.h>

#include <nav_msgs/Odometry.h>

#include <mtsp_msgs/TspProblem.h>

#include <mrs_lib/param_loader.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//}

/* defines //{ */

#define TARGET_NUM 1000
#define COLLISION_PENALTY 10.0
#define MISSED_TARGET_PENALTY 5.0

//}

namespace mtsp_state_machine
{

/* class MtspStateMachine //{ */

// state machine
typedef enum
{

  IDLE_STATE,
  PLANNING_STATE,
  LOADING_STATE,
  VALIDATING_STATE,
  TAKEOFF_STATE,
  FLY_TO_START_STATE,
  FOLLOWING_STATE,
  LANDING_STATE

} State_t;

const char* state_names[8] = {

    "IDLING", "PLANNING TRAJECTORIES", "LOADING TRAJECTORIES", "VALIDATING CURRENT POSITION", "TAKING OFF", "FLYING TO START", "FOLLOWING", "LANDING"};

class MtspStateMachine : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

  int  service_call_repeat_;
  bool land_ = false;

  bool start_position_1_valid = false;
  bool start_position_2_valid = false;

  Eigen::MatrixXd safety_area_points;

  // | --------------------- service clients -------------------- |
  ros::ServiceClient service_client_load_problem;

  ros::ServiceClient service_client_land_1;
  ros::ServiceClient service_client_land_2;

  ros::ServiceClient service_client_fly_to_start_1;
  ros::ServiceClient service_client_fly_to_start_2;

  ros::ServiceClient service_client_start_following_1;
  ros::ServiceClient service_client_start_following_2;

  ros::ServiceClient service_client_load_trajectory_1;
  ros::ServiceClient service_client_load_trajectory_2;

  ros::ServiceClient service_client_validate_start_position_1;
  ros::ServiceClient service_client_validate_start_position_2;

  // | --------------------- service servers -------------------- |
  ros::ServiceServer service_server_start;

  // | ----------------------- subscribers ---------------------- |
  ros::Subscriber subscriber_mavros_state;

  ros::Subscriber subscriber_control_manager_diagnostics_1;
  ros::Subscriber subscriber_control_manager_diagnostics_2;

  ros::Subscriber subscriber_mpc_diagnostics_1;
  ros::Subscriber subscriber_mpc_diagnostics_2;

  ros::Subscriber subscriber_trajectory_1;
  ros::Subscriber subscriber_trajectory_2;

  ros::Subscriber subscriber_odom_1;
  ros::Subscriber subscriber_odom_2;

  ros::Subscriber subscriber_problem;

  // | ----------------------- publishers ----------------------- |
  ros::Publisher publisher_rviz;

  ros::Publisher publisher_ready_to_takeoff_1;
  ros::Publisher publisher_ready_to_takeoff_2;

  // | ----------------------- main timer ----------------------- |
  ros::Timer main_timer;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

  // | ----------------------- rviz timer ----------------------- |
  ros::Timer rviz_timer;
  void       rvizTimer(const ros::TimerEvent& event);
  double     rviz_timer_rate_;

  // | ----------------- distance checker timer ----------------- |
  ros::Timer distance_timer;
  void       distanceTimer(const ros::TimerEvent& event);
  double     distance_timer_rate_;

  // | --------------------- Oneshot timers --------------------- |
  ros::Timer takeoff_timer_1;
  ros::Timer takeoff_timer_2;

  ros::Timer land_timer_1;
  ros::Timer land_timer_2;

  ros::Timer fly_to_start_timer_1;
  ros::Timer fly_to_start_timer_2;

  ros::Timer start_following_timer_1;
  ros::Timer start_following_timer_2;

  ros::Timer load_trajectory_timer_1;
  ros::Timer load_trajectory_timer_2;

  ros::Timer validate_start_position_timer_1;
  ros::Timer validate_start_position_timer_2;

  // | -------------------- MPC diagnostics 1 ------------------- |
  void                            callbackMpcDiagnostics1(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg);
  std::mutex                      mutex_mpc_diagnostics_1;
  mrs_msgs::MpcTrackerDiagnostics mpc_diagnostics_1;
  bool                            got_mpc_diagnostics_1 = false;

  // | -------------------- MPC diagnostics 2 ------------------- |
  void                            callbackMpcDiagnostics2(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg);
  std::mutex                      mutex_mpc_diagnostics_2;
  mrs_msgs::MpcTrackerDiagnostics mpc_diagnostics_2;
  bool                            got_mpc_diagnostics_2 = false;

  // | -------------------- Tracker status 1 ------------------- |
  void                                callbackControlManagerDiagnostics1(const mrs_msgs::ControlManagerDiagnosticsPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_1;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_1;
  bool                                got_control_manager_diagnostics_1 = false;

  // | -------------------- Tracker status 2 ------------------- |
  void                                callbackControlManagerDiagnostics2(const mrs_msgs::ControlManagerDiagnosticsPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_2;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_2;
  bool                                got_control_manager_diagnostics_2 = false;

  // | ---------------------- Trajectory 1 ---------------------- |
  void                          callbackTrajectory1(const mrs_msgs::TrajectoryReferenceConstPtr& msg);
  bool                          got_trajectory_1    = false;
  bool                          trajectory_1_loaded = false;
  std::mutex                    mutex_trajectory_1;
  mrs_msgs::TrajectoryReference trajectory_1;

  // | ---------------------- Trajectory 2 ---------------------- |
  void                          callbackTrajectory2(const mrs_msgs::TrajectoryReferenceConstPtr& msg);
  bool                          got_trajectory_2    = false;
  bool                          trajectory_2_loaded = false;
  std::mutex                    mutex_trajectory_2;
  mrs_msgs::TrajectoryReference trajectory_2;

  // | ----------------------- Odometry 1 ----------------------- |
  void               callbackOdometry1(const nav_msgs::OdometryConstPtr& msg);
  bool               got_odometry_1 = false;
  std::mutex         mutex_odometry_1;
  nav_msgs::Odometry odometry_1;

  // | ----------------------- Odometry 2 ----------------------- |
  void               callbackOdometry2(const nav_msgs::OdometryConstPtr& msg);
  bool               got_odometry_2 = false;
  std::mutex         mutex_odometry_2;
  nav_msgs::Odometry odometry_2;

  // | ------------------------- Problem ------------------------ |
  void                  callbackProblem(const mtsp_msgs::TspProblemConstPtr& msg);
  bool                  got_problem = false;
  std::mutex            mutex_problem;
  mtsp_msgs::TspProblem problem;

  // | --------------------- other callbacks -------------------- |
  bool callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // | ------------------- state machine stuff ------------------ |
  State_t current_state = IDLE_STATE;
  void    switchState(State_t new_state);

  void callForProblem(void);

  // | --------------------- Oneshot timers --------------------- |
  void validateStartPositionTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void takeoffTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void landTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void flyToStartTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void startFollowingTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void loadTrajectoryTimerOneshot(const ros::TimerEvent& event, uint8_t id);

  void validateStartPosition(void);
  void takeoff(void);
  void land(void);
  void flyToStart(void);
  void startFollowing(void);
  void loadTrajectories(void);

  bool StartPositionValid(void);
  bool MpcTrackerActive(void);
  bool MpcFlightInProgress(void);

  // | --------------------- score counting --------------------- |
  void       printFinalScore(void);
  void       resetProgress(void);
  bool       point_distances[TARGET_NUM];
  std::mutex mutex_point_distances;
  ros::Time  start_time;
  bool       collision_avoidance_triggered = false;
};
//}

/* inInit() //{ */

void MtspStateMachine::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  for (int i = 0; i < TARGET_NUM; i++) {
    point_distances[i] = false;
  }

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "MtspStateMachine");

  param_loader.loadParam("main_timer_rate", main_timer_rate_);
  param_loader.loadParam("rviz_timer_rate", rviz_timer_rate_);
  param_loader.loadParam("distance_timer_rate", distance_timer_rate_);
  param_loader.loadParam("service_call_repeat", service_call_repeat_);
  param_loader.loadParam("land", land_);

  safety_area_points = param_loader.loadMatrixDynamic2("safety_area/safety_area", -1, 2);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_mpc_diagnostics_1 = nh_.subscribe("mpc_diagnostics_1_in", 1, &MtspStateMachine::callbackMpcDiagnostics1, this, ros::TransportHints().tcpNoDelay());
  subscriber_mpc_diagnostics_2 = nh_.subscribe("mpc_diagnostics_2_in", 1, &MtspStateMachine::callbackMpcDiagnostics2, this, ros::TransportHints().tcpNoDelay());

  subscriber_control_manager_diagnostics_1 =
      nh_.subscribe("control_manager_diagnostics_1_in", 1, &MtspStateMachine::callbackControlManagerDiagnostics1, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_2 =
      nh_.subscribe("control_manager_diagnostics_2_in", 1, &MtspStateMachine::callbackControlManagerDiagnostics2, this, ros::TransportHints().tcpNoDelay());

  subscriber_trajectory_1 = nh_.subscribe("trajectory_1_in", 1, &MtspStateMachine::callbackTrajectory1, this, ros::TransportHints().tcpNoDelay());
  subscriber_trajectory_2 = nh_.subscribe("trajectory_2_in", 1, &MtspStateMachine::callbackTrajectory2, this, ros::TransportHints().tcpNoDelay());

  subscriber_odom_1 = nh_.subscribe("odometry_1_in", 1, &MtspStateMachine::callbackOdometry1, this, ros::TransportHints().tcpNoDelay());
  subscriber_odom_2 = nh_.subscribe("odometry_2_in", 1, &MtspStateMachine::callbackOdometry2, this, ros::TransportHints().tcpNoDelay());

  subscriber_problem = nh_.subscribe("problem_in", 1, &MtspStateMachine::callbackProblem, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_rviz = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array_out", 1);

  publisher_ready_to_takeoff_1 = nh_.advertise<std_msgs::UInt8>("takeoff_1_out", 1);
  publisher_ready_to_takeoff_2 = nh_.advertise<std_msgs::UInt8>("takeoff_2_out", 1);

  // --------------------------------------------------------------
  // |                       service clients                      |
  // --------------------------------------------------------------

  service_client_load_problem = nh_.serviceClient<std_srvs::Trigger>("load_trajectory_out");

  service_client_land_1 = nh_.serviceClient<std_srvs::Trigger>("land_1_out");
  service_client_land_2 = nh_.serviceClient<std_srvs::Trigger>("land_2_out");

  service_client_fly_to_start_1 = nh_.serviceClient<std_srvs::Trigger>("fly_to_start_1_out");
  service_client_fly_to_start_2 = nh_.serviceClient<std_srvs::Trigger>("fly_to_start_2_out");

  service_client_start_following_1 = nh_.serviceClient<std_srvs::Trigger>("start_following_1_out");
  service_client_start_following_2 = nh_.serviceClient<std_srvs::Trigger>("start_following_2_out");

  service_client_load_trajectory_1 = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("load_trajectory_1_out");
  service_client_load_trajectory_2 = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("load_trajectory_2_out");

  service_client_validate_start_position_1 = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_start_position_1_out");
  service_client_validate_start_position_2 = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_start_position_2_out");

  // --------------------------------------------------------------
  // |                       service servers                      |
  // --------------------------------------------------------------

  service_server_start = nh_.advertiseService("start_in", &MtspStateMachine::callbackStart, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer     = nh_.createTimer(ros::Rate(main_timer_rate_), &MtspStateMachine::mainTimer, this);
  rviz_timer     = nh_.createTimer(ros::Rate(rviz_timer_rate_), &MtspStateMachine::rvizTimer, this);
  distance_timer = nh_.createTimer(ros::Rate(distance_timer_rate_), &MtspStateMachine::distanceTimer, this);

  load_trajectory_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::loadTrajectoryTimerOneshot, this, _1, 1), true, false);
  load_trajectory_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::loadTrajectoryTimerOneshot, this, _1, 2), true, false);

  start_following_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::startFollowingTimerOneshot, this, _1, 1), true, false);
  start_following_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::startFollowingTimerOneshot, this, _1, 2), true, false);

  fly_to_start_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::flyToStartTimerOneshot, this, _1, 1), true, false);
  fly_to_start_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::flyToStartTimerOneshot, this, _1, 2), true, false);

  land_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::landTimerOneshot, this, _1, 1), true, false);
  land_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::landTimerOneshot, this, _1, 2), true, false);

  takeoff_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::takeoffTimerOneshot, this, _1, 1), true, false);
  takeoff_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::takeoffTimerOneshot, this, _1, 2), true, false);

  validate_start_position_timer_1 =
      nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::validateStartPositionTimerOneshot, this, _1, 1), true, false);
  validate_start_position_timer_2 =
      nh_.createTimer(ros::Duration(0), boost::bind(&MtspStateMachine::validateStartPositionTimerOneshot, this, _1, 2), true, false);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MavrosInterface]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[MtspStateMachine]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackMpcDiagnostics1() //{ */

void MtspStateMachine::callbackMpcDiagnostics1(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MtspStateMachine]: getting mpc diagnostics (1)");

  std::scoped_lock lock(mutex_mpc_diagnostics_1);

  got_mpc_diagnostics_1 = true;

  mpc_diagnostics_1 = *msg;

  if (msg->avoiding_collision) {
    collision_avoidance_triggered = true;
    ROS_WARN_THROTTLE(1.0, "[MtspStateMachine]: Potential collision between the UAVs detected!");
  }
}

//}

/* callbackMpcDiagnostics2() //{ */

void MtspStateMachine::callbackMpcDiagnostics2(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MtspStateMachine]: getting mpc diagnostics (2)");

  std::scoped_lock lock(mutex_mpc_diagnostics_2);

  got_mpc_diagnostics_2 = true;

  mpc_diagnostics_2 = *msg;

  if (msg->avoiding_collision) {
    collision_avoidance_triggered = true;
    ROS_WARN_THROTTLE(1.0, "[MtspStateMachine]: Potential collision between the UAVs detected!");
  }
}

//}

/* callbackControlManagerDiagnostics1() //{ */

void MtspStateMachine::callbackControlManagerDiagnostics1(const mrs_msgs::ControlManagerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MtspStateMachine]: getting control manager diagnostics (1)");

  std::scoped_lock lock(mutex_control_manager_diagnostics_1);

  got_control_manager_diagnostics_1 = true;

  control_manager_diagnostics_1 = *msg;
}

//}

/* callbackControlManagerDiagnostics2() //{ */

void MtspStateMachine::callbackControlManagerDiagnostics2(const mrs_msgs::ControlManagerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MtspStateMachine]: getting control manager diagnostics (2)");

  std::scoped_lock lock(mutex_control_manager_diagnostics_2);

  got_control_manager_diagnostics_2 = true;

  control_manager_diagnostics_2 = *msg;
}

//}

/* callbackTrajectory1() //{ */

void MtspStateMachine::callbackTrajectory1(const mrs_msgs::TrajectoryReferenceConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO("[MtspStateMachine]: received trajectory (1)");

  std::scoped_lock lock(mutex_trajectory_1);

  trajectory_1 = *msg;

  got_trajectory_1 = true;
}

//}

/* callbackTrajectory2() //{ */

void MtspStateMachine::callbackTrajectory2(const mrs_msgs::TrajectoryReferenceConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO("[MtspStateMachine]: received trajectory (2)");

  std::scoped_lock lock(mutex_trajectory_2);

  trajectory_2 = *msg;

  got_trajectory_2 = true;
}

//}

/* callbackOdometry1() //{ */

void MtspStateMachine::callbackOdometry1(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MtspStateMachine]: receiving odometry (1)");

  std::scoped_lock lock(mutex_odometry_2);

  odometry_1 = *msg;

  got_odometry_1 = true;
}

//}

/* callbackOdometry2() //{ */

void MtspStateMachine::callbackOdometry2(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MtspStateMachine]: receiving odometry (2)");

  std::scoped_lock lock(mutex_odometry_2);

  odometry_2 = *msg;

  got_odometry_2 = true;
}

//}

/* callbackProblem() //{ */

void MtspStateMachine::callbackProblem(const mtsp_msgs::TspProblemConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MtspStateMachine]: received problem");

  std::scoped_lock lock(mutex_problem);

  problem = *msg;

  mtsp_msgs::TspPoint start_point1;
  start_point1.x = problem.start_points[0].x;
  start_point1.y = problem.start_points[0].y;

  mtsp_msgs::TspPoint start_point2;
  start_point2.x = problem.start_points[1].x;
  start_point2.y = problem.start_points[1].y;

  problem.points.push_back(start_point1);
  problem.points.push_back(start_point2);

  got_problem = true;
}

//}

/* callbackStart() //{ */

bool MtspStateMachine::callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!got_odometry_1 || !got_odometry_2) {

    res.success = false;
    res.message = "missing odometries";

    return true;
  }

  switchState(PLANNING_STATE);

  res.success = true;
  res.message = "stared";

  return true;
}

//}

// --------------------------------------------------------------
// |                        main routines                       |
// --------------------------------------------------------------

/* callForProblem() //{ */

void MtspStateMachine::callForProblem(void) {

  std_srvs::Trigger srv_out;

  for (int i = 0; i < service_call_repeat_; i++) {

    service_client_load_problem.call(srv_out);

    if (!srv_out.response.success) {

      ROS_ERROR("[UavManager]: call for problem failed: %s", srv_out.response.message.c_str());
    } else {

      ROS_INFO("[UavManager]: called for problem");
      break;
    }
  }
}

//}

/* MpcTrackerActive() //{ */

bool MtspStateMachine::MpcTrackerActive(void) {

  std::scoped_lock lock(mutex_mpc_diagnostics_1, mutex_mpc_diagnostics_2);

  if (!got_mpc_diagnostics_1 || !got_mpc_diagnostics_2) {
    return false;
  }

  if (mpc_diagnostics_1.active && mpc_diagnostics_2.active) {
    return true;
  }

  return false;
}

//}

/* MpcFlightInProgress() //{ */

bool MtspStateMachine::MpcFlightInProgress(void) {

  std::scoped_lock lock(mutex_mpc_diagnostics_1, mutex_mpc_diagnostics_2);

  if (control_manager_diagnostics_1.tracker_status.have_goal || control_manager_diagnostics_2.tracker_status.have_goal) {
    return true;
  }

  return false;
}

//}

// | --------------------- oneshot timers --------------------- |

/* validateStartPositionTimerOneshot() //{ */

void MtspStateMachine::validateStartPositionTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  mrs_msgs::ValidateReference ref_out;
  ref_out.request.reference.header.frame_id = "fcu";

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_validate_start_position_1.call(ref_out);
    } else {
      service_client_validate_start_position_2.call(ref_out);
    }

    if (!ref_out.response.success) {

      ROS_ERROR("[UavManager]: current uav position (%d) is not valid: %s", id, ref_out.response.message.c_str());

    } else {

      if (id == 1) {
        start_position_1_valid = true;
      } else {
        start_position_2_valid = true;
      }
      ROS_INFO("[MtspStateMachine]: current uav position (%d) is valid", id);
      break;
    }
  }
}

//}

/* takeoffTimerOneshot() //{ */

void MtspStateMachine::takeoffTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_msgs::UInt8 ready_out;
  ready_out.data = id;

  ROS_INFO("[MtspStateMachine]: vehicle (%d) ready for takeoff", id);

  while (ros::ok()) {

    if (id == 1) {
      std::scoped_lock lock(mutex_mpc_diagnostics_1);
      publisher_ready_to_takeoff_1.publish(ready_out);
      if (mpc_diagnostics_1.active) {
        ROS_INFO("[MtspStateMachine]: takeoff (%d) successful", id);
        break;
      }
    } else {
      std::scoped_lock lock(mutex_mpc_diagnostics_2);
      publisher_ready_to_takeoff_2.publish(ready_out);
      if (mpc_diagnostics_2.active) {
        ROS_INFO("[MtspStateMachine]: takeoff (%d) successful", id);
        break;
      }
    }
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }

  std::scoped_lock lock(mutex_mpc_diagnostics_1, mutex_mpc_diagnostics_2);
  if (id == 1 && !mpc_diagnostics_1.active) {
    ROS_ERROR("[MtspStateMachine]: takeoff (%d) failed", id);
  }
  if (id == 2 && !mpc_diagnostics_2.active) {
    ROS_ERROR("[MtspStateMachine]: takeoff (%d) failed", id);
  }
}

//}

/* landTimerOneshot() //{ */

void MtspStateMachine::landTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_srvs::Trigger srv_out;

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_land_1.call(srv_out);
    } else {
      service_client_land_2.call(srv_out);
    }

    if (!srv_out.response.success) {

      ROS_ERROR("[UavManager]: call for land (%d) failed: %s", id, srv_out.response.message.c_str());
    } else {

      ROS_INFO("[MtspStateMachine]: landing (%d)", id);
      break;
    }
  }
}

//}

/* flyToStartTimerOneshot() //{ */

void MtspStateMachine::flyToStartTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_srvs::Trigger srv_out;

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_fly_to_start_1.call(srv_out);
    } else {
      service_client_fly_to_start_2.call(srv_out);
    }

    if (!srv_out.response.success) {

      ROS_ERROR("[UavManager]: call for flying to start (%d) failed: %s", id, srv_out.response.message.c_str());
    } else {

      ROS_INFO("[MtspStateMachine]: flying to start (%d)", id);
      break;
    }
  }
}

//}

/* startFollowingTimerOneshot() //{ */

void MtspStateMachine::startFollowingTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_srvs::Trigger srv_out;

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_start_following_1.call(srv_out);
    } else {
      service_client_start_following_2.call(srv_out);
    }

    if (!srv_out.response.success) {

      ROS_ERROR("[UavManager]: call for start following (%d) failed: %s", id, srv_out.response.message.c_str());
    } else {

      ROS_INFO("[MtspStateMachine]: started following (%d)", id);
      break;
    }
  }
}

//}

/* loadTrajectoryTimerOneshot() //{ */

void MtspStateMachine::loadTrajectoryTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  mrs_msgs::TrajectoryReferenceSrv srv_out;
  if (id == 1) {
    srv_out.request.trajectory = trajectory_1;
  } else {
    srv_out.request.trajectory = trajectory_2;
  }

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_load_trajectory_1.call(srv_out);
    } else {
      service_client_load_trajectory_2.call(srv_out);
    }

    if (!srv_out.response.success) {
      // returned false because the MPC tracker is not currently active

      for (unsigned int i = 0; i < srv_out.response.tracker_names.size(); i++) {
        if (srv_out.response.tracker_names[i] == "MpcTracker") {
          if (srv_out.response.tracker_successes[i]) {
            // trajectory is feasible for the MPC tracker
            goto loaded;
          }
        }
      }
      ROS_ERROR("[UavManager]: call for loading trajectory (%d) failed", id);
    } else {
    loaded:

      ROS_INFO("[MtspStateMachine]: trajectory loaded (%d)", id);

      if (id == 1) {
        got_trajectory_1    = false;
        trajectory_1_loaded = true;
      } else {
        got_trajectory_2    = false;
        trajectory_2_loaded = true;
      }

      break;
    }
  }
}

//}

// | -------------------- control services -------------------- |

/* validateStartPosition //{ */

void MtspStateMachine::validateStartPosition(void) {
  validate_start_position_timer_1.stop();
  validate_start_position_timer_2.stop();

  validate_start_position_timer_1.start();
  validate_start_position_timer_2.start();
}

//}


/* takeoff() //{ */

void MtspStateMachine::takeoff(void) {

  takeoff_timer_1.stop();
  takeoff_timer_2.stop();

  takeoff_timer_1.start();
  takeoff_timer_2.start();
}

//}

/* land() //{ */

void MtspStateMachine::land(void) {

  land_timer_1.stop();
  land_timer_2.stop();

  land_timer_1.start();
  land_timer_2.start();
}

//}

/* flyToStart() //{ */

void MtspStateMachine::flyToStart(void) {

  resetProgress();

  fly_to_start_timer_1.stop();
  fly_to_start_timer_2.stop();

  fly_to_start_timer_1.start();
  fly_to_start_timer_2.start();
}

//}

/* startFollowing() //{ */

void MtspStateMachine::startFollowing(void) {

  resetProgress();

  start_following_timer_1.stop();
  start_following_timer_2.stop();

  start_following_timer_1.start();
  start_following_timer_2.start();
}

//}

/* loadTrajectories() //{ */

void MtspStateMachine::loadTrajectories(void) {

  load_trajectory_timer_1.stop();
  load_trajectory_timer_2.stop();

  load_trajectory_timer_1.start();
  load_trajectory_timer_2.start();
}

//}

/* dist2d //{ */

static double dist2d(double x1, double y1, double x2, double y2) {

  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

//}

// | ---------------------- state machine --------------------- |

/* switchState() //{ */

void MtspStateMachine::switchState(State_t new_state) {

  ROS_WARN("[MtspStateMachine]: switching state from %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case IDLE_STATE: {

      got_trajectory_1 = false;
      got_trajectory_2 = false;

      got_mpc_diagnostics_1 = false;
      got_mpc_diagnostics_2 = false;

      collision_avoidance_triggered = false;

      break;
    }

    case PLANNING_STATE: {

      callForProblem();

      break;
    }

    case LOADING_STATE: {

      // swap the trajectories according to where the drones are
      double uav1_dist1 =
          dist2d(odometry_1.pose.pose.position.x, odometry_1.pose.pose.position.y, trajectory_1.points[0].position.x, trajectory_1.points[0].position.y);
      double uav2_dist1 =
          dist2d(odometry_2.pose.pose.position.x, odometry_2.pose.pose.position.y, trajectory_1.points[0].position.x, trajectory_1.points[0].position.y);

      if (uav1_dist1 > uav2_dist1) {

        mrs_msgs::TrajectoryReference temp_trajectory = trajectory_1;

        trajectory_1 = trajectory_2;
        trajectory_2 = temp_trajectory;

        ROS_INFO("[MtspStateMachine]: swapping trajectories to minimize fly2start distance.");
      }

      loadTrajectories();

      break;
    }

    case VALIDATING_STATE: {

      validateStartPosition();

      break;
    }
    case TAKEOFF_STATE: {

      takeoff();

      break;
    }

    case FLY_TO_START_STATE: {

      flyToStart();

      break;
    }

    case FOLLOWING_STATE: {

      startFollowing();

      start_time = ros::Time::now();

      break;
    }

    case LANDING_STATE: {

      land();

      break;
    }

    break;
  }

  ros::Duration sleep(1.0);
  sleep.sleep();

  current_state = new_state;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void MtspStateMachine::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  switch (current_state) {

    case IDLE_STATE: {

      if (MpcTrackerActive()) {
        if (got_trajectory_1 && got_trajectory_2) {
          switchState(LOADING_STATE);
        }
      }

      break;
    }

    case PLANNING_STATE: {

      if (got_trajectory_1 && got_trajectory_2) {

        // check the start positions of the trajectories
        bool start1_ok = false;
        bool start2_ok = false;

        if ((dist2d(trajectory_1.points[0].position.x, trajectory_1.points[0].position.y, problem.start_points[0].x, problem.start_points[0].y) <=
             problem.neighborhood_radius) ||
            (dist2d(trajectory_2.points[1].position.x, trajectory_2.points[1].position.y, problem.start_points[0].x, problem.start_points[0].y) <=
             problem.neighborhood_radius)) {
          start1_ok = true;
        }
        if ((dist2d(trajectory_1.points[0].position.x, trajectory_1.points[0].position.y, problem.start_points[1].x, problem.start_points[1].y) <=
             problem.neighborhood_radius) ||
            (dist2d(trajectory_2.points[1].position.x, trajectory_2.points[1].position.y, problem.start_points[1].x, problem.start_points[1].y) <=
             problem.neighborhood_radius)) {
          start2_ok = true;
        }

        if (start1_ok && start2_ok) {

          switchState(LOADING_STATE);

        } else {

          ROS_ERROR("[MtspStateMachine]: Starting point condition is not satisfied!");
          switchState(IDLE_STATE);
        }
      }

      break;
    }

    case LOADING_STATE: {

      if (trajectory_1_loaded && trajectory_2_loaded) {

        if (MpcTrackerActive()) {
          switchState(FLY_TO_START_STATE);
        } else {
          switchState(VALIDATING_STATE);
        }
      }

      break;
    }

    case VALIDATING_STATE: {

      if (start_position_1_valid && start_position_2_valid) {
        switchState(TAKEOFF_STATE);
      } else {
        ROS_ERROR("[MtspStateMachine]: Invalid uav position, cannot takeoff!");
        switchState(IDLE_STATE);
      }

      break;
    }

    case TAKEOFF_STATE: {

      if (MpcTrackerActive() && !MpcFlightInProgress()) {

        switchState(FLY_TO_START_STATE);
      }

      break;
    }

    case FLY_TO_START_STATE: {

      if (!MpcFlightInProgress()) {

        switchState(FOLLOWING_STATE);
      }

      break;
    }

    case FOLLOWING_STATE: {

      if (!MpcFlightInProgress()) {

        printFinalScore();

        if (land_) {
          switchState(LANDING_STATE);
        } else {
          switchState(IDLE_STATE);
        }
      }
      {
        if (got_trajectory_1 && got_trajectory_2) {

          switchState(LOADING_STATE);
        }
      }

      break;
    }

    case LANDING_STATE: {

      switchState(IDLE_STATE);

      break;
    }

    break;
  }
}  // namespace mtsp_state_machine

//}

/* rvizTimer() //{ */

void MtspStateMachine::rvizTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized) {
    return;
  }

  if (!got_problem) {
    return;
  }

  std::scoped_lock lock(mutex_problem);

  visualization_msgs::MarkerArray msg_out;

  int id = 0;

  for (uint i = 0; i < problem.points.size(); i++) {

    bool visied = point_distances[i];

    visualization_msgs::Marker marker;
    marker.header.frame_id    = "common_origin";
    marker.header.stamp       = ros::Time::now();
    marker.ns                 = "mtsp";
    marker.id                 = id++;
    marker.type               = visualization_msgs::Marker::CYLINDER;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.position.x    = problem.points[i].x;
    marker.pose.position.y    = problem.points[i].y;
    marker.pose.position.z    = problem.height;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 2.0 * problem.neighborhood_radius;
    marker.scale.y            = 2.0 * problem.neighborhood_radius;
    marker.scale.z            = 1;
    marker.color.a            = 0.3;
    marker.color.r            = visied ? 0.0 : 1.0;
    marker.color.g            = visied ? 1.0 : 0.0;
    marker.color.b            = 0.0;

    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    msg_out.markers.push_back(marker);
  }

  {
    std::scoped_lock lock(mutex_trajectory_1);

    if (trajectory_1_loaded) {

      visualization_msgs::Marker line_list;
      line_list.header.frame_id    = "common_origin";
      line_list.header.stamp       = ros::Time::now();
      line_list.ns                 = "mtsp";
      line_list.id                 = id++;
      line_list.type               = visualization_msgs::Marker::LINE_STRIP;
      line_list.action             = visualization_msgs::Marker::ADD;
      line_list.pose.position.x    = 0;
      line_list.pose.position.y    = 0;
      line_list.pose.position.z    = 0;
      line_list.pose.orientation.x = 0.0;
      line_list.pose.orientation.y = 0.0;
      line_list.pose.orientation.z = 0.0;
      line_list.pose.orientation.w = 1.0;
      line_list.scale.x            = 0.05;
      line_list.scale.y            = 0.05;
      line_list.scale.z            = 0.05;
      line_list.color.a            = 0.3;
      line_list.color.r            = 0.0;
      line_list.color.g            = 0.0;
      line_list.color.b            = 0.0;

      visualization_msgs::Marker point_list;
      point_list.header.frame_id    = "common_origin";
      point_list.header.stamp       = ros::Time::now();
      point_list.ns                 = "mtsp";
      point_list.id                 = id++;
      point_list.type               = visualization_msgs::Marker::POINTS;
      point_list.action             = visualization_msgs::Marker::ADD;
      point_list.pose.position.x    = 0;
      point_list.pose.position.y    = 0;
      point_list.pose.position.z    = 0;
      point_list.pose.orientation.x = 0.0;
      point_list.pose.orientation.y = 0.0;
      point_list.pose.orientation.z = 0.0;
      point_list.pose.orientation.w = 1.0;
      point_list.scale.x            = 0.1;
      point_list.scale.y            = 0.1;
      point_list.scale.z            = 0.1;
      point_list.color.a            = 1.0;
      point_list.color.r            = 0.0;
      point_list.color.g            = 0.0;
      point_list.color.b            = 0.0;

      for (uint i = 0; i < trajectory_1.points.size(); i++) {

        geometry_msgs::Point point;
        point.x = trajectory_1.points[i].position.x;
        point.y = trajectory_1.points[i].position.y;
        point.z = problem.height;

        line_list.points.push_back(point);
        point_list.points.push_back(point);
      }

      msg_out.markers.push_back(line_list);
      msg_out.markers.push_back(point_list);
    }
  }

  {
    std::scoped_lock lock(mutex_trajectory_2);

    if (trajectory_2_loaded) {

      visualization_msgs::Marker line_list;
      line_list.header.frame_id    = "common_origin";
      line_list.header.stamp       = ros::Time::now();
      line_list.ns                 = "mtsp";
      line_list.id                 = id++;
      line_list.type               = visualization_msgs::Marker::LINE_STRIP;
      line_list.action             = visualization_msgs::Marker::ADD;
      line_list.pose.position.x    = 0;
      line_list.pose.position.y    = 0;
      line_list.pose.position.z    = 0;
      line_list.pose.orientation.x = 0.0;
      line_list.pose.orientation.y = 0.0;
      line_list.pose.orientation.z = 0.0;
      line_list.pose.orientation.w = 1.0;
      line_list.scale.x            = 0.05;
      line_list.scale.y            = 0.05;
      line_list.scale.z            = 0.05;
      line_list.color.a            = 0.3;
      line_list.color.r            = 0.0;
      line_list.color.g            = 0.0;
      line_list.color.b            = 0.0;

      visualization_msgs::Marker point_list;
      point_list.header.frame_id    = "common_origin";
      point_list.header.stamp       = ros::Time::now();
      point_list.ns                 = "mtsp";
      point_list.id                 = id++;
      point_list.type               = visualization_msgs::Marker::POINTS;
      point_list.action             = visualization_msgs::Marker::ADD;
      point_list.pose.position.x    = 0;
      point_list.pose.position.y    = 0;
      point_list.pose.position.z    = 0;
      point_list.pose.orientation.x = 0.0;
      point_list.pose.orientation.y = 0.0;
      point_list.pose.orientation.z = 0.0;
      point_list.pose.orientation.w = 1.0;
      point_list.scale.x            = 0.1;
      point_list.scale.y            = 0.1;
      point_list.scale.z            = 0.1;
      point_list.color.a            = 1.0;
      point_list.color.r            = 0.0;
      point_list.color.g            = 0.0;
      point_list.color.b            = 0.0;

      for (uint i = 0; i < trajectory_2.points.size(); i++) {

        geometry_msgs::Point point;
        point.x = trajectory_2.points[i].position.x;
        point.y = trajectory_2.points[i].position.y;
        point.z = problem.height;

        line_list.points.push_back(point);
        point_list.points.push_back(point);
      }

      msg_out.markers.push_back(line_list);
      msg_out.markers.push_back(point_list);
    }
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id    = "common_origin";
  line_list.header.stamp       = ros::Time::now();
  line_list.ns                 = "mtsp";
  line_list.id                 = id++;
  line_list.type               = visualization_msgs::Marker::LINE_STRIP;
  line_list.action             = visualization_msgs::Marker::ADD;
  line_list.pose.position.x    = 0;
  line_list.pose.position.y    = 0;
  line_list.pose.position.z    = 0;
  line_list.pose.orientation.x = 0.0;
  line_list.pose.orientation.y = 0.0;
  line_list.pose.orientation.z = 0.0;
  line_list.pose.orientation.w = 1.0;
  line_list.scale.x            = 0.2;
  line_list.scale.y            = 0.2;
  line_list.scale.z            = 0.2;
  line_list.color.a            = 0.3;
  line_list.color.r            = 1.0;
  line_list.color.g            = 0.0;
  line_list.color.b            = 0.0;

  visualization_msgs::Marker point_list;
  point_list.header.frame_id    = "common_origin";
  point_list.header.stamp       = ros::Time::now();
  point_list.ns                 = "mtsp";
  point_list.id                 = id++;
  point_list.type               = visualization_msgs::Marker::POINTS;
  point_list.action             = visualization_msgs::Marker::ADD;
  point_list.pose.position.x    = 0;
  point_list.pose.position.y    = 0;
  point_list.pose.position.z    = 0;
  point_list.pose.orientation.x = 0.0;
  point_list.pose.orientation.y = 0.0;
  point_list.pose.orientation.z = 0.0;
  point_list.pose.orientation.w = 1.0;
  point_list.scale.x            = 0.5;
  point_list.scale.y            = 0.5;
  point_list.scale.z            = 0.5;
  point_list.color.a            = 1.0;
  point_list.color.r            = 1.0;
  point_list.color.g            = 0.0;
  point_list.color.b            = 0.0;

  for (uint i = 0; i < safety_area_points.rows(); i++) {

    geometry_msgs::Point point;
    point.x = safety_area_points(i, 0);
    point.y = safety_area_points(i, 1);
    point.z = problem.height;

    line_list.points.push_back(point);
    point_list.points.push_back(point);
  }

  geometry_msgs::Point point;
  point.x = safety_area_points(0, 0);
  point.y = safety_area_points(0, 1);
  point.z = problem.height;

  line_list.points.push_back(point);
  point_list.points.push_back(point);

  msg_out.markers.push_back(line_list);
  msg_out.markers.push_back(point_list);

  publisher_rviz.publish(msg_out);
}

//}

/* distanceTimer() //{ */

void MtspStateMachine::distanceTimer([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized) {
    return;
  }

  if (current_state == FOLLOWING_STATE) {

    std::scoped_lock lock(mutex_problem, mutex_odometry_1, mutex_odometry_2, mutex_point_distances);

    for (uint i = 0; i < problem.points.size(); i++) {

      if (dist2d(problem.points[i].x, problem.points[i].y, odometry_1.pose.pose.position.x, odometry_1.pose.pose.position.y) < problem.neighborhood_radius ||
          dist2d(problem.points[i].x, problem.points[i].y, odometry_2.pose.pose.position.x, odometry_2.pose.pose.position.y) < problem.neighborhood_radius) {
        point_distances[i] = true;
      }
    }
  }
}

//}

// | --------------------- other routines --------------------- |

/* printFinalScore() //{ */

void MtspStateMachine::printFinalScore(void) {

  ros::Duration flight_time       = ros::Time::now() - start_time;
  double        collision_penalty = 0;
  double        target_penalty    = 0;

  ROS_INFO("[MtspStateMachine]: ");
  ROS_INFO("[MtspStateMachine]: Flight time: %.2f s", flight_time.toSec());

  if (collision_avoidance_triggered) {
    collision_penalty = COLLISION_PENALTY;
    ROS_WARN("[MtspStateMachine]: Potential collision detected, +%.2f s", collision_penalty);
  }

  ROS_INFO("[MtspStateMachine]: ");
  for (uint i = 0; i < problem.points.size(); i++) {

    if (point_distances[i]) {
      ROS_INFO("[MtspStateMachine]: target #%d = \e[32mREACHED\e[39m", i);
    } else {
      target_penalty += MISSED_TARGET_PENALTY;
      ROS_INFO("[MtspStateMachine]: target #%d = \e[31mMISSED\e[39m +%.2f s", i, MISSED_TARGET_PENALTY);
    }
  }

  ROS_INFO("[MtspStateMachine]: ");
  ROS_INFO("[MtspStateMachine]: TOTAL SCORE = %.2f", flight_time.toSec() + collision_penalty + target_penalty);
  ROS_INFO("[MtspStateMachine]: ");
}

//}

/* resetProgress() //{ */

void MtspStateMachine::resetProgress(void) {

  std::scoped_lock lock(mutex_point_distances);

  for (uint i = 0; i < TARGET_NUM; i++) {
    point_distances[i] = false;
  }

  collision_avoidance_triggered = false;
}

//}

}  // namespace mtsp_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtsp_state_machine::MtspStateMachine, nodelet::Nodelet)
