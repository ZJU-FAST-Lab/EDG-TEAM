#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/CentralPos.h>
#include <traj_utils/CentralState.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      CEN_REPLAN_TRAJ,
      CEN_EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool flag_escape_emergency_;
    double group_dist_;
    int min_num_, max_num_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d end_pt_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    std::vector<Eigen::Vector3d> wps_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, reach_goal_pub_;

    /* record plan state */
    // ros::Publisher plan_state_pub_;

    /* group planning parameters */
    bool flag_central_plan_;
    bool flag_main_compute_;
    bool have_rec_local_map_;
    bool flag_cen_replan_;
    bool have_recv_central_traj_;
    bool de_reach_goal_, cen_reach_goal_;

    ros::Subscriber other_odoms_sub_, central_state_sub_, cen_replan_sub_;
    ros::Publisher central_state_pub_, cen_replan_pub_;
    ros::Time central_start_time_;

    std::vector<int> lastGroup_, groupSet_; //group check
    std::vector<int> recSet_;

    /* multi-agent pathfinding state */
    std::vector<Eigen::Vector3d> ecbs_start_pts_, ecbs_start_vels_, ecbs_start_accs_;
    std::vector<Eigen::Vector3d> ecbs_end_pts_, ecbs_end_vels_;

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* local group planning */
    bool callEcbsReplan();
    void shareLocalInfo();
    bool planEcbsLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const quadrotor_msgs::GoalSetPtr &msg);
    // void planGlobalTrajbyGivenWps();
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp);

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void RecvBroadcastCentralStateCallback(const traj_utils::CentralStateConstPtr &msg);
    void RecvBroadcastCenReplanCallback(const std_msgs::UInt8MultiArrayConstPtr &msg);
    void otherOdomsCallback(const traj_utils::CentralPosPtr &msg);
    void groupCheck(std::vector<int> &openSet, std::vector<int> &closeSet, std::vector<Eigen::Vector3d> &other_odoms, std::vector<int>& reach_goal);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);
    void centralTraj2ROSMsg(traj_utils::MINCOTraj &MINCO_msg, poly_traj::Trajectory &traj, int drone_id);
    void groupSet2ROSMsg(std_msgs::UInt8MultiArray &group_msg);
  };

} // namespace ego_planner

#endif