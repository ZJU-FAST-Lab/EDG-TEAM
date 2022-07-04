
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;

    flag_central_plan_ = false;
    flag_main_compute_ = false; 
    have_rec_local_map_ = false;
    flag_cen_replan_ = false;
    have_recv_central_traj_ = false;
    // de_reach_goal_ = false;
    // cen_reach_goal_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/group_dist", group_dist_, 4.0); // group_dist_ - safe distance
    nh.param("fsm/min_num", min_num_, 3);
    nh.param("fsm/max_num", max_num_, 8);
    
    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* record plan state */
    // plan_state_pub_ = nh.advertise<std_msgs::Int8>("planning/plan_state", 0);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    /* Use share local information(start, local target, and local map) in group planning */
    central_state_pub_ = nh.advertise<traj_utils::CentralState>("/local_target_from_planner", 10);
    central_state_sub_ = nh.subscribe("/local_target_to_planner", 10, 
                                                  &EGOReplanFSM::RecvBroadcastCentralStateCallback, 
                                                  this, 
                                                  ros::TransportHints().tcpNoDelay());

    /* Call replan in group planning */
    cen_replan_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("/cen_replan_from_planner", 10);
    cen_replan_sub_ = nh.subscribe("/cen_replan_to_planner", 10, 
                                                  &EGOReplanFSM::RecvBroadcastCenReplanCallback, 
                                                  this, 
                                                  ros::TransportHints().tcpNoDelay());

    other_odoms_sub_ = nh.subscribe("/all_odoms", 5, &EGOReplanFSM::otherOdomsCallback, this,
                                                      ros::TransportHints().tcpNoDelay());

    reach_goal_pub_ = nh.advertise<std_msgs::Int32>("/reach_goal_flag", 10);

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);


    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      // ROS_INFO("Waiting for trigger from RC");

      // while (ros::ok() && (!have_odom_ || !have_trigger_))
      // {
      //   ros::spinOnce();
      //   ros::Duration(0.001).sleep();
      // }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    // record plan state
    // std_msgs::Int8 state_msg;
    // state_msg.data = exec_state_;
    // plan_state_pub_.publish(state_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_WARN("Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (planFromLocalTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);

      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wpt_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wpt_id_++;
        std::cout << "wpt_id_ : " << wpt_id_ << std::endl;
        planNextWaypoint(wps_[wpt_id_]);
        ROS_WARN("DECENTRAL PLAN WAYPOINTS!");
      }
      else if (touch_the_goal && ((pos - wps_[waypoint_num_ - 1]).norm() < 1.5))
      {
        if(!de_reach_goal_)
        {
          std_msgs::Int32 reach_msg;
          reach_msg.data = planner_manager_->pp_.drone_id;
          reach_goal_pub_.publish(reach_msg);
          de_reach_goal_ = true;

          ROS_WARN("DECENTRAL REACH GOAL");
        }


        if(t_cur > info->duration - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }

      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        if (planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
        {
          have_target_ = false;
          have_trigger_ = false;
          ROS_ERROR("The goal is in obstacles, finish the planning.");
          callEmergencyStop(odom_pos_);

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else
        {
          // pass;
        }
      }
      else if (t_cur > replan_thresh_ ||
               (!touch_the_goal && planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case CEN_REPLAN_TRAJ:
    {
      // Core drone
      if(flag_main_compute_ && have_rec_local_map_) 
      {
        if(planEcbsLocalTraj(1))
        {
          changeFSMExecState(CEN_EXEC_TRAJ, "FSM");
          have_rec_local_map_ = false;
        }
        else
        {
          if(planner_manager_->pp_.drone_id == groupSet_[0])
          {
            ROS_WARN("PLAN ECBS FAILED!");
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
      }

      break;
    }

    case CEN_EXEC_TRAJ:
    {
      // the agents in a group/team;
      if(have_recv_central_traj_ && flag_central_plan_)
      {
        LocalTrajData *info = &planner_manager_->traj_.local_traj;
        double t_cur = ros::Time::now().toSec() - info->start_time;
        Eigen::Vector3d pos;
        bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);
        if(t_cur >= 0)
        {
          t_cur = min(info->duration, t_cur);
          pos = info->traj.getPos(t_cur);

          if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
              (wpt_id_ < waypoint_num_ - 1) &&
              (end_pt_ - pos).norm() < no_replan_thresh_)
          {
            wpt_id_++;
            std::cout << "wpt_id_ : " << wpt_id_ << std::endl;
            planNextWaypoint(wps_[wpt_id_]);
            ROS_WARN("CENTRAL PLAN WAYPOINTS!");
          }
          else if (touch_the_goal && ((pos - wps_[waypoint_num_ - 1]).norm() < 1.5)) // local target close to the global target
          {
            if(!cen_reach_goal_)
            {
              std_msgs::Int32 reach_msg;
              reach_msg.data = planner_manager_->pp_.drone_id;
              reach_goal_pub_.publish(reach_msg);
              cen_reach_goal_ = true;

              ROS_WARN("CENTRAL REACH GOAL");
            }

            if(t_cur > info->duration - 1e-2)
            {
              have_target_ = false;
              have_trigger_ = false;

              /* The navigation task completed */
              changeFSMExecState(WAIT_TARGET, "FSM");
              ROS_WARN("CENTRAL REACH TARGET!");
              goto force_return;
            }

          }
          else if(t_cur > replan_thresh_)
          {
            if(planner_manager_->pp_.drone_id == groupSet_[0])
            {
              std_msgs::UInt8MultiArray group_msg;
              groupSet2ROSMsg(group_msg);
              cen_replan_pub_.publish(group_msg);
            }
            have_recv_central_traj_ = false;
          }
        }
        else
        {
          ROS_WARN_ONCE("received central traj, but not arrive at the start time!");
          pos = odom_pos_;

          if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
              (wpt_id_ < waypoint_num_ - 1) &&
              (end_pt_ - pos).norm() < no_replan_thresh_)
          {
            wpt_id_++;
            planNextWaypoint(wps_[wpt_id_]);
            ROS_WARN("CENTRAL PLAN WAYPOINTS[ODOM]!");
          }
        }
      }
      else if(flag_central_plan_)
      {
        ROS_WARN_ONCE("waiting to receive central traj! ");
      }
      else if(!flag_central_plan_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[9] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "CEN_REPLAN_TRAJ", "CEN_EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]"
         << ", Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[9] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "CEN_REPLAN_TRAJ", "CEN_EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    // static int last_printed_state = -1, dot_nums = 0;

    // if (exec_state_ != last_printed_state)
    //   dot_nums = 0;
    // else
    //   dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", Drone:" << planner_manager_->pp_.drone_id;

    // last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_ || !have_target_ || !have_trigger_ || (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_))
    {
      cout << ". Waiting for ";
    }
    if (!have_odom_)
    {
      cout << "odom,";
    }
    if (!have_target_)
    {
      cout << "target,";
    }
    if (!have_trigger_)
    {
      cout << "trigger,";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << "prev traj,";
    }

    cout << endl;

    // cout << string(dot_nums, '.');

    // fflush(stdout);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    // bool close_to_the_end_of_safe_segment = (pts_chk.back().back().first - t_cur) < emergency_time_;
    // // bool close_to_goal = (info->traj.getPos(info->duration) - end_pt_).norm() < 1e-5;
    // if (close_to_the_end_of_safe_segment)
    // {
    //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   return;

    //   // if (!close_to_goal)
    //   // {
    //   //   // ROS_INFO("current position is close to the safe segment end.");
    //   //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //   return;
    //   // }
    //   // else
    //   // {
    //   //   double t_step = map->getResolution() / planner_manager_->pp_.max_vel_;
    //   //   for (double t = pts_chk.back().back().first; t < info->duration; t += t_step)
    //   //   {
    //   //     if (map->getInflateOccupancy(info->traj.getPos(t)))
    //   //     {
    //   //       if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_)
    //   //       {
    //   //         ROS_ERROR("Dense obstacles close to the goal, stop planning.");
    //   //         callEmergencyStop(odom_pos_);
    //   //         have_target_ = false;
    //   //         changeFSMExecState(WAIT_TARGET, "SAFETY");
    //   //         return;
    //   //       }
    //   //       else
    //   //       {
    //   //         changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //         return;
    //   //       }
    //   //     }
    //   //   }
    //   // }
    // }

    /* ---------- check trajectory ---------- */
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    double t_temp = t_cur; // t_temp will be changed in the next function!
    int i_start = info->traj.locatePieceIdx(t_temp);

    if (i_start >= pts_chk.size())
    {
      // ROS_ERROR("i_start >= pts_chk.size()");
      return;
    }
    size_t j_start = 0;
    // cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
    for (; i_start < pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    // Eigen::Vector3d last_pt = pts_chk[0][0].second;
    // for (size_t i = 0; i < pts_chk.size(); ++i)
    // {
    //   cout << "--------------------" << endl;
    //   for (size_t j = 0; j < pts_chk[i].size(); ++j)
    //   {
    //     cout << pts_chk[i][j].first << " @ " << pts_chk[i][j].second.transpose() << " @ " << (pts_chk[i][j].second - last_pt).transpose() << " @ " << map->getInflateOccupancy(pts_chk[i][j].second) << endl;
    //     last_pt = pts_chk[i][j].second;
    //   }
    // }

    // cout << "pts_chk[i_start][j_start].first - t_cur = " << pts_chk[i_start][j_start].first - t_cur << endl;
    // cout << "devi = " << (pts_chk[i_start][j_start].second - info->traj.getPos(t_cur)).transpose() << endl;

    // cout << "pts_chk.size()=" << pts_chk.size() << " i_start=" << i_start << endl;
    // Eigen::Vector3d p_last = pts_chk[i_start][j_start].second;
    const bool touch_the_end = ((local_target_pt_ - end_pt_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;
        // if ( (p - p_last).cwiseAbs().maxCoeff() > planner_manager_->grid_map_->getResolution() * 1.05 )
        // {
        //   ROS_ERROR("BBBBBBBBBBBBBBBBBBBBBBBBBBB");
        //   cout << "p=" << p.transpose() << " p_last=" << p_last.transpose() << " dist=" << (p - p_last).cwiseAbs().maxCoeff() << endl;
        // }
        // p_last = p;

        // cout << "t=" << t << " @ "
        //      << "p=" << p.transpose() << endl;
        // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        // if (t_cur < t_2_3 && t >= t_2_3)
        //   break;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        // cout << "p=" << p.transpose() << endl;

        // if (occ)
        // {
        //   ROS_WARN("AAAAAAAAAAAAAAAAAAA");
        //   cout << "pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
        //   cout << "i=" << i << " j=" << j << " i_start=" << i_start << " j_start=" << j_start << endl;
        //   cout << "pts_chk.size()=" << pts_chk.size() << endl;
        //   cout << "t=" << t << endl;
        //   cout << "from t=" << info->traj.getPos(t).transpose() << endl;
        //   cout << "from rec=" << p.transpose() << endl;
        // }

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          // double t_X = t - (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();

            if (dist < CLEARANCE)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          // central drone collision
          if(flag_central_plan_)
          {
            ROS_WARN("[central drone collision] send flag_cen_replan.");
            std_msgs::UInt8MultiArray group_msg;
            groupSet2ROSMsg(group_msg);
            cen_replan_pub_.publish(group_msg);
          }

          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }


          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {
      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  bool EGOReplanFSM::callEcbsReplan()
  {
    // std::cout << "===============callEcbsReplan================" << std::endl;
    bool plan_success = planner_manager_->ecbsReplan(
        ecbs_start_pts_, ecbs_start_vels_, ecbs_start_accs_,
        ecbs_end_pts_, ecbs_end_vels_, groupSet_, central_start_time_);

    if (plan_success)
    {
      // send joint optimization trajectories to each one.
      for(int i = 0; i < (int)groupSet_.size(); i++){
        // std::cout << "groupSet[" << i << "]: " << groupSet_[i] << std::endl;// [Debug] groupSet OK
        poly_traj::Trajectory traj = planner_manager_->ploy_traj_opt_->jerkOpts_[i].getTraj();
        traj_utils::MINCOTraj MINCO_msg;
        centralTraj2ROSMsg(MINCO_msg, traj, groupSet_[i]);
        broadcast_ploytraj_pub_.publish(MINCO_msg);
      }
    }

    return plan_success;
  }

  void EGOReplanFSM::shareLocalInfo()
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    // int id = planner_manager_->pp_.drone_id;

    double t_to_start = central_start_time_.toSec() - info->start_time;
    if(t_to_start < 0)
    {
      ROS_ERROR("local traj error!");
    }
    start_pt_ = info->traj.getPos(t_to_start);
    start_vel_ = info->traj.getVel(t_to_start);
    start_acc_ = info->traj.getAcc(t_to_start);

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    // ROS_WARN("share drone[%d] local info.", planner_manager_->pp_.drone_id);

    traj_utils::CentralState msg;
    msg.drone_id = planner_manager_->pp_.drone_id;
    msg.start_p[0] = start_pt_(0);
    msg.start_p[1] = start_pt_(1);
    msg.start_p[2] = start_pt_(2);

    msg.start_v[0] = start_vel_(0);
    msg.start_v[1] = start_vel_(1);
    msg.start_v[2] = start_vel_(2);

    msg.start_a[0] = start_acc_(0);
    msg.start_a[1] = start_acc_(1);
    msg.start_a[2] = start_acc_(2);

    msg.target_p[0] = local_target_pt_(0);
    msg.target_p[1] = local_target_pt_(1);
    msg.target_p[2] = local_target_pt_(2);

    msg.target_v[0] = local_target_vel_(0);
    msg.target_v[1] = local_target_vel_(1);
    msg.target_v[2] = local_target_vel_(2);

    // core compute drone groupSet_[0] don't send local map.
    if(planner_manager_->pp_.drone_id != groupSet_[0])
    {
      msg.local_bound_min[0] = planner_manager_->grid_map_->md_.local_bound_min_(0);
      msg.local_bound_min[1] = planner_manager_->grid_map_->md_.local_bound_min_(1);
      msg.local_bound_min[2] = planner_manager_->grid_map_->md_.local_bound_min_(2);
      msg.local_bound_max[0] = planner_manager_->grid_map_->md_.local_bound_max_(0);
      msg.local_bound_max[1] = planner_manager_->grid_map_->md_.local_bound_max_(1);
      msg.local_bound_max[2] = planner_manager_->grid_map_->md_.local_bound_max_(2);
      
      int size = planner_manager_->grid_map_->md_.local_buffer_.size();
      msg.local_buffer.resize(size);
      for(int i = 0; i < size; i++)
        msg.local_buffer[i] = planner_manager_->grid_map_->md_.local_buffer_[i];

      // std::cout << "local_buffer_size: " << size << std::endl;
    }

    if((planner_manager_->pp_.drone_id == groupSet_[0]) || (msg.local_buffer.size() != 0))
      central_state_pub_.publish(msg);

  }

  bool EGOReplanFSM::planEcbsLocalTraj(const int trial_times) 
  {
    bool success;
    for (int i = 0; i < trial_times; i++){
      success = callEcbsReplan();
      if(success)
        break;
    }
    if(!success){
      return false;
    }
    return true;
  }

  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    // success = planner_manager_->planGlobalTrajWaypoints(
    //     odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
    //     one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while ((exec_state_ != EXEC_TRAJ) && (exec_state_ != CEN_EXEC_TRAJ))
        {
          // std::cout << "state: " << exec_state_ << std::endl;
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id || msg->goal[2] < -0.1)
      return;

    ROS_INFO("Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);

    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp))
    {
      have_trigger_ = true;
    }
  }

  // void EGOReplanFSM::planGlobalTrajbyGivenWps()
  // {
  //   std::vector<Eigen::Vector3d> wps(waypoint_num_);
  //   for (int i = 0; i < waypoint_num_; i++)
  //   {
  //     wps[i](0) = waypoints_[i][0];
  //     wps[i](1) = waypoints_[i][1];
  //     wps[i](2) = waypoints_[i][2];

  //     end_pt_ = wps.back();
  //   }
  //   bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero(), wps,
  //                                                            Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero());

  //   for (size_t i = 0; i < (size_t)waypoint_num_; i++)
  //   {
  //     visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
  //     ros::Duration(0.001).sleep();
  //   }

  //   if (success)
  //   {

  //     /*** display ***/
  //     constexpr double step_size_t = 0.1;
  //     int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
  //     std::vector<Eigen::Vector3d> gloabl_traj(i_end);
  //     for (int i = 0; i < i_end; i++)
  //     {
  //       gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
  //     }

  //     have_target_ = true;
  //     have_new_target_ = true;

  //     // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
  //     ros::Duration(0.001).sleep();
  //     visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
  //     ros::Duration(0.001).sleep();
  //   }
  //   else
  //   {
  //     ROS_ERROR("Unable to generate global trajectory!");
  //   }
  // }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_]);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    const size_t recv_id = (size_t)msg->drone_id;
    // if ((int)recv_id == planner_manager_->pp_.drone_id)
    //   return;

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    poly_traj::Trajectory trajectory = MJO.getTraj();
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    // send trajectory to trajServer
    if(msg->is_central_traj && (int)recv_id == planner_manager_->pp_.drone_id)
    {
      planner_manager_->setLocalTrajFromOpt(MJO, false, msg->start_time.toSec());

      traj_utils::PolyTraj poly_msg;
      poly_msg.drone_id = recv_id;
      // poly_msg.traj_id = msg->traj_id;
      poly_msg.start_time = ros::Time(msg->start_time);
      poly_msg.order = 5; // todo, only support order = 5 now.

      Eigen::VectorXd durs = trajectory.getDurations();
      int piece_num = trajectory.getPieceNum();
      poly_msg.duration.resize(piece_num);
      poly_msg.coef_x.resize(6 * piece_num);
      poly_msg.coef_y.resize(6 * piece_num);
      poly_msg.coef_z.resize(6 * piece_num);
      for (int i = 0; i < piece_num; ++i)
      {
        poly_msg.duration[i] = durs(i);

        poly_traj::CoefficientMat cMat = trajectory.getPiece(i).getCoeffMat();
        int i6 = i * 6;
        for (int j = 0; j < 6; j++)
        {
          poly_msg.coef_x[i6 + j] = cMat(0, j);
          poly_msg.coef_y[i6 + j] = cMat(1, j);
          poly_msg.coef_z[i6 + j] = cMat(2, j);
        }
      }

      have_recv_central_traj_ = true;
      changeFSMExecState(CEN_EXEC_TRAJ, "CENTRAL_NO_CORE");
      // ROS_WARN("------update have_recv_central_traj_------: %d.", have_recv_central_traj_);
      poly_traj_pub_.publish(poly_msg);
      // ros::Duration(0.001).sleep();
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            break;
          }

          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  void EGOReplanFSM::RecvBroadcastCentralStateCallback(const traj_utils::CentralStateConstPtr &msg)
  {
    if(groupSet_.empty())
      return;

    if(planner_manager_->pp_.drone_id != groupSet_[0])
      return;
    
    int recv_id = msg->drone_id;
    // drone not in the same group, but in the same time
    if(std::find(groupSet_.begin(), groupSet_.end(), recv_id) == groupSet_.end())
      return;

    // avoid to receive empty map; if access empty RAM -> Game Over! ^^
    if((msg->local_buffer.size() == 0) && (recv_id != groupSet_[0]))
      return;

    if(recSet_.size() != groupSet_.size())
    {
      ecbs_start_pts_.resize(groupSet_.size());
      ecbs_start_vels_.resize(groupSet_.size());
      ecbs_start_accs_.resize(groupSet_.size());
      ecbs_end_pts_.resize(groupSet_.size());
      ecbs_end_vels_.resize(groupSet_.size());
      recSet_.resize(groupSet_.size());
    }

    std::vector<int>::iterator it = std::find(groupSet_.begin(), groupSet_.end(), recv_id);
    int i = std::distance(groupSet_.begin(), it);

    recSet_[i] = recv_id;

    ecbs_start_pts_[i](0) = msg->start_p[0];
    ecbs_start_pts_[i](1) = msg->start_p[1];
    ecbs_start_pts_[i](2) = msg->start_p[2];

    ecbs_start_vels_[i](0) = msg->start_v[0];
    ecbs_start_vels_[i](1) = msg->start_v[1];
    ecbs_start_vels_[i](2) = msg->start_v[2];

    ecbs_start_accs_[i](0) = msg->start_a[0];
    ecbs_start_accs_[i](1) = msg->start_a[1];
    ecbs_start_accs_[i](2) = msg->start_a[2];
    
    ecbs_end_pts_[i](0) = msg->target_p[0];
    ecbs_end_pts_[i](1) = msg->target_p[1];
    ecbs_end_pts_[i](2) = msg->target_p[2];

    ecbs_end_vels_[i](0) = msg->target_v[0];
    ecbs_end_vels_[i](1) = msg->target_v[1];
    ecbs_end_vels_[i](2) = msg->target_v[2];

    if(recv_id != planner_manager_->pp_.drone_id)
    {
      Eigen::Vector3i local_min, local_max;
      local_min(0) = msg->local_bound_min[0];
      local_min(1) = msg->local_bound_min[1];
      local_min(2) = msg->local_bound_min[2];

      local_max(0) = msg->local_bound_max[0];
      local_max(1) = msg->local_bound_max[1];
      local_max(2) = msg->local_bound_max[2];

      planner_manager_->grid_map_->mp_.local_map_voxel_num_ = local_max - local_min + Eigen::Vector3i::Ones();

      int local_idx;
      for (int x = local_min(0); x <= local_max(0); ++x)
        for (int y = local_min(1); y <= local_max(1); ++y)
          for (int z = local_min(2); z <= local_max(2); ++z)
          {
            local_idx = planner_manager_->grid_map_->toLocAddress(x - local_min(0), y - local_min(1), z - local_min(2));

            if((msg->local_buffer[local_idx / 8] & (1 << (local_idx % 8))) == (1 << (local_idx % 8)))
              planner_manager_->grid_map_->md_.occupancy_buffer_inflate_[planner_manager_->grid_map_->toAddress(x, y, z)] = 1;
          }
    }

    if(groupSet_ == recSet_)
    {
      recSet_.clear();
      have_rec_local_map_ = true;
      ROS_WARN("------have_rec_local_map_------: %d.", have_rec_local_map_);
      changeFSMExecState(CEN_REPLAN_TRAJ, "CENGROUP_CORE");// core drone
    }

  }

  void EGOReplanFSM::RecvBroadcastCenReplanCallback(const std_msgs::UInt8MultiArrayConstPtr &msg)
  {
    std::vector<int> replanSet;
    replanSet.resize(msg->data.size());
    for(int i = 0; i < (int)msg->data.size(); i++)
      replanSet[i] = msg->data[i];

    if(replanSet == groupSet_)
    {
      flag_cen_replan_ = true;
      ROS_WARN("------flag_cen_replan_------: %d.", flag_cen_replan_);
    }
  }

  void EGOReplanFSM::otherOdomsCallback(const traj_utils::CentralPosPtr &msg)
  {
    int group_num = msg->drone_nums;
    // std::cout << "group_num: " << group_num << std::endl;
    std::vector<Eigen::Vector3d> other_odoms;
    std::vector<int> reach_goal;
    other_odoms.resize(group_num);
    reach_goal.resize(group_num);
    for(int i = 0; i < group_num; i++)
    {
      other_odoms[i](0) = msg->pos_x[i];
      other_odoms[i](1) = msg->pos_y[i];
      other_odoms[i](2) = msg->pos_z[i];
      reach_goal[i] = msg->reach_goal[i];
    }

    // Group Check Module
    std::vector<int> openSet, closeSet;
    groupCheck(openSet, closeSet, other_odoms, reach_goal);

    groupSet_.assign(closeSet.begin(), closeSet.end());
       // group planning is triggered --- 1. form the new group; 2. reach replan time-flag_cen_replan_ = true;
    if(((lastGroup_ != closeSet) || flag_cen_replan_) && (closeSet.size() >= min_num_)){
      flag_cen_replan_ = false;
      central_start_time_ = msg->cen_start_time;
      // std::cout << "central_start_time_: " << central_start_time_ << std::endl;

      // share start, local target, and local map
      shareLocalInfo();
    }

    // lastGroup_ = closeSet;
    lastGroup_.assign(closeSet.begin(), closeSet.end());

  }

  void EGOReplanFSM::groupCheck(std::vector<int> &openSet, std::vector<int> &closeSet, std::vector<Eigen::Vector3d> &other_odoms,
                                  std::vector<int>& reach_goal)
  {
    // ROS_WARN("[Group Check]===========================");
    
    openSet.clear();
    closeSet.clear();
    // the node(=1) reaching goal cannot add openSet; the node have stopped
    if(reach_goal[planner_manager_->pp_.drone_id] == 0)
    {
      openSet.push_back(planner_manager_->pp_.drone_id);
    }
    int current;
    while(!openSet.empty()){
      current = openSet.front();

      openSet.erase(openSet.begin());
      closeSet.push_back(current);
      for(int i = 0; i < (int)other_odoms.size(); i++){
        if(reach_goal[i] == 1)
          continue;
        
        if((std::find(openSet.begin(), openSet.end(), i) != openSet.end()) || 
          (std::find(closeSet.begin(), closeSet.end(), i) != closeSet.end()))
          continue;

        if((other_odoms[current] - other_odoms[i]).norm() < group_dist_){
          openSet.push_back(i);
        }

      }
    }

    // select lowest id do central plan;
    if((closeSet.size() >= min_num_))
    {
      flag_central_plan_ = true;
      std::sort(closeSet.begin(), closeSet.end());

      if(closeSet.size() > max_num_)
      {
        closeSet.erase(closeSet.begin() + max_num_, closeSet.end());
      }

      int id = planner_manager_->pp_.drone_id;
      if(std::find(closeSet.begin(), closeSet.end(), id) == closeSet.end())
      {
        flag_central_plan_ = false;
        flag_main_compute_ = false;
        closeSet.clear();
      }
      else
      {
        // for(int i = 0; i < (int)closeSet.size(); i++){
        //   std::cout << "cloSet[" << i << "]: " << closeSet[i] << std::endl;
        // }
        if(id == closeSet[0])
        {
          flag_main_compute_ = true;
        }
        else
        {
          flag_main_compute_ = false;
        }
      }
    }
    else
    {
      flag_central_plan_ = false;
      flag_main_compute_ = false;
      closeSet.clear();
    }

    // std::cout << "planner_manager_->pp_.drone_id: " << planner_manager_->pp_.drone_id << std::endl;
    // std::cout << "flag_central_plan_: " << flag_central_plan_ << std::endl;
    // std::cout << "flag_main_compute_: " << flag_main_compute_ << std::endl;
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.is_central_traj = false;
    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  void EGOReplanFSM::centralTraj2ROSMsg(traj_utils::MINCOTraj &MINCO_msg, poly_traj::Trajectory &traj, int drone_id)
  {

    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();

    MINCO_msg.is_central_traj = true;
    MINCO_msg.drone_id = drone_id;
    // MINCO_msg.traj_id = traj_id;
    MINCO_msg.start_time = central_start_time_;
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = traj.getPos(0); 
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = traj.getPos(traj.getTotalDuration());
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = traj.getVel(traj.getTotalDuration());
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = traj.getAcc(traj.getTotalDuration());
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  void EGOReplanFSM::groupSet2ROSMsg(std_msgs::UInt8MultiArray &group_msg)
  {
    group_msg.data.resize(groupSet_.size());
    for(int i = 0; i < (int)groupSet_.size(); i++)
      group_msg.data[i] = groupSet_[i];
  }
} // namespace ego_planner
