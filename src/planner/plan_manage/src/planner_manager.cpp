// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    nh.param("manager/drone_radius", ep_.drone_radius_, 0.15);
    nh.param("manager/polyTraj_piece_time", ep_.polyTraj_piece_time, 10.0);
    nh.param("manager/ecbs_w", ep_.ecbs_w, 1.3);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if ( t_to_lc_end < 0 )
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::ecbsReplan(
      const std::vector<Eigen::Vector3d> &start_pts, const std::vector<Eigen::Vector3d> &start_vels,
      const std::vector<Eigen::Vector3d> &start_accs, const std::vector<Eigen::Vector3d> &local_target_pts,
      const std::vector<Eigen::Vector3d> &local_target_vels, const std::vector<int> &closeSet, ros::Time &cen_start_time)
  {

    ep_.central_num_ = closeSet.size();
    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;
    std::vector<poly_traj::MinJerkOpt> initMJOs;
    initMJOs.resize(ep_.central_num_);
    if (!ecbsInitState(start_pts, start_vels, start_accs, local_target_pts, local_target_vels,
                          ts, initMJOs))
    {
      ROS_ERROR("ECBS Init trajectory failed!");
      return false;
    }

    std::vector<Eigen::MatrixXd> cstr_ptss;
    cstr_ptss.resize(ep_.central_num_);
    ploy_traj_opt_->resizeTrajPara(ep_.central_num_);

    for(int i = 0; i < ep_.central_num_; i++){
      cstr_ptss[i] = (initMJOs[i].getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_()));
      vector<std::pair<int, int>> segments;
      if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(ploy_traj_opt_->cpss_[i], segments, initMJOs[i], true) == PolyTrajOptimizer::CHK_RET::ERR)
      {
        ROS_ERROR("central init trajs CheckAndSet constraint failed!");
        return false;
      }
    }

    visualization_->displayCentralInitPathList(cstr_ptss, 0.2);

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;

    std::vector<poly_traj::Trajectory> initTrajs;
    std::vector<Eigen::Matrix<double, 3, 3>> headStates, tailStates;
    std::vector<Eigen::MatrixXd> innerPtss;
    std::vector<Eigen::VectorXd> initTs;
    initTrajs.resize(ep_.central_num_);
    headStates.resize(ep_.central_num_);
    tailStates.resize(ep_.central_num_);
    innerPtss.resize(ep_.central_num_);
    initTs.resize(ep_.central_num_);

    for(int i = 0; i < ep_.central_num_; i ++){
      initTrajs[i] = initMJOs[i].getTraj();
      int PN = initTrajs[i].getPieceNum();
      Eigen::MatrixXd all_pos = initTrajs[i].getPositions();

      innerPtss[i] = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTrajs[i].getJuncPos(0), initTrajs[i].getJuncVel(0), initTrajs[i].getJuncAcc(0);
      tailState << initTrajs[i].getJuncPos(PN), initTrajs[i].getJuncVel(PN), initTrajs[i].getJuncAcc(PN);

      headStates[i] = headState;
      tailStates[i] = tailState;
      initTs[i] = initTrajs[i].getDurations();

      traj_.swarm_traj[closeSet[i]].drone_id = closeSet[i];
      traj_.swarm_traj[closeSet[i]].start_time = cen_start_time.toSec();
    }

    double final_cost;
    // Group trajectories joint optimization
    flag_success = ploy_traj_opt_->ecbsOptTraj(headStates, tailStates,
                                                      innerPtss, initTs,
                                                      cstr_ptss, final_cost, closeSet);
    
    visualization_->displayCentralList(cstr_ptss, 0);

    return true;
  }

  bool EGOPlannerManager::ecbsInitState(
    const std::vector<Eigen::Vector3d> &start_pts, const std::vector<Eigen::Vector3d> &start_vels,
    const std::vector<Eigen::Vector3d> &start_accs, const std::vector<Eigen::Vector3d> &local_target_pts,
    const std::vector<Eigen::Vector3d> &local_target_vels, const double &ts,
    std::vector<poly_traj::MinJerkOpt> &initMJOs)
  {
    // search multiple collision-free paths
    std::vector<PlanResult<State, Action, double>> solution;
    if(!ecbsSearch(solution, start_pts, local_target_pts)){
      ROS_ERROR("ECBS search failed!");
      return false;
    }

    // construct multiple initial trajectories
    for(int i = 0; i < ep_.central_num_; i ++) {
      auto nodes = solution[i].states;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      headState << start_pts[i], start_vels[i], start_accs[i];
      tailState << local_target_pts[i], local_target_vels[i], Eigen::Vector3d::Zero();

      int t_piece = ep_.polyTraj_piece_time;
      piece_nums = round((nodes.back().first.time - nodes.front().first.time) / t_piece);

      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);

      int id = 0;
      Eigen::Vector3d inn_pos;
      for (int j = 0; j < piece_nums - 1; j++) {
        grid_map_->indexToPos(Eigen::Vector3i(nodes[t_piece * j].first.x, 
            nodes[t_piece * j].first.y, 
            nodes[t_piece * j].first.z), inn_pos);
        innerPs.col(id++) = inn_pos;
      }

      piece_dur_vec(piece_dur_vec.size()-1) = (nodes.back().first.time - nodes[(piece_nums - 1) * t_piece].first.time) * ts / t_piece;

      initMJOs[i].reset(headState, tailState, piece_nums);
      initMJOs[i].generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  bool EGOPlannerManager::ecbsSearch(std::vector<PlanResult<State, Action, double>> &solution,
  const std::vector<Eigen::Vector3d> &start_pts,
  const std::vector<Eigen::Vector3d> &local_target_pts)
  {
    // set start and goal position
    std::vector<Location> startLocations, goalLocations;
    // Note: vector element(State) is struct, do not use resize,must use push_bcak()
    std::vector<State> startStates;
    Eigen::Vector3i start_idx, goal_idx;
    for(int i = 0; i < ep_.central_num_; i++)
    {
      grid_map_->posToIndex(start_pts[i], start_idx);
      startLocations.push_back(Location(start_idx(0), start_idx(1), start_idx(2)));
      startStates.push_back(State(0, start_idx(0), start_idx(1), start_idx(2)));

      grid_map_->posToIndex(local_target_pts[i], goal_idx);
      goalLocations.push_back(Location(goal_idx(0), goal_idx(1), goal_idx(2)));
    }

    // small drone d = 0.11; big drone d = 0.25
    std::vector<double> quad_size(ep_.central_num_, ep_.drone_radius_);

    // realize ECBS in 3D gridmap
    Environment mapf(startLocations, goalLocations, quad_size,
                    grid_map_->mp_.resolution_, grid_map_);
    ECBS<State, Action, double, Conflict, Constraints, Environment> ecbs(mapf, ep_.ecbs_w);
    bool success = ecbs.search(startStates, solution, 1);

    if (!success) {
        ROS_ERROR("ecbsSearch  failed!");
        return false;
    }

    return true;
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal, const double &world_time)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret)
      traj_.setLocalTraj(traj, pts_to_check, world_time);

    return ret;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {

    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    ploy_traj_opt_->setIfTouchGoal(touch_goal);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      // continous_failures_count_++;
      // return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(ploy_traj_opt_->cps_, segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;

    if (pp_.use_distinctive_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(),
                                               cstr_pts, final_cost))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(),
                                                        cstr_pts, final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();

      t_opt = ros::Time::now() - t_start;
    }

    // // save and display planned results
    cout << "plan_success=" << flag_success << endl;
    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).toSec()
         << "\033[0m,init:" << t_init.toSec()
         << ",optimize:" << t_opt.toSec()
         << ",avg_time=" << sum_time / count_success << endl;

    setLocalTrajFromOpt(best_MJO, touch_goal, ros::Time::now().toSec());
    visualization_->displayOptimalList(cstr_pts, 0);

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false, ros::Time::now().toSec());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner
