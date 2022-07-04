#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <traj_utils/MINCOTraj.h>
#include <traj_utils/CentralPos.h>
#include <optimizer/poly_traj_utils.hpp>
#include <std_msgs/Int32.h>

using namespace std;

struct Traj_t
{
  poly_traj::Trajectory traj;
  bool valid;
  ros::Time start_time;
  double duration;
  double last_yaw;
};

vector<Traj_t> trajs_;
ros::Subscriber one_traj_sub_;
ros::Publisher other_odoms_pub_;
ros::Subscriber reach_goal_sub_;

vector<int> reach_goal_;

void reach_goal_sub_cb(const std_msgs::Int32Ptr &msg)
{
  // the node reaching goal sets 1;
  int id = msg->data;
  reach_goal_[id] = 1;
}

void one_traj_sub_cb(const traj_utils::MINCOTrajPtr &msg)
{
  // ROS_WARN("sub traj");

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

  const int recv_id = msg->drone_id;

  /* Fill up the buffer */
  if ((int)trajs_.size() <= recv_id)
  {
    for (int i = trajs_.size(); i <= recv_id; i++)
    {
      Traj_t blank;
      blank.valid = false;
      trajs_.push_back(blank);
    }
  }

  /* Store data */;
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

  trajs_[recv_id].traj = MJO.getTraj();
  trajs_[recv_id].start_time = msg->start_time;
  trajs_[recv_id].valid = true;
  trajs_[recv_id].duration = trajs_[recv_id].traj.getTotalDuration();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj2odom");
  ros::NodeHandle nh("~");

  double odom_hz, future_time;
  int drone_num;
  nh.param("odom_hz", odom_hz, 100.0);
  nh.param("future_time", future_time, 0.6);
  nh.param("drone_num", drone_num, 10);
  reach_goal_.resize(drone_num, 0);

  one_traj_sub_ = nh.subscribe("/broadcast_traj_to_planner", 100, one_traj_sub_cb, ros::TransportHints().tcpNoDelay());
  reach_goal_sub_ = nh.subscribe("/reach_goal_flag", 100, reach_goal_sub_cb, ros::TransportHints().tcpNoDelay());
  // other_odoms_pub_ = nh.advertise<nav_msgs::Odometry>("/others_odom", 100);
  other_odoms_pub_ = nh.advertise<traj_utils::CentralPos>("/all_odoms", 1);


  ros::Rate loop_rate(odom_hz);
  while (ros::ok())
  {
    auto t_now = ros::Time::now() + ros::Duration(future_time);

    traj_utils::CentralPos msg;
    msg.drone_nums = trajs_.size();
    msg.cen_start_time = t_now;
    msg.reach_goal.resize(trajs_.size());
    msg.pos_x.resize(trajs_.size());
    msg.pos_y.resize(trajs_.size());
    msg.pos_z.resize(trajs_.size());
    // msg.start_time.resize(trajs_.size());
    for (int id = 0; id < (int)trajs_.size(); ++id)
    {
      if (trajs_[id].valid)
      {
        msg.reach_goal[id] = reach_goal_[id];

        // publish next 50ms position
        double t_to_start = (t_now - trajs_[id].start_time).toSec();
        double t = t_to_start < trajs_[id].duration ? t_to_start : trajs_[id].duration;
        // if(t_to_start > trajs_[id].duration)
        // {
        //   ROS_WARN("t_to_start > trajs_[%d].duration. trajs_[id].duration = %f.", id, trajs_[id].duration);
        // }
        Eigen::Vector3d p = trajs_[id].traj.getPos(t);

        msg.pos_x[id] = p(0);
        msg.pos_y[id] = p(1);
        msg.pos_z[id] = p(2);
        // msg.start_time[id] = t;

      }
    }
    
    if((int)trajs_.size() == drone_num)
    {
      other_odoms_pub_.publish(msg);
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
