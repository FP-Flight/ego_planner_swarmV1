// # 接收 消息 TODO
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <traj_utils/MultiBsplines.h>
#include <traj_utils/Bspline.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <Eigen/Eigen>
#include <traj_utils/planning_visualization.h>

// ros::Publisher drone0_trajs_pub_, drone1_trajs_pub_, drone2_trajs_pub_, drone3_trajs_pub_;
ros::Publisher drone_odom_pub_;

ros::Subscriber one_traj_sub_;
ros::Subscriber other_odoms_sub_;
ego_planner::PlanningVisualization::Ptr visualization_;
int drone_id_;

void multitraj_sub_tcp_cb(const traj_utils::MultiBsplinesPtr &msg)
{

}

void odom_sub_cb(const nav_msgs::OdometryPtr &msg)
{
    std::string drone_id = msg->child_frame_id;
    if(drone_id == std::string("drone_") + std::to_string(drone_id_))
    {
        drone_odom_pub_.publish(msg);
    }

}

void one_traj_sub_cb(const traj_utils::BsplinePtr &msg)
{
  ROS_INFO("recv traj");
  Eigen::MatrixXd ctrl_pts = Eigen::MatrixXd::Zero(3, msg->pos_pts.size());
  for (int i = 0; i < msg->pos_pts.size(); ++i)
  {   
      ctrl_pts(0,i) = msg->pos_pts.at(i).x;
      ctrl_pts(1,i) = msg->pos_pts.at(i).y;
      ctrl_pts(2,i) = msg->pos_pts.at(i).z;
      }
  visualization_->displayOptimalList(ctrl_pts, 0);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosmsg_tcp_bridge");
  ros::NodeHandle nh("~");

  visualization_.reset(new ego_planner::PlanningVisualization(nh));

  nh.param("drone_id", drone_id_, -1);
  // bsplines_msg_.reset(new traj_utils::MultiBsplines);
  // odom_msg_.reset(new nav_msgs::Odometry);
  // stop_msg_.reset(new std_msgs::Empty);
  // bspline_msg_.reset(new traj_utils::Bspline);

  other_odoms_sub_ = nh.subscribe("/others_odom", 10, odom_sub_cb, ros::TransportHints().tcpNoDelay());
  drone_odom_pub_ = nh.advertise<nav_msgs::Odometry>("drone_odom_topic", 10);

  one_traj_sub_ = nh.subscribe("/broadcast_bspline2", 100, one_traj_sub_cb, ros::TransportHints().tcpNoDelay());

  ros::Duration(0.1).sleep();
  std::cout << "[swarm_terminal] start running" << std::endl;
  ros::spin();
  return 0;
}
