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
#include "bspline_opt/uniform_bspline.h"
using ego_planner::UniformBspline;
// ros::Publisher drone0_trajs_pub_, drone1_trajs_pub_, drone2_trajs_pub_, drone3_trajs_pub_;
ros::Publisher drone_odom_pub_;
ros::Publisher exp_pos_marker_pub;

ros::Subscriber one_traj_sub_;
ros::Subscriber other_odoms_sub_;
ego_planner::PlanningVisualization::Ptr visualization_;
int drone_id_;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
static double color_r, color_g, color_b, color_a;
bool receive_traj_ = false;

void multitraj_sub_tcp_cb(const traj_utils::MultiBsplinesPtr &msg)
{
}

void odom_sub_cb(const nav_msgs::OdometryPtr &msg)
{
    std::string drone_id = msg->child_frame_id;
    if (drone_id == std::string("drone_") + std::to_string(drone_id_))
    {
        drone_odom_pub_.publish(msg);
    }
}

void one_traj_sub_cb(const traj_utils::BsplinePtr &msg)
{
    if (msg->drone_id == drone_id_)
    {
          start_time_ = msg->start_time;

        ROS_INFO("recv traj %d", drone_id_);
        Eigen::MatrixXd ctrl_pts = Eigen::MatrixXd::Zero(3, msg->pos_pts.size());
        for (int i = 0; i < msg->pos_pts.size(); ++i)
        {
            ctrl_pts(0, i) = msg->pos_pts.at(i).x;
            ctrl_pts(1, i) = msg->pos_pts.at(i).y;
            ctrl_pts(2, i) = msg->pos_pts.at(i).z;
        }
        visualization_->displayOptimalList(ctrl_pts, 0);

        Eigen::VectorXd knots(msg->knots.size());
        for (size_t i = 0; i < msg->knots.size(); ++i)
        {
            knots(i) = msg->knots[i];
        }
        UniformBspline pos_traj(ctrl_pts, msg->order, 0.1);
        pos_traj.setKnot(knots);
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());
        traj_.push_back(traj_[1].getDerivative());
        traj_duration_ = traj_[0].getTimeSum();
        receive_traj_ = true;

    }
}
void exp_pos_Callback(const ros::TimerEvent &e)
{
    if (!receive_traj_)
        return;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    Eigen::Vector3d pos(Eigen::Vector3d::Zero());

    static ros::Time time_last = ros::Time::now();
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        pos = traj_[0].evaluateDeBoorT(t_cur);
    }
    else if (t_cur >= traj_duration_)
    {
        /* hover when finish traj_ */
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_now;
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = 0;
    sphere.color.r = color_r;
    sphere.color.g = color_g;
    sphere.color.b = color_b;
    sphere.color.a = color_a > 1e-5 ? color_a : 1.0;
    sphere.scale.x = 0.4;
    sphere.scale.y = 0.4;
    sphere.scale.z = 0.4;
    sphere.pose.position.x = pos(0);
    sphere.pose.position.y = pos(1);
    sphere.pose.position.z = pos(2);

    // geometry_msgs::Point pt;

    // pt.x = pos(0);
    // pt.y = pos(1);
    // pt.z = pos(2);
    // sphere.points.push_back(pt);

    exp_pos_marker_pub.publish(sphere);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_terminal");
    ros::NodeHandle nh("~");

    visualization_.reset(new ego_planner::PlanningVisualization(nh));

    nh.param("drone_id", drone_id_, -1);
    nh.param("color/r", color_r, 1.0);
    nh.param("color/g", color_g, 0.0);
    nh.param("color/b", color_b, 0.0);
    nh.param("color/a", color_a, 1.0);
    // bsplines_msg_.reset(new traj_utils::MultiBsplines);
    // odom_msg_.reset(new nav_msgs::Odometry);
    // stop_msg_.reset(new std_msgs::Empty);
    // bspline_msg_.reset(new traj_utils::Bspline);
    exp_pos_marker_pub = nh.advertise<visualization_msgs::Marker>("exp_pos", 50);
    other_odoms_sub_ = nh.subscribe("/others_odom", 10, odom_sub_cb, ros::TransportHints().tcpNoDelay());
    drone_odom_pub_ = nh.advertise<nav_msgs::Odometry>("drone_odom_topic", 10);

    one_traj_sub_ = nh.subscribe("/broadcast_bspline2", 100, one_traj_sub_cb, ros::TransportHints().tcpNoDelay());
    ros::Timer exp_pos_timer = nh.createTimer(ros::Duration(0.01), exp_pos_Callback);

    ros::Duration(0.1).sleep();
    std::cout << "[swarm_terminal] start running" << std::endl;
    ros::spin();
    return 0;
}
