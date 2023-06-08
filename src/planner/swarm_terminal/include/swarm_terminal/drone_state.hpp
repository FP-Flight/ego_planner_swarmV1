#ifndef __DRONE_STATE_HPP_
#define __DRONE_STATE_HPP_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/time.h>
class drone_state
{
private:
    int drone_id_ =-1;
    ros::Time last_update_time;
    Eigen::Vector3d exp_pos_;
    Eigen::Vector3d real_pos_;
    bool selected_;


public:
    drone_state(int drone_id,Eigen::Vector3d real_pos);
    ~drone_state();
    void set_new_command(Eigen::Vector3d exp_pos);
    void update_odom(Eigen::Vector3d real_pos);
    bool is_alive();
    bool is_arrive_goal();
    int get_drone_id();
    void set_selected();
    bool get_selected();
    bool is_takeoff_done();
    std::vector<Eigen::Vector3d> mission_pionts;


};


#endif
