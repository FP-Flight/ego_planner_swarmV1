#include "swarm_terminal/drone_state.hpp"
#define TAKEOFF_HEIGHT 0.7
drone_state::drone_state(int drone_id,Eigen::Vector3d real_pos)
{
    drone_id_ = drone_id;
    real_pos_ = real_pos;
    last_update_time = ros::Time::now()- ros::Duration(1e3);

    selected_=false;


}
void drone_state::set_new_command(Eigen::Vector3d exp_pos)
{
    if((exp_pos - exp_pos_).norm()>0.3)
    {
        exp_pos_ = exp_pos;
    }
}
void drone_state::update_odom(Eigen::Vector3d real_pos)
{
    real_pos_ = real_pos;
    last_update_time = ros::Time::now();
}
bool drone_state::is_alive()
{
    if((ros::Time::now() - last_update_time)<ros::Duration(1)){
        return true;
    }
    else{
        return false;
    }
}
bool drone_state::is_arrive_goal()
{
    if((exp_pos_ - real_pos_).norm()<0.3 && is_alive()){
        return true;
    }
    else{
        return false;
    }
}
bool drone_state::is_takeoff_done()
{
    if(abs(TAKEOFF_HEIGHT - real_pos_(2))<0.15){
        return true;
    }
    else{
        return false;
    }
}
int drone_state::get_drone_id()
{
    return drone_id_;
}
void drone_state::set_selected()
{
    selected_ = true;
}
bool drone_state::get_selected()
{
    return selected_;
}
drone_state::~drone_state()
{

}