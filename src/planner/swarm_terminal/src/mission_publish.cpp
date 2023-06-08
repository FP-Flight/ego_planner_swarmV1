#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include "swarm_terminal/drone_state.hpp"
#include "swarm_terminal/read_mission.hpp"
std::vector<drone_state> drones_state;
std::vector<std::pair<int,std::vector<Eigen::Vector3d>>> dronePoints;

ros::Publisher goal_pub,takeoff_pub,land_pub;
void one_goal_publish(int id,Eigen::Vector3d dronePoint){
    std_msgs::Float32MultiArray goal_msgs;
    goal_msgs.data = {
        static_cast<float>(id),
        static_cast<float>(dronePoint(0)),
        static_cast<float>(dronePoint(1)),
        static_cast<float>(dronePoint(2))};
    goal_pub.publish(goal_msgs);
}
void takeoff_publish(std::vector<std::pair<int,std::vector<Eigen::Vector3d>>> dronePoints){
    std_msgs::Float32MultiArray takeoff_msgs;
    takeoff_msgs.data.push_back(static_cast<float>(dronePoints.size()));
    for(auto i:dronePoints)
    {
        takeoff_msgs.data.push_back(static_cast<float>(i.first));
    }
    takeoff_pub.publish(takeoff_msgs);
}

void land_publish(std::vector<std::pair<int,std::vector<Eigen::Vector3d>>> dronePoints){
    std_msgs::Float32MultiArray land_msgs;
    land_msgs.data.push_back(static_cast<float>(dronePoints.size()));
    for(auto i:dronePoints)
    {
        land_msgs.data.push_back(static_cast<float>(i.first));
    }
    land_pub.publish(land_msgs);
}


nav_msgs::Odometry other_odom;
void others_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    other_odom = *msg;
    std::string drone_id = other_odom.child_frame_id;
    // Eigen::Vector3d pos;
    // pos<<msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z;
    // int id=1;
    // for(auto i:drones_state){
    //     if(i.get_drone_id() == id)
    //     i.update_odom(pos);
    // }

}


int main(int argc, char **argv)
{    
    std::string mission_file_path = "/home/qyswarm/param_files/template/mission.yaml";
    if(!read_mission(mission_file_path, dronePoints))
        return 1;
    ros::init(argc, argv, "mission_publish");
    ros::NodeHandle nh("~");
    // std::cout<<dronePoints<<std::endl;
    print_mission(dronePoints);


    ros::Subscriber others_odom_sub = nh.subscribe("/others_odom", 10, others_odom_cb, ros::TransportHints().tcpNoDelay());
    goal_pub = nh.advertise<std_msgs::Float32MultiArray>("/swarm_command", 10);
    takeoff_pub = nh.advertise<std_msgs::Float32MultiArray>("/swarm_takeoff", 10);
    land_pub = nh.advertise<std_msgs::Float32MultiArray>("/swarm_land", 10);
    
    
    // Check 
    int swarm_num = dronePoints.size();
    int real_num = 0;
    while (ros::ok() && 0)
    {
        real_num=0;
        for(int i=0;i<dronePoints.size();i++)
        {
            for(int j=0;j<drones_state.size();j++)
            {
                if( dronePoints.at(i).first == drones_state.at(j).get_drone_id() )
                {
                    real_num++;
                    drones_state.at(j).set_selected();
                    drones_state.at(j).mission_pionts=dronePoints.at(i).second;

                }
            }

        }
        if(real_num == swarm_num)
        {
            break;
        }
        std::cout << "\033[31m" << "here are "<<swarm_num<<" planes to perform the mission, but only "<<real_num<<" is online" << "\033[0m" << std::endl; 
        std::cout << "\033[31m" << "Try Check Again after 5 seconds "<< "\033[0m" << std::endl; 
        ros::spinOnce();
        ros::Duration(5).sleep();
    }

    // takeoff
    if(ros::ok())
    {
        std::cout << "\033[32m" << "Check Complete" << "\033[0m" << std::endl; 

        std::cout << "\033[32m" << "All Drones will TAKEOFF after 5 seconds" << "\033[0m" << std::endl; 
        for(int i=5;i>0;i--){
            ros::spinOnce();
            ros::Duration(1).sleep();
            std::cout << i << std::endl; 
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
        std::cout << "\033[32m" << "TAKEOFF" << "\033[0m" << std::endl; 
        takeoff_publish(dronePoints);
        //waiting all drones takeoff 
        int takeoff_over_flag = 0;
        while (ros::ok())
        {
            for(int j=0;j<drones_state.size();j++)
            {
                if(drones_state.at(j).get_selected() && drones_state.at(j).is_takeoff_done())
                {
                    takeoff_over_flag++;
                }
            }
            if(takeoff_over_flag == swarm_num)
            {
                break;
            }
            takeoff_over_flag = 0;
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }





    // mission
    int point_th = 0;
    int mission_pub_num = 0;
    int mission_complete_num=0;
    while (ros::ok())
    {
        mission_pub_num = 0;
        for(auto one_drone_state:drones_state)
        {
            if(one_drone_state.get_selected())
            {
                if(point_th < one_drone_state.mission_pionts.size())
                {
                    mission_pub_num++;
                    one_goal_publish(one_drone_state.get_drone_id(),one_drone_state.mission_pionts.at(point_th));
                    one_drone_state.set_new_command(one_drone_state.mission_pionts.at(point_th));
                    ros::Duration(0.05).sleep();
                    ros::spinOnce();
                }

            }
        }
        if(mission_pub_num == 0)
        {
            // if all mission points is published,exit
            break;
        }
        point_th++;
        // waiting mission complete
        while(ros::ok()){
            for(auto one_drone_state:drones_state)
            {
                if(one_drone_state.get_selected() && one_drone_state.is_arrive_goal())
                {      
                    mission_complete_num++;
                }
                
            }
            if(mission_complete_num==mission_pub_num)
            {
                break;
            }
            mission_complete_num = 0;
            ros::Duration(0.3).sleep();
            ros::spinOnce();
        }
    } 



    // land
    if (ros::ok())
    {        
        std::cout << "\033[32m" << "Ready to LAND" << "\033[0m" << std::endl; 
        land_publish(dronePoints);
        ros::Duration(3).sleep();
    }



    return 0;

}