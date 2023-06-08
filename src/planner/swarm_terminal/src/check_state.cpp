#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <swarm_terminal/FSM_move_to.hpp>
#define ROS_RATE 20.0
using namespace std;

nav_msgs::Odometry other_odom;
void others_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    other_odom = *msg;
}

int main(int argc, char **argv)
{
    /*--------Read File-------------------------------------------------*/
    int droneNum;
    std::vector<int> droneid;
    std::vector<std::vector<std::vector<int>>> dronePoints;
    ifstream ifs;
    ifs.open("/home/admin/swarm_ws/src/param_files/template/mission.txt", ios::in);

    if (!ifs.is_open())
    {
        std::cerr << "Failed to open file" << std::endl;
        return 1;
    }

    if (ifs.is_open())
    {
        string line;
        while (getline(ifs, line))
        {
            size_t pos = line.find(':');
            if (pos != std::string::npos)
            {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);

                // 去除空格
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));

                if (key == "drone_num")
                {
                    droneNum = std::stoi(value);
                }
                else if (key == "drone_points")
                {
                    // 解析 drone_points 参数
                    std::vector<std::vector<int>> points;
                    while (getline(ifs, line))
                    {
                        int idx = line.find("id");
                        size_t idstart = line.find_first_of(':') + 1;
                        std::string idStr = line.substr(idstart, line.length());
                        droneid.push_back(std::stoi(idStr));
                        if (idx > -1 && idx < 200)
                        {
                            getline(ifs, line); // 读取 id 行;
                            for (int i = 0; i < 3; i++)
                            {
                                getline(ifs, line); // 读取 points 行
                                std::vector<int> point;
                                size_t start = line.find_first_of('[') + 1;
                                size_t end = line.find_last_of(']');
                                std::string pointStr = line.substr(start, end - start);

                                int commaPos = pointStr.find(',');
                                while (commaPos > -1 && commaPos < 200)
                                {
                                    int value = std::stoi(pointStr.substr(0, commaPos));
                                    point.push_back(value);
                                    pointStr.erase(0, commaPos + 1);
                                    commaPos = pointStr.find(',');
                                }
                                int lastValue = std::stoi(pointStr);
                                point.push_back(lastValue);
                                points.push_back(point);
                            }
                        }
                    }
                    dronePoints.push_back(points);
                }
            }
        }
    }
    ifs.close();
    // cout << "droneNum:" << droneNum << endl;
    // cout << "droneid:" << droneid[0] << droneid[1] << droneid[2] << endl;
    // cout << "points:" << endl;
    // for (auto col : dronePoints)
    // {
    //     for (auto row : col)
    //     {
    //         for (auto temp : row)
    //         {
    //             std::cout << temp;
    //         }
    //         std::cout << std::endl;
    //     }
    //     std::cout << std::endl;
    // }
    /*---End of read file ----------------------------------------------*/

    ros::init(argc, argv, "check_state");
    ros::NodeHandle nh("~");
    ros::Subscriber others_odom_sub = nh.subscribe("/others_odom", 10, others_odom_cb, ros::TransportHints().tcpNoDelay());
    ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("/swarm_cmd", 10);
    ros::Rate rate(ROS_RATE);
    double takeoff_height = 0.7;

    FSM_MOVE_TO fsm_1;
    FSM_MOVE_TO fsm_2;
    FSM_MOVE_TO fsm_3;
    bool ALL_TAKE_OFF = false;
    bool ALL_MOVE_TO_POINT_1 = false;
    bool ALL_MOVE_TO_POINT_2 = false;
    bool ALL_MOVE_TO_POINT_3 = false;
    std_msgs::Float32MultiArray goal_msgs;

    FSM_MOVE_TO::State_t now_state;
    FSM_MOVE_TO::State_t last_state;

    while (ros::ok())
    {
        if (other_odom.child_frame_id == "drone_1")
        {
            // check state for take off
            if ((abs(other_odom.pose.pose.position.z - takeoff_height) < 0.1) && fsm_1.now_state == "NOW_STATE")
            {
                fsm_1.set_takeoff_done_flag(true);

                goal_msgs.data = {
                    static_cast<float>(droneid[1]),
                    static_cast<float>(dronePoints[1][1][1]),
                    static_cast<float>(dronePoints[1][1][2]),
                    static_cast<float>(dronePoints[1][1][3])};
                goal_pub.publish(goal_msgs);
            }

            if ((abs(other_odom.pose.pose.position.x - dronePoints[1][1][1]) > 0.1 ||
                 abs(other_odom.pose.pose.position.y - dronePoints[1][1][2]) > 0.1 ||
                 abs(other_odom.pose.pose.position.z - dronePoints[1][1][3]) > 0.1) &&
                fsm_1.now_state)
            {
                fsm_1.set_move_to_point_1_flag(true);
                goal_msgs.data = {
                    static_cast<float>(droneid[1]),
                    static_cast<float>(dronePoints[1][2][1]),
                    static_cast<float>(dronePoints[1][2][2]),
                    static_cast<float>(dronePoints[1][2][3])};
                goal_pub.publish(goal_msgs);
            }

            if ((abs(other_odom.pose.pose.position.x - dronePoints[1][1][1]) > 0.1 ||
                 abs(other_odom.pose.pose.position.y - dronePoints[1][1][2]) > 0.1 ||
                 abs(other_odom.pose.pose.position.z - dronePoints[1][1][3]) > 0.1) &&
                fsm_1.now_state)
            {
                fsm_1.set_move_to_point_2_flag(true);
                goal_msgs.data = {
                    static_cast<float>(droneid[1]),
                    static_cast<float>(dronePoints[1][3][1]),
                    static_cast<float>(dronePoints[1][3][2]),
                    static_cast<float>(dronePoints[1][3][3])};
                goal_pub.publish(goal_msgs);
            }
        }

        if (other_odom.child_frame_id == "drone_2")
        {
            // check state for take off
            if (abs(other_odom.pose.pose.position.z - takeoff_height) < 0.1)
            {
                fsm_2.set_takeoff_done_flag(true);
                std_msgs::Float32MultiArray goal_msgs;
                goal_msgs.data = {
                    static_cast<float>(droneid[2]),
                    static_cast<float>(dronePoints[2][1][1]),
                    static_cast<float>(dronePoints[2][1][2]),
                    static_cast<float>(dronePoints[2][1][3])};
                goal_pub.publish(goal_msgs);
            }
        }

        if (other_odom.child_frame_id == "drone_3")
        {
            // check state for take off
            if (abs(other_odom.pose.pose.position.z - takeoff_height) < 0.1)
            {
                fsm_3.set_takeoff_done_flag(true);
                std_msgs::Float32MultiArray goal_msgs;
                goal_msgs.data = {
                    static_cast<float>(droneid[3]),
                    static_cast<float>(dronePoints[3][1][1]),
                    static_cast<float>(dronePoints[3][1][2]),
                    static_cast<float>(dronePoints[3][1][3])};
                goal_pub.publish(goal_msgs);
            }
        }
        if (fsm_1.get_takeoff_done_flag() && fsm_2.get_takeoff_done_flag() && fsm_3.get_takeoff_done_flag())
        {
            ALL_MOVE_TO_POINT_1 = true;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
