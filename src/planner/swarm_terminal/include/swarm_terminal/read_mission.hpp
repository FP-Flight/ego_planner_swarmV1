#ifndef __READ_MISSION_HPP_
#define __READ_MISSION_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Eigen>
int print_mission(std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &dronePoints){
    std::cout << "\033[32m" << "Get Mission"<< "\033[0m" << std::endl;

    for (auto i : dronePoints) { // i 表示 vector 中的每个元素
        std::cout<<"Drone_id: "<<i.first<<std::endl;
        for(auto j : i.second)
            std::cout <<j(0) <<" "<<j(1)<<" "<<j(2) << std::endl;
    }
    return 0;
}
int get_space_num(std::string line)
{
    int count = 0;      // 声明一个计数器变量，并初始化为 0
    for (int i = 0; i < line.size(); i++)
    { // 使用 for 循环遍历字符串中的每个字符
        if (isspace(line[i]))
        {            // 如果当前字符是空白字符，则调用 isspace 函数
            count++; // 累加计数器的值
        }
        else
        {
            break;; // 如果当前字符不是空白字符，则跳出循环
        }
    }
    // std::cout << "The number of leading spaces is: " << count << std::endl; // 输出计数器的值
    return count; // 如果当前字符不是空白字符，则跳出循环
}
bool read_mission(std::string filename, std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &dronePoints)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios::in);

    int droneNum;
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open file" << std::endl;
        return false;
    }

    std::string line;
    // find drone_num:
    std::streampos last_pos = ifs.tellg(); // 保存当前位置
    size_t pos;
    while (getline(ifs, line))
    {
        pos = line.find('#');
        if (pos != std::string::npos)
        {
            continue;
        }
        // std::cout << line << std::endl;

        pos = line.find(':');
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
                break;
            }
        }
    }
    // ifs.seekg(last_pos);
    ifs.clear(); // 清除文件末尾标志
    ifs.seekg(0L, std::ios::beg); // 返回到文件开头
    // find drone_points:
    while (getline(ifs, line))
    {
        pos = line.find('#');
        if (pos != std::string::npos)
        {
            continue;
        }
        size_t pos = line.find(':');
        if (pos != std::string::npos)
        {
            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos + 1);

            // 去除空格
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));

            if (key == "drone_points")
            {
                // 解析 drone_points 参数
                std::vector<Eigen::Vector3d> points;
                while (getline(ifs, line))
                {
                    if (line.empty() || std::all_of(line.begin(), line.end(), isspace))
                    {
                        continue;
                    }
                    // std::cout << line << std::endl;

                    int idx = line.find("id");
                    size_t idstart = line.find_first_of(':') + 1;
                    int drone_id_temp = std::stoi(line.substr(idstart, line.length()));

                    if (idx > -1 && idx < 200)
                    {
                        getline(ifs, line); // throw out "points "line
                        int refer_space_num = get_space_num(line);

                        while (true)                   
                        {

                            last_pos = ifs.tellg();
                            getline(ifs, line); // 读取 points 行
                            if(get_space_num(line) - refer_space_num<=0 || line.empty() || std::all_of(line.begin(), line.end(), isspace))
                            {
                                ifs.seekg(last_pos);
                                break;
                            }
  
                            // std::vector<int> point;
                            Eigen::Vector3d point;
                            size_t start = line.find_first_of('[') + 1;
                            size_t end = line.find_last_of(']');
                            std::string pointStr = line.substr(start, end - start);
                            
                            // find x y z
                            int commaPos = pointStr.find(',');
                            if (commaPos > -1 && commaPos < 200)
                            {
                                point(0) = std::stof(pointStr.substr(0, commaPos));
                            }
                            pointStr.erase(0, commaPos + 1);
                            commaPos = pointStr.find(',');
                            if (commaPos > -1 && commaPos < 200)
                            {
                                point(1) = std::stof(pointStr.substr(0, commaPos));
                            }
                            pointStr.erase(0, commaPos + 1);

                            point(2) = std::stof(pointStr.substr(0, commaPos));

                            points.push_back(point);
                        }
                    }
                    dronePoints.push_back(std::make_pair(drone_id_temp, points));
                    points.clear();
                }
            }
        }
    }

    ifs.close();
    return true;
}

#endif