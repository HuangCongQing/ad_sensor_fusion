/*
 * @Description: 
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2021-11-06 21:17:24
 * @LastEditTime: 2021-11-08 11:23:09
 * @FilePath: /ad_sensor_fusion/src/lidar_radar_fusion/src/main.cpp
 */
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "utils.hpp"
#include "lidar_radar_fusion_task.hpp"  // task
using namespace lidar_radar_fusion;  // 命名空间 隔离其他

int main() 
{
    const std::string lidar_data_path = "/home/kavin/data/practice_2_6_radar_lidar_fusion/lidar_data_txt"; // 修改路径
    const std::string radar_data_path = "/home/kavin/data/practice_2_6_radar_lidar_fusion/radar_data_txt";

    // load lidar data 激光雷达
    std::vector<cv::String> lidar_data_list;
    GetFileLists(lidar_data_path, lidar_data_list);
    std::cout << "lidar_data_list.size() = " << lidar_data_list.size() << std::endl;

    // load radar data 毫米波雷达
    std::vector<cv::String> radar_data_list;
    GetFileLists(radar_data_path,radar_data_list);
    std::cout << "radar_data_list.size() = " << radar_data_list.size() << std::endl;
    
    bool ret = false;
    std::shared_ptr<LidarRadarFusionTask> task = std::make_shared<LidarRadarFusionTask>(); // src/lidar_radar_fusion/src/lidar_radar_fusion_task.cpp
    ret = task->Init(lidar_data_list,radar_data_list);  //Init操作 src/lidar_radar_fusion/src/lidar_radar_fusion_task.cpp
    ret = task->Run();
    

    return 0;
}