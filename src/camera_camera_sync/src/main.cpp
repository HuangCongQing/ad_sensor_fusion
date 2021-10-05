/*
 * @Description: 
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2021-09-21 14:57:27
 * @LastEditTime: 2021-10-05 17:02:45
 * @FilePath: /ad_sensor_fusion/src/camera_camera_sync/src/main.cpp
 */
#include "camera_camera_sync/camera_camera_sync.hpp"

int main()
{
    CameraCameraSync camera_camera_sync_; // 封装的相机同步类
    // 文件夹路径修改 // left时间为基准找right
    // std::string oriDirs = "/disk3/sensor_fusion/datasets/src/rosbag_dump/2021-07-13-10-34-45/camera_c5_front_left_60"; // left时间为基准找right
    // std::string dstDirs = "/disk3/sensor_fusion/datasets/src/rosbag_dump/2021-07-13-10-34-45/camera_c6_front_right_60";
    std::string oriDirs = "/media/hcq/hcq4T/多传感器融合资料/07多相机间的同步实战/practice_1_1_multi_camera_sync/camera_front_left_60"; // left时间为基准找right
    std::string dstDirs = "/media/hcq/hcq4T/多传感器融合资料/07多相机间的同步实战/practice_1_1_multi_camera_sync/camera_front_right_60";
    camera_camera_sync_.getImageTimeStamp(oriDirs, dstDirs);
    std::vector<std::pair<std::string, std::string> > syncImageLists;
    int number = camera_camera_sync_.getImageNumber();
    if (number > 0)
    {
        syncImageLists = camera_camera_sync_.imageTimeStampSyncFuncion();  // 1 时间同步   返回配对符合要求的list
    }

    // 2 空间同步（estimate pitch and rol角）
    for(auto syncPair : syncImageLists)// 遍历 2000多张
    {
        cv::Mat image1 = cv::imread(syncPair.first, cv::IMREAD_GRAYSCALE);
        cv::Mat image2 = cv::imread(syncPair.second, cv::IMREAD_GRAYSCALE);
        if( !image1.data || !image2.data )
        { 
            std::cout<< " --(!) Error reading images " << std::endl; 
            return -1;
        }
        
        camera_camera_sync_.spatialSynchronization(image1, image2);  //  空间同步函数
    }

    // cv::Mat image1 = cv::imread("/disk3/sensor_fusion/datasets/src/rosbag_dump/2021-07-13-10-34-45/camera_c5_front_left_60/1625566154.370427.png", cv::IMREAD_GRAYSCALE);
    // cv::Mat image2 = cv::imread("/disk3/sensor_fusion/datasets/src/rosbag_dump/2021-07-13-10-34-45/camera_c6_front_right_60/1625566154.370326.png", cv::IMREAD_GRAYSCALE);
    // if( !image1.data || !image2.data )
    // { 
    //     std::cout<< " --(!) Error reading images " << std::endl; 
    //     return -1;
    // }
    // camera_camera_sync_.spatialSynchronization(image1, image2);

}
