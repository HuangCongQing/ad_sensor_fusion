/*
 * @Description: 各函数声明
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2021-11-06 21:17:24
 * @LastEditTime: 2021-11-08 11:47:38
 * @FilePath: /ad_sensor_fusion/src/lidar_radar_fusion/include/lidar_radar_fusion.hpp
 */
#ifndef __LIDAR_RADAR_FUSION_HPP__
#define __LIDAR_RADAR_FUSION_HPP__
#include "utils.hpp"
#include "ekfInterface.hpp"


namespace lidar_radar_fusion {
    class LidarRadarFusion {
    public:    
        LidarRadarFusion();
        ~LidarRadarFusion();

        bool Init(const std::map<TimeStamp, std::vector<ObjectInfo> >& lidar_objects,  // lidar有高度信息，radar没有高度信息，激光雷达的z信息设置为0
                  const std::map<TimeStamp, std::vector<ObjectInfo> >& radar_objects); // 初始化是对齐后的时间戳
        bool Run();
        std::vector<TrackObject> GetTrackedObjectPoints();
        std::vector<ObjectInfo> GetFilteredObjectPoints();

    private:
        float CalcuProjectScore(const ObjectInfo& lidar_object,
                                const ObjectInfo& radar_object);
        void MatchWith3D(const std::vector<ObjectInfo>& lidar_objects,
                         const std::vector<ObjectInfo>& radar_objects);
        void Match(const std::vector<ObjectInfo>& lidar_objects,
                             const std::vector<ObjectInfo>& radar_objects,
                             std::vector<int>& lidar2radar, 
                             std::vector<int>& radar2lidar);
        void HungarianMatcher(Eigen::MatrixXd &scores, 
                            std::vector<int> &t2d, std::vector<int> &d2t,
					        double thres = 0.2);

        bool _is_initialized = false;
        std::shared_ptr<EKF_API> _ekf_api;// = new EKF_API();

        std::map<TimeStamp, std::vector<ObjectInfo> > _lidar_objects;
        std::map<TimeStamp, std::vector<ObjectInfo> > _radar_objects;
       
        std::map<int , int > _lidar_radar_match;
        std::vector<TrackObject> _tracked_list;
        std::vector<ObjectInfo> _filtered_list;
        int _object_id = -1;
        

    };
}

#endif // __LIDAR_RADAR_FUSION_HPP__