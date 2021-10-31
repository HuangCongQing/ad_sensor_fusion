<!--
 * @Description: 
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2021-09-21 14:57:27
 * @LastEditTime: 2021-10-31 16:37:38
 * @FilePath: /ad_sensor_fusion/README.md
-->
# ad_sensor_fusion
个人学习分支：https://github.com/HuangCongQing/ad_sensor_fusion/tree/hcq

# version and commit
20210727 first commit： commit the "icp" and "camera imu time sync" with compilte  

20210805 second commit: commit the "camera camera sync"  

20210814 3rd commit: commit the "update camera imu time sync"  

20210820 4rd commit: commit the "update camera lidar sync"

20210827 5rd commit: commit the "update lidar imu sync and mdt_omp"

20210904 6rd commit：commit the "stereo camera distance"

20210905 7rd commit: commit the "camera imu fusion"

20210918 8rd commit：commit the "camera lidar fusion and camera radar fusion"

# dataset
用于camera_lidar_fusion的数据集采用 kitti_data: 百度网盘https://pan.baidu.com/s/1dtZKKLBJ8DuDxg6NShStDg,提取码：jd88

用于双目和毫米波雷达的数据连接为：https://pan.baidu.com/s/1vvajIvn-rdKwFkhzZIrdxA提取码：otwn

其他数据集为：https://pan.baidu.com/s/1fXj-qupzd0ZeKOYMv3rFXg，提取码为：lwf9

# 环境配置

参考：https://www.yuque.com/huangzhongqing/eozhay/zkh3re

* 注意，在每个模块里CMakeLists.txt添加

```
set(OpenCV_DIR "/media/hcq/hcq4T/多传感器融合资料/07多相机间的同步实战/opencv/3rdparty/share/OpenCV")
```

# learning

* 多相机同步[camera_camera_sync](src/camera_camera_sync/src/main.cpp)
    * 修改路径：src/camera_camera_sync/src/main.cpp
    * `./devel/lib/camera_camera_sync/camera_camera_sync_node`

* 相机雷达同步[camera_lidar_sync](src/camera_lidar_sync/src/camera_lidar_sync.cpp)
    * ` rosbag play '/media/hcq/hcq4T/多传感器融合资料/99 数据/practice_1_2_camera_imu_sync/test.bag'`
    *  `rosrun camera_lidar_sync camera_lidar_sync_node`

* 相机雷达融合[camera_lidar_fusion](src/camera_lidar_fusion/sensor_processing/src/sensor_processing_lib/sensor_fusion.cpp)

```
roslaunch sensor_processing sensor_processing.launch home_dir:=/home/hcq/data/data_ad_sensor_fusion/99data/practice_2_3_camera_lidar/0012
roslaunch detection detection.launch home_dir:=/home/hcq/data/data_ad_sensor_fusion/99data/practice_2_3_camera_lidar/0012
roslaunch tracking tracking.launch home_dir:=/home/hcq/data/data_ad_sensor_fusion/99data/practice_2_3_camera_lidar/0012

```

* 相机毫米波融合[camera_radar_fusion](src/camera_radar_fusion/src/main.cpp)
    * 路径修改：src/camera_radar_fusion/src/main.cpp
    * `rosrun camera_radar_fusion camera_radar_fusion_node`