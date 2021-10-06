#include "camera_imu_time_sync/utils.hpp"
#include <ros/ros.h>

// 计算点云角度变化（ICP）
double calcAngleBetweenPointclouds(
    const pcl::PointCloud<pcl::PointXYZ>& prev_pointcloud,
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud) 
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pointcloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);

    // shared_pointers needed by icp, no-op destructor to prevent them being cleaned up after use
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_pointcloud_ptr(&prev_pointcloud, [](const pcl::PointCloud<pcl::PointXYZ>*) {});
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(&pointcloud, [](const pcl::PointCloud<pcl::PointXYZ>*) {});

    constexpr int kMaxSamples = 2000;
    pcl::RandomSample<pcl::PointXYZ> sample(false);
    sample.setSample(kMaxSamples);

    sample.setInputCloud(prev_pointcloud_ptr);
    sample.filter(*prev_pointcloud_sampled);

    sample.setInputCloud(pointcloud_ptr);
    sample.filter(*pointcloud_sampled);

    icp.setInputSource(prev_pointcloud_sampled);
    icp.setInputTarget(pointcloud_sampled);

    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);

    Eigen::Matrix4f tform = icp.getFinalTransformation();
    double angle = Eigen::AngleAxisd(tform.topLeftCorner<3, 3>().cast<double>()).angle();

    return angle;
}
// 计算图像角度变化
double calcAngleBetweenImages(const cv::Mat& prev_image,
                              const cv::Mat& image, float focal_length) {
  constexpr int kMaxCorners = 100;
  constexpr double kQualityLevel = 0.01;
  constexpr double kMinDistance = 10;

  // 前一帧特征点检测
  std::vector<cv::Point2f> prev_points;
  cv::goodFeaturesToTrack(prev_image, prev_points, kMaxCorners, kQualityLevel, kMinDistance);

  if (prev_points.size() == 0) {
    ROS_ERROR("Tracking has failed cannot calculate angle");
    return 0.0;
  }

  std::vector<cv::Point2f> points;
  std::vector<uint8_t> valid;
  std::vector<float> err;
  // 当前点和前一帧关联
  cv::calcOpticalFlowPyrLK(prev_image, image, prev_points, points, valid, err); // LK光流金字塔算法

  std::vector<cv::Point2f> tracked_prev_points, tracked_points;  //保存下来
  for (size_t i = 0; i < prev_points.size(); ++i) {
    if (valid[i]) {
      tracked_prev_points.push_back(prev_points[i]);
      tracked_points.push_back(points[i]);
    }
  }

  /*cv::Mat viz_image;
  cv::cvtColor(prev_image, viz_image, cv::COLOR_GRAY2BGR);

  for (size_t i = 0; i < tracked_points.size(); ++i) {
    cv::arrowedLine(viz_image, tracked_prev_points[i], tracked_points[i],
                    cv::Scalar(0, 255, 0));
  }

  cv::namedWindow("Tracked Points", cv::WINDOW_AUTOSIZE);
  cv::imshow("Tracked Points", viz_image);
  cv::waitKey(1);*/

  // close enough for most cameras given the low level of accuracy needed
  const cv::Point2f offset(image.cols / 2.0, image.rows / 2.0);
  // 理解为像素坐标系按照小孔成像的原理  转化为相机坐标系 
  // 两个相机间的角度变化关系
  for (size_t i = 0; i < tracked_points.size(); ++i) {
    tracked_prev_points[i] = (tracked_prev_points[i] - offset) / focal_length;  // focal_length焦距
    tracked_points[i] = (tracked_points[i] - offset) / focal_length;
    //std::cout << tracked_prev_points[i].x << " " << tracked_prev_points[i].y << std::endl;
  }

  constexpr double kMaxEpipoleDistance = 1e-3;
  constexpr double kInlierProbability = 0.99;

  std::vector<uint8_t> inliers;
  // findFundamentalMat求解两个图像的局部坐标关系，构建两个图像之间的对极几何矩阵
  // tracked_prev_points：第一个图像特征点  tracked_points：第二个图像特征点
  cv::Mat cv_F = cv::findFundamentalMat(tracked_prev_points, tracked_points, cv::FM_LMEDS,
                                     kMaxEpipoleDistance, kInlierProbability, inliers);
  Eigen::Matrix3d E, W;

  cv::cv2eigen(cv_F, E); // 对极几何矩阵转换成eigen格式
  // svd分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(E, Eigen::ComputeThinU | Eigen::ComputeThinV);
  W << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // 两个不同方向坐标轴的旋转向量
  Eigen::Matrix3d Ra = svd.matrixU() * W * svd.matrixV().transpose();
  Eigen::Matrix3d Rb = svd.matrixU() * W.transpose() * svd.matrixV().transpose();
  // 得到角度变化关系
  double angle = std::min(Eigen::AngleAxisd(Ra).angle(), Eigen::AngleAxisd(Rb).angle());

  return angle;
}