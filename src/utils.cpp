//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>

namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan, float resolution)
{
  // TODO
  // ROS_INFO("angle_min %f, angle_max %f, angle_incre %f", scan->angle_min, scan->angle_max, scan->angle_increment);
  cv::Mat point_mat(scan->ranges.size(), 2, CV_32F);
  for(int i=0; i < scan->ranges.size(); i++)
  { 
    float a = scan->angle_min + scan->angle_increment*i;
    float x, y;
    utils::polarToCartesian(scan->ranges[i], a, x, y);
    point_mat.at<float>(i,0) = x/resolution;
    point_mat.at<float>(i,1) = y/resolution;
  }
  return point_mat;
}

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  // TODO
  // ROS_INFO("angle_min %f, angle_max %f, angle_incre %f", scan->angle_min, scan->angle_max, scan->angle_increment);
  cv::Mat point_mat(scan->ranges.size(), 2, CV_32F);
  for(int i=0; i < scan->ranges.size(); i++)
  { 
    float a = scan->angle_min + scan->angle_increment*i;
    float x, y;
    utils::polarToCartesian(scan->ranges[i], a, x, y);
    point_mat.at<float>(i,0) = x;
    point_mat.at<float>(i,1) = y;
  }
  return point_mat;
}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));
  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam