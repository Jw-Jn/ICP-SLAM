//
// Created by rakesh on 13/08/18.
//
// rosrun stage_ros stageros `rospack find safe_teleop`/worlds/willow-erratic.world 
// roslaunch turtlebot3_bringup turtlebot3_robot.launch
// rosrun stage_ros stageros `rospack find stage`/worlds/simple.world
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>

#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_laser_scan_(new sensor_msgs::LaserScan()),
    last_kf_seq_(0),
    is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{
  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }

  bool is_create_kf;
  if (last_kf_seq_== 0)
  {
    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_tf_map_laser_ = last_kf_tf_odom_laser_;
    tf_map_laser = current_frame_tf_odom_laser;
    last_kf_seq_ = laser_scan->header.seq;
    return true;
  }
  else
    is_create_kf = isCreateKeyframe(current_frame_tf_odom_laser, last_kf_tf_odom_laser_);
  
  if (is_create_kf)
  {
    tf::Transform T_2_1 = last_kf_tf_odom_laser_.inverse()*current_frame_tf_odom_laser;
    tf::StampedTransform temp;
    temp.setData(icpRegistration(last_kf_laser_scan_, laser_scan, T_2_1));

    tf_map_laser.setData(last_kf_tf_map_laser_ * (temp.inverse()));
    last_kf_tf_map_laser_ = tf_map_laser;
    *last_kf_laser_scan_ = *laser_scan;
    last_kf_tf_odom_laser_ = current_frame_tf_odom_laser;
    last_kf_seq_ = laser_scan->header.seq;
  }
  else
  {
    tf::Transform T_2_1 = last_kf_tf_odom_laser_.inverse()*current_frame_tf_odom_laser;
    tf_map_laser.setData(last_kf_tf_map_laser_ * (T_2_1.inverse()));
  }

  return is_create_kf;
  // TODO: find the pose of laser in map frame
  // if a new keyframe is created, run ICP
  // if not a keyframe, obtain the laser pose in map frame based on odometry update
}

bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);

  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  tfScalar dist_diff = sqrt(pow(current_frame_tf.getOrigin().getX()-last_kf_tf.getOrigin().getX(), 2) + pow(current_frame_tf.getOrigin().getY()-last_kf_tf.getOrigin().getY(), 2));
  bool check_dist = dist_diff > max_keyframes_distance_;

  tfScalar angle_diff = abs(tf::getYaw(current_frame_tf.getRotation()) * 180 / M_PI - tf::getYaw(last_kf_tf.getRotation()) * 180 / M_PI);
  bool check_angle = angle_diff > max_keyframes_angle_;

  double time_diff = current_frame_tf.stamp_.toSec() - last_kf_tf.stamp_.toSec();
  bool check_time = time_diff > max_keyframes_time_;

  return check_dist || check_angle ;//|| check_time;
}

tf::Transform ICPSlam::icpRegistration(const sensor_msgs::LaserScanConstPtr &laser_scan1,
                                      const sensor_msgs::LaserScanConstPtr &laser_scan2,
                                      const tf::Transform &T_2_1)
{ 
  cv::Mat point_mat1 = utils::laserScanToPointMat(laser_scan1);
  cv::Mat point_mat2 = utils::laserScanToPointMat(laser_scan2);
  tf::Transform T_2_1_ = T_2_1;

  int max_iter = 10;
  float pre_mean = 0.0;
  float std_dev, mean;
  float threshold = 0;

  std::vector<int> closest_indices;
  std::vector<float> closest_distances_2;
  
  for (int iter=0; iter<max_iter; iter++)
  {
    cv::Mat transform_point_mat1 = utils::transformPointMat(T_2_1_, point_mat1);

    closestPoints(transform_point_mat1, point_mat2, closest_indices, closest_distances_2); // for each mat 1

    utils::meanAndStdDev<float>(closest_distances_2, mean, std_dev);

    // cv::Mat crp_point_mat2(transform_point_mat1.size(), CV_32F);

    // for (int i=0; i<closest_distances_2.size(); i++)
    //   point_mat2.row(closest_indices[i]).copyTo(crp_point_mat2.row(i));

    // T_2_1_ = icpIteration(point_mat1, crp_point_mat2);
    // std::cout<<"iter "<<iter<<' '<<mean<<'\n';

    // outlier rejection
    int count = 0;
    threshold = mean + 2*std_dev;
    for (int i=0; i<closest_distances_2.size(); i++)
      if (closest_distances_2[i] > threshold)
        count++;

    cv::Mat crp_point_mat1(transform_point_mat1.rows-count, transform_point_mat1.cols, CV_32F);
    cv::Mat crp_point_mat2(transform_point_mat1.rows-count, transform_point_mat1.cols, CV_32F);

    for (int i=0, j=0; i<closest_distances_2.size(); i++)
    {
      if (closest_distances_2[i] <= threshold)
      {
        point_mat2.row(closest_indices[i]).copyTo(crp_point_mat2.row(j));
        point_mat1.row(i).copyTo(crp_point_mat1.row(j));
        j++;
      }
    }

    T_2_1_ = icpIteration(crp_point_mat1, crp_point_mat2);
  }

  // visualize closest points
  cv::Mat transform_point_mat1 = utils::transformPointMat(T_2_1_, point_mat1);
  closestPoints(transform_point_mat1, point_mat2, closest_indices, closest_distances_2);
  cv::Mat crp_point_mat2(transform_point_mat1.size(), CV_32F);

  for (int i=0; i<closest_distances_2.size(); i++)
      point_mat2.row(closest_indices[i]).copyTo(crp_point_mat2.row(i));

  vizClosestPoints(point_mat1, crp_point_mat2, T_2_1_);

  return T_2_1_;
}


tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2)
{ 
//   // overwrite bad points
  std::vector<int> badIndices ;
  std::vector<int> goodIndices ;
  
  for (int j=0; j<point_mat1.rows; j++)
    if(  std::abs( point_mat1.at<float>(0,j) ) > 1e2 ||  std::abs( point_mat1.at<float>(1,j) ) > 1e2  )
      badIndices.push_back( j ) ;
    else goodIndices.push_back( j ) ;
  
  for (int j=0; j<badIndices.size(); j++){
    point_mat1.at<float>(0, badIndices[j] ) =  point_mat1.at<float>(0, goodIndices[j] )  ;
    point_mat1.at<float>(1, badIndices[j] ) =  point_mat1.at<float>(1, goodIndices[j] )  ;
  }

// // std::cout << "badIndices.size() = " << badIndices.size() <<std::endl;
// // std::cout << "goodIndices.size() = " << goodIndices.size() <<std::endl;

  badIndices.clear() ;
  goodIndices.clear() ;
  
  for (int j=0; j<point_mat2.rows; j++)
    if(  std::abs( point_mat2.at<float>(0,j) )> 1e2 ||  std::abs( point_mat2.at<float>(1,j)) > 1e2  )
      badIndices.push_back( j ) ;
    else goodIndices.push_back( j ) ;
  
  for (int j=0; j<badIndices.size(); j++){
    point_mat2.at<float>(0, badIndices[j] ) =  point_mat2.at<float>(0, goodIndices[j] )  ;
    point_mat2.at<float>(1, badIndices[j] ) =  point_mat2.at<float>(1, goodIndices[j] )  ;

  }


  // std::cout << "badIndices.size() = " << badIndices.size() <<std::endl;
  // std::cout << "goodIndices.size() = " << goodIndices.size() <<std::endl;

  tf::Transform T_2_1;
  float mx1, my1, mx2, my2, std_devx1, std_devy1, std_devx2, std_devy2;

  utils::meanAndStdDev<float>(point_mat1.col(0), mx1, std_devx1);
  utils::meanAndStdDev<float>(point_mat1.col(1), my1, std_devy1);
  utils::meanAndStdDev<float>(point_mat2.col(0), mx2, std_devx2);
  utils::meanAndStdDev<float>(point_mat2.col(1), my2, std_devy2);

  cv::Mat p(point_mat1.size(), CV_32F );
  float center1[2] = {mx1, my1};
  for (int i=0; i<point_mat1.cols; i++)
    for (int j=0; j<point_mat1.rows; j++)
    {
      p.at<float>(i,j) = point_mat1.at<float>(i,j) - center1[i];
      //std::cout << "p.at<float>(i,j) " << p.at<float>(i,j)<< std::endl;
    }

  cv::Mat x(point_mat2.size(), CV_32F);
  float center2[2] = {mx2, my2};
  for (int i=0; i<point_mat2.cols; i++)
    for (int j=0; j<point_mat2.rows; j++)
    {
      x.at<float>(i,j) = point_mat2.at<float>(i,j) - center2[i];
      //std::cout << "x.at<float>(i,j) " << x.at<float>(i,j)<< std::endl;
    }


  // float maxValue=0;
  // float minValue=0;

  // for (int i=0; i<p.cols; i++)
  //   for (int j=0; j<p.rows; j++)
  //   {
  //     maxValue = std::max( maxValue,  p.at<float>(i,j) ) ;
  //     minValue = std::min( minValue,  p.at<float>(i,j) ) ;
  //     //std::cout << "p.at<float>(i,j) " << p.at<float>(i,j)<< std::endl;
  //   }
      
      
  // for (int i=0; i<x.cols; i++)
  //   for (int j=0; j<x.rows; j++)
  //   {
  //     maxValue = std::max( maxValue,  x.at<float>(i,j) ) ;
  //     minValue = std::min( minValue,  x.at<float>(i,j) ) ;
  //     //std::cout << "x.at<float>(i,j) " << x.at<float>(i,j)<< std::endl;
  //   }

  // std::cout << "maxValue = " << maxValue <<std::endl;
  // std::cout << "minValue = " << minValue <<std::endl;



  cv::Mat W(2, 2, CV_32F);
  //W =  x.t() * p;       ////////////////////////////////////////////

  W.at<float>(0,0) = 0;
  W.at<float>(0,1) = 0;
  W.at<float>(1,1) = 0;
  W.at<float>(1,0) = 0;

  for (int j=0; j<p.rows; j++)
  {
     W.at<float>(0,0) += x.at<float>(0,j) * p.at<float>(0, j) ;
     W.at<float>(0,1) += x.at<float>(0,j) * p.at<float>(1, j) ;
     W.at<float>(1,1) += x.at<float>(1,j) * p.at<float>(1, j) ;
     W.at<float>(1,0) += x.at<float>(1,j) * p.at<float>(0, j) ;
  }



  cv::SVD svd(W);
  cv::Mat R = svd.u*svd.vt;


  // std::cout << "x.at<float>(0,0) = " << x.at<float>(0,0) << std::endl;
  // std::cout << "x.at<float>(0,1) = " << x.at<float>(0,1) << std::endl;



  // std::cout << "center1[0] = " << center1[0]  << std::endl;
  // std::cout << "center1[1] = " << center1[1]  << std::endl;

  // std::cout << "center2[0] = " << center2[0]  << std::endl;
  // std::cout << "center2[1] = " << center2[1]  << std::endl;



  // std::cout << "W.at<float>(0,0) = " << W.at<float>(0,0) << std::endl;
  // std::cout << "W.at<float>(0,1) = " << W.at<float>(0,1) << std::endl;
  // std::cout << "W.at<float>(1,0) = " << W.at<float>(1,0) << std::endl;
  // std::cout << "W.at<float>(1,1) = " << W.at<float>(1,1) << std::endl;


  // std::cout << "R.at<float>(0,0) = " << R.at<float>(0,0) << std::endl;
  // std::cout << "R.at<float>(0,1) = " << R.at<float>(0,1) << std::endl;
  // std::cout << "R.at<float>(1,0) = " << R.at<float>(1,0) << std::endl;
  // std::cout << "R.at<float>(1,1) = " << R.at<float>(1,1) << std::endl;

  float t_x = center2[0] - (center1[0]*R.at<float>(0,0) + center1[1]*R.at<float>(0,1));
  float t_y = center2[1] - (center1[0]*R.at<float>(1,0) + center1[1]*R.at<float>(1,1));

  // std::cout << "t_x = " << t_x << std::endl;
  // std::cout << "t_y = " << t_y << std::endl;


  T_2_1.setOrigin(tf::Vector3((t_x), (t_y), 0.0));
  T_2_1.setBasis(tf::Matrix3x3((R.at<float>(0,0)), (R.at<float>(0,1)), 0.0,
                              (R.at<float>(1,0)), (R.at<float>(1,1)), 0.0,
                              0.0, 0.0, 1.0));
  return T_2_1;
}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
  }

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1)
{
  assert(point_mat1.size == point_mat2.size);

  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

    cv::Mat img2(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1.inverse(), point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);
    
    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    // std::cout<<point1<<' '<<point2<<'\n';

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1); //r
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1); //b

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2); //g
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  cv::imwrite("./icp_laser.png", img);
}

} // namespace icp_slam