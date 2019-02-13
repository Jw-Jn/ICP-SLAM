//
// Created by rakesh on 27/08/18.
//

#include "../include/icp_slam/mapper.h"
#include "../include/icp_slam/utils.h"

namespace icp_slam
{
    
Mapper::Mapper(){
    is_initialized_ = 0;
}

cv::Mat Mapper::getMapCopy(){
    cv::Mat res(width_, height_, CV_8UC1);
    for (int i = 0; i < width_; i++) {
        for (int j = 0; j < height_; j++) {
            if (map_.at<int32_t>(i, j) == -1) {
                res.at<uchar>(i,j) = 125;
            } else if (map_.at<int32_t>(i, j) > 200) {
                res.at<uchar>(i,j) = 0;
            } else {
                res.at<uchar>(i,j) = 255;
            }
        }
    }
    return res;
}
void Mapper::initMap(int width, int height, float resolution,
               double origin_x_meters, double origin_y_meters,
               uint8_t *pointer, unsigned char unknown_cost_value){
                map_ = cv::Mat(height, width, CV_32SC1,-1);
                height_ = height;
                width_ = width;
                resolution_ = resolution;
                origin_x_ = width/2;
                origin_y_ = height/2;
                is_initialized_ = 1;
               }

int Mapper::updateMap(const sensor_msgs::LaserScanConstPtr &laser_scan,
                const tf::StampedTransform &pose) {
    // ROS_INFO("1");
    cv::Mat origin_location(3, 1, CV_32F);
    origin_location.at<float>(0) = origin_x_;
    origin_location.at<float>(1) = origin_y_;
    origin_location.at<float>(2) = 1.0;
    // ROS_INFO("origin loc: (%f, %f, %f)", origin_location.at<float>(0), origin_location.at<float>(1), origin_location.at<float>(2));
    auto T = utils::transformToMatrix(pose, resolution_);
    auto T1 = utils::transformToTransformationMatrix(pose, resolution_);
    float cosTheta = T.at<float>(0,0);
    float sinTheta = T.at<float>(1,0);
    float deg = utils::radianToDegree<float>(std::atan2(sinTheta, cosTheta));


    // ROS_INFO("2");
    cv::Mat origin = T1 * origin_location;
    cv::Mat laser_locations = utils::laserScanToPointMat(laser_scan, resolution_);
    // ROS_INFO("laser scan size: %d %d", laser_locations.rows, laser_locations.cols);
    // ROS_INFO("laser scan data %f, %f", laser_locations.at<float>(0), laser_locations.at<float>(1));
    // ROS_INFO("3");
    cv::Mat map_laser_location = utils::transformPointMat(pose, laser_locations);

    // ROS_INFO("4");
    cv::Mat temp_map_locs, temp_origin;
    origin.convertTo(temp_origin, CV_32SC1);
    map_laser_location.convertTo(temp_map_locs, CV_32SC1);
    // ROS_INFO("5");
    // ROS_INFO("size: %d %d", T.rows, T.cols);
    // for (int k = 0; k < 3; k++) {
    //     ROS_INFO("T %d: %f, %f, %f ", k, T.at<float>(k, 0), T.at<float>(k, 1), T.at<float>(k, 2));
    // }
    // ROS_INFO("deg: %f", deg);
    // ROS_INFO("origin after transform: (%f, %f, %f)", origin.at<float>(0), origin.at<float>(1), origin.at<float>(2));
    for (int i = 0; i < map_laser_location.size().height; i++){
        // ROS_INFO("origin %d : (%d, %d)", i, temp_origin.at<int32_t>(0), temp_origin.at<int32_t>(1));
        // ROS_INFO("locs %d : (%d, %d)", i, temp_map_locs.at<int32_t>(i, 0), temp_map_locs.at<int32_t>(i, 1));
        Mapper::drawScanLine(temp_origin.at<int32_t>(0), temp_origin.at<int32_t>(1), temp_map_locs.at<int32_t>(i, 0),temp_map_locs.at<int32_t>(i, 1));
    }
    // ROS_INFO("6");

}

// int updateLaserScan(const sensor_msgs::LaserScanConstPtr &laser_scan, robot_pose_t robot_pose){
    
// }


int Mapper::drawScanLine(int x1, int y1, int x2, int y2){
    cv::LineIterator it(map_, cv::Point(x1, y1), cv::Point(x1+x2, y1+y2));

    for(int i = 0; i < it.count-1; i++, ++it) {
        cv::Point point = it.pos(); // (point.x, point.y)
        // ROS_INFO("%d %d", point.x, point.y);
        if(map_.at<int32_t>(point.x, point.y) == -1){
            map_.at<int32_t>(point.x, point.y) = 0;
        }
        map_.at<int32_t>(point.x, point.y) += 0;
    }
    map_.at<int32_t>(x1+x2, y1+y2) += 100;
    // map_.at<int32_t>(x1+x2+1, y1+y2) += 50;
    // map_.at<int32_t>(x1+x2, y1+y2+1) += 50;
    // map_.at<int32_t>(x1+x2-1, y1+y2) += 50;
    // map_.at<int32_t>(x1+x2, y1+y2-1) += 50;
    // if (map_.at<int32_t>(x1+x2, y1+y2) >= 100)
    //     map_.at<int32_t>(x1+x2, y1+y2) = 100;
}

int Mapper::convertToGridCoords(double x, double y, int &grid_x, int &grid_y){

}

int Mapper::convertToWorldCoords(int grid_x, int grid_y, double &x, double &y){

}

// static robot_pose_t Mapper::poseFromGeometryPoseMsg(const geometry_msgs::Pose &pose_msg){

// }

// static robot_pose_t Mapper::poseFromTf(const tf::StampedTransform &tf_pose){

// }


} // namespace icp_slam
