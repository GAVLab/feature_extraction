/*
 * Header file for feature extraction node
 *
 * Dan Pierce
 * 2017-06-01
 */
#ifndef _FEATURE_EXTRACTION_NODE_H_
#define _FEATURE_EXTRACTION_NODE_H_
// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
 #include "tf/transform_datatypes.h"
// STL
#include <iostream>
// PCL
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>

/*! /brief Primary class for the feature extraction node class
*
*/
class FeatureExtractionNode
{

    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
    typedef pcl::PointXYZI Point;

  public:

    FeatureExtractionNode();
    ~FeatureExtractionNode();

  private:

    void rotateCloud (const PointCloud &cloud,PointCloud::Ptr transformed_cloud);

    void filterCloud (PointCloud::Ptr cloud);

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    
    // Publishers
    ros::Publisher kp_pub; /*!< keypoint publisher */
    ros::Publisher filt_pub; /*!< keypoint publisher */
    ros::Subscriber pc_sub;
    
    ros::Subscriber imu_sub;
    
    // // Instance of Harris detector
    // pcl::HarrisKeypoint3D<pcl::PointXYZI,pcl::PointXYZI> detector;
  
    int numThreads;
    bool refine, nonMaxSupression;
    double radius,threshold,zMin,zMax;

    double roll,pitch;

};

#endif