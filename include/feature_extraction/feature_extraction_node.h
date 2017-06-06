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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>


#include <pcl/io/pcd_io.h>

/*! /brief Primary class for the feature extraction node class
*
*/
class FeatureExtractionNode
{

    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::Normal Normal;
    typedef pcl::PointCloud<Normal> NormalCloud;

    typedef pcl::PointXYZINormal PointNormal;
    typedef pcl::PointCloud<PointNormal> PointNormalCloud;

    typedef pcl::SHOT352 Descriptor;
    typedef pcl::PointCloud<Descriptor> DescriptorCloud;
    
  public:

    FeatureExtractionNode();
    ~FeatureExtractionNode();

  private:

    void printRosParameters (void);

    // void rotateCloud (const PointCloud &cloud, PointCloud::Ptr transformed_cloud);
    void rotateCloud (PointCloud::Ptr cloud);

    void filterCloud (PointCloud::Ptr cloud);

    void estimateNormals (const PointCloud::Ptr cloud, NormalCloud::Ptr normals);

    void handle2d (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointNormalCloud::Ptr pt_normals2d);

    void estimateKeypoints (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr keypoints);

    void estimateOtherKeypoints (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr keypoints);

    void estimateDescriptors (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, const PointCloud::Ptr keypoints, DescriptorCloud::Ptr descriptors);

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    
    // --- Publisher
    ros::Publisher feature_pub;         // Feature publisher
    ros::Publisher kp_pub;              // Keypoint publisher
    ros::Publisher filt_pub;            // Filtered point cloud publisher
    ros::Publisher norm_pub;           
    ros::Publisher norm2d_pub;           

    // --- Subscribers
    ros::Subscriber pc_sub;             // Point cloud subscriber
    ros::Subscriber imu_sub;            // IMU subscriber (for roll/pitch)
    
    // --- Class variables
    // Pass through filter
    double zMin,zMax,xMin,xMax,yMin,yMax; // Bounds of point cloud pass through filter
    double roll,pitch;                  // Roll/pitch estimate for rotating point cloud to local-level frame
    // Normal estimation
    double normRadius;                       // number of neighbors used to estimate surface normal
    // Detector
    int kpNumThreads;                   // number of threads in calculating harris keypoints
    bool kpRefine;                      // keypoint refine boolean
    bool kpNonMaxSupression;            // keypoint detection non max supression boolean
    double kpThreshold;                 // keypoint detection threshold for non max supression 
    double kpRadius;                    // radius (in meters) for gathering neighbors
    
    bool init;
    // Descriptor


};

#endif