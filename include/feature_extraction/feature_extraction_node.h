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
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/features/spin_image.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>


/*! /brief Primary class for the feature extraction node class
*
*/
class FeatureExtractionNode
{

  public:

    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::Normal Normal;
    typedef pcl::PointCloud<Normal> NormalCloud;

    typedef pcl::PointXYZINormal PointNormal;
    typedef pcl::PointCloud<PointNormal> PointNormalCloud;

    typedef pcl::SHOT352 Descriptor;
    typedef pcl::PointCloud<Descriptor> DescriptorCloud;

    typedef pcl::Histogram<153> SpinImage;
    typedef pcl::PointCloud<SpinImage> SpinImageCloud;
    
    FeatureExtractionNode();
    ~FeatureExtractionNode();

    void filterCloud (PointCloud::Ptr cloud);

  private:

    void printRosParameters (void);

    void rotateCloud (PointCloud::Ptr cloud);

    void estimateNormals (const PointCloud::Ptr cloud, NormalCloud::Ptr normals);

    void estimateKeypoints (const PointCloud::Ptr cloud, PointCloud::Ptr keypoints);

    void estimateDescriptors (const PointCloud::Ptr cloud, const PointCloud::Ptr keypoints, DescriptorCloud::Ptr descriptors);

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    
    // --- Publisher
    ros::Publisher feature_pub;         // Feature publisher
    ros::Publisher kp_pub;              // Keypoint publisher
    ros::Publisher filt_pub;            // Filtered point cloud publisher

    // --- Subscribers
    ros::Subscriber pc_sub;             // Point cloud subscriber
    ros::Subscriber imu_sub;            // IMU subscriber (for roll/pitch)
    
    // --- Class variables
    // Pass through filter
    double zMin,zMax,xMin,xMax,yMin,yMax; // Bounds of point cloud pass through filter
    double roll,pitch;                  // Roll/pitch estimate for rotating point cloud to local-level frame
    // Normal estimation
    double normRadius;                  // neighbors within this radius of keypoint used to estimate surface normal
    // Detector
    double clusterTolerance;            // Tolerance for point cloud segmentation (as a measure in L2 Euclidean space)
    int clusterMinCount;                 // Minimum number of points in a cluster
    int clusterMaxCount;                // Maximum number of points in a cluster
    
    double clusterRadiusThreshold;      

    // Descriptor
    double descriptorRadius;
    // Other
    bool init;
    
};

#endif