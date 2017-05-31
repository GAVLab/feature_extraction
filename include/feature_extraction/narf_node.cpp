#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

// #include <boost/thread/thread.hpp>
// #include <pcl/range_image/range_image.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/range_image_visualizer.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/features/range_image_border_extractor.h>
// #include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>


// #include <pcl/keypoints/harris_3d.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// --------------------
// -----Parameters-----
// --------------------
// float angular_resolution = 0.5f;
// float support_size = 0.2f;
// bool setUnseenToMaxRange = false;


ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
  PointCloud::Ptr cloud_xyz(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_xyz);

  // Parameters for sift computation
  // const float min_scale = 0.01f;
  // const int n_octaves = 3;
  // const int n_scales_per_octave = 4;
  // const float min_contrast = 0.001f;

  const float min_scale = 0.05f;
  const int n_octaves = 3;
  const int n_scales_per_octave = 4;
  const float min_contrast = 0.005f;
  
  // Estimate the normals of the cloud_xyz
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

  ne.setInputCloud(cloud_xyz);
  ne.setSearchMethod(tree_n);
  // ne.setRadiusSearch(0.2); // original
  ne.setRadiusSearch(0.5);
  ne.compute(*cloud_normals);

  // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i<cloud_normals->points.size(); ++i)
  {
    cloud_normals->points[i].x = cloud_xyz->points[i].x;
    cloud_normals->points[i].y = cloud_xyz->points[i].y;
    cloud_normals->points[i].z = cloud_xyz->points[i].z;
  }

  // Estimate the sift interest points using normals values from xyz as the Intensity variants
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_normals);
  sift.compute(result);

  // Copying the pointwithscale to pointxyz so as visualize the cloud
  PointCloud::Ptr msg (new PointCloud);
  copyPointCloud(result, *msg);


  // pcl::fromPCLPointCloud2(pcl_pc2,*cloud_xyz);
  // pcl::PCLPointCloud2 cloud_temp2;
  // pcl::toPCLPointCloud2(cloud_temp,*cloud_temp2);


  // PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = cloud_msg->header.frame_id;
  // msg->height = msg->width = 1;
  // msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  pub.publish (msg);
  //   ros::spinOnce ();
  //   loop_rate.sleep ();
  // }

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("keypoints", 1);

  // Spin
  ros::spin ();
}