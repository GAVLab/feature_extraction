#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

#include <pcl/console/parse.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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
  PointCloud::Ptr keypoints (new PointCloud);
  copyPointCloud(result, *keypoints);

  std::cout << "keypoints detected: " << keypoints->size() << std::endl;
  std::cout << keypoints->points[0].x << " "<< keypoints->points[0].y << " "<< keypoints->points[0].z << " " << std::endl;


  keypoints->header.frame_id = cloud_msg->header.frame_id;

  pub.publish (keypoints);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "feature_extraction_node");
  ros::NodeHandle nh("~");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("keypoints", 1);

  // Spin
  ros::spin ();
}