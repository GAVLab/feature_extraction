// ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/range_image/range_image_spherical.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud_i;

ros::Publisher pub;
ros::Publisher ri_pub;

pcl::visualization::RangeImageVisualizer viewer("Range image");

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{

  pcl::PCLPointCloud2 cloud2;
  PointCloud::Ptr cloud(new PointCloud);
  // sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
  pcl_conversions::toPCL(*msg,cloud2);

  // pcl::PCLPointCloud2 to PointCloud
  pcl::fromPCLPointCloud2(cloud2,*cloud);

  float angularResolutionX = (float)(0.8f * (M_PI / 180.0f));
  float angularResolutionY = (float)(2.0f * (M_PI / 180.0f));

  float maxAngleW = (float)(180.0f * (M_PI / 180.0f));
  float maxAngleH = (float)(30.0f * (M_PI / 180.0f));


// Sensor pose. Thankfully, the cloud includes the data.
// Eigen::Affine3f sensorPose = //Eigen::Affine3f(cloud->sensor_orientation_);
// Eigen::Affine3f(Eigen::AngleAxisf ((45*M_PI/180.0f), Eigen::Vector3f::UnitZ()));
Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity ();


// Noise level. If greater than 0, values of neighboring points will be averaged.
// This would set the search radius (e.g., 0.03 == 3cm).
float noiseLevel = 0.0f;

// Minimum range. If set, any point closer to the sensor than this will be ignored.
float minimumRange = 0.0f;

// Border size. If greater than 0, a border of "unobserved" points will be left
// in the image when it is cropped.
int borderSize = 0;

// Range image object.
pcl::RangeImage rangeImage;
// pcl::RangeImageSpherical rangeImage;

rangeImage.createFromPointCloud(*cloud, angularResolutionX, maxAngleW, maxAngleH, sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                noiseLevel, minimumRange, borderSize);
rangeImage.setUnseenToMaxRange ();


// Visualize the image.
viewer.showRangeImage(rangeImage);

viewer.spinOnce();
// // Sleep 100ms to go easy on the CPU.
pcl_sleep(0.1);




pcl::PointCloud<int>::Ptr keypoints_idx(new pcl::PointCloud<int>);
// Border extractor object.
pcl::RangeImageBorderExtractor borderExtractor;
// Keypoint detection object.
pcl::NarfKeypoint detector(&borderExtractor);
detector.setRangeImage(&rangeImage);
detector.getParameters().support_size = 0.2f;

detector.compute(*keypoints_idx);



PointCloud::Ptr keypoints (new PointCloud);
keypoints->points.resize (keypoints_idx->points.size ());
for (size_t i=0; i<keypoints_idx->points.size (); ++i)
  keypoints->points[i].getVector3fMap () = rangeImage.points[keypoints_idx->points[i]].getVector3fMap ();


  std::cout << "keypoints detected: " << keypoints->size() << std::endl;

  keypoints->header.frame_id = cloud->header.frame_id;

  pub.publish (keypoints);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "narf_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/feature_extraction_node/cloud_filt", 0, cloud_cb);
  // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 0, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("keypoints", 0);
  ri_pub = nh.advertise<sensor_msgs::Image> ("range_image", 0);

  // Spin
  ros::spin ();
}