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

ros::Publisher pub;
ros::Publisher ri_pub;

pcl::visualization::RangeImageVisualizer viewer("Range image");

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // std::cout << "here" << std::endl;

  // Object for storing the keypoints' indices.
  pcl::PointCloud<int>::Ptr keypoints_idx(new pcl::PointCloud<int>);
  PointCloud::Ptr cloud(new PointCloud);
  
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

  
// Parameters needed by the range image object:
// Angular resolution is the angular distance between pixels.
// Kinect: 57° horizontal FOV, 43° vertical FOV, 640x480 (chosen here).
// Xtion: 58° horizontal FOV, 45° vertical FOV, 640x480.
// Velodyne: 360° azimuth FOV, ~7200 points per scan = ~450 points per channel = ~0.8 degrees / point
// Velodyne: 30° elevation FOV, 16 channels = ~2 degrees / channel

// float angularResolutionX = (float)(57.0f / 640.0f * (M_PI / 180.0f));
// float angularResolutionY = (float)(43.0f / 480.0f * (M_PI / 180.0f));

float angularResolutionX = (float)(0.8f * (M_PI / 180.0f));
float angularResolutionY = (float)(2.0f * (M_PI / 180.0f));

// float angularResolutionX = (float)(0.1f * (M_PI / 180.0f)); // divide by 8
// float angularResolutionY = (float)(0.25f * (M_PI / 180.0f));

// Maximum horizontal and vertical angles. For example, for a full panoramic scan,
// the first would be 360º. Choosing values that adjust to the real sensor will
// decrease the time it takes, but don't worry. If the values are bigger than
// the real ones, the image will be automatically cropped to discard empty zones.

float maxAngleX = (float)(360.0f * (M_PI / 180.0f));
float maxAngleY = (float)(30.0f * (M_PI / 180.0f));

// float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
// float maxAngleY = (float)(50.0f * (M_PI / 180.0f));

// Sensor pose. Thankfully, the cloud includes the data.
Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
cloud->sensor_origin_[1],
cloud->sensor_origin_[2])) *
Eigen::Affine3f(cloud->sensor_orientation_);

// Noise level. If greater than 0, values of neighboring points will be averaged.
// This would set the search radius (e.g., 0.03 == 3cm).
float noiseLevel = 0.0f;

// Minimum range. If set, any point closer to the sensor than this will be ignored.
float minimumRange = 0.0f;

// Border size. If greater than 0, a border of "unobserved" points will be left
// in the image when it is cropped.
int borderSize = 1;

// Range image object.
// pcl::RangeImage rangeImage;
pcl::RangeImageSpherical rangeImage;

rangeImage.createFromPointCloud(*cloud, angularResolutionX, angularResolutionY,
                                maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                noiseLevel, minimumRange, borderSize);
rangeImage.setUnseenToMaxRange ();


// Visualize the image.
viewer.showRangeImage(rangeImage);

viewer.spinOnce();
// Sleep 100ms to go easy on the CPU.
pcl_sleep(0.1);




// Border extractor object.
pcl::RangeImageBorderExtractor borderExtractor;
// Keypoint detection object.
pcl::NarfKeypoint detector(&borderExtractor);
detector.setRangeImage(&rangeImage);
detector.getParameters().support_size = 0.2f;

detector.compute(*keypoints_idx);

// PointCloud::Ptr keypoints_ptr (new PointCloud);
// PointCloud& keypoints = *keypoints_ptr;
// keypoints.points.resize (keypoints_idx->points.size ());
// for (size_t i=0; i<keypoints_idx->points.size (); ++i)
//   keypoints.points[i].getVector3fMap () = rangeImage.points[keypoints_idx->points[i]].getVector3fMap ();

PointCloud::Ptr keypoints (new PointCloud);
keypoints->points.resize (keypoints_idx->points.size ());
for (size_t i=0; i<keypoints_idx->points.size (); ++i)
  keypoints->points[i].getVector3fMap () = rangeImage.points[keypoints_idx->points[i]].getVector3fMap ();


  std::cout << "keypoints detected: " << keypoints->size() << std::endl;

  keypoints->header.frame_id = cloud_msg->header.frame_id;

  pub.publish (keypoints);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "narf_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 0, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("keypoints", 0);
  ri_pub = nh.advertise<sensor_msgs::Image> ("range_image", 0);

  // Spin
  ros::spin ();
}