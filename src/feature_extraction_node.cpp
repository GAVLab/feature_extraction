#include "feature_extraction/feature_extraction_node.h"

FeatureExtractionNode::FeatureExtractionNode()
{

  ros::NodeHandle nh("~");

  ///////////////////////////////////
  /* Feature Extraction Parameters */
  ///////////////////////////////////
  nh.param("harris_number_of_threads", numThreads, 0);
  nh.param("harris_refine", refine, false);
  nh.param("harris_non_max_supression", nonMaxSupression, true);
  nh.param("harris_radius", radius, 1.0);
  nh.param("harris_threshold", threshold, 0.1);
  nh.param("z_min", zMin, 2.0);
  nh.param("z_max", zMax, 2.0);
  
  std::cout << "Cloud Filter Parameters:" << std::endl;
  std::cout << "    z_min: " << zMin << std::endl;
  std::cout << "    z_max: " << zMax << std::endl;

  std::cout << "Harris Parameters:" << std::endl;
  std::cout << "    number_of_threads: " << numThreads << std::endl;
  std::cout << "    refine: " << refine << std::endl;
  std::cout << "    non_max_supression: " << nonMaxSupression << std::endl;
  std::cout << "    radius: " << radius << std::endl;
  std::cout << "    threshold: " << threshold << std::endl;

  // detector.setNumberOfThreads(numThreads);
  // detector.setRefine(refine);
  // detector.setNonMaxSupression(nonMaxSupression); 
  // detector.setRadius(radius); 
  // detector.setThreshold(threshold); 

  ////////////////////////////////
  /* ROS Publishers/Subscribers */
  ////////////////////////////////
  pc_sub = nh.subscribe ("/velodyne_points", 0, &FeatureExtractionNode::cloudCallback, this);

  kp_pub = nh.advertise<PointCloud> ("keypoints", 0);
  filt_pub = nh.advertise<PointCloud> ("cloud_filt", 0);

}

FeatureExtractionNode::~FeatureExtractionNode()
{
  std::cout << "FeatureExtractionNode::~FeatureExtractionNode" << std::endl;
}

void FeatureExtractionNode::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // definitions
  pcl::PCLPointCloud2 cloud2;
  PointCloud::Ptr cloud(new PointCloud);
  PointCloud::Ptr keypoints(new PointCloud());
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

  // sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
  pcl_conversions::toPCL(*msg,cloud2);

  // pcl::PCLPointCloud2 to PointCloud
  // pcl::fromPCLPointCloud2(cloud2,*cloud);

  // // Filter object.
  // pcl::PassThrough<Point> filter;
  // filter.setInputCloud(cloud);
  // filter.setFilterFieldName("z");
  // filter.setFilterLimits(zMin, zMax);
  // filter.filter(*cloud);

  // filter.setFilterLimitsNegative(true);
  // filter.setFilterFieldName("x");
  // filter.setFilterLimits(-1.0, 1.0);
  // filter.filter(*cloud);

  // filter.setFilterFieldName("y");
  // filter.setFilterLimits(-1.0, 1.0);
  // filter.filter(*cloud);

  // cloud->header.frame_id = msg->header.frame_id;

  cloud->width  = 2;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 4.0f;
    cloud->points[i].y = 3.0f;
    cloud->points[i].z = 2.0f;
    cloud->points[i].intensity = 0.0f;
  }

  std::cout << cloud->points[0].x << " "<< cloud->points[0].y << " "<< cloud->points[0].z << " " << std::endl;

  filt_pub.publish (cloud);


  // Instance of Harris detector
  pcl::HarrisKeypoint3D<Point,Point> detector;
  
  detector.setNumberOfThreads(numThreads);
  detector.setRefine(refine);
  detector.setNonMaxSupression(nonMaxSupression); 
  detector.setRadius(radius); 
  detector.setThreshold(threshold); 
  detector.setInputCloud(cloud);

  detector.compute(*keypoints); 

  std::cout << "keypoints detected: " << keypoints->size() << std::endl;

  // std::cout << keypoints->points[0].x << " "<< keypoints->points[0].y << " "<< keypoints->points[0].z << " " << std::endl;

  keypoints->header.frame_id = msg->header.frame_id;
  // keypoints->header.stamp = msg->header.stamp;

  kp_pub.publish (keypoints);

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "feature_extraction_node");
  FeatureExtractionNode node;

  // Spin
  ros::spin ();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
