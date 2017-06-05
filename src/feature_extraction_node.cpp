#include "feature_extraction/feature_extraction_node.h"

FeatureExtractionNode::FeatureExtractionNode()
{

  ros::NodeHandle nh("~");

  /////////////////////////////
  /* Cloud Filter Parameters */
  /////////////////////////////
  nh.param("x_min", xMin, 0.0);
  nh.param("x_max", xMax, 100.0);
  nh.param("y_min", yMin, -50.0);
  nh.param("y_max", yMax, 50.0);
  nh.param("z_min", zMin, -1.15);
  nh.param("z_max", zMax, 5.0);
  std::cout << "Cloud Filter Parameters:" << std::endl;
  std::cout << "    x_min: " << xMin << std::endl;
  std::cout << "    x_max: " << xMax << std::endl;
  std::cout << "    y_min: " << yMin << std::endl;
  std::cout << "    y_max: " << yMax << std::endl;
  std::cout << "    z_min: " << zMin << std::endl;
  std::cout << "    z_max: " << zMax << std::endl;

  //////////////////////////////////
  /* Normal Estimation Parameters */
  //////////////////////////////////
  nh.param("normal_number_neighbors", nnNormal, 5);
  std::cout << "Normal Estimation Parameters:" << std::endl;
  std::cout << "    normal_number_neighbors: " << nnNormal << std::endl;

  ///////////////////////////////////
  /* Keypoint Detection Parameters */
  ///////////////////////////////////
  nh.param("harris_number_of_threads", numThreads, 0);
  nh.param("harris_refine", refine, false);
  nh.param("harris_non_max_supression", nonMaxSupression, true);
  nh.param("harris_radius", keypointRadius, 1.0);
  nh.param("harris_threshold", threshold, 0.1);

  std::cout << "Keypoint Detection Parameters:" << std::endl;
  std::cout << "    number_of_threads: " << numThreads << std::endl;
  std::cout << "    refine: " << refine << std::endl;
  std::cout << "    non_max_supression: " << nonMaxSupression << std::endl;
  std::cout << "    threshold: " << threshold << std::endl;

  ///////////////////////////////////
  /* Feature Descriptor Parameters */
  ///////////////////////////////////
  std::cout << "Feature Descriptor Parameters:" << std::endl;


  ////////////////////////////////
  /* ROS Publishers/Subscribers */
  ////////////////////////////////
  pc_sub = nh.subscribe ("/velodyne_points", 0, &FeatureExtractionNode::cloudCallback, this);
  imu_sub = nh.subscribe ("/xsens/data", 0, &FeatureExtractionNode::imuCallback, this);

  feature_pub = nh.advertise<pcl::PointCloud<pcl::SHOT352> > ("features", 0);
  kp_pub = nh.advertise<PointCloud> ("keypoints", 0);
  filt_pub = nh.advertise<PointCloud> ("cloud_filt", 0);

}

FeatureExtractionNode::~FeatureExtractionNode()
{
  std::cout << "FeatureExtractionNode::~FeatureExtractionNode" << std::endl;
}

void FeatureExtractionNode::imuCallback (const sensor_msgs::ImuConstPtr& msg)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double yaw,tmproll;
  tf::Matrix3x3(quat).getRPY(tmproll, pitch, yaw);
  roll = tmproll - M_PI;

}

void FeatureExtractionNode::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{

  /////////////////
  /* Conversions */
  /////////////////

  // definitions
  pcl::PCLPointCloud2 cloud2;
  PointCloud::Ptr cloud_in(new PointCloud);
  
  // sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
  pcl_conversions::toPCL(*msg,cloud2);

  // pcl::PCLPointCloud2 to PointCloud
  pcl::fromPCLPointCloud2(cloud2,*cloud_in);

  
  PointCloud::Ptr cloud(new PointCloud);
  rotateCloud(*cloud_in,cloud);


  ////////////////////////
  /* Filter Point Cloud */
  ////////////////////////
  filterCloud(cloud);

  cloud->header.frame_id = msg->header.frame_id;
  
  filt_pub.publish (cloud);

  /////////////////////////////
  /* Point Normal Estimation */
  /////////////////////////////
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  // Estimate the normals.
  pcl::NormalEstimation<Point, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.setKSearch(nnNormal);
  normalEstimation.compute(*normals);

  ////////////////////////
  /* Keypoint detection */
  ////////////////////////
  PointCloud::Ptr keypoints(new PointCloud());

  pcl::HarrisKeypoint3D <Point, Point> detector;
  
  detector.setSearchMethod(kdtree); // set pointer to the kD tree
  detector.setKSearch(nnKeypoint); // number of neighbors to gather (for normal estimation or non max suppression?)
  detector.setNormals(normals);

  detector.setNumberOfThreads(numThreads);
  detector.setRefine(refine);
  detector.setNonMaxSupression(nonMaxSupression); 
  detector.setRadius(keypointRadius); 

  detector.setThreshold(threshold); 
  detector.setInputCloud(cloud);
 
  pcl::StopWatch watch;
  detector.compute (*keypoints);
  // pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices ();
  
  keypoints->header.frame_id = msg->header.frame_id;
  // // keypoints->header.stamp = msg->header.stamp;
  kp_pub.publish (keypoints);

  /////////////////////
  /* SHOT Descriptor */
  /////////////////////
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

  pcl::SHOTEstimation<Point, pcl::Normal, pcl::SHOT352> shot;

  shot.setInputCloud(keypoints);
  shot.setSearchSurface(cloud);
  shot.setInputNormals(normals);

  // The radius that defines which of the keypoint's neighbors are described.
  // If too large, there may be clutter, and if too small, not enough points may be found.
  shot.setRadiusSearch(0.5);

  //shot.setKSearch(50);
  
  shot.compute(*descriptors);

  std::cout << "descriptor size: " << descriptors->size() << std::endl;
  std::cout << "cloud size: " << cloud->size() << std::endl;
  
  pcl::console::print_highlight ("Extracted %zd points (out of %zd) in %lfs\n", keypoints->size (), cloud->size (), watch.getTimeSeconds ());
  
  std::cout << descriptors->points[0] << std::endl;
  
  descriptors->header.frame_id = msg->header.frame_id;

  feature_pub.publish(descriptors);

}

void FeatureExtractionNode::rotateCloud (const PointCloud &cloud, PointCloud::Ptr transformed_cloud)
{

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of 0.0 meters on the x, y, and z axis.
  transform.translation() << 0.0, 0.0, 0.0;

  // The rotation matrix theta radians arround Z axis
  transform.rotate (  Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY())*
                      Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX())   );

  // Executing the transformation
  pcl::transformPointCloud (cloud, *transformed_cloud, transform);
}

void FeatureExtractionNode::filterCloud (PointCloud::Ptr cloud){

// Filter out distance points
pcl::PassThrough<Point> filter;
filter.setInputCloud(cloud);
filter.setFilterFieldName("z");
filter.setFilterLimits(zMin, zMax);
filter.filter(*cloud);

filter.setFilterFieldName("y");
filter.setFilterLimits(yMin, yMax);
filter.filter(*cloud);

filter.setFilterFieldName("x");
filter.setFilterLimits(xMin, xMax);
filter.filter(*cloud);

// For Close range objects
filter.setFilterLimitsNegative(true);
filter.setFilterFieldName("x");
filter.setFilterLimits(-1.0, 1.0);
filter.filter(*cloud);

filter.setFilterFieldName("y");
filter.setFilterLimits(-1.0, 1.0);
filter.filter(*cloud);

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
