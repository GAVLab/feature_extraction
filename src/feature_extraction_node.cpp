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

  //////////////////////////////////
  /* Normal Estimation Parameters */
  //////////////////////////////////
  nh.param("normal_number_neighbors", nnNormal, 5);

  ///////////////////////////////////
  /* Keypoint Detection Parameters */
  ///////////////////////////////////
  nh.param("harris_number_of_threads", kpNumThreads, 0);
  nh.param("harris_refine", kpRefine, false);
  nh.param("harris_non_max_supression", kpNonMaxSupression, true);
  nh.param("harris_radius", kpRadius, 1.0);
  nh.param("harris_threshold", kpThreshold, 0.1);

  ///////////////////////////////////
  /* Feature Descriptor Parameters */
  ///////////////////////////////////


  ////////////////////////////////
  /* ROS Publishers/Subscribers */
  ////////////////////////////////
  pc_sub = nh.subscribe ("/velodyne_points", 0, &FeatureExtractionNode::cloudCallback, this);
  imu_sub = nh.subscribe ("/xsens/data", 0, &FeatureExtractionNode::imuCallback, this);

  feature_pub = nh.advertise< DescriptorCloud > ("features", 0);
  kp_pub = nh.advertise<PointCloud> ("keypoints", 0);
  filt_pub = nh.advertise<PointCloud> ("cloud_filt", 0);
  norm_pub = nh.advertise<PointNormalCloud> ("normals", 0);
  norm2d_pub = nh.advertise<PointNormalCloud> ("normals2d", 0);

}

FeatureExtractionNode::~FeatureExtractionNode()
{
  std::cout << "FeatureExtractionNode::~FeatureExtractionNode" << std::endl;
}

void FeatureExtractionNode::imuCallback (const sensor_msgs::ImuConstPtr& msg)
{
  tf::Quaternion quat;
  double yaw,tmproll;
  
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(tmproll, pitch, yaw);
  roll = tmproll - M_PI;
}

void FeatureExtractionNode::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{

  ////////////////////////////
  /* Point Cloud Conversion */
  ////////////////////////////
  PointCloud::Ptr cloud(new PointCloud);

  pcl::PCLPointCloud2 cloud2;
  pcl_conversions::toPCL(*msg,cloud2); // sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
  pcl::fromPCLPointCloud2(cloud2,*cloud); // pcl::PCLPointCloud2 to PointCloud

  pcl::StopWatch watch;
  ////////////////////////
  /* Rotate Point Cloud */
  ////////////////////////
  rotateCloud(cloud);

  ////////////////////////
  /* Filter Point Cloud */
  ////////////////////////
  filterCloud(cloud);

  cloud->header.frame_id = msg->header.frame_id;
  filt_pub.publish (cloud);

  /////////////////////////////
  /* Point Normal Estimation */
  /////////////////////////////
  NormalCloud::Ptr normals(new NormalCloud);

  estimateNormals(cloud,normals);

  PointNormalCloud::Ptr pt_normals(new PointNormalCloud);

  pcl::concatenateFields(*cloud, *normals, *pt_normals);

  pt_normals->header.frame_id = msg->header.frame_id;
  norm_pub.publish (pt_normals);
  
  ////////////////////
  /* 2D Point Cloud */
  ////////////////////
  PointCloud::Ptr cloud2d(new PointCloud);
  *cloud2d = *cloud;

  for(size_t i = 0; i<cloud->points.size(); ++i)
  {
    cloud2d->points[i].z = (zMin+zMax)/2.0;
  }

  NormalCloud::Ptr normals2d(new NormalCloud);

  estimateNormals(cloud2d,normals2d);

  PointNormalCloud::Ptr pt_normals2d(new PointNormalCloud);

  pcl::concatenateFields(*cloud2d, *normals2d, *pt_normals2d);

  pt_normals2d->header.frame_id = msg->header.frame_id;
  norm2d_pub.publish (pt_normals2d);
  
  ////////////////////////
  /* Keypoint detection */
  ////////////////////////
  PointCloud::Ptr keypoints(new PointCloud());

  estimateKeypoints(cloud,normals,keypoints);

  keypoints->header.frame_id = msg->header.frame_id;
  kp_pub.publish (keypoints);

  /////////////////////
  /* SHOT Descriptor */
  /////////////////////
  DescriptorCloud::Ptr descriptors(new DescriptorCloud());

  estimateDescriptors(cloud,normals,keypoints,descriptors);
  
  descriptors->header.frame_id = msg->header.frame_id;
  feature_pub.publish(descriptors);

  pcl::console::print_highlight ("Extracted %zd points (out of %zd) in %lfs\n", keypoints->size (), cloud->size (), watch.getTimeSeconds ());

}

void FeatureExtractionNode::rotateCloud (PointCloud::Ptr cloud)
{
  PointCloud transformed_cloud;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
 
  transform.translation() << 0.0, 0.0, 0.0;
 
  transform.rotate (  Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY())*
                      Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX())   );
 
  pcl::transformPointCloud (*cloud, transformed_cloud, transform);
 
  *cloud = transformed_cloud;
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

void FeatureExtractionNode::estimateNormals (const PointCloud::Ptr cloud,NormalCloud::Ptr normals)
{
  pcl::NormalEstimation<Point, Normal> normalEstimation;
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);

  normalEstimation.setInputCloud(cloud);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.setKSearch(nnNormal);
  normalEstimation.compute(*normals);
}

void FeatureExtractionNode::estimateKeypoints (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr keypoints)
{
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);
  pcl::HarrisKeypoint3D <Point, Point> detector;
  
  detector.setSearchMethod(kdtree); // set pointer to the kD tree
  //detector.setKSearch(nnKeypoint); // number of neighbors to gather (for normal estimation or non max suppression?)
  detector.setNormals(normals);

  detector.setNumberOfThreads(kpNumThreads);
  detector.setRefine(kpRefine);
  detector.setNonMaxSupression(kpNonMaxSupression); 
  detector.setRadius(kpRadius); 

  detector.setThreshold(kpThreshold); 
  detector.setInputCloud(cloud);
 
  detector.compute (*keypoints);

}

void FeatureExtractionNode::estimateOtherKeypoints (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr keypoints)
{

}

void FeatureExtractionNode::estimateDescriptors (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, const PointCloud::Ptr keypoints, DescriptorCloud::Ptr descriptors)
{
  pcl::SHOTEstimation<Point, Normal, Descriptor> shot;

  // pcl::SHOTEstimation<Point, pcl::PointXYZINormal, Descriptor> shot2;

  shot.setInputCloud(keypoints);
  shot.setSearchSurface(cloud);
  shot.setInputNormals(normals);
  shot.setRadiusSearch(0.5);

  //shot.setKSearch(50);
  
  shot.compute(*descriptors);
}

void FeatureExtractionNode::printRosParameters (void)
{
  // std::cout << "Cloud Filter Parameters:" << std::endl;
  // std::cout << "    x_min: " << xMin << std::endl;
  // std::cout << "    x_max: " << xMax << std::endl;
  // std::cout << "    y_min: " << yMin << std::endl;
  // std::cout << "    y_max: " << yMax << std::endl;
  // std::cout << "    z_min: " << zMin << std::endl;
  // std::cout << "    z_max: " << zMax << std::endl;
  // std::cout << "Normal Estimation Parameters:" << std::endl;
  // std::cout << "    normal_number_neighbors: " << nnNormal << std::endl;
  // std::cout << "Keypoint Detection Parameters:" << std::endl;
  // std::cout << "    number_of_threads: " << numThreads << std::endl;
  // std::cout << "    refine: " << refine << std::endl;
  // std::cout << "    non_max_supression: " << nonMaxSupression << std::endl;
  // std::cout << "    threshold: " << threshold << std::endl;
  // std::cout << "Feature Descriptor Parameters:" << std::endl;
  return;
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
