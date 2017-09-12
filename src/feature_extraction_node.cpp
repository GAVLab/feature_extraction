#include "feature_extraction/feature_extraction_node.h"

FeatureExtractionNode::FeatureExtractionNode()
{
  init = false;

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
  nh.param("normal_radius", normRadius, 0.25);

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
  nh.param("estimate_descriptors", descriptorEstimation, true);

  ////////////////////////////////
  /* ROS Publishers/Subscribers */
  ////////////////////////////////
  kp_pub = nh.advertise<PointCloud> ("keypoints", 0);
  filt_pub = nh.advertise<PointCloud> ("cloud_filt", 0);
  norm_pub = nh.advertise<PointNormalCloud> ("normals", 0);
  norm2d_pub = nh.advertise<PointNormalCloud> ("normals2d", 0);
  if (descriptorEstimation)
    feature_pub = nh.advertise< DescriptorCloud > ("features", 0);

  pc_sub = nh.subscribe ("/velodyne_points", 0, &FeatureExtractionNode::cloudCallback, this);
  imu_sub = nh.subscribe ("/xsens/data", 0, &FeatureExtractionNode::imuCallback, this);


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

  if (!init){
    init=true;
    pcl::io::savePCDFileASCII("output.pcd", *cloud);
  }

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
  NormalCloud::Ptr normals2d(new NormalCloud);
  PointNormalCloud::Ptr pt_normals2d(new PointNormalCloud);

  handle2d(cloud,normals,cloud2d,normals2d,pt_normals2d);

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
  if (descriptorEstimation) {
    DescriptorCloud::Ptr descriptors(new DescriptorCloud());

    estimateDescriptors(cloud,normals,keypoints,descriptors);
    
    descriptors->header.frame_id = msg->header.frame_id;
    feature_pub.publish(descriptors);
  }

  // pcl::console::print_highlight ("Extracted %zd points (out of %zd) in %lfs\n", keypoints->size (), cloud->size (), watch.getTimeSeconds ());

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
  normalEstimation.setRadiusSearch(normRadius);
  normalEstimation.compute(*normals);
}

void FeatureExtractionNode::handle2d (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr cloud2d, NormalCloud::Ptr normals2d, PointNormalCloud::Ptr pt_normals2d)
{
  PointCloud::Ptr cloud2d_lo(new PointCloud);
  PointCloud::Ptr cloud2d_hi(new PointCloud);
  PointCloud::Ptr cloud2d_full(new PointCloud);
  NormalCloud::Ptr normals2d_full(new NormalCloud);

  *cloud2d = *cloud; // set all fields equal to 3D cloud
  *cloud2d_lo = *cloud; // same for clone copy
  *cloud2d_hi = *cloud; // same for clone copy

  float zNom = (zMin+zMax)/2.0;
  float zLo = zNom - normRadius/2.0;
  float zHi = zNom + normRadius/2.0;
  
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud2d->points[i].z = zNom;
    cloud2d_lo->points[i].z = zLo;
    cloud2d_hi->points[i].z = zHi;
  }

  *cloud2d_full = *cloud2d + *cloud2d_lo;

  *cloud2d_full = *cloud2d_full + *cloud2d_hi;

  estimateNormals(cloud2d_full,normals2d_full);

  for(size_t i = 0; i<cloud2d->points.size(); ++i){normals2d->points.push_back(normals2d_full->points[i]);}

  pcl::concatenateFields(*cloud2d, *normals2d, *pt_normals2d);
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

  // -----
  // kdtree->setInputCloud(cloud); // do I need this or did detector handle it?
  
  // // kdtree->setInputCloud(keypoints); // do I need this or did detector handle it?
  // // kdtree->setSearchCloud(cloud);
  
  // std::vector<int> indices;
  // std::vector<float> squaredDistances;
  
  // // Placeholder for the 3x3 covariance matrix at each surface patch
  // Eigen::Matrix< float, 3, 3 > covariance_matrix;
  // // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  // Eigen::Vector4f xyz_centroid;
  
  // for(size_t i = 0; i<cloud->points.size(); ++i){
  //   if (kdtree->radiusSearch(cloud->points[i], kpRadius, indices, squaredDistances) > 0)
  //   {
  //     PointCloud::Ptr cloud_local(new PointCloud);

  //     // computePointNormal (*cloud, indices, Eigen::Vector4f &plane_parameters, float &curvature);
  //     for(size_t j = 0; j<indices.size(); ++j)
  //       cloud_local->points.push_back(cloud->points[j]);

  //     // Estimate the XYZ centroid
  //     pcl::compute3DCentroid (*cloud_local, xyz_centroid);
  //     // Compute the 3x3 covariance matrix
  //     pcl::computeCovarianceMatrix (*cloud_local, xyz_centroid, covariance_matrix);
      
  //   }
  // }

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
