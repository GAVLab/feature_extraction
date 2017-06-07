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
  nh.param("cluster_tolerance", clusterTolerance, 1.2);
  nh.param("cluster_min_count", clusterMinCount, 5);
  nh.param("cluster_max_count", clusterMaxCount, 50);
  nh.param("cluster_radius_threshold", descriptorRadius, 2.5);

  ///////////////////////////////////
  /* Feature Descriptor Parameters */
  ///////////////////////////////////
  nh.param("descriptor_radius", clusterRadiusThreshold, 0.5);

  ////////////////////////////////
  /* ROS Publishers/Subscribers */
  ////////////////////////////////
  pc_sub = nh.subscribe ("/velodyne_points", 0, &FeatureExtractionNode::cloudCallback, this);
  imu_sub = nh.subscribe ("/xsens/data", 0, &FeatureExtractionNode::imuCallback, this);

  feature_pub = nh.advertise<DescriptorCloud> ("features", 0);
  kp_pub = nh.advertise<PointCloud> ("keypoints", 0);
  filt_pub = nh.advertise<PointCloud> ("cloud", 0);
  
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
  PointCloud::Ptr cloud_full(new PointCloud);

  pcl::PCLPointCloud2 cloud2;
  pcl_conversions::toPCL(*msg,cloud2); // sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
  pcl::fromPCLPointCloud2(cloud2,*cloud_full); // pcl::PCLPointCloud2 to PointCloud

  pcl::StopWatch watch;
  ////////////////////////
  /* Rotate Point Cloud */
  ////////////////////////
  rotateCloud(cloud_full);

  // if (!init){
  //   init=true;
  //   pcl::io::savePCDFileASCII("output.pcd", *cloud);
  // }

  ////////////////////////
  /* Filter Point Cloud */
  ////////////////////////
  PointCloud::Ptr cloud(new PointCloud);
  *cloud = *cloud_full;
  filterCloud(cloud);

  cloud->header.frame_id = msg->header.frame_id;
  filt_pub.publish (cloud);
  
  ////////////////////////
  /* Keypoint detection */
  ////////////////////////
  PointCloud::Ptr keypoints(new PointCloud());

  estimateKeypoints(cloud,keypoints);

  keypoints->header.frame_id = msg->header.frame_id;
  kp_pub.publish (keypoints);

  /////////////////////
  /* SHOT Descriptor */
  /////////////////////
  DescriptorCloud::Ptr descriptors(new DescriptorCloud());

  estimateDescriptors(cloud,keypoints,descriptors);
  
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
  normalEstimation.setRadiusSearch(normRadius);
  normalEstimation.compute(*normals);
}

void FeatureExtractionNode::estimateKeypoints (const PointCloud::Ptr cloud, PointCloud::Ptr keypoints)
{
  std::vector<pcl::PointIndices> clusters;

  // kd-tree object for searches.
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);
  kdtree->setInputCloud(cloud);

  // Euclidean clustering object.
  pcl::EuclideanClusterExtraction<Point> clustering;

  // Set cluster tolerance to 2cm (small values may cause objects to be divided
  // in several clusters, whereas big values may join objects in a same cluster).
  clustering.setClusterTolerance(clusterTolerance);
  // Set the minimum and maximum number of points that a cluster can have.
  clustering.setMinClusterSize(clusterMinCount);
  clustering.setMaxClusterSize(clusterMaxCount);
  clustering.setSearchMethod(kdtree);
  clustering.setInputCloud(cloud);

  clustering.extract(clusters);

  if (clusters.size()<=0)
    return;

  Point pt_centroid;

  double centerDist,maxCenterDist;
  maxCenterDist = 0.0;

  for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
  {
    // ...add all its points to a new cloud...
    PointCloud::Ptr cluster(new PointCloud);

    for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
      cluster->points.push_back(cloud->points[*point]);
    
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    if (cluster->points.size() > 0){
    
      // Object to store the centroid coordinates.
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);

      for (int ii = 0; ii<cluster->points.size(); ++ii){
        centerDist = pow(pow(cluster->points[ii].x-centroid[0],2)+pow(cluster->points[ii].y-centroid[1],2),0.5);
        if (centerDist > maxCenterDist)
          maxCenterDist = centerDist;
      }
      if (maxCenterDist < clusterRadiusThreshold){
        pt_centroid.x = centroid[0];
        pt_centroid.y = centroid[1];
        pt_centroid.z = centroid[2];
        pt_centroid.intensity = centroid[3];

        keypoints->points.push_back(pt_centroid);
      }

    }

  }
  
}

void FeatureExtractionNode::estimateDescriptors (const PointCloud::Ptr cloud, const PointCloud::Ptr keypoints, DescriptorCloud::Ptr descriptors)
{
  NormalCloud::Ptr normals(new NormalCloud);

  estimateNormals(cloud,normals);

  pcl::SHOTEstimation<Point, Normal, Descriptor> shot;

  shot.setInputCloud(keypoints);
  shot.setSearchSurface(cloud);
  shot.setInputNormals(normals);
  shot.setRadiusSearch(descriptorRadius);

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
