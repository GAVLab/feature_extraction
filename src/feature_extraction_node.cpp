#include "feature_extraction/feature_extraction_node.h"

FeatureExtractionNode::FeatureExtractionNode()
{
  init = false;

  ros::NodeHandle nh("~");

  nh.param("cloud_leveling", levelCloud, true);

  /////////////////////////////
  /* Cloud Filter Parameters */
  /////////////////////////////
  nh.param("x_min", xMin, 0.0);
  nh.param("x_max", xMax, 75.0);
  nh.param("y_min", yMin, -30.0);
  nh.param("y_max", yMax, 30.0);
  nh.param("z_min", zMin, -1.5);
  nh.param("z_max", zMax, 5.0);

  ///////////////////////////////////
  /* Keypoint Detection Parameters */
  ///////////////////////////////////
  nh.param("cluster_tolerance", clusterTolerance, 0.65);
  nh.param("cluster_min_count", clusterMinCount, 5);
  nh.param("cluster_max_count", clusterMaxCount, 50);
  nh.param("cluster_radius_threshold", clusterRadiusThreshold, 0.15);
  nh.param("number_detection_channels", detectionChannelThreshold, 1);

  ///////////////////////////////////
  /* Feature Descriptor Parameters */
  ///////////////////////////////////
  nh.param("estimate_descriptors", descriptorEstimation, true);
  nh.param("descriptor_radius", descriptorRadius, 2.5);

  // printRosParameters();

  ////////////////////////////////
  /* ROS Publishers/Subscribers */
  ////////////////////////////////
  kp_pub = nh.advertise<PointCloud> ("keypoints", 0);
  kpc_pub = nh.advertise<PointCloud> ("keypoint_cloud", 0);
  cloud_pub = nh.advertise<PointCloud> ("cloud", 0);
  if (descriptorEstimation)
    feature_pub = nh.advertise<PointDescriptorCloud> ("features", 0);
  
  pc_sub = nh.subscribe ("/velodyne_points", 0, &FeatureExtractionNode::cloudCallback, this);
  imu_sub = nh.subscribe ("/xsens/data", 0, &FeatureExtractionNode::imuCallback, this);

}

FeatureExtractionNode::~FeatureExtractionNode()
{
  std::cout << "FeatureExtractionNode::~FeatureExtractionNode" << std::endl;
}

void FeatureExtractionNode::imuCallback (const sensor_msgs::ImuConstPtr& msg)
{
  if (levelCloud){
    tf::Quaternion quat;
    double yaw,tmproll;
    
    tf::quaternionMsgToTF(msg->orientation, quat);
    tf::Matrix3x3(quat).getRPY(tmproll, pitch, yaw);
    roll = tmproll - M_PI;
  }else{
    roll = 0.0;
    pitch = 0.0;
  }
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
  //////////////////////////
  /* Get Elevation Angles */
  //////////////////////////
  getElevationAngles(cloud_full);

  ////////////////////////
  /* Rotate Point Cloud */
  ////////////////////////
  rotateCloud(cloud_full);

  ////////////////////////
  /* Filter Point Cloud */
  ////////////////////////
  PointCloud::Ptr cloud(new PointCloud);
  *cloud = *cloud_full;
  filterCloud(cloud);

  ////////////////////////
  /* Keypoint detection */
  ////////////////////////
  PointCloud::Ptr keypoints(new PointCloud());
  PointCloud::Ptr keypoint_cloud(new PointCloud());

  estimateKeypoints(cloud,keypoints,keypoint_cloud);

  ////////////////
  /* Descriptor */
  ////////////////
  if (descriptorEstimation) {
    DescriptorCloud::Ptr descriptors(new DescriptorCloud());

    estimateDescriptors(cloud_full,keypoints,descriptors);

    PointDescriptorCloud::Ptr pt_descriptors(new PointDescriptorCloud());

    pcl::concatenateFields(*keypoints, *descriptors, *pt_descriptors);

    pt_descriptors->header.frame_id = msg->header.frame_id;
    pcl_conversions::toPCL(msg->header.stamp, pt_descriptors->header.stamp);
    feature_pub.publish(pt_descriptors);
  }
  
  ////////////////
  /* Publishers */
  ////////////////
  keypoints->header.frame_id = msg->header.frame_id;
  pcl_conversions::toPCL(msg->header.stamp, keypoints->header.stamp);
  kp_pub.publish (keypoints);

  keypoint_cloud->header.frame_id = msg->header.frame_id;
  pcl_conversions::toPCL(msg->header.stamp, keypoint_cloud->header.stamp);
  kpc_pub.publish (keypoint_cloud);

  cloud->header.frame_id = msg->header.frame_id;
  pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
  cloud_pub.publish (cloud);
  
  /* end */

  // pcl::console::print_highlight ("Loop time: %lfs\n", watch.getTimeSeconds ());
  
}

void FeatureExtractionNode::getElevationAngles (PointCloud::Ptr cloud){
  double x,xp,y,z,az,el_deg;
  for (int i = 0; i<cloud->points.size(); i++){
    x = cloud->points[i].x; y = cloud->points[i].y; z = cloud->points[i].z;
    az = atan2(y,x); // azimuth
    xp = cos(az)*x + sin(az)*y; 
    el_deg = atan2(z,xp)*180/M_PI; // elevation (in degrees)
    cloud->points[i].intensity = el_deg;
  }
}


void FeatureExtractionNode::rotateCloud (PointCloud::Ptr cloud){
  PointCloud transformed_cloud;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.0, 0.0, 0.0;
  transform.rotate (  Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY())*
                      Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX())   );
  pcl::transformPointCloud (*cloud, transformed_cloud, transform);
  *cloud = transformed_cloud;
}

void FeatureExtractionNode::filterCloud (PointCloud::Ptr cloud){
  pcl::PassThrough<Point> filter;
  filter.setInputCloud(cloud);
  /* Filter distant returns */
  filter.setFilterFieldName("z");
  filter.setFilterLimits(zMin, zMax);
  filter.filter(*cloud);
  filter.setFilterFieldName("y");
  filter.setFilterLimits(yMin, yMax);
  filter.filter(*cloud);
  filter.setFilterFieldName("x");
  filter.setFilterLimits(xMin, xMax);
  filter.filter(*cloud);
  /* end */
}

void FeatureExtractionNode::estimateKeypoints (const PointCloud::Ptr cloud, PointCloud::Ptr keypoints, PointCloud::Ptr keypoint_cloud)
{
  /* Keypoint Detection */
  pcl::PassThrough<Point> filter;
  filter.setInputCloud(cloud);
  filter.setFilterFieldName("intensity");

  PointCloud::Ptr keypoints_full(new PointCloud);
  double channelElevationDegrees;

  for (int i = 0; i<16; ++i){
    PointCloud::Ptr cylinderCentroids(new PointCloud);
    PointCloud::Ptr cylinderCloud(new PointCloud);
    PointCloud::Ptr channel(new PointCloud);
    
    channelElevationDegrees = (i-7)*2-1;
    filter.setFilterLimits(channelElevationDegrees-1.0, channelElevationDegrees+1.0); // group by channels
    filter.filter(*channel);
    getCylinderSegments(channel,cylinderCentroids,cylinderCloud);

    *keypoints_full += *cylinderCentroids;
    *keypoint_cloud += *cylinderCloud;
  }

  if (keypoints_full->points.size()<=0)
    return;

  /* Combine keypoints within same proximity */
  double zhold[keypoints_full->points.size()];

  for (int i = 0; i<keypoints_full->points.size(); ++i){
    zhold[i] = keypoints_full->points[i].z;
    keypoints_full->points[i].z = keypoints_full->points[i].intensity*0.75*clusterRadiusThreshold/2;
  }

  IndicesClustersPtr clusterIndices (new IndicesClusters);

  pcl::EuclideanClusterExtraction<Point> clustering; // Euclidean clustering object

  clustering.setInputCloud(keypoints_full);
  clustering.setClusterTolerance(clusterRadiusThreshold);
  clustering.setMinClusterSize(detectionChannelThreshold);
  clustering.setMaxClusterSize(16);

  clustering.extract(*clusterIndices);

  for (int i = 0; i<keypoints_full->points.size(); ++i)
    keypoints_full->points[i].z = zhold[i];

  if (clusterIndices->size()<=0)
    return;

  /* Keypoints from cluster indices */
  for (int i = 0; i < clusterIndices->size (); ++i){
    
    Point pt_centroid; // stores point of the cluster centroid

    double sumx = 0.0, sumy = 0.0, sumz = 0.0;
    int clusterSize = (*clusterIndices)[i].indices.size ();

    for (int j = 0; j < clusterSize; ++j){
      sumx += keypoints_full->points[(*clusterIndices)[i].indices[j]].x;
      sumy += keypoints_full->points[(*clusterIndices)[i].indices[j]].y;
      sumz += keypoints_full->points[(*clusterIndices)[i].indices[j]].z;
    }

    pt_centroid.x = sumx/( (double) clusterSize );
    pt_centroid.y = sumy/( (double) clusterSize );
    pt_centroid.z = sumz/( (double) clusterSize );
    pt_centroid.intensity = keypoints_full->points[(*clusterIndices)[i].indices[0]].intensity;

    keypoints->points.push_back(pt_centroid);
  }
  /* end */
}

void FeatureExtractionNode::getCylinderSegments (const PointCloud::Ptr cloud, PointCloud::Ptr keypoints, PointCloud::Ptr keypoint_cloud){
  
  if (cloud->points.size()<=0)
    return;

  /* Perform Euclidean clustering */
  IndicesClustersPtr clusterIndices (new IndicesClusters);

  pcl::EuclideanClusterExtraction<Point> clustering; // Euclidean clustering object

  clustering.setInputCloud(cloud);
  clustering.setClusterTolerance(clusterTolerance);
  clustering.setMinClusterSize(clusterMinCount);
  clustering.setMaxClusterSize(clusterMaxCount);

  clustering.extract(*clusterIndices);

  if (clusterIndices->size()<=0)
    return;

  /* Keypoints from cluster indices (with conditioning on max radius) */
  for (int i = 0; i < clusterIndices->size (); ++i){
    
    Point pt_centroid; // stores point of the cluster centroid
    PointCloud::Ptr cluster(new PointCloud); // the point cloud of the current cluster

    double x,y,z;
    double sumx = 0.0, sumy = 0.0, sumz = 0.0;
    double minx = 1000.0, maxx = -1000.0;
    double miny = 1000.0, maxy = -1000.0;
    int clusterSize = (*clusterIndices)[i].indices.size ();

    for (int j = 0; j < clusterSize; ++j){
      x = cloud->points[(*clusterIndices)[i].indices[j]].x;
      y = cloud->points[(*clusterIndices)[i].indices[j]].y;
      z = cloud->points[(*clusterIndices)[i].indices[j]].z;
      sumx += x; sumy += y; sumz += z;
      if (x<minx)
        minx=x;
      if (y<miny)
        miny=y;
      if (x>maxx)
        maxx=x;
      if (y>maxy)
        maxy=y;

      Point pt; // stores point of the cluster centroid
      pt.x = x; pt.y = y; pt.z = z; 
      pt.intensity = cloud->points[(*clusterIndices)[i].indices[j]].intensity;
      cluster->points.push_back(pt);

    }

    double diameter = pow(pow(maxx-minx,2)+pow(maxy-miny,2),0.5);
    
    if (diameter<(2*clusterRadiusThreshold)) {
      pt_centroid.x = sumx/( (double) clusterSize );
      pt_centroid.y = sumy/( (double) clusterSize );
      pt_centroid.z = sumz/( (double) clusterSize );
      pt_centroid.intensity = cloud->points[(*clusterIndices)[i].indices[0]].intensity;

      keypoints->points.push_back(pt_centroid);
      *keypoint_cloud += *cluster;
    }
  }

}

void FeatureExtractionNode::estimateDescriptors (const PointCloud::Ptr cloud, const PointCloud::Ptr keypoints, DescriptorCloud::Ptr descriptors)
{
  if (keypoints->points.size()<=0)
    return;

  NormalCloud::Ptr normals(new NormalCloud);
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);

  Normal axis;
  axis.normal_x = 0.0f; axis.normal_y = 0.0f; axis.normal_z = 1.0f;
  for (int i = 0; i<cloud->points.size(); ++i)
    normals->points.push_back(axis);

  // 3DSC estimation object.
  pcl::ShapeContext3DEstimation<Point, Normal, Descriptor> sc3d;
  sc3d.setInputCloud(keypoints);
  sc3d.setSearchSurface(cloud);

  sc3d.setInputNormals(normals);
  sc3d.setSearchMethod(kdtree);

  sc3d.setRadiusSearch(descriptorRadius);
  sc3d.setMinimalRadius(descriptorRadius/10.0);
  sc3d.setPointDensityRadius(descriptorRadius/5.0);
  sc3d.compute(*descriptors);

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
  // std::cout << "    number_detection_channels: " << detectionChannelThreshold << std::endl;
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
