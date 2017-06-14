// ROS
#include <ros/ros.h>
// STL
#include <iostream>
#include <sstream>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/extract_clusters.h>

class PclTune
{

    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<Point> PointCloud;
    
  public:

    PclTune(char** argv);
    ~PclTune();

  private:
    
    int prmIdx,maxNumClusters;

    double zMin,zMax,xMin,xMax,yMin,yMax; // Bounds of point cloud pass through filter

    bool init;

    bool enforceMinHeight;

    // Detector
    double clusterTolerance;
    int clusterMinCount,clusterMaxCount;
    double clusterRadiusThreshold;

    std::string pcdPath;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer ();

    void keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

    void processCloud (PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters, PointCloud::Ptr keypoints);

    void filterCloud (PointCloud::Ptr cloud);

    void segmentCloud (const PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters);

    bool checkClusterCondition (const PointCloud::Ptr cluster);

};

PclTune::PclTune(char** argv)
{
  ros::NodeHandle nh("~");

  init = false;

  /////////////////////////////
  /* Cloud Filter Parameters */
  /////////////////////////////
  nh.param("x_min", xMin, -1.5);
  nh.param("x_max", xMax, 100.0);
  nh.param("y_min", yMin, -50.0);
  nh.param("y_max", yMax, 50.0);
  nh.param("z_min", zMin, -1.15);
  nh.param("z_max", zMax, 2.0);

  /////////////////////////////
  /* Segmentation Parameters */
  /////////////////////////////
  nh.param("cluster_tolerance", clusterTolerance, 1.2);
  nh.param("cluster_min_count", clusterMinCount, 10);
  nh.param("cluster_max_count", clusterMaxCount, 250);
  nh.param("cluster_radius_threshold", clusterRadiusThreshold, 0.15);

  enforceMinHeight = true;

  prmIdx = 1;

  maxNumClusters = 100;
  
  pcdPath = argv[1];
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

}

PclTune::~PclTune(){}


void PclTune::processCloud (PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters, PointCloud::Ptr keypoints){

  //////////////////////
  /* Load Point Cloud */
  //////////////////////
  pcl::io::loadPCDFile<Point>(pcdPath, *cloud);

  ////////////////////////
  /* Filter Point Cloud */
  ////////////////////////
  filterCloud(cloud);

  segmentCloud(cloud,clusters);

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclTune::createViewer ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  
  viewer->registerKeyboardCallback(&PclTune::keyboardCallback, *this, viewer.get());

  //////////////////////
  /* Load Point Cloud */
  //////////////////////
  PointCloud::Ptr cloud(new PointCloud);
  pcl::io::loadPCDFile<Point>(pcdPath, *cloud);

  // initialize cluster visualizations
  std::stringstream ss;
  for (int i=0;i<maxNumClusters;++i){
    ss << "cluster" << i;
    viewer->addPointCloud<Point>(cloud, ss.str() );
    ss.str("");
  }

  // initialize filtered cloud viz
  viewer->addPointCloud<Point>(cloud, "cloud" );

  // visualize full cloud  
  pcl::visualization::PointCloudColorHandlerCustom<Point> single_color(cloud, 128, 128, 128);

  viewer->addPointCloud<Point>(cloud,single_color, "cloud_full" );
  viewer->setPointCloudRenderingProperties
  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_full");

    
  return (viewer);
}

void PclTune::keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  
  int adjDir = 0;

  double prm_step[] = {0.05, // clusterTolerance
                       1, // clusterMinCount
                       1, // clusterMaxCount
                       0.01, // clusterRadiusThreshold
                       0.05, // zMin
                       0.05, // zMax
                       0.0}; // (bool) enforce height TH
                       
  int nPrm = (int) sizeof(prm_step)/sizeof(prm_step[0]);

  if (event.keyDown ()){
    if ( !(event.getKeySym() == "w") &
         !(event.getKeySym() == "a") &
         !(event.getKeySym() == "s") &
         !(event.getKeySym() == "d") )
      return;
     
    // ---- Change parameter to adjust
    if (event.getKeySym() == "a"){
      if (prmIdx>0)
        prmIdx--;
    }
    if (event.getKeySym() == "d"){
      if (prmIdx<(nPrm-1))
        prmIdx++;
    }
    // ---- Increase/decrease parameter
    if (event.getKeySym() == "w")
      adjDir = 1;
    if (event.getKeySym() == "s")
      adjDir = -1;
    
    switch (prmIdx){
      case 0:
        clusterTolerance += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "clusterTolerance = " << clusterTolerance << std::endl << std::endl;
        break;
      case 1:
        clusterMinCount += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "clusterMinCount = " << clusterMinCount << std::endl << std::endl;
        break;
      case 2:
        clusterMaxCount += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "clusterMaxCount = " << clusterMaxCount << std::endl << std::endl;
        break;
      case 3:
        clusterRadiusThreshold += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "clusterRadiusThreshold = " << clusterRadiusThreshold << std::endl << std::endl;
        break;
      case 4:
        zMin += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "zMin = " << zMin << std::endl << std::endl;
        break;
      case 5:
        zMax += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "zMax = " << zMax << std::endl << std::endl;
        break;
      case 6:
        if (adjDir!=0)
          enforceMinHeight = !enforceMinHeight;
        std::cout << std::endl << "enforceMinHeight = " << enforceMinHeight << std::endl << std::endl;
        break;

    }



    std::stringstream ss;
    for (int i=0;i<maxNumClusters;++i){
      ss << "cluster" << i;
      viewer->removePointCloud(ss.str(),0);
      ss.str("");
    }


    


    
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr keypoints(new PointCloud);
    std::vector<pcl::PointIndices> clusterIndices;

    processCloud(cloud,clusterIndices,keypoints);

    if ((prmIdx==4) || (prmIdx==5) || !init){
      viewer->removePointCloud("cloud",0);
      viewer->addPointCloud<Point>(cloud, "cloud" );
      viewer->setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
      init = true;
    }

    int numClusters = clusterIndices.size();

    std::cout << "Number of clusters: " << numClusters << std::endl;

    if (numClusters<=0)
        return;

    // --- Get colors for each cluster
    std::vector<int> red (numClusters);
    std::vector<int> blue (numClusters);
    std::vector<int> green (numClusters);

    int step = (int) 255/(numClusters+1);

    for (int i = 0; i < numClusters; ++i){
      red[i] = (i+1)*step;
      green[i] = (i%2)*255;
      blue[i] = (numClusters-i)*step;
    }

    // --- Plot clusters
    for (int i=0;i<numClusters;++i){
      PointCloud::Ptr cluster(new PointCloud);
      for (int ii=0; ii<clusterIndices[i].indices.size(); ii++)
        cluster->points.push_back(cloud->points[clusterIndices[i].indices[ii]]);
      
      ss << "cluster" << i;
      pcl::visualization::PointCloudColorHandlerCustom<Point> single_color(cluster, red[i], green[i], blue[i]);

      viewer->addPointCloud<Point>(cluster,single_color, ss.str() );
      viewer->setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());

      ss.str("");
    }

  }

}

void PclTune::segmentCloud (const PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusterIndices)
{

  std::vector<pcl::PointIndices> clusterIndicesFull;

  PointCloud::Ptr cloud2d(new PointCloud);
  
  *cloud2d = *cloud;
  for (int ii = 0; ii<cloud->points.size(); ++ii)
    cloud2d->points[ii].z = 0.0;
      

  // kd-tree object for searches.
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);
  kdtree->setInputCloud(cloud2d);

  // Euclidean clustering object.
  pcl::EuclideanClusterExtraction<Point> clustering;

  // Set cluster tolerance to 2cm (small values may cause objects to be divided
  // in several clusters, whereas big values may join objects in a same cluster).
  clustering.setClusterTolerance(clusterTolerance);
  // Set the minimum and maximum number of points that a cluster can have.
  clustering.setMinClusterSize(clusterMinCount);
  clustering.setMaxClusterSize(clusterMaxCount);
  clustering.setSearchMethod(kdtree);
  clustering.setInputCloud(cloud2d);

  clustering.extract(clusterIndicesFull);

  // clusterIndices = clusterIndicesFull;

  for (int i=0; i<clusterIndicesFull.size(); i++){

    PointCloud::Ptr cluster(new PointCloud);
    for (int ii=0; ii<clusterIndicesFull[i].indices.size(); ii++)
      cluster->points.push_back(cloud->points[clusterIndicesFull[i].indices[ii]]);
    
    if(checkClusterCondition(cluster))
      clusterIndices.push_back(clusterIndicesFull[i]);
  
  }

}

bool PclTune::checkClusterCondition (const PointCloud::Ptr cluster){

  if (cluster->points.size()<=0)
    return false;

  double range,heightThreshold;
  double centerDist,height;
  double maxCenterDist = 0.0;
  double maxHeight = 0.0;
  Eigen::Vector4f centroid;
  
  pcl::compute3DCentroid(*cluster, centroid);

  for (int ii = 0; ii<cluster->points.size(); ++ii){
    centerDist = pow(pow(cluster->points[ii].x-centroid[0],2)+pow(cluster->points[ii].y-centroid[1],2),0.5);
    height = pow(pow(cluster->points[ii].z-centroid[2],2),0.5);
    if (centerDist > maxCenterDist)
      maxCenterDist = centerDist;
    if (height > maxHeight)
      maxHeight = height;
  }

  range = pow(pow(centroid[0],2)+pow(centroid[1],2),0.5);
  if (enforceMinHeight){
    heightThreshold = 0.5*range*tan(2*M_PI/180.0);
  }else{
    heightThreshold = 0.0;
  }

  return ((maxCenterDist<clusterRadiusThreshold) && (maxHeight>heightThreshold));
}


void PclTune::filterCloud (PointCloud::Ptr cloud){

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

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tune");

  PclTune node(argv);
  return 0;
}