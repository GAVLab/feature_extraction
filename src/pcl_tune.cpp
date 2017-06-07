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

    // Detector
    double clusterTolerance;
    int clusterMinCount,clusterMaxCount;

    std::string pcdPath;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer ();

    void keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

    void processCloud (PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters, PointCloud::Ptr keypoints);

    void filterCloud (PointCloud::Ptr cloud);

    void segmentCloud (const PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters);

};

PclTune::PclTune(char** argv)
{
  ros::NodeHandle nh("~");

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

  prmIdx = 1;

  maxNumClusters = 20;
  
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

  PointCloud::Ptr cloud_full(new PointCloud);

  //////////////////////
  /* Load Point Cloud */
  //////////////////////
  pcl::io::loadPCDFile<Point>(pcdPath, *cloud_full);

  *cloud = *cloud_full;
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

  PointCloud::Ptr cloud(new PointCloud);
  PointCloud::Ptr keypoints(new PointCloud);
  std::vector<pcl::PointIndices> clusters;

  processCloud(cloud,clusters,keypoints);


  
  viewer->addPointCloud<Point>(cloud, "cloud");
  
  std::stringstream ss;
  for (int i=0;i<maxNumClusters;++i){
    ss << "cluster" << i;
    viewer->addPointCloud<Point>(cloud, ss.str() );
    ss.str("");
  }
    
  return (viewer);
}

void PclTune::keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  
  int adjDir = 0;

  double prm_step[] = {0.05, // clusterTolerance
                       1, // clusterMinCount
                       1}; // clusterMaxCount
                       
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

    }


    std::stringstream ss;
    for (int i=0;i<maxNumClusters;++i){
      ss << "cluster" << i;
      viewer->removePointCloud(ss.str(),0);
      ss.str("");
    }





    
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr keypoints(new PointCloud);
    std::vector<pcl::PointIndices> clusters;

    processCloud(cloud,clusters,keypoints);

    int numClusters = clusters.size();
  
    // std::cout << << numClusters << std::endl;

    if (numClusters<=0)
      return;

    std::vector<int> red (numClusters);
    std::vector<int> blue (numClusters);
    std::vector<int> green (numClusters);

    int step = (int) 255/(numClusters+1);

    for (int i = 0; i < numClusters; ++i){
      red[i] = (i+1)*step;
      green[i] = (i%2)*255;
      blue[i] = (numClusters-i)*step;
      std::cout << "green[i] = " << green[i] << std::endl;
      
    }
    
    // For every cluster.
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
    {
      // ...add all its points to a new cloud...
      PointCloud::Ptr cluster(new PointCloud);

      for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
        cluster->points.push_back(cloud->points[*point]);
      
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      if (cluster->points.size() <= 0)
        break;
      std::cout << "Cluster " << clusterId << " has " << cluster->points.size() << " points." << std::endl;
      

      //
      ss << "cluster" << clusterId;
      
      pcl::visualization::PointCloudColorHandlerCustom<Point> single_color(cluster, red[clusterId], green[clusterId], blue[clusterId]);

      viewer->addPointCloud<Point>(cluster,single_color, ss.str() );
      viewer->setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());

      ss.str("");

      clusterId++;
    }

  }

}

void PclTune::segmentCloud (const PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters)
{

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

  int numClusters = clusters.size();
  std::cout << numClusters << std::endl;


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