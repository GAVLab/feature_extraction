// ROS
#include <ros/ros.h>
// STL
#include <iostream>
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

class PclTune
{

    typedef pcl::PointXYZI Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::Normal Normal;
    typedef pcl::PointCloud<Normal> NormalCloud;

    typedef pcl::PointXYZINormal PointNormal;
    typedef pcl::PointCloud<PointNormal> PointNormalCloud;

    typedef pcl::SHOT352 Descriptor;
    typedef pcl::PointCloud<Descriptor> DescriptorCloud;
    
  public:

    PclTune(char** argv);
    ~PclTune();

  private:
    
    int prmIdx;

    double zMin,zMax,xMin,xMax,yMin,yMax; // Bounds of point cloud pass through filter
    double normRadius;

    // Detector
    int kpNumThreads;                   // number of threads in calculating harris keypoints
    bool kpRefine;                      // keypoint refine boolean
    bool kpNonMaxSupression;            // keypoint detection non max supression boolean
    double kpThreshold;                 // keypoint detection threshold for non max supression 
    double kpRadius;                    // radius (in meters) for gathering neighbors
    
    std::string pcdPath;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer ();

    void keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

    void processCloud (const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, NormalCloud::Ptr normals_out, PointCloud::Ptr keypoints_out);

    void filterCloud (PointCloud::Ptr cloud);

    void estimateNormals (const PointCloud::Ptr cloud, NormalCloud::Ptr normals);

    void estimateKeypoints (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr keypoints);

    void handle2d (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr cloud2d, NormalCloud::Ptr normals2d, PointNormalCloud::Ptr pt_normals2d);

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

  //////////////////////////////////
  /* Normal Estimation Parameters */
  //////////////////////////////////
  nh.param("normal_radius", normRadius, 0.5);

  ///////////////////////////////////
  /* Keypoint Detection Parameters */
  ///////////////////////////////////
  nh.param("harris_number_of_threads", kpNumThreads, 0);
  nh.param("harris_refine", kpRefine, false);
  nh.param("harris_non_max_supression", kpNonMaxSupression, true);
  nh.param("harris_radius", kpRadius, 1.0);
  nh.param("harris_threshold", kpThreshold, 0.1);

  prmIdx = 1;
  
  pcdPath = argv[1];
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createViewer();

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

}

PclTune::~PclTune(){}


void PclTune::processCloud (const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, NormalCloud::Ptr normals_out, PointCloud::Ptr keypoints_out){

  PointCloud::Ptr cloud(new PointCloud);

  *cloud = *cloud_in;
  ////////////////////////
  /* Filter Point Cloud */
  ////////////////////////
  filterCloud(cloud);

  /////////////////////////////
  /* Point Normal Estimation */
  /////////////////////////////
  NormalCloud::Ptr normals(new NormalCloud);
  estimateNormals(cloud,normals);

  std::cout << "\tcloud size:" << cloud->points.size() << std::endl;

  ////////////////////
  /* 2D Point Cloud */
  ////////////////////
  PointCloud::Ptr cloud2d(new PointCloud);
  NormalCloud::Ptr normals2d(new NormalCloud);
  PointNormalCloud::Ptr pt_normals2d(new PointNormalCloud);
  handle2d (cloud,normals,cloud2d,normals2d,pt_normals2d);

  std::cout << "\tcloud2d size:" << cloud2d->points.size() << std::endl;

  ////////////////////////
  /* Keypoint detection */
  ////////////////////////
  PointCloud::Ptr keypoints(new PointCloud());

  estimateKeypoints(cloud,normals,keypoints);
  // estimateKeypoints(cloud2d,normals2d,keypoints);

  ////////////////
  /* Set output */
  ////////////////
  *cloud_out = *cloud2d;
  *normals_out = *normals2d;
  *keypoints_out = *keypoints;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclTune::createViewer ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  
  viewer->registerKeyboardCallback(&PclTune::keyboardCallback, *this, viewer.get());


  PointCloud::Ptr cloud(new PointCloud);
  PointCloud::Ptr cloud2d(new PointCloud);
  NormalCloud::Ptr normals2d(new NormalCloud);
  PointCloud::Ptr keypoints(new PointCloud);

  //////////////////////
  /* Load Point Cloud */
  //////////////////////
  pcl::io::loadPCDFile<Point>(pcdPath, *cloud);

  processCloud(cloud,cloud2d,normals2d,keypoints);


  viewer->addPointCloud<Point>(cloud2d, "cloud");


  viewer->addPointCloudNormals<Point, Normal>(cloud2d, normals2d, 1, 0.5, "normals");// Display one normal out of 20, as a line of length 3cm.

  pcl::visualization::PointCloudColorHandlerCustom<Point> single_color(keypoints, 246, 128, 38);
  viewer->addPointCloud<Point>(keypoints, single_color, "keypoints");
  viewer->setPointCloudRenderingProperties
  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

  return (viewer);
}

void PclTune::keyboardCallback (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  
  int adjDir = 0;

  double prm_step[] = {0.05, // normRadius
                       0.001, // kpThreshold
                       0.05, // kpRadius
                       0.025, // zMin
                       0.025}; // zMax

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
        normRadius += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "normRadius = " << normRadius << std::endl << std::endl;
        break;
      case 1:
        kpThreshold += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "kpThreshold = " << kpThreshold << std::endl << std::endl;
        break;
      case 2:
        kpRadius += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "kpRadius = " << kpRadius << std::endl << std::endl;
        break;
      case 3:
        zMin += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "zMin = " << zMin << std::endl << std::endl;
        break;
      case 4:
        zMax += prm_step[prmIdx]*( (double) adjDir );
        std::cout << std::endl << "zMax = " << zMax << std::endl << std::endl;
        break;

    }








    
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud2d(new PointCloud);
    NormalCloud::Ptr normals2d(new NormalCloud);
    PointCloud::Ptr keypoints(new PointCloud);

    //////////////////////
    /* Load Point Cloud */
    //////////////////////
    pcl::io::loadPCDFile<Point>(pcdPath, *cloud);

    processCloud(cloud,cloud2d,normals2d,keypoints);

    std::cout << "\tkeypoints size:" << keypoints->points.size() << std::endl;


    // Point cloud
    viewer->updatePointCloud<Point>(cloud2d, "cloud");


    // Keypoints
    viewer->removePointCloud("keypoints", 0);
    pcl::visualization::PointCloudColorHandlerCustom<Point> single_color(keypoints, 246, 128, 38);
    viewer->addPointCloud<Point>(keypoints, single_color, "keypoints");
    viewer->setPointCloudRenderingProperties
    (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

    // Normals
    viewer->removePointCloud("normals", 0);
    viewer->addPointCloudNormals<Point, Normal>(cloud2d, normals2d, 1, 0.5, "normals");

  }

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

void PclTune::estimateNormals (const PointCloud::Ptr cloud, NormalCloud::Ptr normals)
{
  pcl::NormalEstimation<Point, Normal> normalEstimation;
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);

  normalEstimation.setInputCloud(cloud);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.setRadiusSearch(normRadius);
  normalEstimation.compute(*normals);
}

void PclTune::handle2d (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr cloud2d, NormalCloud::Ptr normals2d, PointNormalCloud::Ptr pt_normals2d)
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

  // for(size_t i = 0; i<cloud2d_full->points.size(); ++i){normals2d->points.push_back(normals2d_full->points[i]);}
  // *cloud2d = *cloud2d_full;

  for(size_t i = 0; i<cloud2d->points.size(); ++i){normals2d->points.push_back(normals2d_full->points[i]);}

}

void PclTune::estimateKeypoints (const PointCloud::Ptr cloud, const NormalCloud::Ptr normals, PointCloud::Ptr keypoints)
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

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tune");

  PclTune node(argv);
  return 0;
}