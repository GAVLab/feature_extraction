// ROS
#include <ros/ros.h>
#include <ros/ros.h>
// STL
#include <iostream>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;


// pcl::visualization::RangeImageVisualizer viewer("Range image");

int main (int argc, char** argv)
{
  
  // Object for storing the point cloud.
  PointCloud::Ptr cloud(new PointCloud);
  // Object for storing the normals.
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  // Read a PCD file from disk.
  if (pcl::io::loadPCDFile<Point>(argv[1], *cloud) != 0)
  {
  return -1;
  }
  // Object for normal estimation.
  pcl::NormalEstimation<Point, pcl::Normal> normalEstimation;

  normalEstimation.setInputCloud(cloud);
  // For every point, use all neighbors in a radius of 3cm.
  normalEstimation.setRadiusSearch(0.5);
  
  
  // A kd-tree is a data structure that makes searches efficient. More about it later.
  // The normal estimation object will use it to find nearest neighbors.
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);
  normalEstimation.setSearchMethod(kdtree);
  // Calculate the normals.
  normalEstimation.compute(*normals);
  

  // Visualize them.
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
  viewer->addPointCloud<Point>(cloud, "cloud");
  
  // Display one normal out of 20, as a line of length 3cm.
  viewer->addPointCloudNormals<Point, pcl::Normal>(cloud, normals, 1, 0.5, "normals");
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  return 0;
}