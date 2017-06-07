// ROS
#include <ros/ros.h>
// STL
#include <iostream>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

double xMin = 0.0;
double xMax = 50.0;
double yMin = -25.0;
double yMax = 25.0;
double zMin = 0.13;
double zMax = 1.87;


void filterCloud (PointCloud::Ptr cloud){

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

int main(int argc, char** argv)
{
// Object for storing the point cloud.
PointCloud::Ptr cloud(new PointCloud);

// Read a PCD file from disk.
if (pcl::io::loadPCDFile<Point>(argv[1], *cloud) != 0)
{
return -1;
}

filterCloud(cloud);

// kd-tree object for searches.
pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);
kdtree->setInputCloud(cloud);

// Euclidean clustering object.
pcl::EuclideanClusterExtraction<Point> clustering;

// Set cluster tolerance to 2cm (small values may cause objects to be divided
// in several clusters, whereas big values may join objects in a same cluster).
clustering.setClusterTolerance(1.2);
// Set the minimum and maximum number of points that a cluster can have.
clustering.setMinClusterSize(10);
clustering.setMaxClusterSize(250);
clustering.setSearchMethod(kdtree);
clustering.setInputCloud(cloud);

std::vector<pcl::PointIndices> clusters;
clustering.extract(clusters);

// For every cluster...
int currentClusterNum = 1;
for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
{
  // ...add all its points to a new cloud...
  PointCloud::Ptr cluster(new PointCloud);

  for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
    cluster->points.push_back(cloud->points[*point]);
  cluster->width = cluster->points.size();
  cluster->height = 1;
  cluster->is_dense = true;
  // ...and save it to disk.
  if (cluster->points.size() <= 0)
    break;
  std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
  std::string fileName = "/home/danpierce/devel/lidar_ws/data/clusters/cluster" + boost::to_string(currentClusterNum) + ".pcd";
  std::cout << fileName << std::endl;
  pcl::io::savePCDFileASCII(fileName, *cluster);
  currentClusterNum++;
}



}