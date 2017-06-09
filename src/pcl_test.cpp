// ROS
#include <ros/ros.h>
// STL
#include <iostream>
// PCL
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/filters/conditional_removal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

namespace pcl
{

struct PointDescriptor
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float intensity;
  float descriptor[1980];
  float rf[9];
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

}

POINT_CLOUD_REGISTER_POINT_STRUCT (PointDescriptor,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float[1980], descriptor, shape_context)
                                   (float[9], rf, rf)
)

class PclTest
{

  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> PointCloud;

  typedef pcl::Normal Normal;
  typedef pcl::PointCloud<Normal> NormalCloud;

  typedef pcl::PointXYZINormal PointNormal;
  typedef pcl::PointCloud<PointNormal> PointNormalCloud;

  typedef pcl::ShapeContext1980 Descriptor;
  typedef pcl::PointCloud<Descriptor> DescriptorCloud;

  typedef pcl::PointDescriptor PointDescriptor;
  typedef pcl::PointCloud<PointDescriptor> PointDescriptorCloud;

  public:

    PclTest(char** argv);
    ~PclTest();

  private:
    
};

PclTest::PclTest(char** argv)
{
  ros::NodeHandle nh("~");
  
  DescriptorCloud::Ptr descriptors(new DescriptorCloud());
  PointCloud::Ptr keypoints(new PointCloud());
  PointDescriptorCloud::Ptr pt_descriptors(new PointDescriptorCloud());

  pcl::concatenateFields(*keypoints, *descriptors, *pt_descriptors);
  
}

PclTest::~PclTest(){}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_test");

  PclTest node(argv);
  return 0;
}