# feature_extraction #

A ROS package for feature extraction through PCL. The "features" are tall, cylindrical objects such as light posts or trees.

## Dependencies ##
- pcl

## Description ##

Feature extraction is performed as follows:

1) The point cloud is rotated into a local-level frame based on the roll/pitch of the LiDAR.

2) The point cloud is filtered based on user defined cartesian thresholds (min/max xyz).

3) Features (aka keypoints) are detected from the filtered point cloud (as desribed in the following section).

4) A descriptor is formed for each keypoint based on the neighbors (points within a radius). The neighboring points are gathered from the full, unfiltered point cloud.

### Detector ###

a) For each of the 16 channels, segmentation is performed conditioned on
- min/max number of points
- cluster tolerance (min distance from one cluster to the next)
- cluster radius threshold 