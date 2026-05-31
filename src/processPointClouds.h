// PCL lib Functions for processing point clouds 


#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

template<typename PointT>
class ProcessPointClouds {
public:
    // constructor / destructor
    ProcessPointClouds();
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    // Filtering
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                      float filterRes,
                                                      Eigen::Vector4f minPoint,
                                                      Eigen::Vector4f maxPoint);

    // Helpers
    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
    SeparateClouds(pcl::PointIndices::Ptr inliers,
                   typename pcl::PointCloud<PointT>::Ptr cloud);

    // Segmentation (PCL RANSAC)
    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                 int maxIterations,
                 float distanceThreshold);

    // Segmentation (Custom RANSAC Plane)  <-- NEW
    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
    SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud,
                       int maxIterations,
                       float distanceThreshold);

    // Clustering
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
               float clusterTolerance,
               int minSize,
               int maxSize);

    // Bounding box
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    // Oriented bounding box in XY (yaw-only) using PCA
    BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

    // I/O
    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);
    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

#endif /* PROCESSPOINTCLOUDS_H_ */