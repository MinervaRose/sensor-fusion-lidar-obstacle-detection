// PCL lib Functions for processing point clouds 


#include "processPointClouds.h"
#include <unordered_set>
#include <cmath>
#include <pcl/common/centroid.h>
#include <limits>
#include <Eigen/Dense>
#include "quiz/cluster/kdtree.h"



// --------------------------------------------------------------
// Constructor / Destructor
// --------------------------------------------------------------
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


// --------------------------------------------------------------
// Count points
// --------------------------------------------------------------
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


// --------------------------------------------------------------
// Filtering (Voxel Grid + ROI CropBox + Roof removal)
// --------------------------------------------------------------
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                        float filterRes,
                                        Eigen::Vector4f minPoint,
                                        Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    // 1) Downsample
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // 2) Crop to ROI
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // 3) Remove ego roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5f, -1.7f, -1.0f, 1.0f));
    roof.setMax(Eigen::Vector4f( 2.6f,  1.7f, -0.4f, 1.0f));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    inliers->indices.insert(inliers->indices.end(), indices.begin(), indices.end());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsed.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


// --------------------------------------------------------------
// Separate plane vs obstacles using inlier indices
// --------------------------------------------------------------
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());

    for (int idx : inliers->indices)
        planeCloud->points.push_back(cloud->points[idx]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    return {obstCloud, planeCloud};
}


// --------------------------------------------------------------
// PCL's built-in RANSAC plane segmentation
// --------------------------------------------------------------
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                         int maxIterations,
                                         float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
        std::cout << "Could not estimate a planar model for the given dataset.\n";

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsed.count() << " milliseconds" << std::endl;

    return SeparateClouds(inliers, cloud);
}


// --------------------------------------------------------------
// Custom RANSAC plane segmentation (3D)
// --------------------------------------------------------------
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud,
                                               int maxIterations,
                                               float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> bestInliers;
    if (!cloud || cloud->points.size() < 3)
    {
        std::cerr << "[RANSAC] Not enough points.\n";
        return {cloud, typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>())};
    }

    const int N = static_cast<int>(cloud->points.size());
    srand((unsigned)time(nullptr));

    for (int it = 0; it < maxIterations; ++it)
    {
        // Sample 3 distinct points
        int i1 = rand() % N, i2 = rand() % N, i3 = rand() % N;
        if (i1 == i2 || i1 == i3 || i2 == i3) { --it; continue; }

        const auto& p1 = cloud->points[i1];
        const auto& p2 = cloud->points[i2];
        const auto& p3 = cloud->points[i3];

        // Plane from 3 points
        float v1x = p2.x - p1.x, v1y = p2.y - p1.y, v1z = p2.z - p1.z;
        float v2x = p3.x - p1.x, v2y = p3.y - p1.y, v2z = p3.z - p1.z;

        float A = v1y * v2z - v1z * v2y;
        float B = v1z * v2x - v1x * v2z;
        float C = v1x * v2y - v1y * v2x;

        float norm = std::sqrt(A*A + B*B + C*C);
        if (norm == 0.0f) { --it; continue; }   // Colinear sample

        float D = -(A * p1.x + B * p1.y + C * p1.z);

        // Score inliers
        std::unordered_set<int> inliers;
        inliers.reserve(N);
        for (int idx = 0; idx < N; ++idx)
        {
            const auto& p = cloud->points[idx];
            float dist = std::fabs(A*p.x + B*p.y + C*p.z + D) / norm;
            if (dist <= distanceThreshold)
                inliers.insert(idx);
        }

        if (inliers.size() > bestInliers.size())
            bestInliers.swap(inliers);
    }

    // Convert to PointIndices and split
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices.reserve(bestInliers.size());
    for (int idx : bestInliers) inliers->indices.push_back(idx);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC plane (custom) took " << elapsed.count()
              << " ms with " << bestInliers.size() << " inliers" << std::endl;

    return SeparateClouds(inliers, cloud);
}


// ---- Custom Euclidean clustering using our KD-Tree ----
template <typename PointT>
void proximityHelper(int idx,
                     typename pcl::PointCloud<PointT>::Ptr cloud,
                     std::vector<int>& cluster,
                     std::vector<bool>& processed,
                     KdTree& tree,
                     float distanceTol)
{
    processed[idx] = true;
    cluster.push_back(idx);

    std::vector<float> point = {cloud->points[idx].x,
                                cloud->points[idx].y,
                                cloud->points[idx].z};
    std::vector<int> nearby = tree.search(point, distanceTol);

    for (int n : nearby)
    {
        if (!processed[n])
            proximityHelper<PointT>(n, cloud, cluster, processed, tree, distanceTol);
    }
}

template <typename PointT>
std::vector<std::vector<int>> euclideanClusterIndices(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        KdTree& tree,
        float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(cloud->points.size(), false);

    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
    {
        if (processed[i]) continue;

        std::vector<int> cluster;
        proximityHelper<PointT>(i, cloud, cluster, processed, tree, distanceTol);
        clusters.push_back(std::move(cluster));
    }
    return clusters;
}


// --------------------------------------------------------------
// Clustering 
// --------------------------------------------------------------
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       float clusterTolerance,
                                       int minSize,
                                       int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    if (!cloud || cloud->empty())
        return clusters;

    // 1) Build KD-Tree from the obstacle cloud
    KdTree tree;
    tree.k_ = 3; // ensure 3D splits
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
    {
        const auto& p = cloud->points[i];
        tree.insert({p.x, p.y, p.z}, i);
    }

    // 2) Cluster indices using our KD-Tree neighbor search
    std::vector<std::vector<int>> clusterIndices =
        euclideanClusterIndices<PointT>(cloud, tree, clusterTolerance);

    // 3) Convert index clusters -> PCL clouds and filter by size
    for (const auto& indices : clusterIndices)
    {
        if (static_cast<int>(indices.size()) < minSize ||
            static_cast<int>(indices.size()) > maxSize)
            continue;

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        cluster->points.reserve(indices.size());
        for (int idx : indices)
            cluster->points.push_back(cloud->points[idx]);

        cluster->width = static_cast<uint32_t>(cluster->points.size());
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering (custom) took " << elapsed.count()
              << " ms and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


// --------------------------------------------------------------
// Axis-aligned bounding box for a cluster
// --------------------------------------------------------------
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x; box.y_min = minPoint.y; box.z_min = minPoint.z;
    box.x_max = maxPoint.x; box.y_max = maxPoint.y; box.z_max = maxPoint.z;
    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    BoxQ boxq;
    if (!cluster || cluster->empty())
    {
        boxq.bboxTransform = Eigen::Vector3f::Zero();
        boxq.bboxQuaternion = Eigen::Quaternionf::Identity();
        boxq.cube_length = boxq.cube_width = boxq.cube_height = 0.f;
        return boxq;
    }

    // 1) Centroid
    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*cluster, centroid4);
    Eigen::Vector3f centroid = centroid4.head<3>();

    // 2) PCA in XY to get heading/yaw
    double mx = 0, my = 0;
    const size_t N = cluster->points.size();
    for (const auto& p : cluster->points) { mx += p.x; my += p.y; }
    mx /= (double)N; my /= (double)N;

    double sxx = 0, sxy = 0, syy = 0;
    for (const auto& p : cluster->points)
    {
        double dx = p.x - mx;
        double dy = p.y - my;
        sxx += dx*dx; sxy += dx*dy; syy += dy*dy;
    }
    sxx /= (double)N; sxy /= (double)N; syy /= (double)N;

    Eigen::Matrix2d cov; cov << sxx, sxy, sxy, syy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    // principal eigenvector (column with larger eigenvalue)
    Eigen::Vector2d evec = es.eigenvectors().col(1);
    float yaw = std::atan2((float)evec.y(), (float)evec.x());

    // 3) Rotation Rz(yaw), transform points to local PCA frame
    float c = std::cos(yaw), s = std::sin(yaw);
    Eigen::Matrix3f R;
    R <<  c, -s, 0,
          s,  c, 0,
          0,  0, 1;

    float min_x =  std::numeric_limits<float>::max();
    float min_y =  std::numeric_limits<float>::max();
    float min_z =  std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (const auto& p : cluster->points)
    {
        Eigen::Vector3f q = R.transpose() * (Eigen::Vector3f(p.x, p.y, p.z) - centroid);
        min_x = std::min(min_x, q.x()); max_x = std::max(max_x, q.x());
        min_y = std::min(min_y, q.y()); max_y = std::max(max_y, q.y());
        min_z = std::min(min_z, q.z()); max_z = std::max(max_z, q.z());
    }

    // 4) Center in local frame -> world
    Eigen::Vector3f local_center( (min_x+max_x)*0.5f,
                                  (min_y+max_y)*0.5f,
                                  (min_z+max_z)*0.5f );
    Eigen::Vector3f world_center = centroid + R * local_center;

    // 5) Sizes
    float length = (max_x - min_x);
    float width  = (max_y - min_y);
    float height = (max_z - min_z);

    // 6) Fill BoxQ
    boxq.bboxTransform  = world_center;
    boxq.bboxQuaternion = Eigen::Quaternionf(R);
    boxq.cube_length    = length;
    boxq.cube_width     = width;
    boxq.cube_height    = height;
    return boxq;
}


// --------------------------------------------------------------
// I/O helpers
// --------------------------------------------------------------
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)
        PCL_ERROR("Couldn't read file \n");
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;
    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(
        boost::filesystem::directory_iterator{dataPath},
        boost::filesystem::directory_iterator{});
    sort(paths.begin(), paths.end());
    return paths;
}