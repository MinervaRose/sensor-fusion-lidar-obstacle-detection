/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    Car egoCar(Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1(Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2(Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3(Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars = {egoCar, car1, car2, car3};

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    bool renderScene = false;  // set true if you want cars and highway drawn
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // Segment plane using custom RANSAC
    auto segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstacles", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "road", Color(0,1,0));

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = 
        pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);

        // Alternate color if you have more clusters than colors
        Color color = colors[clusterId % colors.size()];
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), color);

        // Add bounding box around each cluster
        // Box box = pointProcessor->BoundingBox(cluster);
        // renderBox(viewer, box, clusterId);
        // NEW: rotated PCA (yaw-only) bounding box
        BoxQ boxQ = pointProcessor->BoundingBoxQ(cluster);
        renderBox(viewer, boxQ, clusterId);

        ++clusterId;
    }

    if (cloudClusters.empty())
        std::cout << "No clusters found!" << std::endl;
}

// Run the full pipeline on one PointXYZI frame
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // 1) Filter (downsample + ROI + roof removal)
    Eigen::Vector4f minPoint(-10.0f, -6.0f, -2.5f, 1.0f);
    Eigen::Vector4f maxPoint(  30.0f,  6.0f,  1.0f, 1.0f);
    auto filtered = pointProcessorI->FilterCloud(inputCloud, 0.2f, minPoint, maxPoint);

    // 2) Ground segmentation (custom RANSAC or PCL)
    auto seg = pointProcessorI->SegmentPlaneRansac(filtered, 100, 0.2f);
    renderPointCloud(viewer, seg.second, "ground", Color(0,1,0));  // plane
    // renderPointCloud(viewer, seg.first,  "obst",   Color(1,0,0)); // optional

    // 3) Clustering (tuned for data_1)
    auto clusters = pointProcessorI->Clustering(seg.first, 0.5f, 10, 1400);

    // 4) Render clusters + (optionally) oriented boxes
    int id = 0;
    std::vector<Color> colors =
        { Color(1,0,0), Color(0,1,0), Color(0,0,1),
          Color(1,1,0), Color(0,1,1), Color(1,0,1) };

    for (auto& c : clusters)
    {
        Color col = colors[id % colors.size()];
        renderPointCloud(viewer, c, "cluster" + std::to_string(id), col);

        // AABB:
        // Box box = pointProcessorI->BoundingBox(c);
        // renderBox(viewer, box, id);

        // Oriented (yaw-only) PCA box (if you implemented BoundingBoxQ)
        BoxQ obb = pointProcessorI->BoundingBoxQ(c);
        renderBox(viewer, obb, id);

        ++id;
    }
}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    int distance = 16;

    switch (setAngle)
    {
        case XY: viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side: viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS: viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main (int argc, char** argv)
{
    std::cout << "Starting environment..." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;  // try FPS for a front-facing view
    initCamera(setAngle, viewer);

    // Create PointXYZI processor and stream of PCDs
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream =
        pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        // Clear previous frame
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load next frame
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        // Process one frame
        cityBlock(viewer, pointProcessorI, inputCloudI);

        // Advance / wrap
        ++streamIterator;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }

    delete pointProcessorI;
    return 0;
}

