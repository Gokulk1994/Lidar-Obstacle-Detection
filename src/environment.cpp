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

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
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
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidarSensor = new Lidar(cars, 0.0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidarSensor->scan();

    //renderPointCloud(viewer,inputCloud,"gokul");

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
//renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
  
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

int clusterId = 0;
std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
{
      pointProcessor.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

      Box box = pointProcessor.BoundingBox(cluster);
      renderBox(viewer,box,clusterId);

      ++clusterId;
}
    
}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)

/*
Processes input point cloud from the pcd  and render it 
*/
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

  // Filtering the poin cloud. Removes unwanted object like points on the roof
  pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud =  pointProcessorI->FilterCloud(inputCloud, 0.3,
  Eigen::Vector4f(-20,-6,-3,-1), Eigen::Vector4f(30,7,2,1) );
  //renderPointCloud(viewer,FilteredCloud,"FilteredCloud");
  
  // find the points that best fits a plane through cross product
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->Ransac3D(FilteredCloud, 100, 0.2);
  //renderPointCloud(viewer,segmentCloud.first,"segmentCloud");
  KdTree* tree = new KdTree;
  
  // Clustering of points in cloud through Eucliden distance metrics
  for (int i=0; i<segmentedCloud.first->points.size(); i++) 
    tree->insert(segmentedCloud.first->points[i],i);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = 
    pointProcessorI->euclideanCluster(segmentedCloud.first, tree, 0.5, 10, 250);
  
  
  //renderPointCloud(viewer,segmentedCloud.first, "obstacle", Color(0,1,1));
  renderPointCloud(viewer,segmentedCloud.second, "normal", Color(0,1,0));
  
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    renderPointCloud(viewer,cluster,"obstacle"+std::to_string(clusterId),colors[clusterId % 3]);
    
    // Draw the bounding box around the detected obstacle
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer,box,clusterId,colors[clusterId % 3]);
    ++clusterId;
  }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
  
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
      // Remove last displayed point clouds and  bounding boxes
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Fetch individual frame from a pcd stream
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      // Loop back at the end of the stream
      streamIterator++;
      if(streamIterator == stream.end())
          streamIterator = stream.begin();

      viewer->spinOnce ();
      
    } 
}