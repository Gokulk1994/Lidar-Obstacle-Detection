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


// Structure for each node in KD tree 
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left; // left node
	Node* right; //right node

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


// Structure for KD tree variables and its member functions
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
  	
  	// helper Recursive function to insert node into the KD tree
	void insertHelper(Node** node,uint depth,pcl::PointXYZI point, int id)
    {
      // If found a empty node, add the node 
      if(*node == NULL)
        *node = new Node(point,id);
      else
      {
        uint cd = depth % 2;
        
        if(cd == 0) 
        {
            if(point.x < (*node)->point.x) 
            	insertHelper(&(*node)->left, depth + 1, point, id);
            else 
            	insertHelper(&(*node)->right, depth + 1, point, id);
        }
        else
        {
            if(point.y < (*node)->point.y)  
              insertHelper(&(*node)->left, depth + 1, point, id);
            else 
              insertHelper(&(*node)->right, depth + 1, point, id);
        }
      }
      
    }
  	
  	// Insert node into the KD tree
	void insert(pcl::PointXYZI point, int id)
	{		
      insertHelper(&root,0,point,id);
	}

  	// Helper recursive function to search for a target node in the KD tree
    void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node != NULL)
        {
          if((node->point.x >= (target.x-distanceTol)&&(node->point.x <= (target.x+distanceTol)))&&(node->point.y >= (target.y-distanceTol)&&(node->point.y <= (target.y+distanceTol))))
          {
            float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) + (node->point.y - target.y) * (node->point.y - target.y));
            if(distance <= distanceTol) 
        		ids.push_back(node->id);
          }
          if(depth%2 == 0){
            if((target.x - distanceTol) < node->point.x) 
              searchHelper(target, node->left, depth+1, distanceTol, ids);
            if((target.x + distanceTol) > node->point.x) 
              searchHelper(target, node->right, depth+1, distanceTol, ids);
          }
          else{
            if((target.y - distanceTol) < node->point.y) 
              searchHelper(target, node->left, depth+1, distanceTol, ids);
            if((target.y + distanceTol) > node->point.y) 
              searchHelper(target, node->right, depth+1, distanceTol, ids);
          }

        }
    }
	  
	// Search a target node in the KD tree
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);
    
    void clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */