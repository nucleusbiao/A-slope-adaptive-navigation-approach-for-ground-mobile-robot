#pragma once
#include <iostream>
# include <pcl/io/pcd_io.h>
# include <pcl/common/common.h>
# include <pcl/filters/filter.h>
# include <pcl/filters/extract_indices.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/filters/crop_box.h>
# include <pcl/kdtree/kdtree.h>
# include <pcl/segmentation/sac_segmentation.h>
# include <pcl/segmentation/extract_clusters.h>
# include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>

// #define k 2  //聚类数，可在kmeans算法处作参数
using namespace std;//vector在std里面，必须要加，要不然就std::vector

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;


class Slopeclouds
{
    public:
    float slope;
    pcl::PointCloud<PointType>::Ptr _clouds; 

    Slopeclouds(){}
    ~Slopeclouds(){}
    float Dist(float t1,float t2);
    int clusterofTuple(float means[],float t);
    float getVar(float means[],vector<Slopeclouds*> cluster[]);
    float ads(float oldvar,float newvar);
    float getMeans(vector<Slopeclouds*> cluster);
    vector<Slopeclouds*>  Kmeans(vector<Slopeclouds*>& slopeclouds);
};


class Compare
{
    public:
    bool operator()(Slopeclouds* slopeclouds1 , Slopeclouds* slopeclouds2)
    {
        return (*slopeclouds1).slope < (*slopeclouds2).slope;
    }
};
