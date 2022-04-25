// PCL lib Functions for processing point clouds 


#include"costmap_2d/processPointClouds.h"
#include <unordered_set>
//#include "quiz/cluster/kdtree.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloudFiltered);

   // interesting region感兴趣的地区
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
    
    // std::cout<<"cloudRegion  pointclouds  size:  "<<cloudRegion->size()<<std::endl;
  //remove ego car roof points  去除自我车顶点
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename  pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    for(int index : inliers-> indices)
        planeCloud -> points.push_back(cloud -> points[index]);
    //Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

//RANSAC（随机抽样一致性）算法 , 找到一条线， 点云到这条线的距离在很小的范围内，认为是地面
//ransac3d
std::unordered_set<int> inliersResult;//容器中的元素无特别的秩序关系，该容器允许基于值的快速元素检索，同时也支持正向迭代。
srand(time(NULL));
while(maxIterations--){
        std::unordered_set<int> inliers;
        while(inliers.size()<3)
            inliers.insert(rand()%(cloud->points.size()));
        float x1, y1,z1, x2, y2,z2, x3, y3,z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        float i = (y2 - y1)*(z3 - z1) - (z2-z1)*(y3 - y1);
        float j = (z2 - z1)*(x3 - x1) - (x2-x1)*(z3 - z1);
        float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3-x1);
        float D = -(i*x1 + j*y1 + k*z1);
        for(int index=0;index< cloud->points.size();index++){
            if(inliers.count(index)>0)
                continue;
            PointT  point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            float d = fabs(i*x4 + j*y4 + k*z4 +D)/sqrt(i*i + j*j + k*k);
            if(d< distanceThreshold)
                inliers.insert(index);
        }
        if(inliers.size() > inliersResult.size()){
            inliersResult = inliers;
        }
     }


     pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
     for(int index=0; index< cloud->points.size();index++){
        //  PointT  point = cloud->points[index];
         if(inliersResult.count(index)){
             inliers2-> indices.push_back(index);
         }

     }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers2,cloud);//SeparateClouds函数用于将点云数据分为平面和非平面两个类别
    return segResult;
}
 
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    //平面分割将点云分成了两大类别，聚类将点云分成一个个目标集合。
    //最简答的聚类算法为：欧式距离。可以简单的理解为：将距离相近的一些点划归为一类，也可以对一类中的点数进行控制，如果一个类别中点数过少，我们可以认为它是噪声；
    //如果一个类别中点数过多，我们认为它可能是两个障碍物点云的重叠。
    //在众多的点云数据中找到距离相近的点，需要对一帧数据中的点云数据进行遍历，十分耗费计算资源。可以采用kdTree算法进行点云的搜索，这里直接使用PCL中的库函数完成。

    // Time clustering process
    // 记录时间
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // std::cout<<"Clustering input clouds size :"<<std::to_string((cloud->points).size())<<std::endl;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    // 将点云数据按照Tree的方式进行存储，方便后续查询遍历
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);// 设置欧式距离的buffer ,设置近邻搜索半径
    ec.setMinClusterSize(minSize);// 设置一个类别的最小数据点数
    ec.setMaxClusterSize(maxSize);//  设置一个类别的最大数据点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);//得到所有类别点云的索引 clusterIndices，每一类的点云都要用一个索引PointIndices  里面包含了原有点云每个点的的索引值
    // 将符合要求点存储起来//将得到的所有类的索引分别在点云中找到，即每一个索引形成了一个类
    for(pcl::PointIndices getIndices: cluster_indices){//遍历每一类别的点云
        // std::cout<<std::to_string(cluster_indices[0].indices.size())<<std::endl;
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>); //for循环下 堆区内存空间被重新利用，以前的内存内容被覆盖了
        for(int index : getIndices.indices)//getIndices.indices是一个int 的vector
        {
            // std::cout<<std::to_string(index)<<"  ";
            cloudCluster-> points.push_back(cloud->points[index]); //点云中的每个点都push一遍 PointCloud类中的point也是vector，<pcl::pointXYZ>的  每个点云进来都是带着索引值进来的
        }    
        // std::cout<<"     "<<std::endl;
        cloudCluster -> width = cloudCluster -> points.size(); //两个含义：1) 对于无组织的(unorganized)点云,表示点的总数，2）对于有组织的点云，表示宽度（一行有多少点）。
                                                                                                                     //有组织点云表示类似图像/矩阵结构的点云，分成行和列，来自立体相机或TOF相机。对于有组织点云，可以知道其相邻点的关系，最近邻操作更高效。
        cloudCluster -> height = 1;//对于有组织点云，指定高度（行数）；对于无组织点云，height等于1 （可用来检查点云有无组织）
        cloudCluster -> is_dense = true; //如果为true,则points中的数据是有限的，否则一些点的XYZ数值可能包含Inf/ NaN
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[idx] = true;
	cluster.push_back(idx);
	std::vector<int> nearest = tree->search(cloud->points[idx], distanceTol);
	for(auto id : nearest)
	{
		if(!processed[id])
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(),false);
    for(size_t idx=0;idx< cloud->points.size(); ++idx){
        if(processed[idx] == false){
            std::vector<int> cluster_idx;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
            clusterHelper(idx,cloud, cluster_idx, processed, tree, distanceTol);
            if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize){
                for(int i=0; i<cluster_idx.size(); i++){
                    PointT point;
                    point = cloud->points[cluster_idx[i]];
                    cloudCluster->points.push_back(point);
                }
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                clusters.push_back(cloudCluster);
            }else{
                for(int i=1;i<cluster_idx.size();i++){
                    processed[cluster_idx[i]] = false;
                }
            }
        }
    }
    return clusters;
}


// template<typename PointT>
// Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
// {

//     // Find bounding box for one of the clusters
//     PointT minPoint, maxPoint;
//     pcl::getMinMax3D(*cluster, minPoint, maxPoint);

//     Box box;
//     box.x_min = minPoint.x;
//     box.y_min = minPoint.y;
//     box.z_min = minPoint.z;
//     box.x_max = maxPoint.x;
//     box.y_max = maxPoint.y;
//     box.z_max = maxPoint.z;

//     return box;
// }


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}