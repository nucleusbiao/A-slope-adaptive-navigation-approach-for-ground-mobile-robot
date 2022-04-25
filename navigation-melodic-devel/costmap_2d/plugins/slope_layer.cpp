#include <costmap_2d/slope_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <time.h>

#include"costmap_2d/processPointClouds.h"
#include"/home/cuimingyue/catkin_ws/src/navigation-melodic-devel/costmap_2d/src/processPointClouds.cpp"
#include"costmap_2d/slope_layer_cluster.h"


PLUGINLIB_EXPORT_CLASS(costmap_2d::SlopeLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{
SlopeLayer::SlopeLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
    _pCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    _slopecloudspoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    _slopecloudspoints2.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

void SlopeLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  _publisher_cloudslope_points = g_nh.advertise<sensor_msgs::PointCloud2>("/cloudslope_points",10);
  rolling_window_ = layered_costmap_->isRolling();

  default_value_ = NO_INFORMATION;

  SlopeLayer::matchSize();//来自costmap_layer
  //分配costmap地图内存并初始化。参数来自master（layered_costmap_） 的costmap。master的costmap在staticlayer初始化时完成的。
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("slope_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // now we need to split the topics based on whitespace which we can use a stringstream for 现在，我们需要根据空格分割主题，我们可以使用stringstream
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("LaserScan"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    //找出传感器的障碍物范围
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    //获取传感器的光线跟踪范围
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    pointclouds_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));
    
    //根据标记分类
    // check if we'll add this buffer to our marking observation buffers
    if (marking)
      slope_marking_buffers_.push_back(pointclouds_buffers_.back());
    
    message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_sub_;
    tf2_ros::MessageFilter<sensor_msgs::PointCloud2>* point_cloud_filter_;
     
    point_cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50);
    point_cloud_filter_ = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_sub_, *tf_, global_frame_, 50, g_nh);
    //global_frame是map坐标系

    point_cloud_filter_->registerCallback(boost::bind(&SlopeLayer::pointCloud2Callback, this, _1, pointclouds_buffers_.back()));
    //一开始的pointclouds_buffers_.back()所指向的buffer中Observation的list是空的，需要回调的赋值
    }

  dsrv_ = NULL;

}

SlopeLayer::~SlopeLayer()
{
    if (dsrv_)
        delete dsrv_;
}

void SlopeLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  //点云的frame_id仍然是velodyne   不是map。
  dealwithSlopeclouds(*message);

  sensor_msgs::PointCloud2 slopeclouds_message;
  pcl::toROSMsg(*_slopecloudspoints2, slopeclouds_message);
  slopeclouds_message.header.frame_id = "velodyne";
  buffer->lock();
  buffer->bufferCloud(slopeclouds_message);
  //此刻(buffer->observation_list_).size()变为1。
  buffer->unlock();
  slopecloudsv.clear();
}

//将获取传感器传来的障碍物信息经过处理后放入一个观察队列中
void SlopeLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{

  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> slope_observations;

  // get the marking observations获得标记观察结果
  current = current && getMarkingObservations(slope_observations);

  current_ = current;
  for (unsigned int i = 0; i < slope_observations.size(); ++i)
  {
    raytraceFreespace(slope_observations[i], min_x, min_y, max_x, max_y); //首先清理出传感器到被测物之间的区域，标记为FREE_SPACE
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void SlopeLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void SlopeLayer::raytraceFreespace(const Observation& slope_observations, double* min_x, double* min_y,
                                              double* max_x, double* max_y)  //rviz加载地图时的外廓最大值最小值
//会首先处理测量值越界的问题
{
  double ox = slope_observations.origin_.x;
  double oy = slope_observations.origin_.y; //近似为0 
  
  const sensor_msgs::PointCloud2 &cloud = *(slope_observations.cloud_);
  unsigned int cell_raytrace_range = cellDistance(slope_observations.raytrace_range_);//cell_raytrace_range大小为：40  根据yaml文件配置  计算中除以0.1的分辨率
  
  _publisher_marking_points.publish(cloud);

  // get the map coordinates of the origin of the sensor
  //获取传感器原点的地图坐标  这个x0 y0值是不变的
  unsigned int x0, y0;

  if (!worldToMap(ox, oy, x0, y0))  //x0大小为： 30  y0大小为： 29
  {
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  //我们可以在内部循环之外预先计算地图的端点…我们一会儿需要这些
  
  touch(ox, oy, min_x, min_y, max_x, max_y);
  
  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  //对于云中的每个点，我们希望从原点开始沿着一条线走，并清除沿途的障碍
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  //点云中的每个点都遍历一遍并填充点云
  for (; iter_x != iter_x.end();)
  {
    double wx0 = *iter_x;
    double wy0 = *iter_y;
    double wx1 = *(++iter_x);
    double wy1 = *(++iter_y);
 
    double wx1_3 = (2*wx0+wx1)/3;
    double wy1_3 = (2*wy0+wy1)/3;
    double wx2_3 = (wx0+2*wx1)/3;
    double wy2_3 = (wy0+2*wy1)/3;

    raytrace(x0 , y0 , ox , oy , wx0 , wy0 , cell_raytrace_range);
    raytrace(x0 , y0 , ox , oy , wx1_3 , wy1_3 , cell_raytrace_range);
    raytrace(x0 , y0 , ox , oy , wx2_3 , wy2_3 , cell_raytrace_range);

  }
  //打印代价值
  // testcostmap();
  offsets.clear();
}

void SlopeLayer::raytrace(unsigned int x0 , unsigned int y0 , double ox , double oy , double wx , double wy , unsigned int cell_raytrace_range)
{
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    //光线跟踪的最小值是原点
    double origin_x = origin_x_, origin_y = origin_y_;
    double map_end_x = origin_x + size_x_ * resolution_; //resolution_为分辨率 0.1  // origin_x 大小为： -2.9  origin_y 大小为： -2.8  size_x_ 60 
    double map_end_y = origin_y + size_y_ * resolution_;
    //现在，我们还需要确保光线跟踪的enpoint没有超出costmap和规格，如果有必要

    if (wx < origin_x) //-2.9
    {
      double t = (origin_x - ox) / a; 
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    if (wx > map_end_x)
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    //现在矢量被正确地缩放了…我们会得到它端点的映射坐标
    unsigned int x1, y1;

    // check for legality just in case
    //检查合法性以防万一
    if (worldToMap(wx, wy, x1, y1))
    {
    // std::cout<<"x1:  "<<std::to_string(x1)<<"y1: "<<std::to_string(y1)<<std::endl;
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    //最后……我们可以通过追踪来清除这条线上的障碍
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range); //会将所有在(x0,y0)>>(x1,y1)之间的所有cell标记为FREE_SPACE  
    //updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);//而updateRaytraceBounds 会根据测量的距离，更新扩张（min_x, min_y, max_x, max_y）
    Calculateindex(x0, y0, x1, y1);  //用来测试的
    }
}

void SlopeLayer::testcostmap()
{
  for(int i = 0 ; i < offsets.size() ; i++)
  {
    std::cout<<(unsigned int)costmap_[offsets[i]]<<"  ";
  }
  std::cout<<std::endl;
}

void SlopeLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);//设置机器人轮廓所在区域为FREE_SPACE
  }

  enabled_ = true;
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}



bool SlopeLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < slope_marking_buffers_.size(); ++i) //marking_buffers_的size为1
  {
    slope_marking_buffers_[i]->lock();
    slope_marking_buffers_[i]->getObservations(marking_observations);
    current = slope_marking_buffers_[i]->isCurrent() && current;
    slope_marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end()); //static_marking_observations_大小为0

  return current;
}


void SlopeLayer::dealwithSlopeclouds(const sensor_msgs::PointCloud2& cloudmessage)
{
  pcl::fromROSMsg(cloudmessage , *_pCloud);  //pcl与ros接口  1编译遇到fromRosMsg不是pcl的成员函数  2程序运行到此处智能指针没有初始化，并且初始化的方式和之前不同，用reset()。

  //滤波
  pcl::PointCloud<PointType>::Ptr _cloudFiltered = _pointProcessorI->FilterCloud(_pCloud,0.1,Eigen::Vector4f(-4, -1, -0.5, 1), Eigen::Vector4f( 4, 1, 1.5, 1));//设置滤波范围 （x1 , y1 , z1 ,1）

  //地面分割
  std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> segmentCloud = _pointProcessorI->SegmentPlane(_cloudFiltered, 50, 0.05);//最大迭代次数，距离阈值

  if(segmentCloud.first->points.size() !=0)
  {
  //聚类
  std::vector<pcl::PointCloud<PointType>::Ptr> cloudClusters = _pointProcessorI->Clustering(segmentCloud.first, 0.33, 5, 500);

  //聚类后坡道点云是分开的 后续合成一个
  // std::cout<<cloudClusters.size()<<std::endl;
  if(cloudClusters.size()>1)
    {
    //投影
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	  coefficients->values.push_back(0.0);
	  coefficients->values.push_back(0.0);
   	coefficients->values.push_back(1.0);
  	coefficients->values.push_back(0.0);
    
    std::vector<pcl::PointCloud<PointType>::Ptr> projectionClouds;
    for(pcl::PointCloud<PointType>::Ptr _cluster : cloudClusters)
    {
      typename pcl::PointCloud<PointType>::Ptr cloud_projected(new pcl::PointCloud<PointType>());
      typename pcl::ProjectInliers<pcl::PointXYZI>::Ptr proj(new pcl::ProjectInliers<pcl::PointXYZI>()); //缺少相对应的头文件
	    proj->setInputCloud(_cluster);
	    proj->setModelCoefficients(coefficients);
	    proj->setModelType(pcl::SACMODEL_PLANE);//在使用pcl提取点云空间中的平面时，我们一般采用SACMODEL_PLANE模型，利用采样一致性算法（RANSAC）算法进行提取
	    proj->filter(*cloud_projected);
        projectionClouds.push_back(cloud_projected);
    }
    
    //拟合直线
    //ax+by+c = 0
    
    fittingStraightline(projectionClouds);
    
    vector<Slopeclouds*> slope(slopecloudsv.size() , nullptr);//预设大小和附初值 空指针
    for(int i = 0 ; i<slopecloudsv.size() ; i++)
    {
        slope[i] = &slopecloudsv[i];   //slope存的是斜率点云的地址
    }
    
    // 对斜率进行聚类
    std::vector<Slopeclouds*>  slopeclouds = _slopeclouds->Kmeans(slope);  //kmeans算法

    //整合斜坡点云
    if(slopeclouds.size() !=0)
    {
    if(_slopecloudspoints->points.size() != 0)
    {
      _slopecloudspoints->points.clear(); //因为以前的书写格式不能编译通过，只能这样写了
      _slopecloudspoints2->points.clear();
    }
    for(int i = 0 ; i < slopeclouds.size() ; i++)
    {
        for(int j = 0 ; j < ((*slopeclouds[i])._clouds)->points.size() ; j++)
        {
            _slopecloudspoints->points.push_back(((*slopeclouds[i])._clouds)->points[j]);
        }
    } 
    }
    //添加反方向点云
    for(int i = 0 ; i < _slopecloudspoints->points.size() ; i++)
    {
      pcl::PointXYZI point;
      point.x =  -(_slopecloudspoints->points[i].x);
      point.y =  -(_slopecloudspoints->points[i].y);
      point.z =  _slopecloudspoints->points[i].z;
      point.intensity =  _slopecloudspoints->points[i].intensity;
      _slopecloudspoints2->points.push_back(point);
      _slopecloudspoints2->points.push_back(_slopecloudspoints->points[i]);
    } 

    //发布斜坡点云
    sensor_msgs::PointCloud2 cloudslope_points;
    pcl::toROSMsg(*_slopecloudspoints2, cloudslope_points);
    cloudslope_points.header.frame_id = "velodyne";
    _publisher_cloudslope_points.publish(cloudslope_points);
  }
  else
  {
    std::cout<<"聚类无法判定是否是斜坡"<<std::endl;
  }
}
  else
{ 
   std::cout<<"无可分割"<<std::endl;
}
}

//拟合直线
//ax+by+c = 0
void SlopeLayer::fittingStraightline(std::vector<pcl::PointCloud<PointType>::Ptr>& projectionCloud)
{ 
    for(pcl::PointCloud<PointType>::Ptr  project : projectionCloud)
    {
        Slopeclouds slopeclouds;
        float a, b, c;
        float x_mean = 0;
        float y_mean = 0;
        int size = project->points.size();
        for(int i=0 ; i <size ; i++)
        {
            // std::cout<<"    x轴坐标值："<<project->points[i].x<<"    y轴坐标值："<<project->points[i].y<<endl;
            x_mean += project->points[i].x;
            y_mean += project->points[i].y;
        }
        x_mean /= size;
        y_mean /= size;
        float Dxx = 0, Dxy = 0, Dyy = 0;
        for(int i = 0; i <size; i++)
        {
            Dxx += (project->points[i].x - x_mean) * (project->points[i].x - x_mean);
            Dxy += (project->points[i].x - x_mean) * (project->points[i].y - y_mean);
            Dyy += (project->points[i].y - y_mean) * (project->points[i].y - y_mean);
        }
        float lambda = ( (Dxx + Dyy) - sqrt( (Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy) ) / 2.0;
        float den = sqrt( Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx) );

        if(fabs(den) < 1e-5)
        {
            if( fabs(Dxx / Dyy - 1) < 1e-5) //这时没有一个特殊的直线方向，无法拟合
            {
                std::cout<<"直线无法拟合"<<std::endl;
            }
            else
            {
               a = 1;
               b = 0;
               c = - x_mean;
            }
        }
       else
        {
            a = Dxy / den;
            b = (lambda - Dxx) / den;
            c = - a * x_mean - b * y_mean;
        }

        pcl::PointXYZI point;
        for(int i=0 ; i <size ; i++)
        {
          point.x = (project->points[i].x+project->points[i+1].x)/2;
          point.y = (project->points[i].y+project->points[i+1].y)/2;
          point.z = project->points[i].z;
          point.intensity =  project->points[i].intensity;
          project->points.push_back(point);
        }

        float k = -b/a;
        std::vector<float> slopev;
        slopev.push_back(k);

        slopeclouds.slope = k;
        slopeclouds._clouds = project;
        slopecloudsv.push_back(slopeclouds); //是不是直接取地址不行
        
    }

}


void SlopeLayer::Calculateindex(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1)
{
      int dx = x1 - x0;
      int dy = y1 - y0;
      unsigned int max_length = 40;
      unsigned int abs_dx = abs(dx);
      unsigned int abs_dy = abs(dy);
      int offset_dx = signs(dx);
      int offset_dy = signs(dy) * size_x_;
      unsigned int offset = y0 * size_x_ + x0;  //size_x_ 值为60  offset就是代价地图的索引值

      // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
      //我们需要根据线段的最大长度来选择我们的主导尺寸的比例
      double dist = hypot(dx, dy);//计算三角形的斜边长
      double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

      // if x is dominant
      if (abs_dx >= abs_dy)
      {
        int error_y = abs_dx / 2;
        bresenham(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
      }
      // otherwise y is dominant
      else 
      {
        int error_x = abs_dy / 2;
        bresenham(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
      }
}
      
void SlopeLayer::bresenham(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                            int offset_b, unsigned int offset, unsigned int max_length)
{
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
      offset += offset_a;
      error_b += abs_db;
      if ((unsigned int)error_b >= abs_da)
      {
        offset += offset_b;
        error_b -= abs_da;
      }
      offsets.push_back(offset);
    }
    // std::cout<<"斜坡栅格数量："<<offsets.size()<<std::endl;
}
}
