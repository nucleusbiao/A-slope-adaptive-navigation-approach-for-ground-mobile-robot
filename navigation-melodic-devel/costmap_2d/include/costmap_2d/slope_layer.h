#ifndef COSTMAP_2D_OBSTACLE_LAYER_H_
#define COSTMAP_2D_OBSTACLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint.h>

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

#include"costmap_2d/processPointClouds.h"
#include"costmap_2d/slope_layer_cluster.h"

namespace costmap_2d
{
typedef pcl::PointXYZI PointType;

class SlopeLayer : public CostmapLayer
{
public:
  SlopeLayer();
  virtual ~SlopeLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                           const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

// 获得用于标记空间的观测值
// 对一个vector的引用，它将被填充观测值
// 如果所有观察缓冲区都是当前的，则为True，否则为false
bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;
void dealwithSlopeclouds(const sensor_msgs::PointCloud2& cloudmessage);
void fittingStraightline(std::vector<pcl::PointCloud<PointType>::Ptr>& projectionCloud);
void Calculateindex(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1);
void bresenham(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,int offset_b, unsigned int offset, unsigned int max_length);
void raytrace(unsigned int x0 , unsigned int y0 , double ox , double oy , double wx , double wy , unsigned int cell_raytrace_range);
void testcostmap();
virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
                       double* max_x, double* max_y);

  double max_obstacle_height_;  ///< @brief Max Obstacle Height

  laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds

  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters用于观察消息过滤器
  std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor用于确保变换对每个传感器可用
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > pointclouds_buffers_;  ///< @brief Used to store observations from various sensors用于存储来自各种传感器的观测结果
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > slope_marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles用于存储观察缓冲区，用于标记障碍物
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles用于存储用于清除障碍物的观察缓冲区

  // Used only for testing purposes
  std::vector<costmap_2d::Observation> static_clearing_observations_, static_marking_observations_;

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig> *dsrv_;

  int combination_method_;

// 转换后的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr _pCloud;
//斜率点云
pcl::PointCloud<pcl::PointXYZI>::Ptr  _slopecloudspoints;
//填充后斜坡点云
pcl::PointCloud<pcl::PointXYZI>::Ptr  _slopecloudspoints2;
//处理点云的函数的对象
ProcessPointClouds<pcl::PointXYZI>* _pointProcessorI; //类模板已经确定类别，函数模板的类别自动确定

Slopeclouds* _slopeclouds; 


std::vector<Slopeclouds> slopecloudsv;

ros::Publisher  _publisher_cloudslope_points;
ros::Publisher _publisher_marking_points;
ros::Publisher _publisher_filtered_points;
ros::Publisher _publisher_segment_points;

vector<int> offsets;

int signs(int x)
{
  return x > 0 ? 1.0 : -1.0;
}

};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_OBSTACLE_LAYER_H_
