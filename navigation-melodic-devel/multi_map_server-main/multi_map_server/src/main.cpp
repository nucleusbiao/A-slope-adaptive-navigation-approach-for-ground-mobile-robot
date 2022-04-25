#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include<map>
#include "std_msgs/Int8.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res)
    {
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      // private_nh.param("frame_id", frame_id, std::string("map"));
      deprecated = (res != 0);
      if (!deprecated) {
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }
#ifdef HAVE_NEW_YAMLCPP
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try {
          doc["resolution"] >> res;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["negate"] >> negate;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["occupied_thresh"] >> occ_th;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["free_thresh"] >> free_th;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try {
            doc["map_id"] >> map_id;
            // std::cout<<"类型： "<<typeid(map_id).name()<<std::endl;
        } catch (YAML::InvalidScalar) {
            ROS_WARN("The map does not contain a map_id tag or it is invalid.");
	          map_id = 0;
        }
        try {
          doc["origin"][0] >> origin[0];
          doc["origin"][1] >> origin[1];
          doc["origin"][2] >> origin[2];
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try {
          doc["image"] >> mapfname;
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
      } else {
        private_nh.param("negate", negate, 0);
        private_nh.param("occupied_thresh", occ_th, 0.65);
        private_nh.param("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      if (map_id !=1)
      {
        frame_id = "map" + std::to_string(map_id);
      }
      else
      {
        frame_id = "map";
      }
      
      std::cout<<frame_id<<std::endl;

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin);
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      std::cout<<"frame_id:  "<<std::to_string(map_id)<<"    "<<map_resp_.map.header.frame_id<<std::endl;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      metadata_pub.publish( meta_data_message_ );

    }

  void SwitchPublisher(bool up) {
      if( up ) {
          // Latched publisher for data
          map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
          map_pub.publish( map_resp_.map );
          // std::cout<<"frame_id:  "<<map_resp_.map.header.frame_id<<std::endl;
      } else {
          map_pub.shutdown();
      }
  }

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    int map_id;

  private:
    ros::NodeHandle n;
    ros::Publisher metadata_pub;
    ros::Publisher map_pub;
    bool deprecated;

    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

};

std::map< int, MapServer> msv;
int current_map;
int laster_map;

bool mapCallback(nav_msgs::GetMap::Request  &req,
		 nav_msgs::GetMap::Response &res ) {
  if(msv.find(current_map) != msv.end()) {
    msv.find(current_map)->second.mapCallback(req,res);
  }
  return true;
}

void MapSelect(const std_msgs::Int8& msg) 
{
  // std::cout<<"msg.data类型：  "<<typeid(msg.data).name()<<std::endl;
  if(msg.data != 0  && msv.find(msg.data) != msv.end())
  {
    current_map = msg.data;
    
    laster_map = msg.data;
  }
  else
  {
    current_map = laster_map;
  }
  msv.find(current_map)->second.SwitchPublisher(true);
  // ROS_INFO("map %d is available.", current_map);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  ros::ServiceServer service;
  ros::Subscriber sub;
  service = n.advertiseService("static_map", &mapCallback);
  sub = n.subscribe("/aruco_simple/judge", 1, MapSelect);

  try{
      for(int i=1; i < argc; i++) {
          MapServer ms = MapServer(argv[i],0.0);
          int id = ms.map_id;
          std::cout<<"id:  "<<id<<std::endl;
          msv.insert(std::pair<int, MapServer>(id, ms));
          if(i==1) {
    	        current_map = id;
              laster_map = id;
    	        msv.find(id)->second.SwitchPublisher(true);
        }
    }
  ros::spin();
  }
  catch(std::runtime_error& e){
      ROS_ERROR("map_server exception: %s", e.what());
      return -1;
  }

  return 0;
}
