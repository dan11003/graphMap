#ifndef GRAPH_MAP_FUSER_H
#define GRAPH_MAP_FUSER_H
#include "stdio.h"
#include "iostream"
#include "graphfactory.h"
#include "graph_map/map_type.h"
#include "graph_map/graph_map.h"
#include "graph_map/reg_type.h"
#include "ndt2d/ndtd2d_reg_type.h"
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "gnuplot-iostream.h"
#include "ndt_map/ndt_map.h"



namespace libgraphMap{
using namespace std;
using Eigen::Affine3d;
class graphMapFuser{
public:
  graphMapFuser(string maptype, string registratorType,Affine3d initPose,const Affine3d &sensorPose);
  virtual void ProcessFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow, const Affine3d & Tmotion); //cloud is the current scan in robot frame,  Tnow is the current pose in world frame
  virtual bool ErrorStatus(string status="");
protected:
  //virtual void GetParameters();//Get fuser specific parameters, map specific parameters are preferably read inside a class derived from map_paramers
  void plotGTCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
  ros::NodeHandle n_;
  string maptype_,registratorType_;
  Eigen::Affine3d initPose_,sensorPose_;
  mapParamPtr mapParam_;
  GraphMapPtr graph_;
  GraphParamPtr graph_param_;
  regParamPtr regParam_;
  regTypePtr registrator_;
  unsigned int frameNr_;

  pcl::PointCloud<pcl::PointXYZ> cloud;

};

}
#endif // GRAPH_MAP_FUSER_H
