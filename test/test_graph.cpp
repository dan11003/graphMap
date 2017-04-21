#include "stdio.h"
#include "iostream"
#include "graph_map/graph_map.h"
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "graphfactory.h"
#include "ros/ros.h"
#include "Eigen/Geometry"
#include "visualization/graph_plot.h"
using namespace libgraphMap;
using namespace std;
using namespace Eigen;
int main(int argc, char **argv){
  ros::init(argc, argv, "testGraphLib");
  ros::NodeHandle n;
  string maptype;
  n.param<std::string>("map_type",maptype,"template");
  cout<<"Starting graph node with maptype: "<<maptype<<endl;
  Eigen::Affine3d initPose;

  initPose=Affine3d::Identity();
  initPose.translation()<<2.5,2.5,0.01;
  Eigen::Affine3d diff;
  diff=Affine3d::Identity();

  diff= AngleAxisd(0.0*M_PI, Vector3d::UnitX())
      * AngleAxisd(0.0*M_PI, Vector3d::UnitY())
      * AngleAxisd(0.2*M_PI, Vector3d::UnitZ())*Translation3d(1,1,0);

  Matrix6d cov;
  Eigen::DiagonalMatrix<double,6> diag1;

  diag1.diagonal()<<0.15,0.15,0.15,0.01,0.01,0.01;
  cov=diag1;

  mapParamPtr params=graphfactory::CreateMapParam(maptype);
  GraphMapPtr graph=graphfactory::CreateGraph(initPose,params);

  cout<<"size of graph :"<<graph->MapSize()<<endl;

  graph->AddMapNode(params,diff,cov);
  graph->AddMapNode(params,diff,cov);
  graph->AddMapNode(params,diff,cov);
  cout<<"size of graph :"<<graph->MapSize()<<endl;
  graphPlot::PlotPoseGraph(graph);
  cout<<graph->ToString()<<endl;
  pcl::PointCloud<pcl::PointXYZ> cloud;

}

