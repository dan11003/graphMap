#ifndef GRAPH_PLOT_H
#define GRAPH_PLOT_H
#include "graphfactory.h"
#include "ros/ros.h"
#include "ros/publisher.h"
#include "Eigen/Dense"
#include "visualization_msgs/MarkerArray.h"
#include "ndt_map/ndt_map.h"
#include "geometry_msgs/PoseArray.h"
#include "eigen_conversions/eigen_msg.h"

#define NDT_MAP_TOPIC "NDTglobalMap"
#define GRAPH_TOPIC "graphMap"
namespace libgraphMap{
using namespace std;
class graphPlot
{
  graphPlot();
public:
  static void sendMapToRviz( lslgeneric::NDTMap *mapPtr, ros::Publisher &mapPublisher,int color);
  static void CovarToMarker(const Eigen::Matrix3d &cov,const Eigen::Vector3d &mean,visualization_msgs::Marker &marker);
  static void PlotPoseGraph(GraphMapPtr graph);
protected:
  static void Initialize();
  static bool initialized_;
  static ros::NodeHandle *nh_;
  static ros::Publisher *mapPublisher_,*graphPublisher_;

};

}
#endif // GRAPH_PLOT_H
