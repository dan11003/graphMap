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

#define NDT_GLOBAL_MAP_TOPIC "NDTglobalMap"
#define NDT_GLOBAL2_MAP_TOPIC "NDTglobal2Map"
#define NDT_LOCAL_MAP_TOPIC  "NDTlocalMap"
#define GRAPH_POSE_TOPIC "graphMap"
#define GRAPH_INFO_TOPIC "graphInfo"
namespace libgraphMap{
using namespace std;
using Eigen::Affine3d;
typedef std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > cov_vector;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > mean_vector;
class graphPlot
{
  graphPlot();
public:
  static void sendMapToRviz(mean_vector &mean, cov_vector &cov, ros::Publisher *mapPublisher, string frame, int color, const Affine3d &offset=Affine3d::Identity(), string ns="ndt", int markerType=visualization_msgs::Marker::SPHERE);
  static void SendLocalMapToRviz(lslgeneric::NDTMap *mapPtr, int color,const Affine3d &offset=Affine3d::Identity());
  static void SendGlobalMapToRviz(lslgeneric::NDTMap *mapPtr, int color,const Affine3d &offset=Affine3d::Identity());
  static void SendGlobal2MapToRviz(lslgeneric::NDTMap *mapPtr, int color,const Affine3d &offset=Affine3d::Identity());
  static void SendGlobal2MapToRviz(std::vector<lslgeneric::NDTCell*>cells, int color,const Affine3d &offset=Affine3d::Identity());

  static void GetAllCellsMeanCov(const lslgeneric::NDTMap *mapPtr, cov_vector &cov, mean_vector &mean);
  static void GetAllCellsMeanCov( std::vector<lslgeneric::NDTCell*>cells,cov_vector &cov, mean_vector &mean);
  static void CovarToMarker(const Eigen::Matrix3d &cov,const Eigen::Vector3d &mean,visualization_msgs::Marker &marker);
  static void PlotPoseGraph(GraphMapPtr graph);

protected:
  static void Initialize();
  static bool initialized_;
  static ros::NodeHandle *nh_;
  static ros::Publisher *localMapPublisher_;
  static ros::Publisher *globalMapPublisher_,*global2MapPublisher_;
  static ros::Publisher *graphPosePublisher_, *graphInfoPublisher_;
};

}
#endif // GRAPH_PLOT_H
