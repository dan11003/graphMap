#include "visualization/graph_plot.h"
#include "graph_map/graph_map.h"
namespace libgraphMap{
bool graphPlot::initialized_=false;
ros::NodeHandle* graphPlot::nh_=NULL;
ros::Publisher* graphPlot::localMapPublisher_=NULL;
ros::Publisher* graphPlot::graphPublisher_=NULL;
ros::Publisher* graphPlot::globalMapPublisher_=NULL;

void graphPlot::Initialize(){
  cout<<"graph_plot: initialize"<<endl;
  nh_=new ros::NodeHandle("");
  graphPublisher_=    new ros::Publisher();
  localMapPublisher_= new ros::Publisher();
  globalMapPublisher_=new ros::Publisher();
  *localMapPublisher_=  nh_->advertise<visualization_msgs::MarkerArray>(NDT_LOCAL_MAP_TOPIC,10);
  *globalMapPublisher_= nh_->advertise<visualization_msgs::MarkerArray>(NDT_GLOBAL_MAP_TOPIC,10);
  *graphPublisher_=nh_->advertise<geometry_msgs::PoseArray>(GRAPH_TOPIC,10);
  initialized_=true;
}
void graphPlot::PlotPoseGraph(GraphMapPtr graph){
  if(!initialized_)
    Initialize();
  geometry_msgs::PoseArray poseArr;
  poseArr.poses.clear();
  poseArr.header.stamp=ros::Time::now();
  poseArr.header.frame_id="/world";

  geometry_msgs::Pose pose;
  for(int i=0;i<graph->MapSize();i++){
    Affine3d pose_tmp= graph->GetNodePose(i);
    tf::poseEigenToMsg(pose_tmp,pose);
    poseArr.poses.push_back(pose);
  }
  for(int i=0;i<5;i++){
    graphPublisher_->publish(poseArr);
    sleep(1);
  }
}

void graphPlot::CovarToMarker(const Eigen::Matrix3d &cov,const Eigen::Vector3d &mean,visualization_msgs::Marker &marker){
  if(!initialized_)
    Initialize();

  Eigen::Matrix2d mat= cov.block(0,0,2,2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(mat);
  Eigen::Vector2d eigValues = eig.eigenvalues();
  Eigen::Matrix2d eigVectors= eig.eigenvectors();

  double angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));
  marker.scale.x=2*sqrt(eigValues(0));
  marker.scale.y=2*sqrt(eigValues(1));
  marker.scale.z=0.02;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = sin(angle*0.5);
  marker.pose.orientation.w = cos(angle*0.5);
  marker.pose.position.x=mean(0);
  marker.pose.position.y=mean(1);
  marker.pose.position.z=mean(2);

}
void graphPlot::sendMapToRviz(lslgeneric::NDTMap *mapPtr, ros::Publisher *mapPublisher, string frame, int color){
  if(!initialized_)
    Initialize();

  std::vector<lslgeneric::NDTCell*> ndts;
  ndts = mapPtr->getAllCells();

  visualization_msgs::MarkerArray marray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.ns = "NDT";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  if(color==0){
    marker.color.a = 0.75;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else if(color==1){
    marker.color.a = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  else if(color==2){
    marker.color.a = 0.6 ;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  for(unsigned int i=0;i<ndts.size();i++){
    Eigen::Matrix3d cov = ndts[i]->getCov();
    Eigen::Vector3d m = ndts[i]->getMean();
    CovarToMarker(cov,m,marker);
    marker.id = i;
    marray.markers.push_back(marker);
  }
  mapPublisher->publish(marray);

}
void graphPlot::SendLocalMapToRviz(lslgeneric::NDTMap *mapPtr,int color){
  if(!initialized_)
    Initialize();
  sendMapToRviz(mapPtr,localMapPublisher_,"fuser",color);
}
void graphPlot::SendGlobalMapToRviz(lslgeneric::NDTMap *mapPtr,int color){
  if(!initialized_)
    Initialize();
  sendMapToRviz(mapPtr,globalMapPublisher_,"world",color);
}
}
