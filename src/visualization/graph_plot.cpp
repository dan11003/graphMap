#include "visualization/graph_plot.h"
#include "graph_map/graph_map.h"
namespace libgraphMap{
bool graphPlot::initialized_=false;
ros::NodeHandle* graphPlot::nh_=NULL;
ros::Publisher* graphPlot::mapPublisher_=NULL;
ros::Publisher* graphPlot::graphPublisher_=NULL;
void graphPlot::Initialize(){
  nh_=new ros::NodeHandle("");
  graphPublisher_=new ros::Publisher();
  mapPublisher_=new ros::Publisher();
  *mapPublisher_=  nh_->advertise<visualization_msgs::MarkerArray>(NDT_MAP_TOPIC,10);
  *graphPublisher_=nh_->advertise<geometry_msgs::PoseArray>(GRAPH_TOPIC,10);
  initialized_=true;
  sleep(1);
}
void graphPlot::PlotPoseGraph(GraphMapPtr graph){
  if(!initialized_)
    Initialize();
  geometry_msgs::PoseArray poseArr;
    poseArr.poses.clear();
  poseArr.header.stamp=ros::Time::now();
  poseArr.header.frame_id="/world";
  ROS_INFO_STREAM("poseArray.header: frame=" << poseArr.header.frame_id);

  geometry_msgs::Pose pose;
  for(int i=0;i<graph->MapSize();i++){
    Affine3d pose_tmp= graph->GetNodePose(i);
    tf::poseEigenToMsg(pose_tmp,pose);
    poseArr.poses.push_back(pose);
    cout<<"packing \n"<<pose_tmp.translation()<<endl;
    cout<<"with orientation \n"<<pose_tmp.linear()<<endl;

  }
  cout<<"senging "<<graph->MapSize()<<" poses"<<endl;
  for(int i=0;i<5;i++){
  graphPublisher_->publish(poseArr);
  cout<<"sending"<<endl;
  sleep(1);
  }
}

void graphPlot::CovarToMarker(const Eigen::Matrix3d &cov,const Eigen::Vector3d &mean,visualization_msgs::Marker &marker){
  if(!initialized_)
    Initialize();

  Eigen::Matrix2d mat= cov.block(0,0,2,2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(mat);
  //cout<<"solving :\n"<<mat<<endl;
  Eigen::Vector2d eigValues = eig.eigenvalues();
  Eigen::Matrix2d eigVectors= eig.eigenvectors();
  //cout<<"solution =\n "<<eigVectors<<endl;
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
void graphPlot::sendMapToRviz( lslgeneric::NDTMap *mapPtr, ros::Publisher &mapPublisher,int color){
  if(!initialized_)
    Initialize();

  std::vector<lslgeneric::NDTCell*> ndts;
  ndts = mapPtr->getAllCells();
  std::cout<<"sending map to rviz"<<std::endl;
  fprintf(stderr,"SENDING MARKER ARRAY MESSAGE (%zu components)\n",ndts.size());
  visualization_msgs::MarkerArray marray;

  for(unsigned int i=0;i<ndts.size();i++){
    Eigen::Matrix3d cov = ndts[i]->getCov();
    Eigen::Vector3d m = ndts[i]->getMean();

    visualization_msgs::Marker marker;
    CovarToMarker(cov,m,marker);
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "NDT";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    if(color==0){
      marker.color.a = 0.9;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    else if(color==1){
      marker.color.a = 0.9;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else if(color==2){
      marker.color.a = 0.9;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }
    marray.markers.push_back(marker);
  }

  mapPublisher_->publish(marray);
  for(unsigned int i=0;i<ndts.size();i++){
    delete ndts[i];
  }

}
}
