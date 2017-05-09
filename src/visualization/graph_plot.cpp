#include "visualization/graph_plot.h"
#include "graph_map/graph_map.h"
namespace libgraphMap{
bool graphPlot::initialized_=false;
ros::NodeHandle* graphPlot::nh_=NULL;
ros::Publisher* graphPlot::localMapPublisher_=NULL;
ros::Publisher* graphPlot::graphPosePublisher_=NULL;
ros::Publisher* graphPlot::graphInfoPublisher_=NULL;
ros::Publisher* graphPlot::globalMapPublisher_=NULL;
ros::Publisher* graphPlot::global2MapPublisher_=NULL;

void graphPlot::Initialize(){
  cout<<"graph_plot: initialize"<<endl;
  nh_=new ros::NodeHandle("");
  graphPosePublisher_=    new ros::Publisher();
  localMapPublisher_= new ros::Publisher();
  globalMapPublisher_=new ros::Publisher();
  global2MapPublisher_=new ros::Publisher();
  graphInfoPublisher_=new  ros::Publisher();

  *localMapPublisher_=  nh_->advertise<visualization_msgs::MarkerArray>(NDT_LOCAL_MAP_TOPIC,10);
  *globalMapPublisher_= nh_->advertise<visualization_msgs::MarkerArray>(NDT_GLOBAL_MAP_TOPIC,10);
  *global2MapPublisher_= nh_->advertise<visualization_msgs::MarkerArray>(NDT_GLOBAL2_MAP_TOPIC,10);
  *graphPosePublisher_=nh_->advertise<geometry_msgs::PoseArray>(GRAPH_POSE_TOPIC,10);
  *graphInfoPublisher_=nh_->advertise<visualization_msgs::MarkerArray>(GRAPH_INFO_TOPIC,10);

  initialized_=true;
}
void graphPlot::PlotPoseGraph(GraphMapPtr graph){
  if(!initialized_)
    Initialize();
  //Publish poses
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

  visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker_array.markers.clear();
    marker.header.frame_id = "/world";
    marker.header.stamp = poseArr.header.stamp;
    marker.ns = "graphInfo";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime=ros::Duration();
    marker.scale.z=0.2;
    marker.color.a=1;
    marker.color.b=1;

    Affine3d pose_tmp= graph->GetCurrentNodePose();
    tf::poseEigenToMsg(pose_tmp,pose);
    marker.pose.position.x=pose_tmp.translation()(0);
    marker.pose.position.y=pose_tmp.translation()(1);
    marker.pose.position.z=pose_tmp.translation()(2)+0.5;

    std::ostringstream os;
    os << std::setw(3)<< "(x,y)=("<<marker.pose.position.x<<","<<marker.pose.position.y<<")"<<endl;


    marker.text= os.str();
    marker_array.markers.push_back(marker);
    marker.id=1;
    graphPosePublisher_->publish(poseArr);
    graphInfoPublisher_->publish(marker_array);


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
void graphPlot::sendMapToRviz(mean_vector &mean, cov_vector &cov, ros::Publisher *mapPublisher, string frame, int color,const Affine3d &offset,string ns,int markerType){
  static unsigned int max_size=0;
  if(!initialized_)
    Initialize();


  if(max_size<mean.size())
    max_size=mean.size();

  cout<<"plotting "<<mean.size()<<" cells, however have previously plotted "<<max_size<<" cells"<<endl;
  visualization_msgs::MarkerArray marray;
  visualization_msgs::Marker marker;
  marray.markers.clear();
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.type = markerType;

  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime=ros::Duration();

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
    marker.color.a = 0.4 ;
    marker.color.r = 0.4;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
  }
  for(unsigned int i=0;i<mean.size();i++){
    Eigen::Matrix3d cov_tmp=offset.linear()*cov[i]*offset.linear().transpose();
    Eigen::Vector3d m_tmp = offset*mean[i];
    CovarToMarker(cov_tmp,m_tmp,marker);
    marker.id = i;
    marray.markers.push_back(marker);
  }

  marray.markers.push_back(marker);
  for(int i=mean.size();i<max_size;i++){
    marker.id =i;
    marray.markers.push_back(marker);
  }
  mapPublisher->publish(marray);

}
void graphPlot::SendLocalMapToRviz(lslgeneric::NDTMap *mapPtr,int color,const Affine3d &offset){
  cov_vector cov;
  mean_vector mean;
  if(!initialized_)
    Initialize();
  GetAllCellsMeanCov(mapPtr,cov,mean);
  sendMapToRviz(mean,cov,localMapPublisher_,"fuser",color,offset,"local");
}
void graphPlot::GetAllCellsMeanCov(const lslgeneric::NDTMap *mapPtr,cov_vector &cov,mean_vector &mean){
  cov.clear();
  mean.clear();
  std::vector<lslgeneric::NDTCell*>cells=mapPtr->getAllCells();
  NDTCell *cell;
  for(int i=0;i<cells.size();i++){
    cov.push_back(cells[i]->getCov());
    mean.push_back(cells[i]->getMean());
    // delete cells[i];
  }
}
void graphPlot::GetAllCellsMeanCov( std::vector<lslgeneric::NDTCell*>cells,cov_vector &cov,mean_vector &mean){
  cov.clear();
  mean.clear();
  for(int i=0;i<cells.size();i++){
    cov.push_back(cells[i]->getCov());
    mean.push_back(cells[i]->getMean());
    //  delete cells[i];
  }
}

void graphPlot::SendGlobalMapToRviz(lslgeneric::NDTMap *mapPtr, int color,const Affine3d &offset){
  cov_vector cov;
  mean_vector mean;
  if(!initialized_)
    Initialize();

  GetAllCellsMeanCov(mapPtr,cov,mean);
  sendMapToRviz(mean,cov,globalMapPublisher_,"world",color,offset,"current");
}
void graphPlot::SendGlobal2MapToRviz(lslgeneric::NDTMap *mapPtr, int color,const Affine3d &offset){
  cov_vector cov;
  mean_vector mean;
  if(!initialized_)
    Initialize();
  GetAllCellsMeanCov(mapPtr,cov,mean);
  sendMapToRviz(mean,cov,globalMapPublisher_,"world",color,offset,"prev",visualization_msgs::Marker::CUBE);
}
void graphPlot::SendGlobal2MapToRviz(  std::vector<lslgeneric::NDTCell*>cells, int color,const Affine3d &offset){
  cov_vector cov;
  mean_vector mean;

  if(!initialized_)
    Initialize();
  GetAllCellsMeanCov(cells,cov,mean);
  sendMapToRviz(mean,cov,globalMapPublisher_,"world",color,offset,"prev",visualization_msgs::Marker::CUBE);
}

}
