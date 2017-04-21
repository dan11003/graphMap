#include "graph_map_fuser.h"
namespace libgraphMap{
using namespace std;
graphMapFuser::graphMapFuser(string maptype, string registratorType,Eigen::Affine3d initPose,Eigen::Affine3d sensorPose){

  registratorType_=registratorType;
  params_=graphfactory::CreateMapParam(maptype);
  graph_ =graphfactory::CreateGraph(initPose,params_);//vart ifrån kommer init pose;
  registrator_=graphfactory::CreateRegistrationType(registratorType_);
  frameNr_=0;
}

void graphMapFuser::processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow, const Eigen::Affine3d & Tmotion){
  //in sensor frame
  if(frameNr_>=0){
    registrator_->Register(graph_->GetCurrentNode()->GetMap(),Tnow,Tmotion,cloud);
  }
  graph_->GetCurrentNode()->updateMap(Tnow,cloud);
  frameNr_++;
}

}
