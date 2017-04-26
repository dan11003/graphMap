#include "graph_map_fuser.h"
namespace libgraphMap{
using namespace std;
graphMapFuser::graphMapFuser(string maptype, string registratorType,Eigen::Affine3d initPose,Eigen::Affine3d sensorPose){

  registratorType_=registratorType;
  mapParam_=graphfactory::CreateMapParam(maptype);
  graph_ =graphfactory::CreateGraph(initPose,mapParam_);
  regParam_=graphfactory::CreateRegParam(registratorType);
  registrator_=graphfactory::CreateRegistrationType(regParam_);
  sensorPose_=sensorPose;
  frameNr_=0;
}


void graphMapFuser::ProcessFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow, const Eigen::Affine3d & Tmotion){
  //in sensor frame
  if(frameNr_>=0){
    lslgeneric::transformPointCloudInPlace(sensorPose_, cloud);//Transform cloud into robot frame before registrating

    Eigen::Affine3d prevPose=Tnow;
    cout<<"fuser: register"<<endl;
    registrator_->Register(graph_->GetCurrentNode()->GetMap(),Tnow,Tmotion,cloud);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration
    Eigen::Affine3d diff=prevPose.inverse()*Tnow;
  }

  lslgeneric::transformPointCloudInPlace(Tnow, cloud);//Transform cloud to world frame, the cloud should have the correct placment in the world
  Eigen::Affine3d scanSourcePose=Tnow*sensorPose_;//calculate the source of the scan

  graph_->GetCurrentNode()->updateMap(scanSourcePose,cloud);
  NDT2DMapPtr mapPtr = boost::dynamic_pointer_cast< NDT2DMapType >(graph_->GetCurrentNode()->GetMap());
  graphPlot::SendGlobalMapToRviz(mapPtr->GetMap(),1);
  frameNr_++;
}

bool graphMapFuser::ErrorStatus(string status){
    if(graph_!=NULL && registrator_!=NULL){
      return false;
    }
    else{
      status="No object instance found for graph or registrator";
      return true;
    }

  }
}
