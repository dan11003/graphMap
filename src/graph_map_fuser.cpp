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

/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 * \param Tmotion: world
 */
void graphMapFuser::ProcessFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow, const Eigen::Affine3d & Tmotion){
  //in sensor frame
  Eigen::Affine3d T_world_to_local_map=graph_->GetCurrentNodePose().inverse(); //transformation from node to world frame
  Tnow=T_world_to_local_map*Tnow;//change frame to local map
    cout<<"New frame:global Tnow=\n"<<(T_world_to_local_map.inverse()*Tnow).translation() <<endl;
  if(frameNr_>=0){
    lslgeneric::transformPointCloudInPlace(sensorPose_, cloud);//Transform cloud into robot frame before registrating
    cout<<"fuser: register"<<endl;
    registrator_->Register(graph_->GetCurrentNode()->GetMap(),Tnow,Tmotion,cloud);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration
    graph_->AutomaticMapInterchange(Tnow,unit_covar,T_world_to_local_map);
  }

  lslgeneric::transformPointCloudInPlace(Tnow, cloud);//Transform cloud to map frame, The cloud chould new be centered around the robot pose in the map frame
  Eigen::Affine3d scanSourcePose=Tnow*sensorPose_;//calculate the source of the scan

  graph_->GetCurrentNode()->updateMap(scanSourcePose,cloud);//Update map
  Tnow=T_world_to_local_map.inverse()*Tnow;//remap Tnow to global map frame

  NDT2DMapPtr curr_node = boost::dynamic_pointer_cast< NDT2DMapType >(graph_->GetCurrentNode()->GetMap());
  graphPlot::SendGlobalMapToRviz(curr_node->GetMap(),1,T_world_to_local_map.inverse());
  graphPlot::PlotPoseGraph(graph_);

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
