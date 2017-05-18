#include "graph_map_fuser.h"
namespace libgraphMap{
using namespace std;
GraphMapFuser::GraphMapFuser(string maptype, string registratorType, Eigen::Affine3d initPose, const Eigen::Affine3d &sensorPose){

  registratorType_=registratorType;
  mapParam_=GraphFactory::CreateMapParam(maptype);
  graph_param_=GraphFactory::CreateGraphParam();
  graph_ =GraphFactory::CreateGraph(initPose,mapParam_,graph_param_);
  regParam_=GraphFactory::CreateRegParam(registratorType);
  registrator_=GraphFactory::CreateRegistrationType(sensorPose,regParam_);
  sensorPose_=sensorPose;
  frameNr_=0;
}

/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 */
void GraphMapFuser::ProcessFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow){
  //in sensor frame
  bool mapChange=false;
  Eigen::Affine3d T_world_to_local_map=graph_->GetCurrentNodePose().inverse(); //transformation from node to world frame
  Tnow=T_world_to_local_map*Tnow;//change frame to local map
  //cout<<"New frame:global Tnow=\n"<<(T_world_to_local_map.inverse()*Tnow).translation() <<endl;
  if(frameNr_>=0){
    lslgeneric::transformPointCloudInPlace(sensorPose_, cloud);//Transform cloud into robot frame before registrating
    registrator_->Register(graph_->GetCurrentNode()->GetMap(),Tnow,cloud);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration
    mapChange=graph_->AutomaticMapInterchange(Tnow,unit_covar,T_world_to_local_map);
  }

  lslgeneric::transformPointCloudInPlace(Tnow, cloud);// The cloud should now be centered around the robot pose in the map frame
  Eigen::Affine3d scanSourcePose=Tnow*sensorPose_;//calculate the source of the scan
  /*if(mapChange){
    NDT2DMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(graph_->GetCurrentNode()->GetMap());
    GraphPlot::SendGlobal2MapToRviz(curr_node->GetMap(),0,T_world_to_local_map.inverse());
  }*/
  graph_->GetCurrentNode()->updateMap(scanSourcePose,cloud);//Update map
  Tnow=T_world_to_local_map.inverse()*Tnow;//remap Tnow to global map frame
  //--------plot
  NDT2DMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(graph_->GetCurrentNode()->GetMap());
  //GraphPlot::SendGlobalMapToRviz(curr_node->GetMap(),1,T_world_to_local_map.inverse());
  GraphPlot::PlotPoseGraph(graph_);
  //-------end of plo
  frameNr_++;
}
void GraphMapFuser::plotGTCloud(pcl::PointCloud<pcl::PointXYZ> &cloud){
  lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(0.4));
  ndlocal.guessSize(0,0,0,50,50,5);
  ndlocal.loadPointCloud(cloud,30.0);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

  NDTMap * ptrmap=&ndlocal;
  GraphPlot::SendLocalMapToRviz(ptrmap,0);
}

bool GraphMapFuser::ErrorStatus(string status){
  if(graph_!=NULL && registrator_!=NULL){
    return false;
  }
  else{
    status="No object instance found for graph or registrator";
    return true;
  }
}
}
