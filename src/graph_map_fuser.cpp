#include "graph_map_fuser.h"
namespace libgraphMap{
using namespace std;
GraphMapFuser::GraphMapFuser(string maptype, string registratorType, Eigen::Affine3d initPose, const Eigen::Affine3d &sensorPose){
  graph_param_=GraphFactory::CreateGraphParam();
  graph_param_->GetParametersFromRos();
  regParam_=GraphFactory::CreateRegParam(registratorType);
  cout<<"started reading reg par from ros"<<endl;
  regParam_->GetParametersFromRos();
  cout<<"finished reading reg par from ros"<<endl;
  mapParam_=GraphFactory::CreateMapParam(maptype);
  cout<<"started reading map par from ros"<<endl;
  mapParam_->GetParametersFromRos();
cout<<"time to create graph inside fuser"<<endl;
  Graph_nav_ =GraphFactory::CreateGraphNavigator(initPose,mapParam_,graph_param_);
  registrator_=GraphFactory::CreateRegistrationType(sensorPose,regParam_);
  sensorPose_=sensorPose;
  nr_frames_=0;
  initialized_=true;
}
GraphMapFuser::GraphMapFuser(  RegParamPtr regParam,  MapParamPtr mapParam, GraphParamPtr graph_param, Eigen::Affine3d initPose, const Eigen::Affine3d &sensorPose){
  mapParam_=mapParam;
  regParam_=regParam;
  graph_param_=graph_param;
  Graph_nav_ =GraphFactory::CreateGraphNavigator(initPose,mapParam_,graph_param_);
  registrator_=GraphFactory::CreateRegistrationType(sensorPose,regParam_);
  nr_frames_=0;
  initialized_=true;
}

/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 */
void GraphMapFuser::ProcessFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow, const Eigen::Affine3d &Tmotion){

  bool map_node_changed=false,map_node_created,registration_succesfull=false;
  if(!initialized_){
   cerr<<"fuser not initialized"<<endl;
    return;
  }
  //Tnow=Tnow*Tmotion;
  Eigen::Affine3d T_world_to_local_map=Graph_nav_->GetCurrentNodePose().inverse(); //transformation from node to world frame
  Tnow=T_world_to_local_map*Tnow;//change frame to local map
  //cout<<"New frame:global Tnow=\n"<<(T_world_to_local_map.inverse()*Tnow).translation() <<endl;
  if(nr_frames_>=0){
    Matrix6d motion_cov=motion_model_2d_.getCovMatrix6(Tmotion, 1., 1., 1.);
    lslgeneric::transformPointCloudInPlace(sensorPose_, cloud);//Transform cloud into robot frame before registrating
    registration_succesfull = registrator_->Register(Graph_nav_->GetCurrentNode()->GetMap(),Tnow,cloud,motion_cov);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration
    if(Graph_nav_->AutomaticMapInterchange(Tnow,motion_cov,T_world_to_local_map,map_node_changed,map_node_created) && map_node_changed)
    {
      //double score;
      //Affine3d Tdiff=Graph_nav_->GetPreviousNodePose().inverse()*Graph_nav_->GetCurrentNodePose();
      //Tdiff.translation()=Tdiff.translation()+Vector3d(0.1,0.1,0.1);
      //cout<<"Tdiff with offset=\n"<<Tdiff.translation()<<endl;
      //registrator_->RegisterMap2Map(Graph_nav_->GetPreviousNode()->GetMap(),Graph_nav_->GetCurrentNode()->GetMap(),Tdiff,score);
      //Graph_nav_->AddFactor(Graph_nav_->GetPreviousNode(),Graph_nav_->GetCurrentNode(),Tdiff,unit_covar);
    }
    if(!registration_succesfull){
      Tnow=T_world_to_local_map.inverse()*Tnow;//remap Tnow to global map frame
      cerr<<"REGISTRATION ERROR"<<endl;
      return;
    }
  }
cout<<"fuser loop"<<endl;
  lslgeneric::transformPointCloudInPlace(Tnow, cloud);// The cloud should now be centered around the robot pose in the map frame
  Affine3d scanSourcePose=Tnow*sensorPose_;//calculate the source of the scan
  Graph_nav_->GetCurrentNode()->updateMap(scanSourcePose,cloud);//Update map
  Tnow=T_world_to_local_map.inverse()*Tnow;//remap Tnow to global map frame
  pose_last_fuse_=Tnow;

  //--------plot
  //NDT2DMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(Graph_nav_->GetCurrentNode()->GetMap());
//  GraphPlot::SendGlobalMapToRviz(curr_node->GetMap(),1,T_world_to_local_map.inverse());

  GraphPlot::PlotPoseGraph(Graph_nav_);
  //-------end of plot
  nr_frames_++;
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
  if(Graph_nav_!=NULL && registrator_!=NULL){
    return false;
  }
  else{
    status="No object instance found for graph or registrator";
    return true;
  }
}
void GetCovarianceFromMotion(Matrix6d &cov,const Affine3d &Tm){

}
}
