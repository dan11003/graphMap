#include "graph_map_fuser.h"
namespace libgraphMap{
using namespace std;
GraphMapFuser::GraphMapFuser(string maptype, string registratorType, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose){
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
  Graph_nav_ =GraphFactory::CreateGraphNavigator(init_pose,mapParam_,graph_param_);
  registrator_=GraphFactory::CreateRegistrationType(sensorPose,regParam_);
  sensorPose_=sensorPose;
  nr_frames_=0;
  initialized_=true;
}
GraphMapFuser::GraphMapFuser(  RegParamPtr regParam,  MapParamPtr mapParam, GraphParamPtr graph_param, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose){
  mapParam_=mapParam;
  regParam_=regParam;
  graph_param_=graph_param;
  Graph_nav_ =GraphFactory::CreateGraphNavigator(init_pose,mapParam_,graph_param_);
  registrator_=GraphFactory::CreateRegistrationType(sensorPose,regParam_);
  sensorPose_=sensorPose;
  nr_frames_=0;
  initialized_=true;
  pose_last_fuse_=init_pose;
}
//!
//! \brief GraphMapFuser::KeyFrameBasedFuse
//! \param Tnow pose of base specified in the GLOBAL world frame
//! \return true if base has moved outside bounds and can fuse the current frame, otherwise return false
//!
bool GraphMapFuser::KeyFrameBasedFuse(const Affine3d &Tnow ){
  Affine3d diff=pose_last_fuse_.inverse()*Tnow;
  Eigen::Vector3d Tmotion_euler = diff.rotation().eulerAngles(0,1,2);

  ndt_generic::normalizeEulerAngles(Tmotion_euler);
  cout<<"translation diff="<<diff.translation().norm()<<",rotation diff="<<Tmotion_euler.norm()<<endl;
  if(use_keyframe_ ){
    if(diff.translation().norm()>min_keyframe_dist_ || Tmotion_euler.norm()>(min_keyframe_rot_deg_*M_PI/180.0))
      return true;
    else
      return false;
  }
  else
    return true;
}

/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 */

void GraphMapFuser::ProcessFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow, const Eigen::Affine3d &Tmotion){
  //plotGTCloud(cloud);
  bool map_node_created,registration_succesfull=true,fuse_this_frame=false;
  static bool map_node_changed=false;
  if(!initialized_){
    cerr<<"fuser not initialized"<<endl;
    return;
  }
  fuse_this_frame=KeyFrameBasedFuse(Tnow);//fuse frame based on distance traveled
  Eigen::Affine3d T_world_to_local_map=Graph_nav_->GetCurrentNodePose().inverse(); //transformation from node to world frame
  Tnow=T_world_to_local_map*Tnow;//change frame to local map

if(fuse_this_frame)
  cout<<"time to fuse a new frame="<<nr_frames_<<endl;

  Matrix6d motion_cov=motion_model_2d_.getCovMatrix6(Tmotion, 1., 1., 1.);
  lslgeneric::transformPointCloudInPlace(sensorPose_, cloud);//Transform cloud into robot frame before registrating

  if(fuse_this_frame||map_node_changed){
    registration_succesfull = registrator_->Register(Graph_nav_->GetCurrentNode()->GetMap(),Tnow,cloud,motion_cov);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration
  }

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
    nr_frames_++;
    return;
  }
  if(fuse_this_frame||map_node_changed){
    lslgeneric::transformPointCloudInPlace(Tnow, cloud);// The cloud should now be centered around the robot pose in the map frame
    Graph_nav_->GetCurrentNode()->updateMap(Tnow*sensorPose_,cloud);//Update map, provided transform is the pose of the sensor in the world which is where the scan was taken from
  }

  Tnow=T_world_to_local_map.inverse()*Tnow;//remap Tnow to global map frame
  if(fuse_this_frame||map_node_changed)
     pose_last_fuse_=Tnow;

  nr_frames_++;
  if(visualize_){
    NDTMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(Graph_nav_->GetCurrentNode()->GetMap());
    GraphPlot::SendGlobalMapToRviz(curr_node->GetMap(),1,T_world_to_local_map.inverse());
    GraphPlot::PlotPoseGraph(Graph_nav_);
  }
}
void GraphMapFuser::plotGTCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud){
  lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(0.4));
  // for(int i=0;i<cloud.size();i+=500){
  // cout<<cloud[i].x<<","<<cloud[i].y<<","<<cloud[i].z<<endl;
  // }
  ndlocal.guessSize(0,0,0,100,100,8);
  ndlocal.loadPointCloud(cloud,30.0);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
  cout<<"CLOUD SIZE IS="<<cloud.size()<<endl;
  cout<<"ndt cloud size="<<ndlocal.getAllCells().size()<<endl;
  NDTMap * ptrmap=&ndlocal;
  GraphPlot::SendLocalMapToRviz(ptrmap,1);
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
