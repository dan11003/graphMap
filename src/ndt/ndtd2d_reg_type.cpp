#include "ndt/ndtd2d_reg_type.h"
namespace libgraphMap{


NDTD2DRegType::NDTD2DRegType(const Affine3d &sensor_pose, RegParamPtr paramptr):registrationType(sensor_pose,paramptr){

  ndtd2dregParamPtr param_ptr = boost::dynamic_pointer_cast< ndtd2dRegParam >(paramptr);//Should not be NULL
  if(param_ptr!=NULL){
    resolution_=param_ptr->resolution_;
    resolutionLocalFactor_=param_ptr->resolutionLocalFactor_;
    //NDTMatcherD2D_2D parameters
    matcher2D_.ITR_MAX = param_ptr->matcher2D_ITR_MAX;
    matcher2D_.step_control=param_ptr->matcher2D_step_control;
    matcher2D_.n_neighbours=param_ptr->matcher2D_n_neighbours;
    cout<<"succesfully applied ndt d2d registration parameters"<<endl;
  }
  else
    cerr<<"ndtd2d registrator has NULL parameters"<<endl;
}

NDTD2DRegType::~NDTD2DRegType(){}

bool NDTD2DRegType::Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud) {

  if(!enableRegistration_||!maptype->Initialized()){
    cout<<"Registration disabled - motion based on odometry"<<endl;
    return true;
  }
  ///Create local map
  lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution_*resolutionLocalFactor_));
  ndlocal.guessSize(0,0,0,sensorRange_,sensorRange_,mapSizeZ_);
  ndlocal.loadPointCloud(cloud,sensorRange_);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
  Eigen::Affine3d Tinit = Tnow;//registration prediction
  //NDTMap * ptrmap=&ndlocal;
  //graphPlot::SendLocalMapToRviz(ptrmap,0,sensorPose_);

  //Get ndt map pointer
  NDT2DMapPtr MapPtr = boost::dynamic_pointer_cast< NDTMapType >(maptype);
  NDTMap *globalMap=MapPtr->GetMap();
  // cout<<"number of cell in (global/local) map"<<globalMap->getAllCells().size()<<","<<ndlocal.getAllCells().size()<<endl;
  bool matchSuccesfull;
  if(registration2d_){
    matchSuccesfull=matcher2D_.match(*globalMap, ndlocal,Tinit,true);
  }
  else if(!registration2d_){
    matchSuccesfull=matcher3D_.match( *globalMap, ndlocal,Tinit,true);
  }

  if(matchSuccesfull){
    Eigen::Affine3d diff = Tnow.inverse()*Tinit;//difference between prediction and registration
    Vector3d diff_angles=Vector3d(diff.rotation().eulerAngles(0,1,2));
    Eigen::AngleAxisd diff_rotation_(diff.rotation());


    if(checkConsistency_ && diff.translation().norm() > maxTranslationNorm_ ){
      cerr<<"registration failure: Translation too high"<<endl;
      cerr<<"movement="<<diff.translation().norm()<<"m  >  "<<diff.translation().norm()<<endl;
      return false;
    }
    else if(checkConsistency_ &&(diff_rotation_.angle() > maxRotationNorm_) ){
      cerr<<"registration failure: Rotation too high"<<endl;
      cerr<<"movement="<<diff_rotation_.angle()<<"rad  >  "<<maxRotationNorm_<<endl;
      //Tnow = Tnow * Tmotion;
      return false;
    }
    else{
    Tnow = Tinit;//return registered value
    return true;
    }
  }
  else{
  cerr<<"Registration unsuccesfull"<<endl;
  return false;
  }
}







/* ----------- Parameters ------------*/
ndtd2dRegParam::~ndtd2dRegParam(){}
ndtd2dRegParam::ndtd2dRegParam():registrationParameters(){
  GetParametersFromRos();
}
void ndtd2dRegParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");//base class parameters
  nh.param("registration_2D",registration2d_,true);
  nh.param("resolution",resolution_,1.0);
  nh.param("resolutionLocalFactor",resolutionLocalFactor_,1.0);
}



}//end namespace
