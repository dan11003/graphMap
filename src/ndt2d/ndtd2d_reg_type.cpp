
#include "ndt2d/ndtd2d_reg_type.h"
namespace libgraphMap{


ndtd2dRegType::ndtd2dRegType(regParamPtr paramptr):registrationType(paramptr){

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

ndtd2dRegType::~ndtd2dRegType(){}

bool ndtd2dRegType::Register(mapTypePtr maptype,Eigen::Affine3d &Tnow, const Eigen::Affine3d &Tmotion,pcl::PointCloud<pcl::PointXYZ> &cloud) {

  ///Create local map
  lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution_*resolutionLocalFactor_));
  ndlocal.guessSize(0,0,0,sensorRange_,sensorRange_,mapSizeZ_);
  ndlocal.loadPointCloud(cloud,sensorRange_);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
  Eigen::Affine3d Tinit = Tnow * Tmotion;//registration prediction


  if(!enableRegistration_||!maptype->Initialized()){
    cout<<"Registration disabled - motion based on odometry"<<endl;
    Tnow=Tnow*Tmotion;
    return true;
  }


  //Get ndt map pointer
  NDT2DMapPtr MapPtr = boost::dynamic_pointer_cast< NDT2DMapType >(maptype);
  NDTMap *globalMap=MapPtr->GetMap();
  cout<<"number of cell in (global/local) map"<<globalMap->getAllCells().size()<<","<<ndlocal.getAllCells().size()<<endl;

  if(matcher2D_.match(*globalMap, ndlocal,Tinit,true))
  {
    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;//difference between prediction and registration
    if((diff.translation().norm() > maxTranslationNorm_ ||
        diff.rotation().eulerAngles(0,1,2).norm() > maxRotationNorm_) && checkConsistency_){
      fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
      cerr<<"Registration Failure"<<endl;
      Tnow = Tnow * Tmotion;
      return false;
    }else{
      Tnow = Tinit;
      return true;
    }
  }
  cerr<<"Registration unsuccesfully"<<endl;
  return false;
}







/* ----------- Parameters ------------*/
ndtd2dRegParam::~ndtd2dRegParam(){}
ndtd2dRegParam::ndtd2dRegParam():registrationParameters(){
  GetParametersFromRos();
}
void ndtd2dRegParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");//base class parameters
  registration2d_=true;
  nh.param("resolution",resolution_,0.4);
  nh.param("resolutionLocalFactor",resolutionLocalFactor_,1.0);
}



}//end namespace

