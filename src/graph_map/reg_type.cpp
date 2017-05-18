#include "graph_map/reg_type.h"
namespace libgraphMap{





/* -------Registration type---------------- */
registrationType::registrationType(const Eigen::Affine3d &sensor_pose, RegParamPtr regparam){
  if(regparam!=NULL){
  sensorPose_=sensor_pose;
  cout<<"created registration with sensor pose=\n"<<sensor_pose.translation()<<"\n linear=\n"<<sensor_pose.linear()<<endl;
  enableRegistration_ = regparam->enableRegistration_;
  registration2d_     = regparam->registration2d_;
  checkConsistency_   = regparam->checkConsistency_;
  maxTranslationNorm_ = regparam->maxTranslationNorm_;
  maxRotationNorm_    = regparam->maxRotationNorm_;
  translationRegistrationDelta_=regparam->translationRegistrationDelta_;
  rotationRegistrationDelta_=regparam->rotationRegistrationDelta_;
  sensorRange_        =regparam->sensorRange_;
  mapSizeZ_           =regparam->mapSizeZ_;
  failed_registrations_=0;
  succesfull_registrations_=0;
  cout<<"sucessfully applied registration parameters"<<endl;
  }
  else
    cerr<<"Registration parameters cannot be applied to registrator as parameter object does not exist"<<endl;
}
registrationType::~registrationType(){}




/* -------Parameters---------------- */
registrationParameters::registrationParameters(){
  GetParametersFromRos();
}
registrationParameters::~registrationParameters(){}
void registrationParameters::GetParametersFromRos(){

  ros::NodeHandle nh("~");//base class parameters
  cout<<"reading base class registration parameters"<<endl;
  nh.param("enable_registration",enableRegistration_,true);
  nh.param("registration_2D",registration2d_,false);
  nh.param("check_consistency",checkConsistency_,true);
  nh.param("sensor_range",sensorRange_,20.0);
  nh.param("size_z_meters",mapSizeZ_,0.8);
  nh.param("max_translation_norm",maxTranslationNorm_,0.4);
  nh.param("max_rotation_norm",maxRotationNorm_,M_PI/4);

  // translationRegistrationDelta_; vad för värde?
  // rotationRegistrationDelta_;
}



}


