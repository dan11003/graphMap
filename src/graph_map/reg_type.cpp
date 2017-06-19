#include "graph_map/reg_type.h"
namespace libgraphMap{



/* ----------- Parameters ------------*/
NDTDLRegTypeParam::~NDTDLRegTypeParam(){}
NDTDLRegTypeParam::NDTDLRegTypeParam():registrationParameters(){
  GetParametersFromRos();
}
void NDTDLRegTypeParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");//base class parameters
  nh.param<std::string>("super_important_parameter",super_important_parameter_,"default string");

}



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
  do_soft_constraints_=regparam->do_soft_constraints_;
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
bool render_GT_map;
  ros::NodeHandle nh("~");//base class parameters
  cout<<"reading base class registration parameters"<<endl;
  nh.param("enable_registration",enableRegistration_,true);
  nh.param("registration_2D",registration2d_,false);
  nh.param("check_consistency",checkConsistency_,true);
  nh.param("sensor_range",sensorRange_,20.0);
  nh.param("size_z_meters",mapSizeZ_,0.8);
  nh.param("max_translation_norm",maxTranslationNorm_,0.4);
  nh.param("max_rotation_norm",maxRotationNorm_,M_PI/4);
  nh.param("renderGTmap",render_GT_map,false);
  nh.param("do_soft_constraints",do_soft_constraints_,false);
/*
  nh.param<double>("motion_params_Cd", motion_model_2d_.params.Cd, 0.005);
  nh.param<double>("motion_params_Ct", motion_model_2d_.params.Ct, 0.01);
  nh.param<double>("motion_params_Dd", motion_model_2d_.params.Dd, 0.001);
  nh.param<double>("motion_params_Dt", motion_model_2d_.params.Dt, 0.01);
  nh.param<double>("motion_params_Td", motion_model_2d_.params.Td, 0.001);
  nh.param<double>("motion_params_Tt", motion_model_2d_.params.Tt, 0.005);
*/
  if(render_GT_map)
    enableRegistration_=false;
}



}


