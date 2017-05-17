#include "template/template_reg_type.h"


namespace libgraphMap{


TemplateRegType::TemplateRegType(const Affine3d &sensor_pose,regParamPtr paramptr):registrationType(sensor_pose,paramptr){

  TemplateRegTypeParamPtr param = boost::dynamic_pointer_cast< TemplateRegTypeParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Transfer all parameters from param to this class
    cout<<"Created registration type for template"<<endl;
  }
  else
    cerr<<"ndtd2d registrator has NULL parameters"<<endl;
}

TemplateRegType::~TemplateRegType(){}

bool TemplateRegType::Register(mapTypePtr maptype,Eigen::Affine3d &Tnow, const Eigen::Affine3d &Tmotion,pcl::PointCloud<pcl::PointXYZ> &cloud) {

  Eigen::Affine3d Tinit = Tnow * Tmotion;//Prediction for registration
  if(!enableRegistration_||!maptype->Initialized()){
    cout<<"Registration disabled - motion based on odometry"<<endl;
    Tnow=Tinit;
    return false;
  }
  else{
    TemplateMapTypePtr MapPtr = boost::dynamic_pointer_cast< TemplateMapType >(maptype);
    //Perform registration based on prediction "Tinit", your map "MapPtr" and the "cloud"
  }

}






/* ----------- Parameters ------------*/
TemplateRegTypeParam::~TemplateRegTypeParam(){}
TemplateRegTypeParam::TemplateRegTypeParam():registrationParameters(){
  GetParametersFromRos();
}
void TemplateRegTypeParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");//base class parameters
  nh.param<std::string>("super_important_parameter",super_important_parameter_,"default string");

}



}//end namespace

