#include "ndt_dl/ndtdl_reg_type.h"


namespace libgraphMap{


NDTDLRegType::NDTDLRegType(const Affine3d &sensor_pose,RegParamPtr paramptr):registrationType(sensor_pose,paramptr){

  NDTDLRegTypeParamPtr param = boost::dynamic_pointer_cast< NDTDLRegTypeParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Transfer all parameters from param to this class
    cout<<"Created registration type for template"<<endl;
  }
  else
    cerr<<"ndtd2d registrator has NULL parameters"<<endl;
}

NDTDLRegType::~NDTDLRegType(){}

bool NDTDLRegType::Register(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> &cloud, Matrix6d cov) {

  cout<<"registration is disabled until it is implemented for map of type: "<<maptype->GetMapName()<<endl;
  return true;//Remove when registration has been implemented

  if(!enableRegistration_||!maptype->Initialized()){
    cout<<"Registration disabled - motion based on odometry"<<endl;

    return false;
  }
  else{
    NDTDLMapTypePtr MapPtr = boost::dynamic_pointer_cast< NDTDL >(maptype);
    //Perform registration based on prediction "Tinit", your map "MapPtr" and the "cloud"
  }

}








}//end namespace

