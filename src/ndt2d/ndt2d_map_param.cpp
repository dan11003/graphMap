#include "ndt2d/ndt2d_map_param.h"
using namespace std;
namespace libgraphMap{

NDT2DMapParam::NDT2DMapParam(){
  GetParametersFromRos();
}


void NDT2DMapParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;
  nh.param("resolution",resolution_,0.20);
  //nh.param("laser_variance_z",varz,resolution/4);
}

template<class Archive>
void NDT2DMapParam::serialize(Archive & ar, const unsigned int version){
  ar & boost::serialization::base_object<mapParam>(*this);
  //ar & cenx_& ceny_ &cenz_;
}



}
