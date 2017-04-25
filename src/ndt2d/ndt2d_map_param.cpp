#include "ndt2d/ndt2d_map_param.h"
using namespace std;
namespace libgraphMap{

NDT2DMapParam::NDT2DMapParam(){
  GetRosParamNDT2D();
}


void NDT2DMapParam::GetRosParamNDT2D(){
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;

  nh.param("sensor_range",max_range_,30.);
  nh.param("min_laser_range",min_laser_range_,0.5);
  //nh.param("laser_variance_z",varz,resolution/4);
  nh.param("resolution",resolution_,0.20);

  nh.param("size_x_meters",sizex_,20.);
  nh.param("size_y_meters",sizey_,20.);
  nh.param("size_z_meters",sizez_,20.);
  cout<<"map size: x="<<sizex_<<", y="<<sizex_<<", z="<<sizez_<<endl;
}

template<class Archive>
void NDT2DMapParam::serialize(Archive & ar, const unsigned int version){
  ar & boost::serialization::base_object<mapParam>(*this);
  //ar & cenx_& ceny_ &cenz_;
  ar & sizex_&sizey_&sizez_;
  ar & max_range_;
  ar & max_range_;
  ar & directory_;
  ar & saveOnDelete_;
}



}
