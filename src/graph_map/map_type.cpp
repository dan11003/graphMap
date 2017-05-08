#include "graph_map/map_type.h"
namespace libgraphMap{


mapParam::~mapParam(){}

mapParam::mapParam(){
  GetParametersFromRos();
}
void mapParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");//base class parameters
  nh.param("sensor_range",max_range_,30.);
  nh.param("min_laser_range",min_range_,0.5);
  nh.param("size_x_meters",sizex_,20.);
  nh.param("size_y_meters",sizey_,20.);
  nh.param("size_z_meters",sizez_,20.);
  cout<<"read mapType parameters from ros"<<endl;
  cout<<ToString()<<endl;
}
string mapParam::ToString(){
  stringstream ss;
  ss<<"Base map parameters:"<<endl;
  ss<<"Range(max/min)=("<<max_range_<<"/"<<min_range_<<endl;
  ss<<"size(x,y,z)=("<<sizex_<<","<<sizey_<<","<<sizez_<<")";
  return ss.str();
}



mapType::mapType(mapParamPtr param){
  initialized_=false;
  sizex_= param->sizex_;
  sizey_= param->sizey_;
  sizez_= param->sizez_;
  max_range_=param->max_range_;
  min_range_=param->min_range_;
}

bool mapType::CompoundMapsByRadius(mapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){
  cout<<"Compunding map not possible in base class"<<endl;
  return false;
}

}
