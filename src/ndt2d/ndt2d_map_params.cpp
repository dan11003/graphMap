#include "ndt2d/ndt2d_map_params.h"
using namespace std;
namespace libgraphMap{

NDT2DMapParams::NDT2DMapParams(){
    GetRosParamNDT2D();
}

/*void NDT2DMapParams::SetParams(double resolution,
                                         float cenx,
                                         float ceny,
                                         float cenz,
                                         float sizex,
                                         float sizey,
                                         float sizez,
                                         double max_range,
                                         std::string directory,
                                         bool _saveOnDelete){
    resolution_=resolution;
    cenx_=cenx;
    ceny_=ceny;
    cenz_=cenz;
    sizex_=sizex;
    sizey_=sizey;
    sizez_=sizez;
    max_range_=max_range;
    directory_=directory;
    saveOnDelete_=_saveOnDelete;
}*/
void NDT2DMapParams::GetRosParamNDT2D(){
ros::NodeHandle nh;
cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;
nh.param("sensor_range",max_range_,3.);
nh.param("min_laser_range",min_laser_range_,0.1);
//nh.param("laser_variance_z",varz,resolution/4);
nh.param("resolution",resolution_,0.10);
nh.param("match2D",match2D_,false);
nh.param("beHMT",beHMT,false);
nh.param("matchLaser",matchLaser,false);
nh.param("size_x_meters",sizex_,10.);
nh.param("size_y_meters",sizey_,10.);
nh.param("size_z_meters",sizez_,2.);
}

template<class Archive>
void NDT2DMapParams::serialize(Archive & ar, const unsigned int version){
ar & boost::serialization::base_object<mapParams>(*this);
//ar & cenx_& ceny_ &cenz_;
ar & sizex_&sizey_&sizez_;
ar & max_range_;
ar & max_range_;
ar & directory_;
ar & saveOnDelete_;
}



}
