#include "ndt2d/ndt2d_map_type.h"
namespace libgraphMap{
using namespace std;

using namespace lslgeneric;

NDT2DMapType::NDT2DMapType( mapParamPtr paramptr) : mapType(paramptr){
  NDT2DMapParamPtr param = boost::dynamic_pointer_cast< NDT2DMapParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    resolution_=param ->resolution_;
    map_ = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution_));
    map_->initialize(0.0,0.0,0.0,param->sizex_,param->sizey_,param->sizez_);
    cout<<"created ndt2dmap"<<endl;
  }
  else
    cerr<<"Cannot create instance of NDTmapHMT"<<std::endl;
}
NDT2DMapType::~NDT2DMapType(){}

void NDT2DMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

  if(initialized_){
    Eigen::Vector3d localMapSize(max_range_,max_range_,sizez_);
    map_->addPointCloudMeanUpdate(Tsensor.translation(),cloud,localMapSize, 1e5, 25, 2*sizez_, 0.06);

  }else{
    InitializeMap(Tsensor,cloud);
    initialized_ = true;
  }
}
void NDT2DMapType::InitializeMap(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){
  cout<<"initialize map"<<endl;
  map_->addPointCloud(Tsensor.translation(),cloud, 0.1, 100.0, 0.1);
  map_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(), 0.1);
}

}
