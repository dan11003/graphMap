#include "ndt2d/ndt2d_map_type.h"
namespace libgraphMap{
using namespace std;

using namespace lslgeneric;

NDT2DMapType::NDT2DMapType(const Eigen::Affine3d &mapPose, NDT2DMapParamPtr param) : mapType(mapPose){
  if(param!=NULL){
    resolution_=param->resolution_;
    sizeX_= param->sizex_;
    sizeY_= param->sizey_;
    sizeZ_= param->sizez_;
    sensorRange_=param->max_range_;
    map_ = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution_));
    map_->initialize(mapPose.translation()(0),mapPose.translation()(1),mapPose.translation()(2),param->sizex_,param->sizey_,param->sizez_);
    cout<<"created ndt2dmap type at pose= \n"<<mapPose.translation()<<endl;
  }
  else
    cerr<<"Cannot create instance of NDTmapHMT"<<std::endl;
}
NDT2DMapType::~NDT2DMapType(){}

void NDT2DMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

  if(initialized_){
    Eigen::Vector3d localMapSize(sensorRange_,sensorRange_,sizeZ_);
    map_->addPointCloudMeanUpdate(Tsensor.translation(),cloud,localMapSize, 1e5, 25, 2*sizeZ_, 0.06);

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
