#include "ndt2d/ndt2d_map_type.h"
namespace libgraphMap{
using namespace std;

using namespace lslgeneric;

NDT2DMapType::NDT2DMapType(const Eigen::Affine3d &mapPose,NDT2DMapParamsPtr params) : mapType(mapPose){
  if(params!=NULL){
    resolution_=params->resolution_;
    sizeX_= params->sizex_;
    sizeY_= params->sizey_;
    sizeZ_= params->sizez_;
    sensorRange_=params->max_range_;
    map_=new NDTMapHMT(params->resolution_,
                       mapPose.translation()(0),
                       mapPose.translation()(1),
                       mapPose.translation()(2),//Add rotation of map here or should rotation only be used in context of nodes in graphs?
                       params->sizex_,
                       params->sizey_,
                       params->sizez_,
                       params->max_range_,
                       params->directory_,
                       params->saveOnDelete_);
    cout<<"created ndt2dmap type"<<endl;
  }
  else
    cerr<<"Cannot create instance of NDTmapHMT"<<std::endl;
}
NDT2DMapType::~NDT2DMapType(){}

void NDT2DMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.
  //Eigen::Affine3d NDTFuserHMT::update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud)
  if(initialized_){
    Eigen::Vector3d localMapSize(sensorRange_,sensorRange_,sizeZ_);
    map_->addPointCloudMeanUpdate(Tsensor.translation(),cloud,localMapSize, 1e5, 25, 2*sizeZ_, 0.06);

  }else{
    InitializeMap(Tsensor,cloud);
    initialized_ = true;
  }
}
void NDT2DMapType::InitializeMap(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){
  map_->addPointCloud(Tsensor.translation(),cloud, 0.1, 100.0, 0.1);
  map_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(), 0.1);
  //      lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
  //lslgeneric::transformPointCloudInPlace(mapPose_, cloud);

  //Tlast_fuse_ = Td;  should probably not be here as they are related to fuser
  //Todom = Tnow;
}

}
