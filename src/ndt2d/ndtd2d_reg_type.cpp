#include "ndt2d/ndtd2d_reg_type.h"
namespace libgraphMap{
ndtd2dRegType::ndtd2dRegType(){}
ndtd2dRegType::~ndtd2dRegType(){
  //map parameters
  resolution_=1.0;
  resolutionLocalFactor_=1.0;
  //Matcher
  matcher2D_.ITR_MAX = 30;
  matcher2D_.step_control=true;
  matcher2D_.n_neighbours=2;
}
bool ndtd2dRegType::Register(mapTypePtr maptype,Eigen::Affine3d Tnow, const Eigen::Affine3d &Tmotion,pcl::PointCloud<pcl::PointXYZ> &cloud) {

  ///Set the cloud to sensor frame with respect to base
  lslgeneric::transformPointCloudInPlace(sensorPose_, cloud);

  ///Create local map
  lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution_*resolutionLocalFactor_));
  ndlocal.guessSize(0,0,0,sensorRange_,sensorRange_,mapSizeZ_);
  ndlocal.loadPointCloud(cloud,sensorRange_);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
  Eigen::Affine3d Tinit = Tnow * Tmotion;//registration prediction

  //retreive node map map
  NDT2DMapPtr MapPtr = boost::dynamic_pointer_cast< NDT2DMapType >(maptype);
  NDTMapHMT *globalMap=MapPtr->GetMap();
  //perform amtching
 if(matcher2D_.match(*globalMap, ndlocal,Tinit,true))
  {
    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;//difference between prediction and registration
    if((diff.translation().norm() > maxTranslationNorm_ ||
        diff.rotation().eulerAngles(0,1,2).norm() > maxRotationNorm_) && checkConsistency_){
      fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
      Tnow = Tnow * Tmotion;
      return false;
    }else{
      Tnow = Tinit;
      lslgeneric::transformPointCloudInPlace(Tnow, cloud);
      Eigen::Affine3d spose = Tnow*sensorPose_;
      /* Eigen::Affine3d diff_fuse = TprevFuse_.inverse()*Tnow;
        if(diff_fuse.translation().norm() > translation_fuse_delta ||
           diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
        {*/
      //map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06)
      return true;
    }
  }
  return false;
}

}//end namespace

