#ifndef REGISTRATIONTYPE_H
#define REGISTRATIONTYPE_H
#include "graphfactory.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
namespace libgraphMap{


class registrationType{
public:

  virtual ~registrationType()=0;
  virtual bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud){}//This methods attempts to register the point cloud versus the map using the affine transformation guess "Tm"
protected:
  bool enableRegistration_;
  bool registration2d_;
  bool checkConsistency_;
  double maxTranslationNorm_,maxRotationNorm_;
  double translationRegistrationDelta_, rotationRegistrationDelta_;
  double sensorRange_;
  double mapSizeZ_;
  unsigned int  failed_registrations_;
  unsigned int  succesfull_registrations_;
  registrationType(const Affine3d &sensor_pose,RegParamPtr regparam);
  Eigen::Affine3d sensorPose_;//Translation between robot and sensor frame
private:
  friend class GraphFactory;
};



class registrationParameters{
public:
  virtual ~registrationParameters()=0;
  virtual void GetParametersFromRos();
  bool enableRegistration_;
  bool registration2d_;
  bool checkConsistency_;
  double maxTranslationNorm_,maxRotationNorm_;
  double translationRegistrationDelta_, rotationRegistrationDelta_;
  double sensorRange_;
  double mapSizeZ_;
protected:
  registrationParameters();
private:
  friend class GraphFactory;
};



}
#endif // REGISTRATIONTYPE_H
