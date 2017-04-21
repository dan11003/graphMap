#ifndef REGISTRATIONTYPE_H
#define REGISTRATIONTYPE_H
#include "graphfactory.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "pcl/io/pcd_io.h"
namespace libgraphMap{
class registrationType{
public:

  virtual ~registrationType()=0;
  virtual bool Register(mapTypePtr maptype,Eigen::Affine3d Tnow, const Eigen::Affine3d &Tmotion,pcl::PointCloud<pcl::PointXYZ> &cloud){}//This methods attempts to register the point cloud versus the map using the affine transformation guess "Tm"
protected:
  bool checkConsistency_;
  double maxTranslationNorm_,maxRotationNorm_;
  double translationRegistrationDelta_, rotationRegistrationDelta_;
  unsigned int nrRegistrations_;
  registrationType();
  Eigen::Affine3d sensorPose_;//Translation between robot and sensor frame
private:
  friend class graphfactory;
};
}


#endif // REGISTRATIONTYPE_H
