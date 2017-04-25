#ifndef MAPTYPE_H
#define MAPTYPE_H
#include "graphfactory.h"
#include "eigen3/Eigen/Dense"
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <stdio.h>



using namespace std;
namespace libgraphMap{


/*!
 * ... Abstract class to implement a map. The class defines an abstraction of a map including generic parameters and methods  ...
 */

class mapParam{
public:
  virtual ~mapParam()=0;
  string getMapName() const{return mapName_;}
protected:
  mapParam(){}
  Eigen::Affine3d mapPose_;
  bool initialized_;
  string mapName_;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){}
  /*-----End of Boost serialization------*/
};




/*!
 * ... Abstract class to implement maps.  ...
 */

class mapType{

public:
  virtual void update(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud)=0;
  virtual void SetMapPose(const Eigen::Affine3d mapPose){mapPose_=mapPose;}
protected:
  mapType(Eigen::Affine3d mapPose){mapPose_=mapPose;initialized_=false;}
  Eigen::Affine3d mapPose_;
  double sizeX_,sizeY_,sizeZ_;
  bool initialized_;
  string mapName_;

  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & sizeX_&sizeY_&sizeZ_;
    ar & initialized_;
  }
  /*-----End of Boost serialization------*/
};





}
#endif // MAPTYPE_H
