/**
 *  file map_type.h.
 */
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
 * ... Abstract class to present parameters for "mapType". this class contain generic parameters forr all map types.  ...
 */

class mapParam{
public:
  virtual ~mapParam()=0;
  string getMapName() const{return mapName_;}
protected:
  mapParam(){}
  bool initialized_;
  string mapName_;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){}
  /*-----End of Boost serialization------*/
};




/*!
 * ... Abstract class to implement local maps.  ...
 */

class mapType{

public:
  /*!
   * \brief update attempts to update the map based on point cloud data and the pose where the scan was taken from(scanner) in the world fram
   * \param Tnow transformation from world to sensor frame
   * \param cloud data to update map with
   */

  virtual void update(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud)=0;
protected:
  mapType(){initialized_=false;}
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
