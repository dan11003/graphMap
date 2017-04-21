#ifndef MAP_NODE_H
#define MAP_NODE_H
#include "graphfactory.h"
#include "graph_map/map_type.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "stdio.h"
#include <iostream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>



namespace libgraphMap{
/*!
* ... Class to represent a node ...
*/
class node{
protected:
public:
  virtual string ToString(){return "base node";}
  virtual Affine3d GetPose() const;
protected:
  Eigen::Affine3d pose_;
};
/*!
* ... Class to represent a map node ...
*/
class mapNode:public node{

public:
  virtual void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud);

  virtual mapTypePtr GetMap(){return map_;}
  virtual string ToString();
protected:
  mapTypePtr map_;
  mapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam);
private:
  friend class graphfactory;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version);
  /*-----End of Boost serialization------*/

};


}
#endif // MAP_NODE_H
