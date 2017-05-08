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
  bool operator ==(const node& node_compare);
  virtual string ToString(){return "base node";}
  virtual Affine3d GetPose() const;
  virtual bool WithinRadius(const Affine3d &pose, const double &radius);
protected:
  node();
  unsigned id_;
  Eigen::Affine3d pose_;
};
/*!
* ... Class to represent a map node ...
*/
class mapNode:public node{

public:
  virtual void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud);
  virtual bool Initialized(){return initialized_;}
  virtual mapTypePtr GetMap(){return map_;}
  virtual string ToString();
protected:
  mapTypePtr map_;
  mapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam);
  bool initialized_=false;
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
