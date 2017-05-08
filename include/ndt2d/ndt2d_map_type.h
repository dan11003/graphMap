#ifndef NDT2DMAP_TYPE_H
#define NDT2DMAP_TYPE_H
#include "graphfactory.h"
#include <graph_map/map_type.h>
#include <ndt2d/ndt2d_map_param.h>
#include <ndt_map/ndt_map_hmt.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map_hmt.h>
#include "visualization/graph_plot.h"
#include "visualization/graph_plot.h"
#include <ndt_map/pointcloud_utils.h>
//#include <ndt_fuser/motion_model_2d.h>
#define ndt_map_type_name "ndt_2d_map"
namespace libgraphMap{
using namespace lslgeneric;

class NDT2DMapType:public mapType{
public:
  ~NDT2DMapType();
  virtual void update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<pcl::PointXYZ> &cloud);
  virtual NDTMap* GetMap() { return map_;}
  //Advanced
  virtual bool CompoundMapsByRadius(mapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius);
  NDT2DMapType(mapParamPtr paramptr);
  NDTMap *map_;

private:
  double resolution_,resolution_local_factor=1.;
  double sensor_range_;
  friend class graphfactory;
  void InitializeMap(const Eigen::Affine3d &Td,pcl::PointCloud<pcl::PointXYZ> &cloud);

  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    //ar & map_ ...
  }
  /*-----End of Boost serialization------*/

};


}
#endif // NDTMAP_TYPE_H
