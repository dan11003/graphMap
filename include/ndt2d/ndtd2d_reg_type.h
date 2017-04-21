#ifndef NDTD2DREGTYPE_H
#define NDTD2DREGTYPE_H
#include "graphfactory.h"
#include "graph_map/reg_type.h"
#include "graph_map/map_type.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include "Eigen/Dense"
#include <ndt_map/ndt_map.h>
#include "ndt_map/lazy_grid.h"
#include "ndt_map/ndt_map_hmt.h"
#include "math.h"
#include <ndt_map/pointcloud_utils.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_sc.h>
#include "ndt2d/ndt2d_map_type.h"
namespace libgraphMap{
using namespace lslgeneric;
class ndtd2dRegType:public registrationType{
public:
  ~ndtd2dRegType();
  bool Register(mapTypePtr maptype, Eigen::Affine3d Tnow, const Eigen::Affine3d &Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess
private:
  NDTMatcherD2D_2D matcher2D_;
  double resolution_,resolutionLocalFactor_;
  double sensorRange_;
  double mapSizeZ_;
  ndtd2dRegType();
  friend class graphfactory;
};
}
#endif // NDTD2DREGTYPE_H

