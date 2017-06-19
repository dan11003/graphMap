#ifndef TEMPLATE_REG_TYPE_H
#define TEMPLATE_REG_TYPE_H

#include "graphfactory.h"
#include "graph_map/reg_type.h"
#include "graph_map/map_type.h"
#include "ndt_dl/ndtdl_map_type.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include "Eigen/Dense"
#include "math.h"
#include "ros/ros.h"
#include "ros/node_handle.h"


#define ndt_d2d_reg_type_name "ndt_d2d_reg"
namespace libgraphMap{
class NDTDLRegType:public registrationType{
public:
  ~NDTDLRegType();
  bool Register(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> &cloud, Matrix6d covar=unit_covar);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess
protected:
  string super_important_parameter_;
  NDTDLRegType(const Affine3d &sensor_pose,RegParamPtr paramptr);
private:
  friend class GraphFactory;
};


class NDTDLRegTypeParam:public registrationParameters{
public:
  ~NDTDLRegTypeParam();
  void GetParametersFromRos();//Get parametes from ros e.g. from the ros parameter server
  //but all your parameters here
  string super_important_parameter_;
protected:
  NDTDLRegTypeParam();//Constructor is protected to allow only graphcatory to instanciate or derived classes create this type
private:
  friend class GraphFactory;

};
}



#endif // TEMPLATE_REG_TYPE_H
