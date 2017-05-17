#ifndef TEMPLATE_REG_TYPE_H
#define TEMPLATE_REG_TYPE_H

#include "graphfactory.h"
#include "graph_map/reg_type.h"
#include "graph_map/map_type.h"
#include "template/template_map_type.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include "Eigen/Dense"
#include "math.h"
#include "ros/ros.h"
#include "ros/node_handle.h"


#define ndt_d2d_reg_type_name "ndt_d2d_reg"
namespace libgraphMap{
class TemplateRegType:public registrationType{
public:
  ~TemplateRegType();
  bool Register(mapTypePtr maptype, Eigen::Affine3d &Tnow, const Eigen::Affine3d &Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess
protected:
  string super_important_parameter_;
  TemplateRegType(const Affine3d &sensor_pose,regParamPtr paramptr);
private:
  friend class graphfactory;
};


class TemplateRegTypeParam:public registrationParameters{
public:
  ~TemplateRegTypeParam();
  void GetParametersFromRos();//Get parametes from ros e.g. from the ros parameter server
  //but all your parameters here
  string super_important_parameter_;
protected:
  TemplateRegTypeParam();//Constructor is protected to allow only graphcatory to instanciate or derived classes create this type
private:
  friend class graphfactory;

};
}





#endif // TEMPLATE_REG_TYPE_H
