#ifndef GRAPHFACTORY_H
#define GRAPHFACTORY_H
#include <stdio.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"

using namespace  std;
namespace libgraphMap{

typedef Eigen::Matrix<double,6,6> Matrix6d;
using Eigen::Affine3d;
class factor;
typedef boost::shared_ptr<factor> factorPtr;

class registrationType;
typedef boost::shared_ptr<registrationType> regTypePtr;

class ndtd2dRegType;
typedef boost::shared_ptr<ndtd2dRegType> ndtd2dregTypePtr;

class node;
typedef boost::shared_ptr<node> NodePtr;

class mapNode;
typedef boost::shared_ptr<mapNode> mapNodePtr;

class GraphMap;
typedef boost::shared_ptr<GraphMap> GraphMapPtr;

class mapType;
typedef boost::shared_ptr<mapType> mapTypePtr;

class mapParams;
typedef boost::shared_ptr<mapParams> mapParamPtr;

class NDT2DMapType;
typedef boost::shared_ptr<NDT2DMapType> NDT2DMapPtr;

class NDT2DMapParams;
typedef boost::shared_ptr<NDT2DMapParams> NDT2DMapParamsPtr;

/*!
 * ... Abstract class to implement map parameters.  ...
 */
class graphfactory{
public:
  static mapParamPtr CreateMapParam(string mapType);
  static mapTypePtr  CreateMap(const Eigen::Affine3d &mapPose,mapParamPtr mapparam);
  static GraphMapPtr CreateGraph(const Eigen::Affine3d &nodepose, mapParamPtr &mapparam);
  static mapNodePtr  CreateMapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam);
  static regTypePtr  CreateRegistrationType(string regTypeName);

  static factorPtr   CreateObservationFactor(mapNodePtr mapPose, NodePtr observationPose,const Eigen::Affine3d &diff,const Matrix6d &covar);
  static factorPtr   CreateMapNodeFactor(mapNodePtr prevMapPose, mapNodePtr nextMapPose,const Eigen::Affine3d &diff,const Matrix6d &covar);

private:
  graphfactory(){}
};





}
#endif // GRAPHFACTORY_H
