#ifndef GRAPHFACTORY_H
#define GRAPHFACTORY_H
#include <stdio.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"

using namespace  std;
namespace libgraphMap{

using Eigen::Vector3d;
using Eigen::Affine3d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
const Matrix6d unit_covar = (Eigen::Matrix<double, 6, 6>() << 0.1,0.0,0.0,0.0,0.0,0.0,
                                                              0.0,0.1,0.0,0.0,0.0,0.0,
                                                              0.0,0.0,0.1,0.0,0.0,0.0,
                                                              0.0,0.0,0.0,0.01,0.0,0.0,
                                                              0.0,0.0,0.0,0.0,0.01,0.0,
                                                              0.0,0.0,0.0,0.0,0.0,0.01).finished();

/*--------------------------- TEMPLATE FOR USAGE OF THE GRAPH LIBRARY ---------------------------------*/
/*!
 * The following types with name template_map_type are presented here as an example for how to create your own map and registration types
 */

/*!
 * \brief templateMapType implements the map or acts as a wrapper to your existing map classes
 * \brief templateMapTypePtr is the general way the map type is passed around, based om shared pointers to ensure memory no memory losses
 */
class TemplateMapType;
typedef boost::shared_ptr<TemplateMapType> TemplateMapTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
class TemplateMapParam;
typedef boost::shared_ptr<TemplateMapParam> TemplateMapParamPtr;

/*!
 * \brief templateRegType implements the registration or act as a wrapper to your existing registration types.
 * \brief templateRegTypePtr
 */
class TemplateRegType;
typedef boost::shared_ptr<TemplateRegType> TemplateRegTypePtr;

/*!
 * \brief templateRegTypeParam implements the parameters for the registration type <templateRegType>
 * \brief regParamPtr
 */
class TemplateRegTypeParam;
typedef boost::shared_ptr<TemplateRegTypeParam> TemplateRegTypeParamPtr;

/*--------------------------------END OF TEMPLATE -------------------------------------------------------*/





class NDT2DMapType;
typedef boost::shared_ptr<NDT2DMapType> NDT2DMapPtr;

class NDT2DMapParam;
typedef boost::shared_ptr<NDT2DMapParam> NDT2DMapParamPtr;

class ndtd2dRegParam;
typedef boost::shared_ptr<ndtd2dRegParam> ndtd2dregParamPtr;

class ndtd2dRegType;
typedef boost::shared_ptr<ndtd2dRegType> ndtd2dregTypePtr;

class factor;
typedef boost::shared_ptr<factor> factorPtr;

/*!
 *\brief registrationType is an abstract class for registration
 *\brief registrationParameters provides paramerters to the registration
 */
class registrationType;
typedef boost::shared_ptr<registrationType> regTypePtr;

class registrationParameters;
typedef boost::shared_ptr<registrationParameters> regParamPtr;

class mapType;
typedef boost::shared_ptr<mapType> mapTypePtr;

class mapParam;
typedef boost::shared_ptr<mapParam> mapParamPtr;


class Node;
typedef boost::shared_ptr<Node> NodePtr;

class MapNode;
typedef boost::shared_ptr<MapNode> mapNodePtr;

class GraphMap;
typedef boost::shared_ptr<GraphMap> GraphMapPtr;

class GraphParam;
typedef boost::shared_ptr<GraphParam> GraphParamPtr;





/*!
 * ... Abstract class to implement map parameters.  ...
 */
class graphfactory{
public:
  static mapParamPtr CreateMapParam(string mapType);
  static mapTypePtr  CreateMap(mapParamPtr mapparam);
  static GraphParamPtr CreateGraphParam();
  static GraphMapPtr CreateGraph(const Eigen::Affine3d &nodepose, mapParamPtr &mapparam,GraphParamPtr graphparam);
  static mapNodePtr  CreateMapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam);

  static regTypePtr  CreateRegistrationType(const Eigen::Affine3d &sensor_pose, regParamPtr regparam);
  static regParamPtr CreateRegParam(string regType);

  static factorPtr   CreateObservationFactor(mapNodePtr mapPose, NodePtr observationPose,const Eigen::Affine3d &diff,const Matrix6d &covar);
  static factorPtr   CreateMapNodeFactor(mapNodePtr prevMapPose, mapNodePtr nextMapPose,const Eigen::Affine3d &diff,const Matrix6d &covar);

private:
  graphfactory(){}
};





}
#endif // GRAPHFACTORY_H
