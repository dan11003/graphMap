#ifndef NDT2DMAPPARAM_H
#define NDT2DMAPPARAM_H
#include "graph_map/map_type.h"
#include <string.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "ros/ros.h"
using namespace std;
namespace libgraphMap{

/*!
 * ... Parameter class for mapType. Class is by choice of design fully public.  ...
 */

class NDT2DMapParam : public mapParam{
public:
  ~NDT2DMapParam(){}
  /*void SetParam(double resolution,
                         float cenx,
                          float ceny,
                           float cenz,
                            float sizex,
                             float sizey,
                              float sizez,
                               double max_range,
                                std::string directory,
                                 bool _saveOnDelete);*/
  void GetParametersFromRos();

  double resolution_;

  std::string directory_;
  bool saveOnDelete_, match2D_,beHMT,matchLaser;
protected:
  NDT2DMapParam();

private:
  friend class graphfactory;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version);
  /*-----End of Boost serialization------*/

};


}
#endif // NDT2DMAPPARAM_H
