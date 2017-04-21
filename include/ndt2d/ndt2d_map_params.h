#ifndef NDT2DMAPPARAMS_H
#define NDT2DMAPPARAMS_H
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

class NDT2DMapParams : public mapParams{
public:
  ~NDT2DMapParams(){}
  /*void SetParams(double resolution,
                         float cenx,
                          float ceny,
                           float cenz,
                            float sizex,
                             float sizey,
                              float sizez,
                               double max_range,
                                std::string directory,
                                 bool _saveOnDelete);*/
  void GetRosParamNDT2D();
  double resolution_;
  double sizex_;
  double sizey_;
  double sizez_;
  double max_range_;
  double min_laser_range_;
  std::string directory_;
  bool saveOnDelete_, match2D_,beHMT,matchLaser;
protected:
  NDT2DMapParams();

private:
  friend class graphfactory;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version);
  /*-----End of Boost serialization------*/

};


}
#endif // NDT2DMAPPARAMS_H
