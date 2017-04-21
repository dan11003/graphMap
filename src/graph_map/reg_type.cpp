#include "graph_map/reg_type.h"
namespace libgraphMap{
registrationType::registrationType(){
  checkConsistency_=true;
  maxTranslationNorm_ = 1.;
  maxRotationNorm_ = M_PI/4;
  nrRegistrations_=0;
}
registrationType::~registrationType(){}

}
