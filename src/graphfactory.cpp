#include "graphfactory.h"
#include "graph_map/factor.h"
#include "graph_map/map_type.h"
#include "ndt2d/ndt2d_map_param.h"
#include "ndt2d/ndt2d_map_type.h"
#include "graph_map/map_node.h"
#include "graph_map/graph_map.h"
#include "graph_map/reg_type.h"
#include "ndt2d/ndtd2d_reg_type.h"

namespace libgraphMap{
mapParamPtr graphfactory::CreateMapParam(string mapname){
  if(mapname.compare(ndt_map_type_name)==0){
    cout<<"Graphfactory: Created parameters for map type: \""<<ndt_map_type_name<<"\""<<endl;
    NDT2DMapParamPtr paramPtr(new NDT2DMapParam());
    return paramPtr;
  }
  else if(mapname.compare("template")==0){
    cerr<<"Graphfactory:  Template, no map parameter instance created"<<endl;
    return NULL;
  }
  else{
    cerr<<"No map type exists with name: \""<<mapname<<"\""<<endl;
    return NULL;
  }
}

mapTypePtr graphfactory::CreateMap(const Eigen::Affine3d &mapPose,mapParamPtr mapparam){
  if(  NDT2DMapParamPtr ndt2MapParam = boost::dynamic_pointer_cast< NDT2DMapParam >(mapparam) ){ //perform typecast and check if not null conversion
    cout<<"Graphfactory: Created map of type: \""<<ndt_map_type_name<<"\""<<endl;
    return  mapTypePtr(new NDT2DMapType(mapPose,ndt2MapParam));
  }
  else if(mapparam->getMapName().compare("template")==0){
    cerr<<"Graphfactory: no map exists for \"template\""<<endl;
    return NULL;
  }
  else{
    cerr<<"Graphfactory: No map type exists for map parameters"<<endl;
    return NULL;
  }

}

GraphMapPtr graphfactory::CreateGraph(const Eigen::Affine3d &nodepose, mapParamPtr &mapparam){

  if(mapparam!=NULL){
    cout<<"Graphfactory: Creating graph"<<endl;
    GraphMapPtr graphPtr=boost::shared_ptr<GraphMap>(new GraphMap(nodepose,mapparam));
    return graphPtr;
  }
  else{
    cout<<"Graphfactory: parameter NULL to graph"<<endl;
    return NULL;
  }


}

mapNodePtr graphfactory::CreateMapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam){//Create a node
  cout<<"Graphfactory: Creating map node"<<endl;
  return boost::shared_ptr<mapNode>(new mapNode(pose,mapparam));
}


regTypePtr graphfactory::CreateRegistrationType(regParamPtr regparam){
  if(  regParamPtr ndt_d2d_reg_param_ptr = boost::dynamic_pointer_cast< ndtd2dRegParam >(regparam) ){ //perform typecast and check if not null conversion
    cout<<"Graphfactory: created ndtd2d registration type from parameter"<<endl;
    return regTypePtr(new ndtd2dRegType(ndt_d2d_reg_param_ptr));
  }
  else
    cerr<<"Failed to create object of registration type"<<endl;
  return NULL;
}
regParamPtr graphfactory::CreateRegParam(string regType){
  if(regType.compare(ndt_d2d_reg_type_name)==0){
    cout<<"Graphfactory: Creating parameters for registration type: \""<<ndt_d2d_reg_type_name<<"\""<<endl;
    return regParamPtr(new ndtd2dRegParam());
  }
  else{
    cerr<<"No registration type with name: \""<<regType<<"\""<<endl;
    return NULL;
  }
}

factorPtr graphfactory::CreateMapNodeFactor(mapNodePtr prevMapPose, mapNodePtr nextMapPose, const Eigen::Affine3d &diff, const Matrix6d &covar){
  factorPtr factorptr=boost::shared_ptr<factor>(new factor(prevMapPose,nextMapPose,diff));
  return factorptr;
}

}
