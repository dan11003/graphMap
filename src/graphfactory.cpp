#include "graphfactory.h"
#include "graph_map/factor.h"
#include "graph_map/map_type.h"
#include "ndt2d/ndt2d_map_params.h"
#include "ndt2d/ndt2d_map_type.h"
#include "graph_map/map_node.h"
#include "graph_map/graph_map.h"
#include "graph_map/reg_type.h"
#include "ndt2d/ndtd2d_reg_type.h"

namespace libgraphMap{
mapParamPtr graphfactory::CreateMapParam(string mapname){
  if(mapname.compare("ndt2dmap")==0){
    cout<<"Graphfactory: created ndt2dmap parameters"<<endl;
    NDT2DMapParamsPtr paramPtr(new NDT2DMapParams());
    return paramPtr;
  }
  else if(mapname.compare("template")==0){
    cout<<"Graphfactory: template"<<endl;
    return NULL;
  }
  else return NULL;
}

mapTypePtr graphfactory::CreateMap(const Eigen::Affine3d &mapPose,mapParamPtr mapparam){
  if(  NDT2DMapParamsPtr ndt2MapParams = boost::dynamic_pointer_cast< NDT2DMapParams >(mapparam) ){ //perform typecast and check if not null conversion
    return  mapTypePtr(new NDT2DMapType(mapPose,ndt2MapParams));
  }
  else if(mapparam->getMapName().compare("template")==0){
    //Dont forget to type cast mapparam and make sure result of cast is non-NULL
    cout<<"create template map"<<endl;
    return NULL;
  }
  else{
    cerr<<"No map type for derived class of map parameter"<<endl;
    return NULL;
  }

}

GraphMapPtr graphfactory::CreateGraph(const Eigen::Affine3d &nodepose, mapParamPtr &mapparam){
    cout<<"Creating graph"<<endl;
  if(mapparam!=NULL){
     GraphMapPtr graphPtr=boost::shared_ptr<GraphMap>(new GraphMap(nodepose,mapparam));
     return graphPtr;
  }
  else{
      cout<<"parameter NULL to graph"<<endl;
      return NULL;
  }


}

mapNodePtr graphfactory::CreateMapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam){//Create a node
  cout<<"creating node";
  return boost::shared_ptr<mapNode>(new mapNode(pose,mapparam));
}


regTypePtr graphfactory::CreateRegistrationType(string regTypeName){
if(regTypeName.compare("ndtd2dreg")==0)
  return regTypePtr(new ndtd2dRegType());

}
factorPtr graphfactory::CreateMapNodeFactor(mapNodePtr prevMapPose, mapNodePtr nextMapPose, const Eigen::Affine3d &diff, const Matrix6d &covar){
factorPtr factorptr=boost::shared_ptr<factor>(new factor(prevMapPose,nextMapPose,diff));
return factorptr;
}

}
