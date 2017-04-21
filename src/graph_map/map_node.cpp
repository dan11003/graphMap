#include "graph_map/map_node.h"
namespace libgraphMap{

mapNode::mapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam){
  pose_=pose;
  map_=graphfactory::CreateMap(pose,mapparam);
}

void mapNode::updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud){
  map_->update(Tnow,cloud);
}
template<class Archive>
void mapNode::serialize(Archive & ar, const unsigned int version){
//ar &map_ & mapPose_ not sure how to do this
}
Affine3d node::GetPose() const{
  return pose_;
}
string mapNode::ToString(){
stringstream ss;
ss<<"map node, translation= "<<pose_.translation();
return ss.str();
}



}
