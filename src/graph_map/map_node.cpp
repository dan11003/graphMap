#include "graph_map/map_node.h"
namespace libgraphMap{


Node::Node(){
  static unsigned int num_nodes=0;
  id_=num_nodes;
  num_nodes++;
}

Affine3d Node::GetPose() const{
  return pose_;
}
bool Node::operator ==(const Node& node_compare){
  if(id_==node_compare.id_ && pose_.isApprox(node_compare.pose_))
    return true;
  else return false;
}

bool Node::WithinRadius(const Affine3d &pose, const double &radius){

  double distance= Eigen::Vector3d(pose.translation()-pose_.translation()).norm();
  if(distance<radius)
    return true;
  else return false;
}




MapNode::MapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam){
  pose_=pose;
  map_=GraphFactory::CreateMap(mapparam);
}

void MapNode::updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud){
  map_->update(Tnow,cloud);
  initialized_=true;
}
template<class Archive>
void MapNode::serialize(Archive & ar, const unsigned int version){
  //ar &map_ & mapPose_ not sure how to do this
}
string MapNode::ToString(){
  stringstream ss;
  ss<<"map node, translation= "<<pose_.translation();
  return ss.str();
}




}
