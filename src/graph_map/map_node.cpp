#include "graph_map/map_node.h"
namespace libgraphMap{


node::node(){
  static unsigned int num_nodes=0;
  id_=num_nodes;
  num_nodes++;
}

Affine3d node::GetPose() const{
  return pose_;
}
bool node::operator ==(const node& node_compare){
  if(id_==node_compare.id_ && pose_.isApprox(node_compare.pose_))
    return true;
  else return false;
}

bool node::WithinRadius(const Affine3d &pose, double &radius){

  double distance= Eigen::Vector3d(pose.translation()-pose_.translation()).norm();
  cout<<"distance"<<distance<<endl;
  if(distance<radius)
    return true;
  else return false;
}




mapNode::mapNode(const Eigen::Affine3d &pose,const mapParamPtr &mapparam){
  pose_=pose;
  map_=graphfactory::CreateMap(mapparam);
}

void mapNode::updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud){
  map_->update(Tnow,cloud);
  initialized_=true;
}
template<class Archive>
void mapNode::serialize(Archive & ar, const unsigned int version){
  //ar &map_ & mapPose_ not sure how to do this
}
string mapNode::ToString(){
  stringstream ss;
  ss<<"map node, translation= "<<pose_.translation();
  return ss.str();
}




}
