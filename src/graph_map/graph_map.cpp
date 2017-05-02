#include "graph_map/graph_map.h"

using namespace std;
namespace libgraphMap{

GraphMap::GraphMap(const Eigen::Affine3d &nodepose,const mapParamPtr &mapparam){
  currentNode_=graphfactory::CreateMapNode(nodepose,mapparam);//The first node to be added
  nodes_.push_back(currentNode_);
}
mapNodePtr GraphMap::GetCurrentNode(){
  return currentNode_;
}

void GraphMap::AddMapNode(const mapParamPtr &mapparam,const Eigen::Affine3d &diff,const Matrix6d &covar){ //Add node with link uncertainty

  Eigen::Affine3d newNodePose=Eigen::Affine3d::Identity();
  newNodePose= currentNode_->GetPose()*diff;
  mapNodePtr newNode=graphfactory::CreateMapNode(newNodePose,mapparam);
  factorPtr sd=graphfactory::CreateMapNodeFactor(currentNode_,newNode,diff,covar);
  factors_.push_back(sd);//Add connection between current and new node with link diff and covariance
  nodes_.push_back(newNode);
  currentNode_=newNode;
}
string GraphMap::ToString(){
  string s="Graph map: \n";
  for(int i=0;i<nodes_.size();i++){
    NodePtr ptr=nodes_[i];
    s=s+ptr->ToString()+"\n";
  }
  return s;
}
Affine3d GraphMap::GetNodePose(int nodeNr){
  if(nodeNr<MapSize())
    return nodes_[nodeNr]->GetPose();
}
Eigen::Affine3d GraphMap::GetCurrentNodePose(){
  return currentNode_->GetPose();
}
}
