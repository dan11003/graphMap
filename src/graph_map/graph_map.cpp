#include "graph_map/graph_map.h"

using namespace std;
namespace libgraphMap{

GraphMap::GraphMap(const Affine3d &nodepose,const mapParamPtr &mapparam){
  currentNode_=graphfactory::CreateMapNode(nodepose,mapparam);//The first node to be added
  nodes_.push_back(currentNode_);
  mapparam_=mapparam;

}
mapNodePtr GraphMap::GetCurrentNode(){
  return currentNode_;
}

void GraphMap::AddMapNode(const mapParamPtr &mapparam,const Affine3d &diff,const Matrix6d &covar){ //Add node with link uncertainty

  Affine3d newNodePose=Affine3d::Identity();
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
Affine3d GraphMap::GetCurrentNodePose(){
  return currentNode_->GetPose();
}
void GraphMap::AutomaticMapInterchange(Affine3d pose,const Matrix6d &covar){
  double min_size=mapparam_->sizex_< mapparam_->sizey_? mapparam_->sizex_ : mapparam_->sizey_; //select min map length
  //condition to never see past map size : radius_+sensorRange < min_size/2 of map
  double keep_map_radius=7;//min_size/2-mapparam_->max_range_;
  cout<<"keep map radius="<<keep_map_radius<<endl;

  if(! currentNode_->WithinRadius(pose,keep_map_radius)){
    cout<<"Outside previous map limits, nodes to search through:="<<nodes_.size()<<endl;
    for(std::vector<NodePtr>::iterator itr_node = nodes_.begin(); itr_node != nodes_.end(); ++itr_node) { //loop thorugh all existing nodes
      if( (*itr_node)->WithinRadius(pose,keep_map_radius) ){ //if a node is within radius of pose and the node is of type "map type"
        cout<<"Pose is within range of previously created map"<<endl;
        if(  mapNodePtr map_node_ptr = boost::dynamic_pointer_cast< mapNode >(*itr_node) ){
          cout<<"correct type"<<endl;
          currentNode_=map_node_ptr; //switch to that node
          return;
        }
        else
          cout<<"wrong type of previously created node"<<endl;
      }
    } // no node available for interchange, create new node
    AddMapNode(mapparam_,currentNode_->GetPose().inverse()*pose,covar); //if no node already exists, create a new node

  }
}

}
