#include "graph_map/graph_map.h"

using namespace std;
namespace libgraphMap{

GraphMap::GraphMap(const Affine3d &nodepose,const mapParamPtr &mapparam){
  prevNode_=NULL;
  currentNode_=graphfactory::CreateMapNode(nodepose,mapparam);//The first node to be added
  nodes_.push_back(currentNode_);
  mapparam_=mapparam;

}
mapNodePtr GraphMap::GetCurrentNode(){
  return currentNode_;
}
mapNodePtr GraphMap::GetPreviousNode(){
  return prevNode_;
}
void GraphMap::AddMapNode(const mapParamPtr &mapparam, const Affine3d &diff, const Matrix6d &cov){ //Add node with link uncertainty

  Affine3d newNodePose=Affine3d::Identity();
  newNodePose= currentNode_->GetPose()*diff;
  mapNodePtr newNode=graphfactory::CreateMapNode(newNodePose,mapparam);
  factorPtr sd=graphfactory::CreateMapNodeFactor(currentNode_,newNode,diff,cov);
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
Affine3d GraphMap::GetPreviousNodePose(){
  return prevNode_->GetPose();
}
bool GraphMap::SwitchToClosestMapNode(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map,const double radius){
  bool node_found=false;
  mapNodePtr closest_map_node=NULL;
  double closest_distance=-1.0;
  cout<<"currently at pose="<<Tnow.translation()<<endl;
  for(std::vector<NodePtr>::iterator itr_node = nodes_.begin(); itr_node != nodes_.end(); ++itr_node) { //loop thorugh all existing nodes
    if( (*itr_node)->WithinRadius(Tnow,radius) ){ //if a node is within radius of pose and the node is of type "map type"
      if(  mapNodePtr found_map_node_ptr = boost::dynamic_pointer_cast< mapNode >(*itr_node) ){ //A map node has now been found within radius)
        double found_distance= Eigen::Vector3d(Tnow.translation()-(*itr_node)->GetPose().translation()).norm();//Euclidian distance between robot pose and map node pose;
        cout<<"Node is within range of previously created map, distance="<<found_distance<<endl;
        if(closest_distance==-1.0|| found_distance<closest_distance){
          node_found=true;
          closest_map_node=found_map_node_ptr;
          closest_distance=found_distance;
          cout<<"closest node found at pose=\n"<<closest_map_node->GetPose().translation()<<endl;
        }
      }
      else
        cout<<"wrong type of previously created node"<<endl;
    }
  }
  if(node_found){//if any node at all was found, switch to the closest and update Tnow & T_world_to_local_map (transformation to current node)
    prevNode_=currentNode_;
    currentNode_=closest_map_node; //switch to that node
  }
  return node_found;
}

void GraphMap::AutomaticMapInterchange(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map){
  Tnow=T_world_to_local_map.inverse()*Tnow;//map Tnow to world frame
  // double min_size=mapparam_->sizex_< mapparam_->sizey_? mapparam_->sizex_ : mapparam_->sizey_; //select min map length
  //condition to never see past map size : radius_+sensorRange < min_size/2 of map
  double keep_map_radius=5;//min_size/2-mapparam_->max_range_;
  cout<<"keep map radius="<<keep_map_radius<<endl;

  if(! currentNode_->WithinRadius(Tnow,keep_map_radius)){ //No longer within radius of node
    cout<<"Left boundries of previous map, will  search through "<<nodes_.size()<<" node(s) to find a map node within range of"<<keep_map_radius<<"m"<<endl;
    if( SwitchToClosestMapNode(Tnow,cov,T_world_to_local_map,keep_map_radius))
      cout<<"switched to node="<<currentNode_->GetPose().translation()<<endl;
    else{
      cout<<"No node was found, will create a new map pose."<<endl;
      prevNode_=currentNode_;
      NDT2DMapPtr prev_ndt_map = boost::dynamic_pointer_cast< NDT2DMapType >(prevNode_->GetMap());
      AddMapNode(mapparam_,T_world_to_local_map*Tnow,cov); //if no node already exists, create a new node
      prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(),prevNode_->GetPose(),currentNode_->GetPose(),5.0);
    }
  }
  T_world_to_local_map=currentNode_->GetPose().inverse();
  Tnow=T_world_to_local_map*Tnow;//map Tnow from global to new local frame
}

}
