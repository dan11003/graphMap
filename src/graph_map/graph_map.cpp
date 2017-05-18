#include "graph_map/graph_map.h"

using namespace std;
namespace libgraphMap{

GraphMap::GraphMap(const Affine3d &nodepose,const MapParamPtr &mapparam,const GraphParamPtr graphparam){
  prevNode_=NULL;
  currentNode_=GraphFactory::CreateMapNode(nodepose,mapparam);//The first node to be added
  nodes_.push_back(currentNode_);
  mapparam_=mapparam;
  use_submap_=graphparam->use_submap_;
  interchange_radius_=graphparam->interchange_radius_;
  compound_radius_=graphparam->compound_radius_;

  cout<<"interchange_radius_ set to: "<<interchange_radius_<<endl;
  cout<<"use_submap_ set to: "<<use_submap_<<endl;

  //double min_size=mapparam->sizex_< mapparam->sizey_? mapparam->sizex_ : mapparam->sizey_; //select min map length
  //interchange_radius_=min_size/2-mapparam_->max_range_;//

}
MapNodePtr GraphMap::GetCurrentNode(){
  return currentNode_;
}
MapNodePtr GraphMap::GetPreviousNode(){
  return prevNode_;
}
void GraphMap::AddMapNode(const MapParamPtr &mapparam, const Affine3d &diff, const Matrix6d &cov){ //Add node with link uncertainty

  Affine3d newNodePose=Affine3d::Identity();
  newNodePose= currentNode_->GetPose()*diff;
  MapNodePtr newNode=GraphFactory::CreateMapNode(newNodePose,mapparam);
  FactorPtr sd=GraphFactory::CreateMapNodeFactor(currentNode_,newNode,diff,cov);
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
  MapNodePtr closest_map_node=NULL;
  double closest_distance=-1.0;
  cout<<"currently at pose="<<Tnow.translation()<<endl;
  for(std::vector<NodePtr>::iterator itr_node = nodes_.begin(); itr_node != nodes_.end(); ++itr_node) { //loop thorugh all existing nodes
    if( (*itr_node)->WithinRadius(Tnow,radius) ){ //if a node is within radius of pose and the node is of type "map type"
      if(  MapNodePtr found_map_node_ptr = boost::dynamic_pointer_cast< MapNode >(*itr_node) ){ //A map node has now been found within radius)
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

bool GraphMap::AutomaticMapInterchange(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map){
  bool mapChanged=false;
  if(use_submap_==false)
    return false;

  Tnow=T_world_to_local_map.inverse()*Tnow;//map Tnow to world frame
  if(! currentNode_->WithinRadius(Tnow,interchange_radius_)){ //No longer within radius of node
    cout<<"Left boundries of previous map, will  search through "<<nodes_.size()<<" node(s) to find a map node within range of"<<interchange_radius_<<"m"<<endl;
    if( SwitchToClosestMapNode(Tnow,cov,T_world_to_local_map,interchange_radius_)){
      cout<<"switched to node="<<currentNode_->GetPose().translation()<<endl;
    }
    else{
      cout<<"No node was found, will create a new map pose."<<endl;
      prevNode_=currentNode_;
      NDT2DMapPtr prev_ndt_map = boost::dynamic_pointer_cast< NDTMapType >(prevNode_->GetMap());
      AddMapNode(mapparam_,T_world_to_local_map*Tnow,cov); //if no node already exists, create a new node
      //Tdiff.translation()=T_target.translation()-T_source.translation();
      //pcl::PointXYZ center_pcl(Tdiff.translation()(0),Tdiff.translation()(1),Tdiff.translation()(2));
      prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(),prevNode_->GetPose(),currentNode_->GetPose(),compound_radius_);
      mapChanged=true;
    }
  }
  T_world_to_local_map=currentNode_->GetPose().inverse();
  Tnow=T_world_to_local_map*Tnow;//map Tnow from global to new local frame
  return mapChanged;
}


GraphParam::GraphParam(){
  GetParametersFromRos();
}
void GraphParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");
  nh.param("use_submap",use_submap_,false);
  nh.param("interchange_radius",interchange_radius_,100.0);
  nh.param("compound_radius",compound_radius_,100.0);
}
}
