#ifndef GRAPH_MAP_H
#define GRAPH_MAP_H
#include "graphfactory.h"
#include "graph_map/map_type.h"
#include "graph_map/map_node.h"
#include "stdio.h"
#include <iostream>
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include <stdint.h>
#include<Eigen/StdVector>


namespace libgraphMap{

typedef std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > EigenAffineVector;

class GraphMap{
public:
  mapNodePtr GetCurrentNode();
  virtual void AddMapNode(const mapParamPtr &mapparam,const Eigen::Affine3d &diff,const Matrix6d &covar);//
  //virtual void OptimizeGraph(){} //Will try to find the optimal links between the local map nodes in the graph
   uint32_t MapSize(){return nodes_.size();}
   virtual string ToString();
   virtual Affine3d GetNodePose(int nodeNr);
   virtual Eigen::Affine3d GetCurrentNodePose();
protected:
  GraphMap(const Eigen::Affine3d &nodepose, const mapParamPtr &mapparam);
  mapNodePtr currentNode_;//The current node
  std::vector<NodePtr> nodes_;//Vector of all nodes in graph
  std::vector<factorPtr> factors_;
private:
  friend class graphfactory;
};
}
#endif // GRAPH_H
