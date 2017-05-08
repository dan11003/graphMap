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
#include "visualization/graph_plot.h"
#include "ndt_map/ndt_map.h"
#include "ndt2d/ndt2d_map_type.h"

namespace libgraphMap{
using namespace lslgeneric;
typedef std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > EigenAffineVector;

class GraphMap{
public:
  /*!
   * \brief UpdateGraph is  high level interface to graph class. It allows for automatic selection and creation of mapnodes.
   */
  /*!
   * \brief AutomaticMapInterchange Automatically interchange between or create new map upon certain conditions
   * \param Tnow pre: Pose is described in the local mapNode frame. Post: Pose is described in the current local mapNode frame and is properly updated if a new frame has been selected
   * \param cov contain the movement uncertainty accumulated since last node
   * \param T_world_to_local pre: Not used: Post T_world_to_local contain the mapping from world to the current local mapNode frame.
   */
  void AutomaticMapInterchange(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map);
  virtual void AddMapNode(const mapParamPtr &mapparam,const Eigen::Affine3d &diff,const Matrix6d &cov);//
  mapNodePtr GetCurrentNode();
  mapNodePtr GetPreviousNode();
  uint32_t MapSize(){return nodes_.size();}
  virtual string ToString();
  virtual Affine3d GetNodePose(int nodeNr);
  virtual Eigen::Affine3d GetCurrentNodePose();
  virtual Eigen::Affine3d GetPreviousNodePose();

protected:
  GraphMap(const Eigen::Affine3d &nodepose, const mapParamPtr &mapparam);
  bool SwitchToClosestMapNode(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map,const double radius);
  mapNodePtr currentNode_,prevNode_;//The current node
  std::vector<NodePtr> nodes_;//Vector of all nodes in graph
  std::vector<factorPtr> factors_;
  mapParamPtr mapparam_;
private:
  friend class graphfactory;
};
}
#endif // GRAPH_H
