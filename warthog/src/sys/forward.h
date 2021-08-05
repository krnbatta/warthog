#ifndef WARTHOG_FORWARD_H
#define WARTHOG_FORWARD_H

// ./memory/forward.h
//
// Forward declarations
//
// @author: dharabor
// @created: 2018-11-07
//

namespace warthog
{

class apriori_filter;
class apex_filter;
class bb_filter;
class dummy_filter;
class expansion_policy;
class euclidean_heuristic;
class gridmap;
class problem_instance;
class search_node;
class zero_heuristic;

template<typename H, typename E, typename Q>
class flexible_astar;

template<typename FILTER>
class graph_expansion_policy;

namespace graph
{

class node;
class edge;
class corner_point_graph;

template<class T_NODE, class T_EDGE>
class xy_graph_base;
typedef xy_graph_base<node, edge> xy_graph;

}

namespace label
{

class af_labelling;
class bb_labelling;
class bbaf_labelling;
class dfs_labelling;
class firstmove_labelling;

}

namespace jps
{

class corner_point_locator;

}

namespace mem
{
}

namespace cbs
{
}

}

#endif
