#ifndef WARTHOG_JPG_H
#define WARTHOG_JPG_H

// contraction/jpg.h
//
// common helper functions that are useful when applying
// jump point pruning rules to corner point graphs
//
// @author: dharabor
// @created: 2017-02--2
//


#include "forward.h"
#include "graph.h"
#include "jps.h"

namespace warthog
{

namespace jpg
{

// we label each edge of the CH with two pieces of information:
// (1) a FIRST label that specifies the direction of the first
// step on the diagonal-first path that begins at the edge's 
// tail node and finishes at its head node. 
// (2) a LAST label that is similar and which specifies the 
// direction of the last step on the path
enum step_type
{
    FIRST = 0,
    LAST = 1
};

// label the edges of the corner point graph with direction labels
// necessary to support jump point pruning. there are two labels:
//  - the direction of the first step on the path represented by the edge
//  - the direction of the last step on the path represented by the edge
void
compute_direction_labels(warthog::graph::corner_point_graph* g);

// see warthog::jpg::compute_direction_labels
void
label_edge(
        warthog::graph::edge* e, uint32_t e_tail_id, 
        warthog::graph::xy_graph* pg);

// see warthog::jpg::compute_direction_labels
void
process_edge(warthog::graph::edge* e, uint32_t e_tail_id,
             warthog::graph::xy_graph* g);

// see warthog::jpg::compute_direction_labels
inline warthog::jps::direction
get_dir(warthog::graph::edge* e, step_type which)
{
    return (warthog::jps::direction)
        ((uint8_t*)(&e->label_))[which && which];
}

// see warthog::jpg::compute_direction_labels
inline void
set_dir(warthog::graph::edge* e, step_type which, 
        warthog::jps::direction d)
{
    ((uint8_t*)(&e->label_))[which && which] = d;
}
 
} // warthog::jpg

} // warthog

#endif

