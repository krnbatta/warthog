#ifndef WARTHOG_CPG_EXPANSION_POLICY_H
#define WARTHOG_CPG_EXPANSION_POLICY_H

// cpg_expansion_policy.h
//
// This expansion policy creates a graph of corner points
// and searches for paths in that graph. Problems where
// the start location or the target location (or both)
// are not corner points are handled by an insertion 
// procedure.
//
// @author: dharabor
// @created: 2016-09-21
//

#include "expansion_policy.h"
#include "forward.h"
#include "gridmap.h"
#include "xy_graph.h"
#include <unordered_map>

namespace warthog
{

class cpg_expansion_policy : public expansion_policy
{
    public:
        cpg_expansion_policy(warthog::graph::corner_point_graph*);

        virtual 
        ~cpg_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t node_id,  int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        inline warthog::graph::corner_point_graph*
        get_graph() { return this->g_; }

        virtual size_t
        mem();

    private:
        warthog::graph::corner_point_graph* g_;
        uint32_t instance_id_at_last_insert_;
};

}

#endif

