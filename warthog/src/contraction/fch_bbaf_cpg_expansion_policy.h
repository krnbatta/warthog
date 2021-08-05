#ifndef WARTHOG_FCH_BBAF_CPG_EXPANSION_POLICY_H
#define WARTHOG_FCH_BBAF_CPG_EXPANSION_POLICY_H

// contraction/fch_bbaf_cpg_expansion_policy.h
//
// Forward-driven search with contraction hiearchies 
// on corner graphs. Combined with arc-flags + rectangular 
// bounding boxes to prune redundant up and down edges
//
// @author: dharabor
// @created: 2016-12-15
//

#include "expansion_policy.h"
#include "forward.h"
#include <vector>

namespace warthog
{

class fch_bbaf_cpg_expansion_policy : public expansion_policy
{
    public:
        fch_bbaf_cpg_expansion_policy(
                warthog::graph::corner_point_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::label::bbaf_labelling* lab);

        ~fch_bbaf_cpg_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        virtual size_t
        mem()
        {
            return expansion_policy::mem() +
                sizeof(this);
        }

    private:
        std::vector<uint32_t>* rank_;
        warthog::graph::corner_point_graph* g_;
        uint32_t search_id_;

        std::set<uint32_t> t_part_;
        warthog::geom::rectangle t_rect_;
        warthog::label::bbaf_labelling* lab_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }

        // customised filtering function;
        // specific for this expansion policy
        bool
        filter(uint32_t node_id, uint32_t edge_index, bool down);
};
}

#endif
