#ifndef WARTHOG_FCH_BBAF_EXPANSION_POLICY_H
#define WARTHOG_FCH_BBAF_EXPANSION_POLICY_H

// contraction/fch_bbaf_expansion_policy.h
//
// Forward-driven search in contraction hiearchies using 
// arc-flags + rectangular bounding boxes to prune redundant 
// up and down edges
//
// @author: dharabor
// @created: 2016-08-23
//

#include "expansion_policy.h"
#include "forward.h"

#include <vector>

namespace warthog
{

class fch_bbaf_expansion_policy : public expansion_policy
{
    public:
        fch_bbaf_expansion_policy(
                warthog::graph::xy_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::label::bbaf_labelling* lab);

        ~fch_bbaf_expansion_policy();

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
        warthog::graph::xy_graph* g_;
        warthog::label::bbaf_labelling* lab_;
        uint32_t search_id_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }

        // customised filter function specialised 
        // to this expansion policy; faster than using bbaf_filter
        bool 
        filter(uint32_t node_id, uint32_t edge_idx, bool down);
        
        uint32_t t_byte_;
        uint32_t t_bitmask_;
        int32_t tx_, ty_;
};
}

#endif
