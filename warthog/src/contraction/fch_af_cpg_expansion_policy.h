#ifndef WARTHOG_FCH_AF_CPG_EXPANSION_POLICY_H
#define WARTHOG_FCH_AF_CPG_EXPANSION_POLICY_H

// contraction/fch_af_cpg_expansion_policy.h
//
// Forward-driven search in contraction hiearchies built
// from corner graphs. Further combined with arc-flags to 
// prune redundant up and down edges
//
// @author: dharabor
// @created: 2016-12-13
//

#include "forward.h"
#include "expansion_policy.h"
#include <vector>

namespace warthog
{

class fch_af_cpg_expansion_policy : public expansion_policy
{
    public:
        fch_af_cpg_expansion_policy(
                warthog::graph::corner_point_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::label::af_labelling* afl);

        ~fch_af_cpg_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        void
        set_apex(uint32_t apex) 
        { 
            apex_ = apex; 
        }

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
        warthog::label::af_labelling* afl_;
        std::set<uint32_t> t_part_;

        uint32_t search_id_;
        bool apex_reached_;
        uint32_t apex_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }

        bool
        filter(uint32_t node_id, uint32_t edge_index);
};
}

#endif
