#ifndef WARTHOG_FCH_X_EXPANSION_POLICY_H
#define WARTHOG_FCH_X_EXPANSION_POLICY_H

// contraction/fch_x_expansion_policy.h
//
// Forward-driven search in contraction hiearchies using 
// an apex filter. the filter assumes the apex of the path
// is known apriori 
//
// This expansion policy is not practically interesting and only useful to 
// investigate the behaviour of FCH search.
//
// @author: dharabor
// @created: 2017-10-09
//

#include "expansion_policy.h"

#include <vector>

namespace warthog
{

class fch_x_expansion_policy : public expansion_policy
{
    public:
        fch_x_expansion_policy(
                warthog::graph::xy_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::apex_filter*);

        ~fch_x_expansion_policy();

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
        warthog::apex_filter* filter_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }
};
}

#endif
