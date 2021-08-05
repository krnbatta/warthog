#ifndef WARTHOG_FCH_AF_EXPANSION_POLICY_H
#define WARTHOG_FCH_AF_EXPANSION_POLICY_H

// contraction/fch_af_expansion_policy.h
//
// Forward-driven search in contraction hiearchies using 
// arc-flags to prune redundant up and down edges
//
// @author: dharabor
// @created: 2016-08-23
//

#include "expansion_policy.h"
#include <vector>

namespace warthog
{

class af_filter;
class problem_instance;
class search_node;
class fch_af_expansion_policy : public expansion_policy
{
    public:
        fch_af_expansion_policy(
                warthog::graph::xy_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::af_filter*);

        ~fch_af_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        void
        set_apex(uint32_t apex) 
        { 
            apex_ = apex; 
        }

        virtual size_t
        mem()
        {
            return expansion_policy::mem() +
                sizeof(this);
        }


    private:
        std::vector<uint32_t>* rank_;
        warthog::graph::xy_graph* g_;
        warthog::af_filter* filter_;
        bool apex_reached_;
        uint32_t apex_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }
};
}

#endif
