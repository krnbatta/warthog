#ifndef WARTHOG_FCH_BB_EXPANSION_POLICY
#define WARTHOG_FCH_BB_EXPANSION_POLICY

// contraction/fch_bb_expansion_policy.h
//
// An expansion policy for forward-driven
// search in contraction hiearchies combined 
// with a bounding-box filtering scheme
//
// @author: dharabor
// @created: 2016-08-02
//

#include "expansion_policy.h"
#include "forward.h"
#include <vector>

namespace warthog
{

class fch_bb_expansion_policy : public expansion_policy
{
    public:
        fch_bb_expansion_policy(
                warthog::graph::xy_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::bb_filter* nf);

        ~fch_bb_expansion_policy();

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
            apex_reached_ = (apex == warthog::INF) ? true : false; 
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
        warthog::bb_filter* nf_;
        uint32_t apex_;
        bool apex_reached_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }
};
}

#endif
