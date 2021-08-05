#ifndef WARTHOG_FCH_BB_CPG_EXPANSION_POLICY
#define WARTHOG_FCH_BB_CPG_EXPANSION_POLICY

// contraction/fch_bb_cpg_expansion_policy.h
//
// Forward-driven search in contraction hiearchies 
// applied to corner point graphs. Plus rectangular
// geometric containers. 
//
// @author: dharabor
// @created: 2016-11-29
//

#include "forward.h"
#include "expansion_policy.h"
#include <vector>

namespace warthog
{

class fch_bb_cpg_expansion_policy : public expansion_policy
{
    public:
        fch_bb_cpg_expansion_policy(
                warthog::graph::corner_point_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::label::bb_labelling* lab);


        ~fch_bb_cpg_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        void
        set_apex(uint32_t apex) 
        { 
            apex_ = apex; 
            apex_reached_ = (apex == warthog::INF) ? true : false; 
        }

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        virtual size_t
        mem() { return expansion_policy::mem() + sizeof(this); }

    private:
        std::vector<uint32_t>* rank_;
        warthog::graph::corner_point_graph* g_;
        warthog::label::bb_labelling* lab_;
        uint32_t apex_;
        bool apex_reached_;
        uint32_t search_id_at_last_insert_;
        std::vector<uint32_t> proxy_xy_;
        warthog::geom::rectangle r_;
        warthog::problem_instance* instance_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }

        bool
        filter(uint32_t node_id, uint32_t edge_id);
};
}

#endif
