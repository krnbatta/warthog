#ifndef WARTHOG_FCH_FM_EXPANSION_POLICY_H
#define WARTHOG_FCH_FM_EXPANSION_POLICY_H

// contraction/fch_fm_expansion_policy.h
//
// An expansion policy that combines FCH with Compressed Path Databases,
// a pruning scheme that eliminates all but the optimal first move toward
// a given target.
//
// @author: dharabor
// @created: 2017-12-02
//

#include "xy_graph.h"
#include "firstmove_labelling.h"
#include "expansion_policy.h"

#include <vector>

namespace warthog
{

class fch_fm_expansion_policy : public expansion_policy
{
    public:
        fch_fm_expansion_policy(
                warthog::graph::xy_graph* graph,
                std::vector<uint32_t>* rank, 
                warthog::label::firstmove_labelling* lab,
                bool sort_successors=true);

        ~fch_fm_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        virtual inline size_t
        mem()
        {
            size_t retval = sizeof(this);
            retval += g_->get_num_nodes() * sizeof(uint8_t);
            retval += expansion_policy::mem();
            return retval;
        }

    private:
        std::vector<uint32_t>* rank_;
        warthog::graph::xy_graph* g_;
        uint8_t* heads_;

        warthog::label::firstmove_labelling* lab_;
        int32_t t_graph_id;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }
};
}

#endif
