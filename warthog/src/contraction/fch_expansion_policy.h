#ifndef WARTHOG_FCH_EXPANSION_POLICY_H
#define WARTHOG_FCH_EXPANSION_POLICY_H

// contraction/fch_expansion_policy.h
//
// An expansion policy for forward-driven
// search in contraction hiearchies.
//
// The idea is simple:
// When expanding a node, look at the rank
// of the parent relative to the current node.
//
// If the parent is smaller the search is traveling up in the hierarchy and
// every neighbour is generated.
//
// If the parent is larger the search is traveling down in the hiearchy
// and only down-ward neighbours are generated.
//
// The approach preserves optimality.
//
// @author: dharabor
// @created: 2016-07-18
//

#include "contraction.h"
#include "expansion_policy.h"
#include <vector>

namespace warthog
{

class fch_expansion_policy : public expansion_policy
{
    public:
        fch_expansion_policy(
                warthog::graph::xy_graph* graph,
                std::vector<uint32_t>* rank,
                warthog::ch::search_direction = warthog::ch::ANY,
                bool sort_successors = false);

        ~fch_expansion_policy();

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
            return expansion_policy::mem() +
                sizeof(this);
        }

        ////////////////////// FCH specific stuff ////////////////////
        inline void
        set_search_direction(warthog::ch::search_direction d)
        { dir_ = d; }

        // return the index of the first down node from among the list
        // of outgoing edges of node @param n_id
        uint32_t
        get_first_down_index(uint32_t n_id) { return down_heads_[n_id] ; }

        warthog::graph::xy_graph*
        get_graph() { return g_; }

        std::vector<uint32_t>*
        get_rank_labels() { return rank_; }


    private:
        std::vector<uint32_t>* rank_;
        warthog::graph::xy_graph* g_;
        uint8_t* down_heads_;

        warthog::ch::search_direction dir_;

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }
};
}

#endif
