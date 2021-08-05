#ifndef WARTHOG_FCH_BB_JPG_EXPANSION_POLICY_H
#define WARTHOG_FCH_BB_JPG_EXPANSION_POLICY_H

// contraction/fch_bb_jpg_expansion_policy.h
//
// Forward-driven with contraction hiearchies applied to 
// corner point graphs. 
//  + jump point pruning.
//  + geometric containers.
//
// @author: dharabor
// @created: 2017-01-25
//

#include "expansion_policy.h"
#include "forward.h"
#include "geom.h"
#include <vector>

namespace warthog
{

class fch_bb_jpg_expansion_policy : public expansion_policy
{
    public:
        fch_bb_jpg_expansion_policy(
                warthog::graph::corner_point_graph* graph,
                std::vector<uint32_t>* rank,
                warthog::label::bb_labelling* lab);

        ~fch_bb_jpg_expansion_policy();

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
            // TODO: implement me
            return 0; 
        }

    private:
        uint32_t search_id_at_last_insert_;
        std::vector<uint32_t>* rank_;
        warthog::graph::corner_point_graph* g_;

        // geometric containers stuff
        warthog::label::bb_labelling* lab_;
        warthog::geom::rectangle r_;
        bool
        filter(uint32_t node_id, uint32_t edge_id);

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }

        void
        init();

};
}

#endif
