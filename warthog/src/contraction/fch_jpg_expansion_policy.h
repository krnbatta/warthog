#ifndef WARTHOG_FCH_JPG_EXPANSION_POLICY_H
#define WARTHOG_FCH_JPG_EXPANSION_POLICY_H

// contraction/fch_jpg_expansion_policy.h
//
// Forward-driven jump point search with contraction hiearchies 
// applied to corner point graphs. 
//
// @author: dharabor
// @created: 2017-01-09
//

#include "expansion_policy.h"
#include "graph.h"
#include <vector>

namespace warthog
{

class fch_jpg_expansion_policy : public expansion_policy
{
    public:
        fch_jpg_expansion_policy(
                warthog::graph::corner_point_graph* graph,
                std::vector<uint32_t>* rank);

        ~fch_jpg_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        // when true, the expansion policy only generates
        // successors reached by down edges
        inline void
        down_successors_only(bool value) 
        {
            down_successors_only_ = value;
        }


        virtual inline size_t
        mem()
        {
            return expansion_policy::mem() +
                sizeof(this);
        }

    private:
        uint32_t search_id_at_last_insert_;
        std::vector<uint32_t>* rank_;
        warthog::graph::corner_point_graph* g_;
        bool down_successors_only_;
        
        // we label each edge of the CH with two pieces of information:
        // (1) a FIRST label that specifies the direction of the first
        // step on the diagonal-first path that begins at the edge's 
        // tail node and finishes at its head node. 
        // (2) a LAST label that is similar and which specifies the 
        // direction of the last step on the path
        enum step_type
        {
            FIRST = 0,
            LAST = 1
        };

        inline uint32_t
        get_rank(uint32_t id)
        {
            return rank_->at(id);
        }

        void
        init();

        void
        compute_direction_labels();

        void
        label_edge(
                warthog::graph::edge* e, uint32_t e_tail_id, 
                warthog::graph::xy_graph* pg);

        void
        process_edge(warthog::graph::edge* e, uint32_t e_tail_id,
                     warthog::graph::xy_graph* g);

        inline warthog::jps::direction
        get_dir(warthog::graph::edge* e, step_type which)
        {
            return (warthog::jps::direction)
                ((uint8_t*)(&e->label_))[which && which];
        }

        inline void
        set_dir(warthog::graph::edge* e, step_type which, 
                warthog::jps::direction d)
        {
            ((uint8_t*)(&e->label_))[which && which] = d;
        }
        
};
}

#endif
