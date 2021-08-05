#ifndef WARTHOG_TXEVL_GRIDMAP_EXPANSION_POLICY_H
#define WARTHOG_TXEVL_GRIDMAP_EXPANSION_POLICY_H

// search/evl_gridmap_expansion_policy.h
//
// An expansion policy for edge-and-vertex labelled manhattan 
// grids. The cost of each move is equal to its corresponding 
// edge label plus the corresponding vertex label of the cell 
// being moved to.
//
// @author: dharabor
// @created: 2018-11-08
//

#include "arraylist.h"
#include "forward.h"
#include "labelled_gridmap.h"
#include "node_pool.h"
#include "problem_instance.h"
#include "search_node.h"

#include <memory>

namespace warthog
{

class evl_gridmap_expansion_policy 
{
	public:
		evl_gridmap_expansion_policy(warthog::evl_gridmap* map);
		~evl_gridmap_expansion_policy();

		inline void
		reset()
		{
			current_ = 0;
            neis_->clear();
		}

		inline void
		first(warthog::search_node*& ret, double& cost)
		{
            current_ = 0;
            n(ret, cost);
		}

		inline void
		n(warthog::search_node*& ret, double& cost)
		{
            if(current_ < neis_->size())
            {
                ret = (*neis_)[current_].node_;
                cost = (*neis_)[current_].cost_;
            }
            else
            {
                ret = 0;
                cost = 0;
            }
		}

		inline void
		next(warthog::search_node*& ret, double& cost)
		{
            current_++;
            n(ret, cost);
		}

		void 
		expand(warthog::search_node*, warthog::problem_instance*);

        void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);
        
        inline warthog::search_node*
        generate(uint32_t node_id)
        {
            warthog::search_node* nei = pool_->generate(node_id);
            return nei;
        }

        inline bool
        is_target(warthog::search_node* n, warthog::problem_instance* pi)
        {
            return (n->get_id() == pi->target_id_);
        }

		size_t 
        mem();

	
	private:
		warthog::evl_gridmap* map_;
        uint32_t map_xy_sz_;
        warthog::mem::node_pool* pool_;

        struct neighbour_record
        {
            neighbour_record(warthog::search_node* node, double cost)
            {
                node_ = node;
                cost_ = cost;
            }
            warthog::search_node* node_;
            double cost_;
        };

        arraylist<neighbour_record>* neis_;
        uint32_t current_;

        inline void 
        add_neighbour(warthog::search_node* nei, double cost)
        {
            neis_->push_back(neighbour_record(nei, cost));
            //std::cout << " neis_.size() == " << neis_->size() << std::endl;
        }
};

}

#endif

