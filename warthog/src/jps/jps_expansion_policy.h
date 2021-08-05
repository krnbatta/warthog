#ifndef WARTHOG_JPS_EXPANSION_POLICY_H
#define WARTHOG_JPS_EXPANSION_POLICY_H

// jps_expansion_policy.h
//
// This expansion policy reduces the branching factor
// of a node n during search by ignoring any neighbours which
// could be reached by an equivalent (or shorter) path that visits
// the parent of n but not n itself.
//
// An extension of this idea is to generate jump nodes located in the
// same direction as the remaining neighbours. 
//
// Theoretical details:
// [Harabor D. and Grastien A., 2011, Online Node Pruning for Pathfinding
// On Grid Maps, AAAI] 
//
// @author: dharabor
// @created: 06/01/2010

#include "expansion_policy.h"
#include "gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "online_jump_point_locator.h"
#include "problem_instance.h"
#include "search_node.h"

#include "stdint.h"

namespace warthog
{

class jps_expansion_policy : public expansion_policy
{
	public:
		jps_expansion_policy(warthog::gridmap* map);
		virtual ~jps_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

        virtual void
        get_xy(uint32_t nid, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

		virtual inline size_t
		mem()
		{
            return expansion_policy::mem() +
                sizeof(*this) + map_->mem() + jpl_->mem();
		}

	private:
		warthog::gridmap* map_;
		warthog::online_jump_point_locator* jpl_;

		// computes the direction of travel; from a node n1
		// to a node n2.
		inline warthog::jps::direction
		compute_direction(uint32_t n1_id, uint32_t n2_id)
		{
			if(n1_id == warthog::NODE_NONE) { return warthog::jps::NONE; }

			int32_t x, y, x2, y2;
			warthog::helpers::index_to_xy(n1_id, map_->width(), x, y);
			warthog::helpers::index_to_xy(n2_id, map_->width(), x2, y2);
			warthog::jps::direction dir = warthog::jps::NONE;
			if(y2 == y)
			{
				if(x2 > x)
					dir = warthog::jps::EAST;
				else
					dir = warthog::jps::WEST;
			}
			else if(y2 < y)
			{
				if(x2 == x)
					dir = warthog::jps::NORTH;
				else if(x2 < x)
					dir = warthog::jps::NORTHWEST;
				else // x2 > x
					dir = warthog::jps::NORTHEAST;
			}
			else // y2 > y 
			{
				if(x2 == x)
					dir = warthog::jps::SOUTH;
				else if(x2 < x)
					dir = warthog::jps::SOUTHWEST;
				else // x2 > x
					dir = warthog::jps::SOUTHEAST;
			}
			assert(dir != warthog::jps::NONE);
			return dir;
		}
};

}

#endif

