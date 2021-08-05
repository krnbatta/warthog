#ifndef WARTHOG_JPS_EXPANSION_POLICY_WGM_H
#define WARTHOG_JPS_EXPANSION_POLICY_WGM_H

// jps_expansion_policy_wgm.h
//
// A variant of Jump Point Search adapted to gridmaps with weighted costs.
// The algorithm works similar to the variant for uniform-cost maps;
// The main difference is that we stop whenever the current node has a 
// successor with a different terrain type.
//
// Further details in: 
// (to appear)
//
// EXPERIMENTAL AND CRAP! DOES NOT WORK
// EXPERIMENTAL AND CRAP! DOES NOT WORK
// EXPERIMENTAL AND CRAP! DOES NOT WORK
// EXPERIMENTAL AND CRAP! DOES NOT WORK
// EXPERIMENTAL AND CRAP! DOES NOT WORK
// EXPERIMENTAL AND CRAP! DOES NOT WORK
// EXPERIMENTAL AND CRAP! DOES NOT WORK
// EXPERIMENTAL AND CRAP! DOES NOT WORK
//
// @author: dharabor
// @created: 2014-09-22
//

#include "expansion_policy.h"
#include "helpers.h"
#include "jps.h"
#include "jps_wgm.h"
#include "online_jump_point_locator_wgm.h"
#include "problem_instance.h"
#include "search_node.h"
#include "labelled_gridmap.h"

#include "stdint.h"

namespace warthog
{
class jps_expansion_policy_wgm : public expansion_policy
{
	public:
		jps_expansion_policy_wgm(warthog::vl_gridmap* map);
		virtual ~jps_expansion_policy_wgm();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

		virtual inline size_t
		mem()
		{
			return expansion_policy::mem() + 
                sizeof(*this) + map_->mem() + jpl_->mem();
		}

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

	private:
		warthog::vl_gridmap* map_;
		warthog::online_jump_point_locator_wgm* jpl_;

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

