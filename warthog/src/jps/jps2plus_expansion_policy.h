#ifndef WARTHOG_JPS2PLUS_EXPANSION_POLICY_H
#define WARTHOG_JPS2PLUS_EXPANSION_POLICY_H

// jps2plus_expansion_policy.h
//
// An experimental variation of warthog::jps_expansion_policy,
// this version is designed for efficient offline jps.
//
// @author: dharabor
// @created: 06/01/2010

#include "expansion_policy.h"
#include "gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "offline_jump_point_locator2.h"
#include "problem_instance.h"
#include "search_node.h"

#include "stdint.h"

namespace warthog
{

class jps2plus_expansion_policy : public expansion_policy
{
	public:
		jps2plus_expansion_policy(warthog::gridmap* map);
		virtual ~jps2plus_expansion_policy();

		virtual void 
		expand(warthog::search_node*, warthog::problem_instance*);

		virtual inline size_t
		mem()
		{
			return 
                expansion_policy::mem() + 
                sizeof(*this) + 
                map_->mem() + jpl_->mem();
		}

        virtual void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y);

        virtual warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        virtual warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);

        // this function gets called whenever a successor node is relaxed. at that
        // point we set the node currently being expanded (==current) as the 
        // parent of n and label node n with the direction of travel, 
        // from current to n
        void
        update_parent_direction(warthog::search_node* n);

	private:
		warthog::gridmap* map_;
		offline_jump_point_locator2* jpl_;
		std::vector<double> costs_;
		std::vector<uint32_t> jp_ids_;
};

}

#endif

