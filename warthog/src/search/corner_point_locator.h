#ifndef WARTHOG_CORNER_POINT_LOCATOR_H
#define WARTHOG_CORNER_POINT_LOCATOR_H

// corner_point_locator.h
//
// Intended for use during the insertion phase of search 
// on corner graphs.
//
// This class takes as input an (m*n) input grid graph and
// creates from it a new grid where each of the (m*n) tiles 
// are marked as either being a corner point (1) or not (0).
// A corner point is any grid cell having two adjacent neighbours
// which are not co-visible.
//
//  The corner grid is scanned in a manner similar to jump point search.
//  The objective is to identify all corner point successors of start
//  or target nodes that are not themselves corner points 
//
//  The implementation here is based on the class
//  warthog::online_jump_point_locator2
//
// @author: dharabor
// @created: 2016-09-22
//

#include "jps.h"
#include "gridmap.h"
#include "xy_graph.h"
#include <vector>

namespace warthog
{

class corner_point_locator
{
	public: 
		corner_point_locator(warthog::gridmap* map);
		~corner_point_locator();

		void
		jump(warthog::jps::direction d, uint32_t node_id, uint32_t goalid, 
				std::vector<uint32_t>& jpoints,
				std::vector<double>& costs);

        inline warthog::gridmap* 
        get_map() { return map_; } 

        inline warthog::gridmap* 
        get_corner_map() { return cnr_map_; }

		uint32_t 
		mem();

	private:
		void
		jump_north(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);
		void
		jump_south(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);
		void
		jump_east(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);
		void
		jump_west(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);
		void
		jump_northeast(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);
		void
		jump_northwest(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);
		void
		jump_southeast(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);
		void
		jump_southwest(
				std::vector<uint32_t>& jpoints, 
				std::vector<double>& costs);

		// these versions can be passed a map parameter to
		// use when jumping. they allow switching between
		// map_ and rmap_ (a rotated counterpart).
		void
		__jump_north(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost,
				warthog::gridmap* map, warthog::gridmap* cnr_map, 
                bool jpcn = false);
		void
		__jump_south(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost, 
				warthog::gridmap* map, warthog::gridmap* cnr_map, 
                bool jpcn = false);
		void
		__jump_east(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost, 
				warthog::gridmap* map, warthog::gridmap* cnr_map, 
                bool jpcn = false);
		void
		__jump_west(uint32_t node_id, uint32_t goal_id, 
				uint32_t& jumpnode_id, double& jumpcost, 
				warthog::gridmap* map, warthog::gridmap* cnr_map, 
                bool jpcn = false);

		// these versions perform a single diagonal jump, returning
		// the intermediate diagonal jump point and the straight 
		// jump points that caused the jumping process to stop
		void
		__jump_northeast(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, double& jumpcost,
				uint32_t& jp1_id, double& jp1_cost,
				uint32_t& jp2_id, double& jp2_cost);
		void
		__jump_northwest(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, double& jumpcost,
				uint32_t& jp1_id, double& jp1_cost,
				uint32_t& jp2_id, double& jp2_cost);
		void
		__jump_southeast(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, double& jumpcost,
				uint32_t& jp1_id, double& jp1_cost,
				uint32_t& jp2_id, double& jp2_cost);
		void
		__jump_southwest(
				uint32_t& node_id, uint32_t& rnode_id, 
				uint32_t goal_id, uint32_t rgoal_id,
				uint32_t& jumpnode_id, double& jumpcost,
				uint32_t& jp1_id, double& jp1_cost,
				uint32_t& jp2_id, double& jp2_cost);

		// functions to convert map indexes to rmap indexes
		inline uint32_t
		map_id_to_rmap_id(uint32_t mapid)
		{
			if(mapid == warthog::INF) { return mapid; }

			uint32_t x, y;
			uint32_t rx, ry;
			map_->to_unpadded_xy(mapid, x, y);
			ry = x;
			rx = map_->header_height() - y - 1;
			return rmap_->to_padded_id(rx, ry);
		}

		// convert rmap indexes to map indexes
		inline uint32_t
		rmap_id_to_map_id(uint32_t rmapid)
		{
			if(rmapid == warthog::INF) { return rmapid; }

			uint32_t x, y;
			uint32_t rx, ry;
			rmap_->to_unpadded_xy(rmapid, rx, ry);
			x = ry;
			y = rmap_->header_width() - rx - 1;
			return map_->to_padded_id(x, y);
		}

        // create a copy of the input grid that is rotated by 90 degrees
		warthog::gridmap*
		create_rmap(warthog::gridmap*);

        // create a grid 
        warthog::gridmap* 
        create_cnr_map(warthog::graph::xy_graph* g);

		warthog::gridmap* map_;
		warthog::gridmap* rmap_;

        warthog::gridmap* cnr_map_;
        warthog::gridmap* cnr_rmap_;
		uint32_t jumplimit_;

		uint32_t current_goal_id_;
		uint32_t current_rgoal_id_;
		uint32_t current_node_id_;
		uint32_t current_rnode_id_;
};

}

#endif

