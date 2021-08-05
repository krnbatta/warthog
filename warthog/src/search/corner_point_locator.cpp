#include "gridmap.h"
#include "jps.h"
#include "corner_point_locator.h"
#include "xy_graph.h"

#include <cassert>
#include <climits>

warthog::corner_point_locator::corner_point_locator(warthog::gridmap* map)
	: map_(map), jumplimit_(UINT32_MAX)
{
	rmap_ = create_rmap(map_);

    cnr_map_ = warthog::jps::create_corner_map(map);
    cnr_rmap_ = create_rmap(cnr_map_);

	current_node_id_ = current_rnode_id_ = warthog::INF;
	current_goal_id_ = current_rgoal_id_ = warthog::INF;
}

warthog::corner_point_locator::~corner_point_locator()
{
	delete rmap_;
    delete cnr_map_;
    delete cnr_rmap_;
}

// create a copy of the grid map which is rotated by 90 degrees clockwise.
// this version will be used when jumping North or South. 
warthog::gridmap*
warthog::corner_point_locator::create_rmap(warthog::gridmap* map)
{
	uint32_t maph = map->header_height();
	uint32_t mapw = map->header_width();
	uint32_t rmaph = mapw;
	uint32_t rmapw = maph;
	warthog::gridmap* rmap = new warthog::gridmap(rmaph, rmapw);

	for(uint32_t x = 0; x < mapw; x++) 
	{
		for(uint32_t y = 0; y < maph; y++)
		{
			uint32_t label = map->get_label(map->to_padded_id(x, y));
			uint32_t rx = ((rmapw-1) - y);
			uint32_t ry = x;
			uint32_t rid = rmap->to_padded_id(rx, ry);
			rmap->set_label(rid, label);
		}
	}
	return rmap;
}

// Finds a jump point successor of node (x, y) in Direction d.
// Also given is the location of the goal node (goalx, goaly) for a particular
// search instance. If encountered, the goal node is always returned as a 
// jump point successor.
//
// @return: the id of a jump point successor or warthog::INF if no jp exists.
void
warthog::corner_point_locator::jump(warthog::jps::direction d,
	   	uint32_t node_id, uint32_t goal_id, 
		std::vector<uint32_t>& jpoints,
		std::vector<double>& costs)
{
	// cache node and goal ids so we don't need to convert all the time
	if(goal_id != current_goal_id_)
	{
		current_goal_id_ = goal_id;
		current_rgoal_id_ = map_id_to_rmap_id(goal_id);
	}

	if(node_id != current_node_id_)
	{
		current_node_id_ = node_id;
		current_rnode_id_ = map_id_to_rmap_id(node_id);
	}

	switch(d)
	{
		case warthog::jps::NORTH:
			jump_north(jpoints, costs);
			break;
		case warthog::jps::SOUTH:
			jump_south(jpoints, costs);
			break;
		case warthog::jps::EAST:
			jump_east(jpoints, costs);
			break;
		case warthog::jps::WEST:
			jump_west(jpoints, costs);
			break;
		case warthog::jps::NORTHEAST:
			jump_northeast(jpoints, costs);
			break;
		case warthog::jps::NORTHWEST:
			jump_northwest(jpoints, costs);
			break;
		case warthog::jps::SOUTHEAST:
			jump_southeast(jpoints, costs);
			break;
		case warthog::jps::SOUTHWEST:
			jump_southwest(jpoints, costs);
			break;
		default:
			break;
	}
}

void
warthog::corner_point_locator::jump_north(
		std::vector<uint32_t>& jpoints,
		std::vector<double>& costs)
{
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
	uint32_t jumpnode_id;
	double jumpcost;

	__jump_north(rnode_id, rgoal_id, jumpnode_id, jumpcost, rmap_, cnr_rmap_, true);

	if(jumpnode_id != warthog::INF)
	{
		jumpnode_id = current_node_id_ - (jumpcost) * map_->width();
		*(((uint8_t*)&jumpnode_id)+3) = warthog::jps::NORTH;
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}

void
warthog::corner_point_locator::__jump_north(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost,
		warthog::gridmap* map, warthog::gridmap* cnr_map, bool jpcn)
{
	// jumping north in the original map is the same as jumping
	// east when we use a version of the map rotated 90 degrees.
	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, map, cnr_map, jpcn);
}

void
warthog::corner_point_locator::jump_south(
		std::vector<uint32_t>& jpoints, 
		std::vector<double>& costs)
{
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
	uint32_t jumpnode_id;
	double jumpcost;

	__jump_south(rnode_id, rgoal_id, jumpnode_id, jumpcost, rmap_, cnr_rmap_, true);

	if(jumpnode_id != warthog::INF)
	{
		jumpnode_id = current_node_id_ + (jumpcost ) * map_->width();
		*(((uint8_t*)&jumpnode_id)+3) = warthog::jps::SOUTH;
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}

void
warthog::corner_point_locator::__jump_south(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost,
		warthog::gridmap* map, warthog::gridmap* cnr_map, bool jpcn)
{
	// jumping north in the original map is the same as jumping
	// west when we use a version of the map rotated 90 degrees.
	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, map, cnr_map, jpcn);
}

void
warthog::corner_point_locator::jump_east(
		std::vector<uint32_t>& jpoints, 
		std::vector<double>& costs)
{
	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t jumpnode_id;
	double jumpcost;

	__jump_east(node_id, goal_id, jumpnode_id, jumpcost, map_, cnr_map_, true);

	if(jumpnode_id != warthog::INF)
	{
		*(((uint8_t*)&jumpnode_id)+3) = warthog::jps::EAST;
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}


void
warthog::corner_point_locator::__jump_east(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost, 
		warthog::gridmap* map, warthog::gridmap* cnr_map, 
        bool jpcn)
{
    // we scan the grid looking for obstacles or jump points
    // NB: we use a 32bit stride
	uint32_t jumpto_id = node_id;
    uint32_t* mem = (uint32_t*)map->get_mem_ptr(jumpto_id);
    uint32_t* jp_mem = (uint32_t*)cnr_map->get_mem_ptr(jumpto_id);

    // we ignore the tiles left of node_id and possibly node_id
    // too depending on jpcn (jpcn = jump past current node)
    uint32_t offset = (jumpto_id & warthog::DBWORD_BITS_MASK);
    uint32_t stop_bits = (~*mem) | (*jp_mem & ~(jpcn << offset));
    stop_bits = (stop_bits >> offset) << offset;

	uint32_t stop_pos;
    uint32_t iters = 0;
	while(true)
	{
		if(stop_bits)
		{
            stop_pos = __builtin_ffs(stop_bits)-1; // ffs returns idx+1
			break;
		}
        stop_bits = ~*(++mem) | *(++jp_mem);
        iters++;
	}

	jumpcost = (32*iters + stop_pos) - offset;
    jumpnode_id = node_id + jumpcost;
	uint32_t goal_dist = goal_id - node_id;
	if(jumpcost > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_dist ;
        return;
	}

    if((~*mem) & (1 << stop_pos)) 
	{
        // no successor if we stopped due to obstacles
		jumpcost -= (1 && jumpcost);
		jumpnode_id = warthog::INF;
	}
}

// analogous to ::jump_east 
void
warthog::corner_point_locator::jump_west(
		std::vector<uint32_t>& jpoints, 
		std::vector<double>& costs)
{
	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t jumpnode_id;
	double jumpcost;

	__jump_west(node_id, goal_id, jumpnode_id, jumpcost, map_, cnr_map_, true);

	if(jumpnode_id != warthog::INF)
	{
		*(((uint8_t*)&jumpnode_id)+3) = warthog::jps::WEST;
		jpoints.push_back(jumpnode_id);
		costs.push_back(jumpcost);
	}
}

void
warthog::corner_point_locator::__jump_west(uint32_t node_id, 
		uint32_t goal_id, uint32_t& jumpnode_id, double& jumpcost, 
		warthog::gridmap* map, warthog::gridmap* cnr_map, 
        bool jpcn)
{
    // scan the grid for obstacles or jump points
    // NB: we use a 32bit stride (cf. dbword which is 8bits) and, 
    // when we begin, we place node_id in the highest byte of 
    // the stride word (hence -3)
    uint32_t* mem = (uint32_t*)(map->get_mem_ptr(node_id)-3);
    uint32_t* jp_mem = (uint32_t*)(cnr_map->get_mem_ptr(node_id)-3);

    // we compute the bit index of node_id in its parent dbword
    // and also the number of bits from node_id to the end of its
    // parent dbword (all inclusive)
    //uint32_t offset = node_id & warthog::DBWORD_BITS_MASK;
    //uint32_t r_offset = warthog::DBWORD_BITS - offset;
    
    // compute the left and right bit index of node_id
    uint32_t l_offset = (node_id & warthog::DBWORD_BITS_MASK) + 24;
    uint32_t r_offset = 31 - l_offset;
    
    // we ignore all bits to right of node_id and possibly the bit
    // value of node_id, depending on jpcn (jump past current node)
    uint32_t stop_bits = (~*mem) | (*jp_mem & ~(jpcn << l_offset));
    stop_bits = (stop_bits << r_offset) >> r_offset;

    uint32_t stop_pos;
    uint32_t iters = 0;
	while(true)
	{
		if(stop_bits)
		{
			stop_pos = __builtin_clz(stop_bits); // first non-zero bit
			break;
		}
        stop_bits = ~*(--mem) | *(--jp_mem);
        iters++;
	}

	jumpcost = (32*iters + stop_pos) - r_offset;
    jumpnode_id = node_id - jumpcost;

	uint32_t goal_dist = node_id - goal_id;
	if(jumpcost > goal_dist)
	{
		jumpnode_id = goal_id;
		jumpcost = goal_dist ;
        return;
	}

    bool obstacle = (~*mem) & (1 << (31 - stop_pos));
	if(obstacle)
	{
		jumpcost -= (1 && jumpcost);
		jumpnode_id = warthog::INF;
	}
}

void
warthog::corner_point_locator::jump_northeast(
		std::vector<uint32_t>& jpoints,
		std::vector<double>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	double jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. node_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 1542) != 1542) { return; }

	while(node_id != warthog::INF)
	{
		__jump_northeast(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, jp1_id, jp1_cost, 
				jp2_id, jp2_cost);

		if(jp1_id != warthog::INF)
		{
			jp1_id = node_id - (jp1_cost ) * map_->width();
			*(((uint8_t*)&jp1_id)+3) = warthog::jps::NORTH;
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF)
		{
			*(((uint8_t*)&jp2_id)+3) = warthog::jps::EAST;
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::corner_point_locator::__jump_northeast(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, double& jumpcost, 
		uint32_t& jp_id1, double& cost1,
		uint32_t& jp_id2, double& cost2)
{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id - mapw + 1;
		rnode_id = rnode_id + rmapw + 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_north(rnode_id, rgoal_id, jp_id1, cost1, rmap_, cnr_rmap_);
		__jump_east(node_id, goal_id, jp_id2, cost2, map_, cnr_map_);
		if((jp_id1 & jp_id2) != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) 
		{ 
			node_id = jp_id1 = jp_id2 = warthog::INF; 
			break; 
		}

	}
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}

void
warthog::corner_point_locator::jump_northwest(
		std::vector<uint32_t>& jpoints,
		std::vector<double>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	double jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. node_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 771) != 771) { return; }

	while(node_id != warthog::INF)
	{
		__jump_northwest(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, jp1_id, jp1_cost, 
				jp2_id, jp2_cost);

		if(jp1_id != warthog::INF)
		{
			jp1_id = node_id - (jp1_cost ) * map_->width();
			*(((uint8_t*)&jp1_id)+3) = warthog::jps::NORTH;
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF)
		{
			*(((uint8_t*)&jp2_id)+3) = warthog::jps::WEST;
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::corner_point_locator::__jump_northwest(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, double& jumpcost,
		uint32_t& jp_id1, double& cost1, 
		uint32_t& jp_id2, double& cost2)

{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id - mapw - 1;
		rnode_id = rnode_id - (rmapw - 1);

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_north(rnode_id, rgoal_id, jp_id1, cost1, rmap_, cnr_rmap_);
		__jump_west(node_id, goal_id, jp_id2, cost2, map_, cnr_map_);
		if((jp_id1 & jp_id2) != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) 
		{ 
			node_id = jp_id1 = jp_id2 = warthog::INF;
		   	break; 
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}

void
warthog::corner_point_locator::jump_southeast(
		std::vector<uint32_t>& jpoints,
		std::vector<double>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	double jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;

	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);
	
	// early return if the first diagonal step is invalid
	// (validity of subsequent steps is checked by straight jump functions)
	if((neis & 394752) != 394752) { return; }

	while(node_id != warthog::INF)
	{
		__jump_southeast(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, jp1_id, jp1_cost, 
				jp2_id, jp2_cost);

		if(jp1_id != warthog::INF)
		{
			jp1_id = node_id + (jp1_cost ) * map_->width();
			*(((uint8_t*)&jp1_id)+3) = warthog::jps::SOUTH;
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; } // no corner cutting
		}

		if(jp2_id != warthog::INF)
		{
			*(((uint8_t*)&jp2_id)+3) = warthog::jps::EAST;
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; } // no corner cutting
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::corner_point_locator::__jump_southeast(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, double& jumpcost,
		uint32_t& jp_id1, double& cost1, 
		uint32_t& jp_id2, double& cost2)

{
	uint32_t num_steps = 0;

	// jump a single step at a time (no corner cutting)
	uint32_t rmapw = rmap_->width();
	uint32_t mapw = map_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id + mapw + 1;
		rnode_id = rnode_id + rmapw - 1;

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_south(rnode_id, rgoal_id, jp_id1, cost1, rmap_, cnr_rmap_);
		__jump_east(node_id, goal_id, jp_id2, cost2, map_, cnr_map_);
		if((jp_id1 & jp_id2) != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) 
		{ 
			node_id = jp_id1 = jp_id2 = warthog::INF; 
			break; 
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}

void
warthog::corner_point_locator::jump_southwest(
		std::vector<uint32_t>& jpoints,
		std::vector<double>& costs)
{
	uint32_t jumpnode_id, jp1_id, jp2_id;
	double jumpcost, jp1_cost, jp2_cost, cost_to_nodeid;
	jumpnode_id = jp1_id = jp2_id = 0;
	jumpcost = jp1_cost = jp2_cost = cost_to_nodeid = 0;

	uint32_t node_id = current_node_id_;
	uint32_t goal_id = current_goal_id_;
	uint32_t rnode_id = current_rnode_id_;
	uint32_t rgoal_id = current_rgoal_id_;
	
	// first 3 bits of first 3 bytes represent a 3x3 cell of tiles
	// from the grid. next_id at centre. Assume little endian format.
	uint32_t neis;
	map_->get_neighbours(node_id, (uint8_t*)&neis);

	// early termination (first step is invalid)
	if((neis & 197376) != 197376) { return; }

	while(node_id != warthog::INF)
	{
		__jump_southwest(
				node_id, rnode_id,
				goal_id, rgoal_id,
				jumpnode_id, jumpcost, 
				jp1_id, jp1_cost, jp2_id, jp2_cost);

		if(jp1_id != warthog::INF)
		{
			jp1_id = node_id + (jp1_cost ) * map_->width();
			*(((uint8_t*)&jp1_id)+3) = warthog::jps::SOUTH;
			jpoints.push_back(jp1_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp1_cost);
			if(jp2_cost == 0) { break; }
		}

		if(jp2_id != warthog::INF)
		{
			*(((uint8_t*)&jp2_id)+3) = warthog::jps::WEST;
			jpoints.push_back(jp2_id);
			costs.push_back(cost_to_nodeid + jumpcost + jp2_cost);
			if(jp1_cost == 0) { break; }
		}
		node_id = jumpnode_id;
		cost_to_nodeid += jumpcost;
	}
}

void
warthog::corner_point_locator::__jump_southwest(
		uint32_t& node_id, uint32_t& rnode_id, 
		uint32_t goal_id, uint32_t rgoal_id,
		uint32_t& jumpnode_id, double& jumpcost,
		uint32_t& jp_id1, double& cost1, 
		uint32_t& jp_id2, double& cost2)
{
	// jump a single step (no corner cutting)
	uint32_t num_steps = 0;
	uint32_t mapw = map_->width();
	uint32_t rmapw = rmap_->width();
	while(true)
	{
		num_steps++;
		node_id = node_id + mapw - 1;
		rnode_id = rnode_id - (rmapw + 1);

		// recurse straight before stepping again diagonally;
		// (ensures we do not miss any optimal turning points)
		__jump_south(rnode_id, rgoal_id, jp_id1, cost1, rmap_, cnr_rmap_);
		__jump_west(node_id, goal_id, jp_id2, cost2, map_, cnr_map_);
		if((jp_id1 & jp_id2) != warthog::INF) { break; }

		// couldn't move in either straight dir; node_id is an obstacle
		if(!(cost1 && cost2)) 
		{ 
			node_id = jp_id1 = jp_id2 = warthog::INF;
		   	break; 
		}
	}
	jumpnode_id = node_id;
	jumpcost = num_steps*warthog::DBL_ROOT_TWO;
}

uint32_t 
warthog::corner_point_locator::mem()
{
    return sizeof(*this) + rmap_->mem();
}
