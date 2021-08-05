#include "jps_expansion_policy_wgm.h"

warthog::jps_expansion_policy_wgm::jps_expansion_policy_wgm(
        warthog::vl_gridmap* map) 
    : expansion_policy(map->height() * map->width())
{
	map_ = map;
	jpl_ = new warthog::online_jump_point_locator_wgm(map);
}

warthog::jps_expansion_policy_wgm::~jps_expansion_policy_wgm()
{
	delete jpl_;
}

void 
warthog::jps_expansion_policy_wgm::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
    reset();

	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c =
	   	this->compute_direction(current->get_parent(), current->get_id());

	// get the tiles around the current node c
    
    warthog::dbword c_tiles[9];
	uint32_t c_ids[9];
	uint32_t current_id = current->get_id();

    map_->get_neighbours(current_id, c_ids, c_tiles);
    assert(current_id == c_ids[4]);

	// look for jump points in the direction of each natural 
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors_wgm(dir_c, c_tiles);

    // jump in the direction of each forced or natural neighbour, generating
    // any jump points we find.
	uint32_t goal_id = problem->target_id_;
	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		if(succ_dirs & d)
		{
			double jumpcost;
			uint32_t succ_id;
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

			if(succ_id != warthog::INF)
			{
                add_neighbour(generate(succ_id), jumpcost);
			}
		}
	}
}

void
warthog::jps_expansion_policy_wgm::get_xy(uint32_t id, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy(id, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::jps_expansion_policy_wgm::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t max_id = map_->header_width() * map_->header_height();
    if(pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id(pi->start_id_);
    return generate(padded_id);
}

warthog::search_node*
warthog::jps_expansion_policy_wgm::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t max_id = map_->header_width() * map_->header_height();
    if(pi->target_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id(pi->target_id_);
    return generate(padded_id);
}
