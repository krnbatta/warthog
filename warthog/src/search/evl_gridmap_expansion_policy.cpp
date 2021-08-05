#include "evl_gridmap_expansion_policy.h"
#include "grid.h"

#include <algorithm>

warthog::evl_gridmap_expansion_policy::evl_gridmap_expansion_policy(
		warthog::evl_gridmap* map)
    : map_(map)
{
    neis_ = new warthog::arraylist<neighbour_record>(32);
    map_xy_sz_ = map->height() * map->width();
    assert(map_xy_sz_ > 0);
    pool_ = new warthog::mem::node_pool(map_xy_sz_);
}

warthog::evl_gridmap_expansion_policy::~evl_gridmap_expansion_policy()
{
    delete pool_;
    delete neis_;
}


void 
warthog::evl_gridmap_expansion_policy::expand(warthog::search_node* current,
		warthog::problem_instance* problem)
{
	reset();

    // get the xy id of the current node and extract current timestep
	uint32_t nodeid = current->get_id();

    // neighbour ids are calculated using nodeid offsets
	uint32_t nid_m_w = nodeid - map_->width();
	uint32_t nid_p_w = nodeid + map_->width();

    // cardinal successors
    warthog::labelled_cell* nei = &map_->get_label(nid_m_w);
    if(nei->v_lab_ != UINT32_MAX)
	{  
		add_neighbour(
            generate(nid_m_w),
            nei->e_lab_[__builtin_ffs(warthog::grid::NORTH)-1] 
            + nei->v_lab_);
	} 

    nei = &map_->get_label(nid_p_w);
    if(nei->v_lab_ != UINT32_MAX)
	{  
		add_neighbour(
            generate(nid_p_w),
            nei->e_lab_[__builtin_ffs(warthog::grid::SOUTH)-1]
            + nei->v_lab_);
	} 

    nei = &map_->get_label(nodeid+1);
    if(nei->v_lab_ != UINT32_MAX)
	{  
		add_neighbour(
            generate(nodeid+1),
            nei->e_lab_[__builtin_ffs(warthog::grid::EAST)-1]
            + nei->v_lab_);
	} 

    nei = &map_->get_label(nodeid-1);
    if(nei->v_lab_ != UINT32_MAX)
	{  
		add_neighbour(
            generate(nodeid-1),
            nei->e_lab_[__builtin_ffs(warthog::grid::WEST)-1]
            + nei->v_lab_);
	} 
}

void
warthog::evl_gridmap_expansion_policy::get_xy(uint32_t nid, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy(nid, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::evl_gridmap_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t max_id = map_->header_width() * map_->header_height();
    if(pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id(pi->start_id_);
    return generate(padded_id);
}

warthog::search_node*
warthog::evl_gridmap_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t max_id = map_->header_width() * map_->header_height();
    if(pi->target_id_ >= max_id) { return 0; }

    uint32_t padded_id = map_->to_padded_id(pi->target_id_);
    return generate(padded_id);
}

size_t
warthog::evl_gridmap_expansion_policy::mem()
{
   size_t total = sizeof(*this) + map_->mem();
   total += pool_->mem();
   total += sizeof(neighbour_record) * neis_->capacity();
   return total;
}
