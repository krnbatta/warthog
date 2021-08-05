#include "cbs.h"
#include "grid.h"
#include "helpers.h"
#include "cbs_ll_expansion_policy.h"
#include "problem_instance.h"

#include <algorithm>

using namespace warthog::cbs;

warthog::cbs_ll_expansion_policy::cbs_ll_expansion_policy(
		warthog::gridmap* map, warthog::cbs_ll_heuristic* h) : map_(map), h_(h)
{
    neis_ = new warthog::arraylist<neighbour_record>(32);

    map_xy_sz_ = map->height() * map->width();
    assert(map_xy_sz_ > 0);

    cons_ = new warthog::cbs::time_constraints(map_xy_sz_);

    // preallocate memory for up to some number of timesteps 
    // in advance. for subsequent timesteps memory is allocated
    // dynamically
    time_map_ = new std::vector<warthog::mem::node_pool*>();
    for(uint32_t i = 0; i < 128; i++)
    {
        time_map_->push_back(new warthog::mem::node_pool(map_xy_sz_));
    }

    // setup some constants to quickly compute the current timestep 
    // and xy-index
    bitwidth_map_ = 32 - __builtin_clz(map->height()*map->width());
    id_mask_ = (1 << bitwidth_map_)-1;
}

warthog::cbs_ll_expansion_policy::~cbs_ll_expansion_policy()
{
    for(uint32_t i = 0; i < time_map_->size(); i++)
    {
        delete time_map_->at(i);
    }
    time_map_->clear();
    delete time_map_;
    delete cons_;
    delete neis_;
}


void 
warthog::cbs_ll_expansion_policy::expand(warthog::search_node* current,
		warthog::problem_instance* problem)
{
	reset();

    // get the xy id of the current node and extract current timestep
	uint32_t nodeid = current->get_id() & id_mask_;
    uint32_t timestep = current->get_id() >> bitwidth_map_;

	// get adjacent grid tiles (bitpacked into one 32bit word)
	uint32_t tiles = 0;
	map_->get_neighbours(nodeid, (uint8_t*)&tiles);

    // neighbour ids are calculated using nodeid offsets
	uint32_t nid_m_w = nodeid - map_->width();
	uint32_t nid_p_w = nodeid + map_->width();

    // edge constraints for the current node
    cell_constraints* cur_cc = cons_->get_constraints(nodeid, timestep);

    // cardinal successors
    cell_constraints* succ_cc = cons_->get_constraints(nid_m_w, timestep+1);
    if( ((tiles & 514) == 514) && // N
        !(cur_cc->e_ & warthog::grid::NORTH) &&  
        !succ_cc->v_ ) 
	{  
		add_neighbour(__generate(nid_m_w, timestep+1), 1);
	} 

    succ_cc = cons_->get_constraints(nodeid + 1, timestep+1);
	if( ((tiles & 1536) == 1536) && // E
        !(cur_cc->e_ & warthog::grid::EAST) && 
        !succ_cc->v_ )
	{
		add_neighbour(__generate(nodeid + 1, timestep+1), 1);
	}

    succ_cc = cons_->get_constraints(nid_p_w, timestep+1);
	if( ((tiles & 131584) == 131584) && // S
        !(cur_cc->e_ & warthog::grid::SOUTH) && 
        !succ_cc->v_ )
	{ 

		add_neighbour(__generate(nid_p_w, timestep+1), 1);
	}

    succ_cc = cons_->get_constraints(nodeid - 1, timestep+1);
	if( ((tiles & 768) == 768) && // W
        !(cur_cc->e_ & warthog::grid::WEST) && 
        !succ_cc->v_ )
	{ 
		add_neighbour(__generate(nodeid - 1, timestep+1), 1);
	}

    // wait successor
    succ_cc = cons_->get_constraints(nodeid, timestep+1);
    if(!succ_cc->v_)
    {
        add_neighbour(__generate(nodeid, timestep+1), 1);
    }
}

void
warthog::cbs_ll_expansion_policy::get_xy(uint32_t nid, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy(nid & id_mask_, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::cbs_ll_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t max_id = map_->header_width() * map_->header_height();
    if(pi->start_id_ >= max_id) { return 0; }
    uint32_t padded_id = map_->to_padded_id(pi->start_id_);
    return __generate(padded_id, 0);
}

warthog::search_node*
warthog::cbs_ll_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t max_id = map_->header_width() * map_->header_height();
    if(pi->target_id_ >= max_id) { return 0; }
    h_->set_current_target(pi->target_id_);

    uint32_t padded_id = map_->to_padded_id(pi->target_id_);
    return __generate(padded_id, 0);
}

size_t
warthog::cbs_ll_expansion_policy::mem()
{
   size_t total = sizeof(*this) + map_->mem();
   uint32_t tm_sz = time_map_->size();
   for(uint32_t i = 0; i < tm_sz; i++)
   {
       total += time_map_->at(i)->mem();
   }
   total += sizeof(neighbour_record) * neis_->capacity();
   return total;
}
