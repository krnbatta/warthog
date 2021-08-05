#include "af_labelling.h"
#include "contraction.h"
#include "corner_point_graph.h"
#include "fch_af_jpg_expansion_policy.h"
#include "jpg.h"
#include "search_node.h"

warthog::fch_af_jpg_expansion_policy::fch_af_jpg_expansion_policy(
        warthog::graph::corner_point_graph* g, std::vector<uint32_t>* rank,
        warthog::label::af_labelling* afl)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    afl_ = afl;
    apex_ = warthog::INF;
    apex_reached_ = false;

    warthog::jpg::compute_direction_labels(g_);
}

warthog::fch_af_jpg_expansion_policy::~fch_af_jpg_expansion_policy()
{
}

void
warthog::fch_af_jpg_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance* problem)
{
    reset();

    // determine which directions contain jump point successors
    // NB: we treat the start node as a special case since it has
    // no parent (i.e. we expand all successors in every direction)
    uint32_t succ_dirs;
    uint32_t current_id = current->get_id();
    if(current_id == problem->start_id_)
    {
        succ_dirs = warthog::jps::direction::ALL;
    }
    else
    {
        // get the direction of travel used to reach the current node
        // (i.e. the last direction we must have traveled in)
        uint32_t parent_id = current->get_parent();
        warthog::graph::node* parent = g_->get_node(parent_id);
        warthog::graph::edge* e_pn = parent->find_edge(current_id);
        warthog::jps::direction d_last = get_dir(e_pn, warthog::jpg::LAST);

        // using d_last we now compute a set of directions in which we 
        // should look for successors; we will only generate successors
        // that can be reached by traveling in those directions.
        int32_t x, y;
        this->get_xy(current_id, x, y);
        x /= warthog::ONE;
        y /= warthog::ONE;
        warthog::gridmap* gm = g_->get_gridmap();
        uint32_t grid_id = gm->to_padded_id(x, y);
        uint32_t tiles;
        gm->get_neighbours(grid_id, (uint8_t*)&tiles);
        succ_dirs = warthog::jps::compute_successors(d_last, tiles);
    }

    warthog::search_node* parent = generate(current->get_parent());
    uint32_t current_rank = get_rank(current_id);
    bool up_travel = !parent || (current_rank > get_rank(parent->get_id()));

    warthog::graph::node* n = g_->get_node(current_id);
    warthog::graph::edge_iter eit = n->outgoing_begin();
    while(succ_dirs)
    {
        uint32_t label = __builtin_ffs(succ_dirs)-1;
        uint32_t first = g_->labelled_edge_offset(current_id, label);
        uint32_t last = g_->labelled_edge_offset(current_id, label+1);
        for( uint32_t i = first; i < last; i++)
        {
            warthog::graph::edge& e = *(eit+i);
            assert(e.node_id_ < g_->get_num_nodes());
            
            // prune the following successors:
            // (i) those that aren't forced or natural (in JPS terminology)
            // (ii) those that don't jive with FCH up/down rules; i.e.
            // traveling up the hierarchy we generate all neighbours;
            // traveling down, we generate only "down" neighbours
#ifndef NDEBUG
            warthog::jps::direction s_dir = get_dir(&e, warthog::jpg::FIRST);
            assert((current_id == problem->start_id_) ||
                    (succ_dirs & s_dir));
#endif
            
            bool down_succ = get_rank(e.node_id_) < current_rank;
            if((up_travel && !down_succ) || 
               (down_succ && !filter(current_id, i)))
            {
                this->add_neighbour(this->generate(e.node_id_), e.wt_);
            }
        }
        succ_dirs = succ_dirs & ~(1 << label);
    }

    // dumb hacky goal test. necessary because the goal is never 
    // inserted into the direction-index maintained by 
    // corner_point_graph (we use the index to limit the set of
    // successors being scanned to just those which are in a 
    // forced or natural direction)
    eit += n->out_degree() - 1;
    if(eit->node_id_ == g_->get_inserted_target_id())
    {
        add_neighbour(generate(eit->node_id_), eit->wt_);
    }
}

void
warthog::fch_af_jpg_expansion_policy::get_xy(
        uint32_t id, int32_t& x, int32_t& y)
{
    g_->get_xy(id, x, y);
}

warthog::search_node* 
warthog::fch_af_jpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

warthog::search_node*
warthog::fch_af_jpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_ = pi->instance_id_;
    }

    t_part_.clear();
    uint32_t t_id = g_->get_inserted_target_id();
    if(t_id != g_->get_dummy_target_id())
    {
        t_part_.insert(afl_->get_partitioning()->at(t_id));
    }
    else
    {
        warthog::graph::node* target = g_->get_node(t_id);
        for( warthog::graph::edge_iter it = target->incoming_begin();
             it != target->incoming_end(); it++ )
        {
            uint32_t nei_part = afl_->get_partitioning()->at(it->node_id_);
            t_part_.insert(nei_part);
        }
    }
    return this->generate(t_id);
}

bool
warthog::fch_af_jpg_expansion_policy::filter(
        uint32_t node_id, uint32_t edge_index)
{
    uint8_t* label = afl_->get_label(node_id, edge_index);
    bool retval = 0;
    for(std::set<uint32_t>::iterator it = t_part_.begin();
            it != t_part_.end(); it++)
    {
        uint32_t part_id = *it;
        uint32_t index = part_id >> 3;
        uint32_t bit_mask = 1 << (part_id & 7);
        retval |= label[index] & bit_mask;
    }
    return retval == 0;
}
