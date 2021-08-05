#include "bb_labelling.h"
#include "contraction.h"
#include "corner_point_graph.h"
#include "fch_bb_jpg_expansion_policy.h"
#include "jpg.h"
#include "search_node.h"

warthog::fch_bb_jpg_expansion_policy::fch_bb_jpg_expansion_policy(
        warthog::graph::corner_point_graph* g, 
        std::vector<uint32_t>* rank,
        warthog::label::bb_labelling* lab)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    lab_ = lab;

    warthog::jpg::compute_direction_labels(g);
    g_->build_edge_label_index();

    init();
}

warthog::fch_bb_jpg_expansion_policy::~fch_bb_jpg_expansion_policy()
{
}

void
warthog::fch_bb_jpg_expansion_policy::init()
{
    // we insert two extra elements in the event that we
    // need to insert the start or target. both have the lowest
    // possible rank in the hierarchy (0 and 1)
    // NB: along the way we need to increase all ranks by 2 
    for(uint32_t i = 0; i < rank_->size(); i++) rank_->at(i)+=2;
    rank_->push_back(0);
    rank_->push_back(1);
    search_id_at_last_insert_ = warthog::INF;
}

// TODO: 
// 1. ::expand
// 2. ::generate / insert
void
warthog::fch_bb_jpg_expansion_policy::expand(
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
        warthog::jps::direction d_last = warthog::jpg::get_dir(
                e_pn, warthog::jpg::LAST);

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
            warthog::jps::direction s_dir = 
                warthog::jpg::get_dir(&e, warthog::jpg::FIRST);
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
warthog::fch_bb_jpg_expansion_policy::get_xy(uint32_t nid, int32_t& x, int32_t& y)
{
    g_->get_xy(nid, x, y);
}

warthog::search_node* 
warthog::fch_bb_jpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_at_last_insert_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

// when inserting the target node there are two cases to consider
// (i) the target node is a corner node.
// (ii) the target not is not a corner node
//
// Consider case (i). Since the target is in the corner graph we need
// only look at the label of the current edge and see if the edge can 
// appear on any optimal path to the target. so far, nothing has changed.
//
// Consider case (ii). Since the target is not in the corner graph
// we need to insert it. Insertion yields a set of successors, S.
// From a goal pruning perspective, the target is now not a single
// node but the entire set of nodes S. During search, if any edge can
// appear on an optimal path to any s \in S, we should relax the edge.
//
// Depending on |S| this might be slow. In the case of geometric containers 
// we can construct a bounding box for all nodes in S \cup { t } and check 
// if the bounding box stored with the current edge overlaps with the 
// bounding box containing the target and all of its successors. 
warthog::search_node*
warthog::fch_bb_jpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_at_last_insert_ = pi->instance_id_;
    }

    r_.clear();
    if(g_->get_inserted_target_id() != g_->get_dummy_target_id())
    {
       int32_t my_x, my_y;
       g_->get_xy(g_->get_inserted_target_id(), my_x, my_y);
       r_.grow(my_x, my_y);
    }
    else
    {
        warthog::graph::node* target = 
            g_->get_node(g_->get_inserted_target_id());
        for( warthog::graph::edge_iter it = target->incoming_begin();
             it != target->incoming_end(); it++ )
        {
            int32_t my_x, my_y;
            g_->get_xy(it->node_id_, my_x, my_y);
            r_.grow(my_x, my_y);
        }
    }

    // update the filter with the new target location
//    {
//        int32_t tx, ty;
//        g_->get_xy(g_->get_inserted_target_id(), tx, ty);
//        nf_->set_target_xy(tx, ty);
//    }

    return this->generate(g_->get_inserted_target_id());
}

bool
warthog::fch_bb_jpg_expansion_policy::filter(
        uint32_t node_id, uint32_t edge_id)
{
    warthog::geom::rectangle rect = 
    lab_->get_label(node_id, edge_id);
    return r_.intersects(rect) == 0;

    //bool retval = 0;
    //for(uint32_t i = 0; i < proxy_xy_.size(); i+=2)
    //{
    //    retval |= rect.contains(proxy_xy_.at(i), proxy_xy_.at(i+1));
    //}
    //return retval == 0;
}
