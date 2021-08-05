#include "contraction.h"
#include "fch_jpg_expansion_policy.h"
#include "corner_point_graph.h"
#include "search_node.h"

warthog::fch_jpg_expansion_policy::fch_jpg_expansion_policy(
        warthog::graph::corner_point_graph* g, std::vector<uint32_t>* rank)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    init();
    compute_direction_labels();
    g_->build_edge_label_index();
}

warthog::fch_jpg_expansion_policy::~fch_jpg_expansion_policy()
{
}

void
warthog::fch_jpg_expansion_policy::init()
{
    // we insert two extra elements in the event that we
    // need to insert the start or target. both have the lowest
    // possible rank in the hierarchy (0 and 1)
    // NB: along the way we need to increase all ranks by 2 
    for(uint32_t i = 0; i < rank_->size(); i++) rank_->at(i)+=2;
    rank_->push_back(0);
    rank_->push_back(1);
    search_id_at_last_insert_ = warthog::INF;
    down_successors_only_ = false;
}

// TODO: 
// 1. ::expand
// 2. ::generate / insert
void
warthog::fch_jpg_expansion_policy::expand(
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
        warthog::jps::direction d_last = get_dir(e_pn, LAST);

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
    up_travel = up_travel && !down_successors_only_;

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
            warthog::jps::direction s_dir = get_dir(&e, FIRST);
            assert((current_id == problem->start_id_) ||
                    (succ_dirs & s_dir));
#endif
            
            if((up_travel || (get_rank(e.node_id_) < current_rank)))
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
warthog::fch_jpg_expansion_policy::get_xy(uint32_t nid, int32_t& x, int32_t& y)
{
    g_->get_xy(nid, x, y);
}

warthog::search_node* 
warthog::fch_jpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_at_last_insert_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

warthog::search_node*
warthog::fch_jpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_at_last_insert_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_target_id());
}

void
warthog::fch_jpg_expansion_policy::compute_direction_labels()
{
    warthog::graph::xy_graph* pg = g_->get_xy_graph();
    for(uint32_t node_id = 0; node_id < g_->get_num_nodes(); node_id++)
    {
        warthog::graph::node* n = g_->get_node(node_id);
        for(warthog::graph::edge_iter eit = n->outgoing_begin();
                eit != n->outgoing_end(); eit++)
        {
            process_edge(eit, node_id, pg);
        }
    }
}

void
warthog::fch_jpg_expansion_policy::process_edge(
        warthog::graph::edge* e, uint32_t e_tail_id,
        warthog::graph::xy_graph* pg)
{
    if(e->label_ != UINTPTR_MAX) { return; } // already labeled

    // unpacking a shortcut edge means finding the two arcs that make 
    // up the shortcut
    std::vector<warthog::graph::edge*> intermediate;
    warthog::ch::unpack_and_list_edges(e, e_tail_id, pg, intermediate, false);

    // edges that cannot be unpacked further are part of the corner point 
    // graph and labelled by analysing their endpoints
    if(intermediate.size() == 0)
    {
        label_edge(e, e_tail_id, pg);
        return;
    }

    // edge is a shortcut; recursively unpack it and label it 
    // using the labels of the unpacked edges
    warthog::graph::edge* e1 = intermediate.at(0);
    warthog::graph::edge* e2 = intermediate.at(1);
    assert(e2->node_id_ == e->node_id_);

    process_edge(e1, e_tail_id, pg);
    process_edge(e2, e1->node_id_, pg);
    assert(get_dir(e1, step_type::FIRST) != 255);
//    {
//        std::cerr << "wtf " << e_tail_id << " " << e->node_id_ << std::endl;
//        assert(false);
//    }
    assert(get_dir(e1, step_type::LAST) != 255);
    set_dir(e, step_type::FIRST, get_dir(e1, step_type::FIRST));
    set_dir(e, step_type::LAST, get_dir(e2, step_type::LAST));
}

void
warthog::fch_jpg_expansion_policy::label_edge(warthog::graph::edge* e, 
        uint32_t e_tail_id, warthog::graph::xy_graph* pg)
{
    // edge is part of the original corner point graph
    // we label it directly by analysing its endpoints
    // (the edge tail node and head node respectively)
    int32_t tx, ty, hx, hy;
    pg->get_xy(e_tail_id, tx, ty);
    pg->get_xy(e->node_id_, hx, hy);
    tx /= warthog::ONE;
    ty /= warthog::ONE;
    hx /= warthog::ONE;
    hy /= warthog::ONE;

    switch(warthog::jps::compute_direction(tx, ty, hx, hy))
    {
        case warthog::jps::NORTH:
        {
            set_dir(e, FIRST, warthog::jps::NORTH);
            set_dir(e, LAST, warthog::jps::NORTH);
            break;
        }
        case warthog::jps::SOUTH:
        {
            set_dir(e, FIRST, warthog::jps::SOUTH);
            set_dir(e, LAST, warthog::jps::SOUTH);
            break;
        }
        case warthog::jps::EAST:
        {
            set_dir(e, FIRST, warthog::jps::EAST);
            set_dir(e, LAST, warthog::jps::EAST);
            break;
        }
        case warthog::jps::WEST:
        {
            set_dir(e, FIRST, warthog::jps::WEST);
            set_dir(e, LAST, warthog::jps::WEST);
            break;
        }
        case warthog::jps::NORTHEAST:
        {
            set_dir(e, FIRST, warthog::jps::NORTHEAST);
            uint32_t dx = hx - tx;
            uint32_t dy = ty - hy;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::NORTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::EAST); }
            else { set_dir(e, LAST, warthog::jps::NORTHEAST); }
            break;
        }
        case warthog::jps::NORTHWEST:
        {
            set_dir(e, FIRST, warthog::jps::NORTHWEST);
            uint32_t dx = tx - hx;
            uint32_t dy = ty - hy;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::NORTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::WEST); }
            else { set_dir(e, LAST, warthog::jps::NORTHWEST); }
            break;
        }
        case warthog::jps::SOUTHEAST:
        {
            set_dir(e, FIRST, warthog::jps::SOUTHEAST);
            uint32_t dx = hx - tx;
            uint32_t dy = hy - ty;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::SOUTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::EAST); }
            else { set_dir(e, LAST, warthog::jps::SOUTHEAST); }
            break;
        }
        case warthog::jps::SOUTHWEST:
        {
            set_dir(e, FIRST, warthog::jps::SOUTHWEST);
            uint32_t dx = tx - hx;
            uint32_t dy = hy - ty;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::SOUTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::WEST); }
            else { set_dir(e, LAST, warthog::jps::SOUTHWEST); }
            break;
        }
        case warthog::jps::ALL:
        case warthog::jps::NONE:
        {
            assert(false);
            break;
        }
    }
}


//bool
//warthog::fch_jpg_expansion_policy::filter(
//        uint32_t node_id, uint32_t edge_index)
//{
//    reset();
//    uint32_t graph_id = current->get_id();
//
//    uint32_t succ_dirs;
//    if(graph_id == problem->start_id_)
//    {
//        // special case for the start node: since there is no parent, 
//        // expand all successors in every direction
//        succ_dirs = warthog::jps::direction::ALL;
//    }
//
//    else
//    {
//        // for all other nodes we need to compute the
//        // travel direction from the parent
//        int32_t px, py, x, y;
//        this->get_xy(graph_id, x, y);
//        this->get_xy(current->get_parent()->get_id(), px, py);
//        warthog::jps::direction dir_c = 
//            warthog::jps::compute_direction((uint32_t)px, (uint32_t)py, 
//                    (uint32_t)x, (uint32_t)y);
//        
//        // using dir_c we now compute a set of directions 
//        // in which we should look for successors
//        warthog::gridmap* gm = g_->get_gridmap();
//        uint32_t grid_id = gm->to_padded_id(x, y);
//        uint32_t tiles;
//        gm->get_neighbours(grid_id, (uint8_t*)&tiles);
//        succ_dirs = warthog::jps::compute_successors(dir_c, tiles);
//    }
//
//    warthog::graph::node* n = g_->get_node(graph_id);
//    warthog::graph::edge_iter eit = n->outgoing_begin();
//    while(succ_dirs)
//    {
//        uint32_t label = __builtin_ffs(succ_dirs)-1;
//        uint32_t first = g_->labelled_edge_offset(graph_id, label);
//        uint32_t last = g_->labelled_edge_offset(graph_id, label+1);
//
//        for( uint32_t i = first; i < last; i++)
//        {
//            add_neighbour(generate((eit+i)->node_id_), (eit+i)->wt_);
//        }
//        succ_dirs = succ_dirs & ~(1 << label);
//    }
//
//    // goal test
//    eit += n->out_degree() - 1;
//    if(eit->node_id_ == g_->get_inserted_target_id())
//    {
//        add_neighbour(generate(eit->node_id_), eit->wt_);
//    }
//}
//
