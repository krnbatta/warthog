#include "contraction.h"
#include "fch_dfs_expansion_policy.h"
#include "xy_graph.h"
#include "search_node.h"

warthog::fch_dfs_expansion_policy::fch_dfs_expansion_policy(
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* rank,
        warthog::label::dfs_labelling* lab,
        bool sort_successors)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    lab_ = lab;
    t_label = s_label = INT32_MAX;
    //filter = &warthog::fch_dfs_expansion_policy::filter_all;
    filter = &warthog::fch_dfs_expansion_policy::filter_id_range_and_bb;
    //filter = &warthog::fch_dfs_expansion_policy::filter_id_range_and_af;
    //filter = &warthog::fch_dfs_expansion_policy::filter_bb_and_af;
    //filter = &warthog::fch_dfs_expansion_policy::filter_bb_only;
    //filter = &warthog::fch_dfs_expansion_policy::filter_af_only;
    //filter = &warthog::fch_dfs_expansion_policy::filter_id_range_only;

    // sort edges s.t. all up successors appear before any down successor
    if(sort_successors) { warthog::ch::fch_sort_successors(g, rank); }

    // store the location of the first down successor 
    heads_ = new uint8_t[g->get_num_nodes()];
    for(uint32_t i = 0; i < g_->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);
        uint32_t i_rank = rank->at(i);

        heads_[i] = n->out_degree(); // begin assuming none
        for(warthog::graph::edge_iter it = n->outgoing_begin();
                it != n->outgoing_end(); it++)
        {
            if(rank_->at(it->node_id_) < i_rank)
            {
                heads_[i] = it - n->outgoing_begin();
                break;
            }
        }
    }
}

warthog::fch_dfs_expansion_policy::~fch_dfs_expansion_policy()
{
    delete [] heads_;
}

void
warthog::fch_dfs_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance*)
{
    reset();

    warthog::search_node* pn = generate(current->get_parent());
    uint32_t current_id = current->get_id();
    uint32_t current_rank = get_rank(current_id);

    warthog::graph::node* n = g_->get_node(current_id);
    warthog::graph::edge_iter begin, end, succ;
    begin = n->outgoing_begin();
    end = n->outgoing_end();
    succ = begin + 
        // adjust the successors pointer by an appropriate offset.
        // traveling up the hierarchy we generate all neighbours;
        // traveling down, we generate only "down" neighbours
        (pn && (current_rank < get_rank(pn->get_id()))) *
        heads_[current_id];

    for( ; succ != end; succ++)
    {
        warthog::graph::edge& e = *succ;
        assert(e.node_id_ < g_->get_num_nodes());
        if(!(this->*filter)(current_id, (succ - begin)))
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
        }
    }
}

void
warthog::fch_dfs_expansion_policy::get_xy(
        uint32_t nid, int32_t& x, int32_t& y)
{
    g_->get_xy(nid, x, y);
}

warthog::search_node* 
warthog::fch_dfs_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    uint32_t s_graph_id = g_->to_graph_id(pi->start_id_);
    if(s_graph_id == warthog::INF) { return 0; }

    s_label = lab_->get_dfs_index(s_graph_id);

    return generate(s_graph_id);
}

warthog::search_node*
warthog::fch_dfs_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id == warthog::INF) { return 0; }

    t_rank = get_rank(t_graph_id);
    t_label = lab_->get_dfs_index(t_graph_id);
    
    uint32_t t_part = lab_->get_partitioning()->at(t_graph_id);
    t_byte_ = t_part >> 3; // div8
    t_bitmask_ = 1 << (t_part & 7); // mod8

    get_xy(t_graph_id, tx_, ty_);

    return generate(t_graph_id);
}
