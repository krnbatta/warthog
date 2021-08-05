#include "contraction.h"
#include "fch_expansion_policy.h"
#include "xy_graph.h"
#include "search_node.h"

warthog::fch_expansion_policy::fch_expansion_policy(
        warthog::graph::xy_graph* g,
        std::vector<uint32_t>* rank,
        warthog::ch::search_direction d,
        bool sort_successors)
    : expansion_policy(g->get_num_nodes()), g_(g)
{
    rank_ = rank;

    // sort edges s.t. all up successors appear before any down successor
    if(sort_successors)
    {
        // TODO: only if sanity check fails
        warthog::ch::fch_sort_successors(g, rank);
    }

    // store the location of the first down successor
    down_heads_ = new uint8_t[g->get_num_nodes()];
    for(uint32_t i = 0; i < g_->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);
        uint32_t i_rank = rank->at(i);

        down_heads_[i] = n->out_degree(); // begin assuming none
        for(warthog::graph::edge_iter it = n->outgoing_begin();
                it != n->outgoing_end(); it++)
        {
            if(rank_->at(it->node_id_) < i_rank)
            {
                down_heads_[i] = it - n->outgoing_begin();
                break;
            }
        }
    }

    dir_ = d;
}

warthog::fch_expansion_policy::~fch_expansion_policy()
{
    delete [] down_heads_;
}

void
warthog::fch_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance*)
{
    reset();

    warthog::search_node* pn = generate(current->get_parent());
    uint32_t current_id = current->get_id();
    uint32_t current_rank = get_rank(current_id);
    warthog::graph::node* n = g_->get_node(current_id);


    // traveling up the hierarchy we generate all neighbours;
    // traveling down, we generate only "down" neighbours
    warthog::graph::edge_iter begin = n->outgoing_begin();
    warthog::graph::edge_iter end = n->outgoing_end();
    bool up_travel = dir_ & (!pn || (current_rank > get_rank(pn->get_id())));
    if(!up_travel) { begin += down_heads_[current_id]; }

    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());
        this->add_neighbour(this->generate(e.node_id_), e.wt_);
    }
}

void
warthog::fch_expansion_policy::get_xy(uint32_t nid, int32_t& x, int32_t& y)
{
    g_->get_xy(nid, x, y);
}

warthog::search_node*
warthog::fch_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    uint32_t s_graph_id = g_->to_graph_id(pi->start_id_);
    if(s_graph_id == warthog::INF) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::fch_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id == warthog::INF) { return 0; }
    return generate(t_graph_id);
}
