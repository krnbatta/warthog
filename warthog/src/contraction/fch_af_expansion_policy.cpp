#include "af_filter.h"
#include "contraction.h"
#include "fch_af_expansion_policy.h"
#include "xy_graph.h"
#include "search_node.h"

warthog::fch_af_expansion_policy::fch_af_expansion_policy(
        warthog::graph::xy_graph* g, std::vector<uint32_t>* rank,
        warthog::af_filter* filter)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    filter_ = filter;
    apex_ = warthog::INF;
    apex_reached_ = false;
}

warthog::fch_af_expansion_policy::~fch_af_expansion_policy()
{
}

void
warthog::fch_af_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance* instance)
{
    reset();

    warthog::search_node* pn = generate(current->get_parent());
    uint32_t current_id = current->get_id();
    uint32_t current_rank = get_rank(current_id);

    warthog::graph::node* n = g_->get_node(current_id);
    warthog::graph::edge_iter begin, end;
    begin = n->outgoing_begin();
    end = n->outgoing_end();

    // traveling up the hierarchy we generate all neighbours;
    // traveling down, we generate only "down" neighbours
    bool up_travel = !pn || (current_rank > get_rank(pn->get_id()));
    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());

        bool dn_succ = (get_rank(e.node_id_) < current_rank);
        if(up_travel || dn_succ)
        {
            if(!filter_->filter(current_id, it - begin))
            {
                // prune up successors above the apex
                if(rank_->at(e.node_id_) > apex_) { continue; }
                this->add_neighbour(this->generate(e.node_id_), e.wt_);
            }
        }
    }
}

void
warthog::fch_af_expansion_policy::get_xy(
        uint32_t id, int32_t& x, int32_t& y)
{
    g_->get_xy(id, x, y);
}

warthog::search_node* 
warthog::fch_af_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    uint32_t s_graph_id = g_->to_graph_id(pi->start_id_);
    if(s_graph_id == warthog::INF) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::fch_af_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id == warthog::INF) { return 0; }
    filter_->set_target(t_graph_id);
    return generate(t_graph_id);
}
