#include "bbaf_filter.h"
#include "bch_bbaf_expansion_policy.h"
#include "contraction.h"
#include "problem_instance.h"
#include "search_node.h"

warthog::bch_bbaf_expansion_policy::bch_bbaf_expansion_policy(
        warthog::graph::xy_graph* g, 
        warthog::bbaf_filter* filter,
        bool backward)
    : expansion_policy(g->get_num_nodes())
{
    g_ = g;
    filter_ = filter;
    backward_ = backward;

    if(backward_)
    {
        fn_begin_iter_ = &warthog::bch_bbaf_expansion_policy::get_bwd_begin_iter;
        fn_end_iter_ = &warthog::bch_bbaf_expansion_policy::get_bwd_end_iter;
        fn_rev_end_iter_ = &warthog::bch_bbaf_expansion_policy::get_fwd_end_iter;
        fn_rev_begin_iter_ = &warthog::bch_bbaf_expansion_policy::get_fwd_begin_iter;
    }
    else
    {
        fn_begin_iter_ = &warthog::bch_bbaf_expansion_policy::get_fwd_begin_iter;
        fn_end_iter_ = &warthog::bch_bbaf_expansion_policy::get_fwd_end_iter;
        fn_rev_begin_iter_ = &warthog::bch_bbaf_expansion_policy::get_bwd_begin_iter;
        fn_rev_end_iter_ = &warthog::bch_bbaf_expansion_policy::get_bwd_end_iter;
    }
}

void
warthog::bch_bbaf_expansion_policy::expand(warthog::search_node* current,
        warthog::problem_instance* problem)
{
    reset();

    uint32_t current_id = current->get_id();
    warthog::graph::node* n = g_->get_node(current_id);
    warthog::graph::edge_iter begin, end;

    // stall-on-demand
    begin = (this->*fn_rev_begin_iter_)(n);
    end = (this->*fn_rev_end_iter_)(n);
    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());
        warthog::search_node* next = this->generate(e.node_id_);
        if(next->get_search_id() == current->get_search_id() &&
                current->get_g() > (next->get_g() + e.wt_))
        {
            return; // stall
        }
    }

    // OK, node doesn't need stalling; generate successors as usual
    begin = (this->*fn_begin_iter_)(n);
    end = (this->*fn_end_iter_)(n);
    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());
        if(!filter_->filter(current_id, it - begin))
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
        }
    }
}

size_t
warthog::bch_bbaf_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + 
        sizeof(this);
}

void
warthog::bch_bbaf_expansion_policy::get_xy(
        uint32_t node_id, int32_t& x, int32_t& y)
{
    g_->get_xy(node_id, x, y);
}

warthog::search_node* 
warthog::bch_bbaf_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    // update the filter with the new target location
    uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id != warthog::INF) 
    {
        filter_->set_target(t_graph_id);
    }

    // generate the start node
    uint32_t s_graph_id = g_->to_graph_id(pi->start_id_);
    if(s_graph_id == warthog::INF) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::bch_bbaf_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id == warthog::INF) { return 0; }
    // update the filter with the new target location
    {
        filter_->set_target(t_graph_id);
    }
    return generate(t_graph_id);
}
