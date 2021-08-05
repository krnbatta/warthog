#include "af_filter.h"
#include "chase_expansion_policy.h"
#include "contraction.h"
#include "problem_instance.h"
#include "search_node.h"

warthog::chase_expansion_policy::chase_expansion_policy(
        warthog::graph::xy_graph* g, 
        warthog::af_filter* filter,
        bool backward) : expansion_policy(g->get_num_nodes())
{
    g_ = g;
    filter_ = filter;
    backward_ = backward;

    if(backward_)
    {
        fn_begin_iter_ = &warthog::chase_expansion_policy::get_bwd_begin_iter;
        fn_end_iter_ = &warthog::chase_expansion_policy::get_bwd_end_iter;
    }
    else
    {
        fn_begin_iter_ = &warthog::chase_expansion_policy::get_fwd_begin_iter;
        fn_end_iter_ = &warthog::chase_expansion_policy::get_fwd_end_iter;
    }

}

void
warthog::chase_expansion_policy::expand(warthog::search_node* current,
        warthog::problem_instance* problem)
{
    reset();

    uint32_t current_id = current->get_id();
    warthog::graph::node* n = g_->get_node(current_id);
   
    warthog::graph::edge_iter begin, end;
    begin = (this->*fn_begin_iter_)(n);
    end = (this->*fn_end_iter_)(n);

    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());
        if(!(this->*fn_filter_arc)(current_id, it - begin))
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
        }
    }
}

size_t
warthog::chase_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + 
        sizeof(this);
}

void
warthog::chase_expansion_policy::get_xy(
        uint32_t node_id, int32_t& x, int32_t& y)
{
    g_->get_xy(node_id, x, y);
}

warthog::search_node* 
warthog::chase_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    // update the filter with the new target location
    uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id != warthog::INF) 
    { 
       filter_->set_target(t_graph_id);
    }
    fn_filter_arc = &warthog::chase_expansion_policy::phase1_filter_fn;

    // generate the start node
    uint32_t s_graph_id = g_->to_graph_id(pi->start_id_);
    if(s_graph_id == warthog::INF) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::chase_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id == warthog::INF) { return 0; }

    // update the filter with the new target location
    filter_->set_target(t_graph_id);
    fn_filter_arc = &warthog::chase_expansion_policy::phase1_filter_fn;

    return generate(t_graph_id);
}
