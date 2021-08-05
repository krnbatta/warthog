#include "corner_point_graph.h"
#include "cpg_expansion_policy.h"
#include "corner_point_locator.h"
#include "online_jump_point_locator2.h"
#include "xy_graph.h"

warthog::cpg_expansion_policy::cpg_expansion_policy(
        warthog::graph::corner_point_graph* g) 
    : expansion_policy(g->get_num_nodes())
{
    g_ = g;
    instance_id_at_last_insert_ = warthog::INF;
}

warthog::cpg_expansion_policy::~cpg_expansion_policy()
{
}

void
warthog::cpg_expansion_policy::get_xy(uint32_t nid, int32_t& x, int32_t& y)
{
    g_->get_xy(nid, x, y);
}

void 
warthog::cpg_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance* problem)
{
    reset();

    uint32_t graph_id = current->get_id();
    warthog::graph::node* n = g_->get_node(graph_id);
    assert(n);
    for(warthog::graph::edge_iter eit = n->outgoing_begin(); 
            eit != n->outgoing_end(); eit++)
    {
        add_neighbour(generate(eit->node_id_), eit->wt_);
    }
}

size_t
warthog::cpg_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + sizeof(*this) + g_->mem();
}

warthog::search_node* 
warthog::cpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != instance_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        instance_id_at_last_insert_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

warthog::search_node*
warthog::cpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != instance_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        instance_id_at_last_insert_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_target_id());
}
