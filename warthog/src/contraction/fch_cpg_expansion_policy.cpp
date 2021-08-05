#include "contraction.h"
#include "fch_cpg_expansion_policy.h"
#include "corner_point_graph.h"
#include "search_node.h"

warthog::fch_cpg_expansion_policy::fch_cpg_expansion_policy(
        warthog::graph::corner_point_graph* g, std::vector<uint32_t>* rank)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    init();
}

warthog::fch_cpg_expansion_policy::~fch_cpg_expansion_policy()
{
}

void
warthog::fch_cpg_expansion_policy::init()
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

void
warthog::fch_cpg_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance*)
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
        
        if(up_travel || (get_rank(e.node_id_) < current_rank))
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
        }
    }
}

void
warthog::fch_cpg_expansion_policy::get_xy(uint32_t nid, int32_t& x, int32_t& y)
{
    g_->get_xy(nid, x, y);
}

warthog::search_node* 
warthog::fch_cpg_expansion_policy::generate_start_node(
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
warthog::fch_cpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_at_last_insert_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_target_id());
}
