#include "ch_cpg_expansion_policy.h"
#include "contraction.h"
#include "problem_instance.h"
#include "search_node.h"

warthog::ch::ch_cpg_expansion_policy::ch_cpg_expansion_policy(
        warthog::graph::corner_point_graph* g, 
        std::vector<uint32_t>* rank, 
        bool backward,
        warthog::ch::search_direction sd)
    : expansion_policy(g->get_num_nodes())
{
    g_ = g;
    rank_ = rank;
    backward_ = backward;
    sd_ = sd;

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
warthog::ch::ch_cpg_expansion_policy::expand(warthog::search_node* current,
        warthog::problem_instance* problem)
{
    reset();

    uint32_t current_id = current->get_id();
    uint32_t current_rank = get_rank(current_id);
    warthog::graph::node* n = g_->get_node(current_id);
   
    warthog::graph::edge_iter begin, end;
    if(backward_)
    {
        begin = n->incoming_begin();
        end = n->incoming_end();
    }
    else
    {
        begin = n->outgoing_begin();
        end = n->outgoing_end();
    }

    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());

        if((sd_ & warthog::ch::UP) && get_rank(e.node_id_) > current_rank)
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
            continue;
        }

        if((sd_ & warthog::ch::DOWN) && get_rank(e.node_id_) < current_rank)
        {
             this->add_neighbour(this->generate(e.node_id_), e.wt_);
        }
    }
}

size_t
warthog::ch::ch_cpg_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + 
        sizeof(this);
}
