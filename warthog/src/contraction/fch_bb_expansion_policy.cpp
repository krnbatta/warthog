#include "contraction.h"
#include "bb_filter.h"
#include "euclidean_heuristic.h"
#include "fch_bb_expansion_policy.h"
#include "xy_graph.h"
#include "search_node.h"

warthog::fch_bb_expansion_policy::fch_bb_expansion_policy(
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* rank,
        warthog::bb_filter* nf)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    nf_ = nf;
    apex_ = warthog::INF;
    apex_reached_ = false;
}

warthog::fch_bb_expansion_policy::~fch_bb_expansion_policy()
{
}

void
warthog::fch_bb_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance* instance)
{
    reset();

    warthog::search_node* pn = generate(current->get_parent());
    uint32_t current_id = current->get_id();
    uint32_t current_rank = get_rank(current_id);

    if(rank_->at(current_id) == apex_)
    {
        apex_reached_ = true;
    }

    warthog::graph::node* n = g_->get_node(current_id);
    warthog::graph::edge_iter begin, end;
    begin = n->outgoing_begin();
    end = n->outgoing_end();

    // determine whether current was reached via an up edge or a down edge
    bool up_travel = !pn || (current_rank > get_rank(pn->get_id()));
    //std::cerr << (up_travel ? "(UPTRAVEL) " : "(DNTRAVEL) ");
    for(warthog::graph::edge_iter it = begin; it != end; it++)
    {
        warthog::graph::edge& e = *it;
        assert(e.node_id_ < g_->get_num_nodes());

        // try to prune every down successor, regardless of 
        // wheter the parent was reached by an up edge or a
        // down edge
        bool down_succ = get_rank(e.node_id_) < current_rank;
        if(down_succ && !nf_->filter(current_id, (it - begin)))
        {
            // prune down successors before the apex is reached
            if(apex_ != warthog::INF && !apex_reached_) { continue; }
            
            // prune down successors below the goal
            if(rank_->at(e.node_id_) < rank_->at(instance->target_id_)) 
            { continue; }

            //std::cerr << " (D) ";

            warthog::search_node* tmp = this->generate(e.node_id_);
            this->add_neighbour(tmp, e.wt_);
            continue;
        }

        // generate up successors only when traveling up
        // (the rest are implicitly pruned)
        else if(up_travel && !down_succ)
        {
            // prune up successors after the apex is reached
            if(apex_ != warthog::INF && apex_reached_) { continue; }
            // prune up successors above the apex
            if(rank_->at(e.node_id_) > apex_) { continue; }

            //std::cerr << " (U) ";

            this->add_neighbour(this->generate(e.node_id_), e.wt_);
            continue;
        }
    }
    //std::cerr << "\n";
}

void
warthog::fch_bb_expansion_policy::get_xy(
        uint32_t id, int32_t& x, int32_t& y)
{
    g_->get_xy(id, x, y);
}

warthog::search_node* 
warthog::fch_bb_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    uint32_t s_graph_id = g_->to_graph_id(pi->start_id_);
    if(s_graph_id == warthog::INF) { return 0; }
    return generate(s_graph_id);
}

warthog::search_node*
warthog::fch_bb_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
    if(t_graph_id == warthog::INF) { return 0; }

    // update the filter with the new target location
    {
        int32_t tx, ty;
        g_->get_xy(t_graph_id, tx, ty);
        nf_->set_target_xy(tx, ty);
    }
    
    // finally, generate the inserted target node
    return generate(t_graph_id);
}
