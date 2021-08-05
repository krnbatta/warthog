#include "contraction.h"
#include "bb_labelling.h"
#include "euclidean_heuristic.h"
#include "fch_bb_cpg_expansion_policy.h"
#include "corner_point_graph.h"
#include "search_node.h"

warthog::fch_bb_cpg_expansion_policy::fch_bb_cpg_expansion_policy(
        warthog::graph::corner_point_graph* g, 
        std::vector<uint32_t>* rank,
        warthog::label::bb_labelling* lab)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    lab_ = lab;
    apex_ = warthog::INF;
    apex_reached_ = false;

    // we insert two extra elements in the event that we
    // need to insert the start or target. both have the lowest
    // possible rank in the hierarchy (0 and 1)
    // NB: along the way we need to increase all ranks by 2 
    for(uint32_t i = 0; i < rank_->size(); i++) rank_->at(i)+=2;
    rank_->push_back(0);
    rank_->push_back(1);
    search_id_at_last_insert_ = warthog::INF;
    instance_ = 0;
}

warthog::fch_bb_cpg_expansion_policy::~fch_bb_cpg_expansion_policy()
{
}

void
warthog::fch_bb_cpg_expansion_policy::expand(
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
        //assert(e.node_id_ < g_->get_num_nodes());

        // the target is inserted into the graph but not into 
        // the filter. one way to get around this (HACK HACK HACK)
        // is to just generate the goal whenever we encounter it, 
        // ignoring the filter altogether
        if(e.node_id_ == g_->get_inserted_target_id())
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
            continue;
        }

        // try to prune every down successor, regardless of 
        // wheter the parent was reached by an up edge or a
        // down edge
        bool down_succ = get_rank(e.node_id_) < current_rank;
        if(down_succ && !filter(current_id, it - begin))
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
warthog::fch_bb_cpg_expansion_policy::get_xy(
        uint32_t id, int32_t& x, int32_t& y)
{
    g_->get_xy(id, x, y);
}

warthog::search_node* 
warthog::fch_bb_cpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_at_last_insert_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

// when inserting the target node there are two cases to consider
// (i) the target node is a corner node.
// (ii) the target not is not a corner node
//
// Consider case (i). Since the target is in the corner graph we need
// only look at the label of the current edge and see if the edge can 
// appear on any optimal path to the target. This is just vanilla geometric
// container pruning. So far, nothing has changed.
//
// Consider case (ii). Since the target is not in the corner graph
// we need to insert it. Insertion yields a node whose set of successors 
// S are all corner nodes.
// From a goal pruning perspective, the target is now not a single
// node but the entire set of nodes S. During search, if any edge can
// appear on an optimal path to any s \in S, we should relax the edge.
//
// Depending on |S| this might be slow. In the case of geometric containers 
// we can construct a bounding box for all nodes in S \cup { t } and check 
// if the bounding box stored with the current edge overlaps with the 
// bounding box containing the target and all of its successors. 
warthog::search_node*
warthog::fch_bb_cpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_at_last_insert_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_at_last_insert_ = pi->instance_id_;
    }

    r_.clear();
    if(g_->get_inserted_target_id() != g_->get_dummy_target_id())
    {
       int32_t my_x, my_y;
       g_->get_xy(g_->get_inserted_target_id(), my_x, my_y);
       r_.grow(my_x, my_y);
    }
    else
    {
        warthog::graph::node* target = 
            g_->get_node(g_->get_inserted_target_id());
        for( warthog::graph::edge_iter it = target->incoming_begin();
             it != target->incoming_end(); it++ )
        {
            int32_t my_x, my_y;
            g_->get_xy(it->node_id_, my_x, my_y);
            r_.grow(my_x, my_y);
        }
    }
    
    // update the filter with the new target location
    //{
    //    int32_t tx, ty;
    //    g_->get_xy(g_->get_inserted_target_id(), tx, ty);
    //    nf_->set_target_xy(tx, ty);
    //}
    
    // finally, generate the inserted target node
    return this->generate(g_->get_inserted_target_id());
}

bool
warthog::fch_bb_cpg_expansion_policy::filter(
        uint32_t node_id, uint32_t edge_id)
{
    warthog::geom::rectangle rect = 
    lab_->get_label(node_id, edge_id);
    return r_.intersects(rect) == 0;

    //bool retval = 0;
    //for(uint32_t i = 0; i < proxy_xy_.size(); i+=2)
    //{
    //    retval |= rect.contains(proxy_xy_.at(i), proxy_xy_.at(i+1));
    //}
    //return retval == 0;
}
