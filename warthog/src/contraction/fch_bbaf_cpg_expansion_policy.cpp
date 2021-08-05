#include "bbaf_labelling.h"
#include "contraction.h"
#include "fch_bbaf_cpg_expansion_policy.h"
#include "corner_point_graph.h"
#include "search_node.h"

warthog::fch_bbaf_cpg_expansion_policy::fch_bbaf_cpg_expansion_policy(
        warthog::graph::corner_point_graph* g, 
        std::vector<uint32_t>* rank,
        warthog::label::bbaf_labelling* lab)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    lab_ = lab;
}

warthog::fch_bbaf_cpg_expansion_policy::~fch_bbaf_cpg_expansion_policy()
{
}

void
warthog::fch_bbaf_cpg_expansion_policy::expand(
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

        // want to generate all start successors, up or down.
        if( current_id == g_->get_inserted_start_id())
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
            continue;
        }

        // the target is inserted into the graph but not into 
        // the filter. one way to get around this (HACK HACK HACK)
        // is to just generate the goal whenever we encounter it, 
        // ignoring the filter altogether. 
        if(e.node_id_ == g_->get_inserted_target_id())
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
            continue;
        }

        bool dn_succ = (get_rank(e.node_id_) < current_rank);
        if(up_travel || dn_succ)
        {
            if(!filter(current_id, it - begin, dn_succ))
            {
                this->add_neighbour(this->generate(e.node_id_), e.wt_);
            }
        }
    }
}

void
warthog::fch_bbaf_cpg_expansion_policy::get_xy(
        uint32_t id, int32_t& x, int32_t& y)
{
    g_->get_xy(id, x, y);
}

warthog::search_node* 
warthog::fch_bbaf_cpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

// when inserting the target node there are two cases to consider
// (i) the target node is a corner node.
// (ii) the target not is not a corner node
//
// Consider case (i). Since the target is in the corner graph we need
// only look at the label of the current edge and see if the edge can 
// appear on any optimal path to the target. so far, nothing has changed.
//
// Consider case (ii). Since the target is not in the corner graph
// we need to insert it. Insertion yields a set of successors, S.
// From a goal pruning perspective, the target is now not a single
// node but the entire set of nodes S. During search, if any edge can
// appear on an optimal path to any s \in S, we should relax the edge.
//
// Depending on |S| this might be slow. In the case of geometric containers 
// we can construct a bounding box for all nodes in S \cup { t } and check 
// if the bounding box stored with the current edge overlaps with the 
// bounding box containing the target and all of its successors. In the case
// of arc flags, we need to take the union of partitions for all nodes in S
// and check each one against the bitflags of the current edge.
warthog::search_node*
warthog::fch_bbaf_cpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_ = pi->instance_id_;
    }

    t_rect_.clear();
    t_part_.clear();
    uint32_t t_id = g_->get_inserted_target_id();
    if(t_id != g_->get_dummy_target_id())
    {
       t_part_.insert(lab_->get_partitioning()->at(t_id));
       int32_t my_x, my_y;
       g_->get_xy(g_->get_inserted_target_id(), my_x, my_y);
       t_rect_.grow(my_x, my_y);
    }
    else
    {
        warthog::graph::node* target = g_->get_node(t_id);
        for( warthog::graph::edge_iter it = target->incoming_begin();
             it != target->incoming_end(); it++ )
        {
            uint32_t nei_part = lab_->get_partitioning()->at(it->node_id_);
            t_part_.insert(nei_part);

            int32_t my_x, my_y;
            g_->get_xy(it->node_id_, my_x, my_y);
            t_rect_.grow(my_x, my_y);
        }
    }
    return this->generate(t_id);
}

bool
warthog::fch_bbaf_cpg_expansion_policy::filter(
        uint32_t node_id, uint32_t edge_index, bool down)
{
    warthog::label::bbaf_label& label = lab_->get_label(node_id, edge_index);

    // try to prune every edge using arcflags
    bool retval = 0;
    uint8_t* eflags = label.flags_;
    for(std::set<uint32_t>::iterator it = t_part_.begin();
            it != t_part_.end(); it++)
    {
        uint32_t part_id = *it;
        uint32_t index = part_id >> 3;
        uint32_t bit_mask = 1 << (part_id & 7);
        retval |= eflags[index] & bit_mask;
    }

    // further pruning for down successors via 
    // geometric containers
    retval = 
        (retval && !down) ||
        (retval && t_rect_.intersects(label.bbox_));
    return !retval;
}
