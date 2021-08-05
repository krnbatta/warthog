#include "af_labelling.h"
#include "contraction.h"
#include "fch_af_cpg_expansion_policy.h"
#include "corner_point_graph.h"
#include "search_node.h"

warthog::fch_af_cpg_expansion_policy::fch_af_cpg_expansion_policy(
        warthog::graph::corner_point_graph* g, std::vector<uint32_t>* rank,
        warthog::label::af_labelling* afl)
    : expansion_policy(g->get_num_nodes()), g_(g) 
{
    rank_ = rank;
    afl_ = afl;
    apex_ = warthog::INF;
    apex_reached_ = false;
}

warthog::fch_af_cpg_expansion_policy::~fch_af_cpg_expansion_policy()
{
}

void
warthog::fch_af_cpg_expansion_policy::expand(
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

        // hack hack hack
        // (start and target are inserted into the corner graph but
        //  nothing is added to the filter. so we generate 
        //  (a) all successors of the start node and;
        //  (b) always generate the target anytime we reach it)
        if( current_id == g_->get_inserted_start_id() ||
            it->node_id_ == g_->get_inserted_target_id())
        {
            this->add_neighbour(this->generate(e.node_id_), e.wt_);
            continue;
        }

        bool dn_succ = (get_rank(e.node_id_) < current_rank);
        if(up_travel || dn_succ)
        {
            if(!filter(current_id, it - begin))
            {
                // prune up successors above the apex
                if(rank_->at(e.node_id_) > apex_) { continue; }
                this->add_neighbour(this->generate(e.node_id_), e.wt_);
            }
        }
    }
}

void
warthog::fch_af_cpg_expansion_policy::get_xy(
        uint32_t id, int32_t& x, int32_t& y)
{
    g_->get_xy(id, x, y);
}

warthog::search_node* 
warthog::fch_af_cpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

warthog::search_node*
warthog::fch_af_cpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != search_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        search_id_ = pi->instance_id_;
    }

    t_part_.clear();
    uint32_t t_id = g_->get_inserted_target_id();
    if(t_id != g_->get_dummy_target_id())
    {
        t_part_.insert(afl_->get_partitioning()->at(t_id));
    }
    else
    {
        warthog::graph::node* target = g_->get_node(t_id);
        for( warthog::graph::edge_iter it = target->incoming_begin();
             it != target->incoming_end(); it++ )
        {
            uint32_t nei_part = afl_->get_partitioning()->at(it->node_id_);
            t_part_.insert(nei_part);
        }
    }
    return this->generate(t_id);
}

bool
warthog::fch_af_cpg_expansion_policy::filter(
        uint32_t node_id, uint32_t edge_index)
{
    uint8_t* label = afl_->get_label(node_id, edge_index);
    bool retval = 0;
    for(std::set<uint32_t>::iterator it = t_part_.begin();
            it != t_part_.end(); it++)
    {
        uint32_t part_id = *it;
        uint32_t index = part_id >> 3;
        uint32_t bit_mask = 1 << (part_id & 7);
        retval |= label[index] & bit_mask;
    }
    return retval == 0;
}
