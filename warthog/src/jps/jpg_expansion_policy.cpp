#include "jpg_expansion_policy.h"
#include "corner_point_graph.h"

warthog::jps::jpg_expansion_policy::jpg_expansion_policy(
        warthog::graph::corner_point_graph* g)
    : expansion_policy(g->get_num_nodes())
{
    g_ = g;
    instance_id_ = warthog::INF;
}

warthog::jps::jpg_expansion_policy::~jpg_expansion_policy()
{
}

void
warthog::jps::jpg_expansion_policy::get_xy(uint32_t id, int32_t& x, int32_t& y)
{
    return g_->get_xy(id, x, y);
}

void 
warthog::jps::jpg_expansion_policy::expand(
        warthog::search_node* current, warthog::problem_instance* problem)
{
    reset();
    uint32_t graph_id = current->get_id();

    uint32_t succ_dirs;
    if(graph_id == problem->start_id_)
    {
        // special case for the start node: since there is no parent, 
        // expand all successors in every direction
        succ_dirs = warthog::jps::direction::ALL;
    }

    else
    {
        // for all other nodes we need to compute the
        // travel direction from the parent
        int32_t px, py, x, y;
        this->get_xy(graph_id, x, y);
        this->get_xy(current->get_parent(), px, py);
        
        // TODO: fix these hacky divs. What's needed here is for
        // warthog::corner_point_graph to allow easy conversion 
        // between the graph id of a node and its grid id
        // then, we call gm->to_unpadded_xy or similar
        x /= warthog::ONE;
        y /= warthog::ONE;
        px /= warthog::ONE;
        py /= warthog::ONE;

        warthog::jps::direction dir_c = 
            warthog::jps::compute_direction((uint32_t)px, (uint32_t)py, 
                    (uint32_t)x, (uint32_t)y);
        
        // using dir_c we now compute a set of directions 
        // in which we should look for successors
        warthog::gridmap* gm = g_->get_gridmap();
        uint32_t grid_id = gm->to_padded_id(x, y);
        uint32_t tiles;
        gm->get_neighbours(grid_id, (uint8_t*)&tiles);
        succ_dirs = warthog::jps::compute_successors(dir_c, tiles);
    }

    warthog::graph::node* n = g_->get_node(graph_id);
    warthog::graph::edge_iter eit = n->outgoing_begin();
    while(succ_dirs)
    {
        uint32_t label = __builtin_ffs(succ_dirs)-1;
        uint32_t first = g_->labelled_edge_offset(graph_id, label);
        uint32_t last = g_->labelled_edge_offset(graph_id, label+1);

        for( uint32_t i = first; i < last; i++)
        {
            add_neighbour(generate((eit+i)->node_id_), (eit+i)->wt_);
        }
        succ_dirs = succ_dirs & ~(1 << label);
    }

    // goal test
    eit += n->out_degree() - 1;
    if(eit->node_id_ == g_->get_inserted_target_id())
    {
        add_neighbour(generate(eit->node_id_), eit->wt_);
    }
}

size_t
warthog::jps::jpg_expansion_policy::mem()
{
    return 
        expansion_policy::mem() + sizeof(*this) + g_->mem();
}

warthog::search_node* 
warthog::jps::jpg_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != instance_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        instance_id_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_start_id());
}

warthog::search_node*
warthog::jps::jpg_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    if(pi->instance_id_ != instance_id_)
    {
        g_->insert(pi->start_id_, pi->target_id_);
        instance_id_ = pi->instance_id_;
    }
    return this->generate(g_->get_inserted_target_id());
}
