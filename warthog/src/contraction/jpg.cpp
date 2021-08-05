#include "contraction.h"
#include "corner_point_graph.h"
#include "jpg.h"

void
warthog::jpg::compute_direction_labels(warthog::graph::corner_point_graph* g_)
{
    warthog::graph::xy_graph* pg = g_->get_xy_graph();
    for(uint32_t node_id = 0; node_id < g_->get_num_nodes(); node_id++)
    {
        warthog::graph::node* n = g_->get_node(node_id);
        for(warthog::graph::edge_iter eit = n->outgoing_begin();
                eit != n->outgoing_end(); eit++)
        {
            warthog::jpg::process_edge(eit, node_id, pg);
        }
    }
}

void
warthog::jpg::process_edge(warthog::graph::edge* e, uint32_t e_tail_id,
             warthog::graph::xy_graph* pg)
{
    if(e->label_ != UINTPTR_MAX) { return; } // already labeled

    // unpacking a shortcut edge means finding the two arcs that make 
    // up the shortcut
    std::vector<warthog::graph::edge*> intermediate;
    warthog::ch::unpack_and_list_edges(e, e_tail_id, pg, intermediate, false);

    // edges that cannot be unpacked further are part of the corner point 
    // graph and labelled by analysing their endpoints
    if(intermediate.size() == 0)
    {
        label_edge(e, e_tail_id, pg);
        return;
    }

    // edge is a shortcut; recursively unpack it and label it 
    // using the labels of the unpacked edges
    warthog::graph::edge* e1 = intermediate.at(0);
    warthog::graph::edge* e2 = intermediate.at(1);
    assert(e2->node_id_ == e->node_id_);

    process_edge(e1, e_tail_id, pg);
    process_edge(e2, e1->node_id_, pg);
    assert(get_dir(e1, step_type::FIRST) != 255);
//    {
//        std::cerr << "wtf " << e_tail_id << " " << e->node_id_ << std::endl;
//        assert(false);
//    }
    assert(get_dir(e1, step_type::LAST) != 255);
    set_dir(e, step_type::FIRST, get_dir(e1, step_type::FIRST));
    set_dir(e, step_type::LAST, get_dir(e2, step_type::LAST));
}

void
warthog::jpg::label_edge(warthog::graph::edge* e, uint32_t e_tail_id, 
        warthog::graph::xy_graph* pg)
{
    // edge is part of the original corner point graph
    // we label it directly by analysing its endpoints
    // (the edge tail node and head node respectively)
    int32_t tx, ty, hx, hy;
    pg->get_xy(e_tail_id, tx, ty);
    pg->get_xy(e->node_id_, hx, hy);
    tx /= warthog::ONE;
    ty /= warthog::ONE;
    hx /= warthog::ONE;
    hy /= warthog::ONE;

    switch(warthog::jps::compute_direction(tx, ty, hx, hy))
    {
        case warthog::jps::NORTH:
        {
            set_dir(e, FIRST, warthog::jps::NORTH);
            set_dir(e, LAST, warthog::jps::NORTH);
            break;
        }
        case warthog::jps::SOUTH:
        {
            set_dir(e, FIRST, warthog::jps::SOUTH);
            set_dir(e, LAST, warthog::jps::SOUTH);
            break;
        }
        case warthog::jps::EAST:
        {
            set_dir(e, FIRST, warthog::jps::EAST);
            set_dir(e, LAST, warthog::jps::EAST);
            break;
        }
        case warthog::jps::WEST:
        {
            set_dir(e, FIRST, warthog::jps::WEST);
            set_dir(e, LAST, warthog::jps::WEST);
            break;
        }
        case warthog::jps::NORTHEAST:
        {
            set_dir(e, FIRST, warthog::jps::NORTHEAST);
            uint32_t dx = hx - tx;
            uint32_t dy = ty - hy;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::NORTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::EAST); }
            else { set_dir(e, LAST, warthog::jps::NORTHEAST); }
            break;
        }
        case warthog::jps::NORTHWEST:
        {
            set_dir(e, FIRST, warthog::jps::NORTHWEST);
            uint32_t dx = tx - hx;
            uint32_t dy = ty - hy;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::NORTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::WEST); }
            else { set_dir(e, LAST, warthog::jps::NORTHWEST); }
            break;
        }
        case warthog::jps::SOUTHEAST:
        {
            set_dir(e, FIRST, warthog::jps::SOUTHEAST);
            uint32_t dx = hx - tx;
            uint32_t dy = hy - ty;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::SOUTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::EAST); }
            else { set_dir(e, LAST, warthog::jps::SOUTHEAST); }
            break;
        }
        case warthog::jps::SOUTHWEST:
        {
            set_dir(e, FIRST, warthog::jps::SOUTHWEST);
            uint32_t dx = tx - hx;
            uint32_t dy = hy - ty;
            if(dx < dy) { set_dir(e, LAST, warthog::jps::SOUTH); }
            else if(dx > dy) { set_dir(e, LAST, warthog::jps::WEST); }
            else { set_dir(e, LAST, warthog::jps::SOUTHWEST); }
            break;
        }
        case warthog::jps::ALL:
        case warthog::jps::NONE:
        {
            assert(false);
            break;
        }
    }
}
