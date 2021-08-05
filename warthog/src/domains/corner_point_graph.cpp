#include "constants.h"
#include "corner_point_graph.h"
#include "corner_point_locator.h"
#include "graph.h"
#include "jps.h"

#include <algorithm>

typedef std::unordered_map<uint32_t, uint32_t>::iterator id_map_iter;

warthog::graph::corner_point_graph::corner_point_graph(
        std::shared_ptr<warthog::gridmap> gm)
{
    g_ = std::shared_ptr<warthog::graph::xy_graph>(
            new warthog::graph::xy_graph());
    cpl_ = new warthog::corner_point_locator(gm.get());

    e_lab_index_ = new std::vector<label_index>(g_->get_num_nodes());


    // do the thing
    construct();

    // we now insert two dummy nodes into the graph which will be
    // used whenever the start and target location are not corner points
    S_DUMMY_ID = this->add_node(0, 0); 
    T_DUMMY_ID = this->add_node(0, 0);

    s_grid_id_ = t_grid_id_ = warthog::INF;
    s_graph_id_ = t_graph_id_ = warthog::INF;
}

warthog::graph::corner_point_graph::corner_point_graph(
        std::shared_ptr<warthog::gridmap> gm, 
        std::shared_ptr<warthog::graph::xy_graph> g)
{
    g_ = g;
    cpl_ = new warthog::corner_point_locator(gm.get());

    s_graph_id_ = t_graph_id_ = warthog::INF;
    e_lab_index_ = new std::vector<label_index>(g_->get_num_nodes());

    // we now insert two dummy nodes into the graph which will be
    // used whenever the start and target location are not corner points
    S_DUMMY_ID = this->add_node(0, 0); 
    T_DUMMY_ID = this->add_node(0, 0);

    s_grid_id_ = t_grid_id_ = warthog::INF;
    s_graph_id_ = t_graph_id_ = warthog::INF;

    // map each graph node id to its corresponding grid id
    for(uint32_t graph_id = 0; graph_id < g_->get_num_nodes(); graph_id++)
    {
        int32_t x, y;
        get_xy(graph_id, x, y);
        assert(x == 0 || x >= warthog::ONE);
        assert(y == 0 || y >= warthog::ONE);
        x /= warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
        y /= warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;

        if(x == INT32_MAX && y == INT32_MAX)
        { continue; } // skip padding at the start of the graph array

        uint32_t gm_id = gm->to_padded_id(x, y);
        id_map_.insert(std::pair<uint32_t, uint32_t>(gm_id, graph_id));
    }
}

warthog::graph::corner_point_graph::~corner_point_graph()
{
    delete e_lab_index_;
    delete cpl_;
}

void
warthog::graph::corner_point_graph::construct()
{
    warthog::gridmap* gm = cpl_->get_map();
    warthog::gridmap* corner_map = cpl_->get_corner_map();
    uint32_t mapwidth = gm->header_width();
    uint32_t mapheight = gm->header_height();
    
    // construct the graph
    for(uint32_t index = 0; index < mapheight*mapwidth; index++)
    {
        // filter out source nodes that aren't corners
        uint32_t gm_id = gm->to_padded_id(index);
        if(!corner_map->get_label(gm_id)) { continue; } 

        // each corner point in the grid becomes a node in 
        // the corresponding planar graph
        uint32_t gm_x, gm_y;
        gm->to_unpadded_xy(gm_id, gm_x, gm_y);

        uint32_t from_id;
        std::unordered_map<uint32_t, uint32_t>::iterator it_from_id;
        it_from_id = id_map_.find(gm_id);
        if(it_from_id == id_map_.end())
        {
            int32_t x = gm_x * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
            int32_t y = gm_y * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
            from_id = this->add_node(x, y, index);
            id_map_.insert(std::pair<uint32_t, uint32_t>(gm_id, from_id));
        }
        else { from_id = it_from_id->second; }

        // identify corner points adjacent to the source by scanning the 
        // map. we add each such point to the graph (if it hasn't been
        // added already) and we add an edge between it and the source
        for(uint32_t i = 0; i < 8; i++)
        {
            warthog::jps::direction d = (warthog::jps::direction)(1 << i);
            std::vector<uint32_t> jpoints;
            std::vector<double> jcosts;
            cpl_->jump(d, gm_id, warthog::INF, jpoints, jcosts);
            for(uint32_t idx = 0; idx < jpoints.size(); idx++)
            {
                //uint8_t pdir = ((uint8_t*)&jpoints[idx])[3];
                uint32_t jp_id = jpoints[idx] & ((1 << 24) - 1);
                std::unordered_map<uint32_t, uint32_t>::iterator it_to_id; 
                it_to_id = id_map_.find(jp_id);

                uint32_t to_id;
                if(it_to_id == id_map_.end())
                {
                    uint32_t jp_x, jp_y;
                    gm->to_unpadded_xy(jp_id, jp_x, jp_y);

                    int32_t x, y;
                    x = jp_x * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
                    y = jp_y * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;

                    to_id = this->add_node(
                            x, y, jp_y * gm->header_width() + jp_x);
                    id_map_.insert(
                            std::pair<uint32_t, uint32_t>(jp_id, to_id));
                }
                else { to_id = it_to_id->second; }

                uint32_t label_id = __builtin_ffs(d)-1;
                uint32_t weight = jcosts[idx] * 
                    warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
                this->add_labeled_outgoing_edge(
                        from_id, warthog::graph::edge(to_id, weight),
                        label_id);
            }
        }
    }
}

// TODO: should this code live in an expansion policy??
// seems like the graph should be oblivious to things like 
// the target needing to be inserted before the start and 
// also dummy targets
void
warthog::graph::corner_point_graph::insert(
        uint32_t start_grid_id, uint32_t target_grid_id)
{
    // clear any previous insertion data
    uninsert();

    // NB: order is important here 
    // (target before start, lest we jump over the target)

    warthog::gridmap* gm = cpl_->get_map();
    uint32_t t_gm_id = gm->to_padded_id(target_grid_id);
    uint32_t s_gm_id = gm->to_padded_id(start_grid_id);

    id_map_iter myit = id_map_.find(t_gm_id);
    if(myit != id_map_.end())
    {
        // target is a node already in the graph; nothing to insert
        t_graph_id_ = myit->second;
        t_grid_id_ = t_gm_id;
    }
    else
    {
        // special case that's sometimes useful: unreachable dummy target
        if(target_grid_id == warthog::INF)
        {
            t_graph_id_ = T_DUMMY_ID;
            t_grid_id_ = target_grid_id;
            g_->set_xy(T_DUMMY_ID, 0, 0);
        }
        else
        {
            insert_target(s_gm_id, t_gm_id);
        }
    }
    
    myit = id_map_.find(s_gm_id);
    if(myit != id_map_.end())
    {
        // nothing to insert
        s_graph_id_ = myit->second;
        s_grid_id_ = s_gm_id;
    }
    else
    {
        insert_start(s_gm_id, t_gm_id);
    }
}

void
warthog::graph::corner_point_graph::insert_start(
        uint32_t start_grid_id, uint32_t target_grid_id)
{
    s_grid_id_ = start_grid_id;
    s_graph_id_ = S_DUMMY_ID;

    warthog::gridmap* gm = cpl_->get_map();
    uint32_t sx, sy;
    gm->to_unpadded_xy(s_grid_id_, sx, sy);
    sx *= warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
    sy *= warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
    g_->set_xy(s_graph_id_, (int32_t)sx, (int32_t)sy);

    id_map_.insert(std::pair<uint32_t, uint32_t>(s_grid_id_, s_graph_id_));

    // compute jump point successors for the start node
    std::vector<uint32_t> corner_neis;
    std::vector<double> costs;
    for(uint32_t i = 0; i < 8; i++)
    {
        warthog::jps::direction d = (warthog::jps::direction)(1 << i);
        cpl_->jump(d, s_grid_id_, target_grid_id, corner_neis, costs);
    }

    warthog::graph::node* s = g_->get_node(s_graph_id_);
    for(uint32_t i = 0; i < corner_neis.size(); i++)
    {
        warthog::jps::direction d = 
            (warthog::jps::direction)(corner_neis[i] >> 24);
        uint32_t corner_id = corner_neis[i] & warthog::jps::ID_MASK;

        uint32_t graph_id;
        if(corner_id == target_grid_id)
        {
            graph_id = t_graph_id_;
        }
        else
        {
           id_map_iter corner_iter = id_map_.find(corner_id);
           assert(corner_iter != id_map_.end());
           graph_id = corner_iter->second;
        }

        uint32_t weight = 
            costs[i] * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
        
        // we affix to each inserted edge a label that indicates the last 
        // direction we travel in when jumping from s to its successor
        // this label is stored in bits 8-15 (the second byte) 
        s->add_outgoing(warthog::graph::edge(graph_id, weight, (d << 8)));
    }
}

void
warthog::graph::corner_point_graph::insert_target(
        uint32_t start_grid_id, uint32_t target_grid_id)
{
    t_graph_id_ = T_DUMMY_ID;
    t_grid_id_ = target_grid_id;
    id_map_.insert(std::pair<uint32_t, uint32_t>(t_grid_id_, t_graph_id_));

    uint32_t tx, ty;
    warthog::gridmap* gm = cpl_->get_map();
    gm->to_unpadded_xy(target_grid_id, tx, ty);
    tx *= warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
    ty *= warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
    g_->set_xy(t_graph_id_, (int32_t)tx, (int32_t)ty);

    std::vector<uint32_t> corner_neis;
    std::vector<double> costs;
    for(uint32_t i = 0; i < 8; i++)
    {
        warthog::jps::direction d = (warthog::jps::direction)(1 << i);
        cpl_->jump(d, target_grid_id, warthog::INF, corner_neis, costs);
    }

    warthog::graph::node* target = g_->get_node(t_graph_id_);
    for(uint32_t i = 0; i < corner_neis.size(); i++)
    {
        uint32_t corner_id = corner_neis[i] & warthog::jps::ID_MASK;
        uint32_t graph_id = (*id_map_.find(corner_id)).second;
        uint32_t weight = costs[i] * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;

        // add the target to the outgoing edges list of every 
        // corner point we just found
        warthog::graph::node* nei = g_->get_node(graph_id);
        warthog::graph::edge_iter eit = nei->add_outgoing(
               warthog::graph::edge(t_graph_id_, weight));
        t_incoming_.push_back(graph_id);
        t_incoming_iters_.push_back(eit);

        // also add every corner point to the incoming successor list
        // of the node we are inserting 
        target->add_incoming(warthog::graph::edge(graph_id, weight));
    }
}

void
warthog::graph::corner_point_graph::uninsert()
{
    if(s_graph_id_ == S_DUMMY_ID)
    {
        id_map_.erase(id_map_.find(s_grid_id_));
        g_->get_node(s_graph_id_)->clear();
    }
    s_graph_id_ = warthog::INF;
    s_grid_id_ = warthog::INF;

    if(t_graph_id_ == T_DUMMY_ID)
    {
        // only when the target is not an unreachable dummy
        if(t_grid_id_ != warthog::INF)
        {
            id_map_.erase(id_map_.find(t_grid_id_));
            g_->get_node(t_graph_id_)->clear();
        }
    }
    t_graph_id_ = warthog::INF;
    t_grid_id_ = warthog::INF;

    for(uint32_t i = t_incoming_.size()-1; i < t_incoming_.size() ; i--)
    {
        warthog::graph::node* nei = g_->get_node(t_incoming_.at(i));
        nei->del_outgoing(t_incoming_iters_.at(i));
    }
    t_incoming_iters_.clear();
    t_incoming_.clear();
}

size_t
warthog::graph::corner_point_graph::mem()
{
    size_t mem_sz = g_->mem() + cpl_->mem() + sizeof(*this);
    if(e_lab_index_)
    {
        mem_sz += sizeof(label_index)*e_lab_index_->capacity();
    }
    return mem_sz;  
}

warthog::graph::edge_iter
warthog::graph::corner_point_graph::add_labeled_outgoing_edge(
        uint32_t node_id, warthog::graph::edge e, uint32_t label_id)
{
    assert(label_id < NUM_LABELS);
    int32_t head = e_lab_index_->at(node_id).head_[label_id];
    
    // insert the edge into the neighbours array at 
    // the position indicated by head
    warthog::graph::node* n = g_->get_node(node_id);
    warthog::graph::edge_iter ret = n->insert_outgoing(e, head);

    // increment subsequent indexes to reflect the insertion
    for(uint32_t i = label_id+1; i < NUM_LABELS; i++)
    {
        assert(e_lab_index_->at(node_id).head_[i] >= head);
        e_lab_index_->at(node_id).head_[i]++;
    }
    return ret;
}

void
warthog::graph::corner_point_graph::build_edge_label_index()
{
    for(uint32_t node_id = 0; node_id < get_num_nodes(); node_id++)
    {
        warthog::graph::node* n = get_node(node_id);

        // sort the edges by label before building the index
        bool (*edge_comparator) 
        (warthog::graph::edge, warthog::graph::edge) =
            [](warthog::graph::edge e1, warthog::graph::edge e2) -> bool
            {
                return ((uint8_t*)&e1.label_)[0] < ((uint8_t*)&e2.label_)[0];
            };
        std::sort(n->outgoing_begin(), n->outgoing_end(), edge_comparator);

        // build the index 
        //std::cerr << "\n node " << node_id;
        for(warthog::graph::edge_iter it = n->outgoing_begin();
                it != n->outgoing_end(); it++)
        {
            warthog::jps::direction dir = 
                (warthog::jps::direction)((uint8_t*)(&(it->label_)))[0];
            //std::cerr << " " << dir;
            uint32_t label_id = __builtin_ffs(dir)-1;
            assert(label_id < NUM_LABELS);

            // increment subsequent indexes 
#ifndef NDEBUG
            int32_t head = e_lab_index_->at(node_id).head_[label_id];
#endif
            for(uint32_t i = label_id+1; i < NUM_LABELS; i++)
            {
                assert(e_lab_index_->at(node_id).head_[i] >= head);
                e_lab_index_->at(node_id).head_[i]++;
            }
        }

        //std::cerr << "\n labels " << node_id;
        //for(uint32_t i = 0; i < NUM_LABELS; i++)
        //{
        //    std::cerr << " " << e_lab_index_->at(node_id).head_[i];
        //}
    }
}

