#ifndef WARTHOG_GRAPH_CORNER_POINT_GRAPH_H
#define WARTHOG_GRAPH_CORNER_POINT_GRAPH_H

// domains/corner_point_graph.h
//
// A corner_point_graph is wrapper around a planar graph created from a grid map.
// Typically the nodes of the wrapped graph are grid locations having 
// two adjacent neighbours that are not co-visible. We refer to all such 
// points as corner points.
//
// The edges of the wrapped graph can be created in different ways. For 
// example, connecting all pairs of co-visible corner points yields a
// visibility graph. Other possibilities include jump point graphs
// and subgoal graph.
//
// Common to all of the above is an insertion operation which is used 
// to handle cases when the start or target location is not a corner point.
// 
// @author: dharabor
// @created: 2016-11-06
//

#include "corner_point_locator.h"
#include "gridmap.h"
#include "xy_graph.h"

#include <unordered_map>
#include <memory>

namespace warthog
{

namespace graph
{

class corner_point_graph
{
    // sometimes we want to label edges with grid-direction values
    // here we index the outgoing edges of each node in order to more 
    // easily access edges with a specific label (cf. iterating all edges)
    static const uint32_t NUM_LABELS = 8;
    struct label_index
    {
        label_index()
        {
            uint32_t num_labels = warthog::graph::corner_point_graph::NUM_LABELS;
            for(uint32_t i = 0; i < num_labels; i++)
                head_[i] = 0;
        }

        warthog::graph::ECAP_T head_[warthog::graph::corner_point_graph::NUM_LABELS];
    };

    public:
        corner_point_graph(std::shared_ptr<warthog::gridmap> gm );
        corner_point_graph(
                std::shared_ptr<warthog::gridmap> gm, 
                std::shared_ptr<warthog::graph::xy_graph> g);

        virtual ~corner_point_graph();

        inline void
        print_dimacs_gr(std::ostream& oss)
        {
            // omit dummy start and target nodes which were added
            g_->print_dimacs_gr(oss, 0, g_->get_num_nodes()-2);
        }

        inline void
        print_dimacs_co(std::ostream& oss)
        {
            // omit dummy start and target nodes which were added
            g_->print_dimacs_co(oss, 0, g_->get_num_nodes()-2);
        }

        // returns number of nodes + ID_OFFSET number of
        // dummy elements located at the start of the nodes array
        inline uint32_t
        get_num_nodes()
        {
            // omit dummy start and target nodes which were added
            return g_->get_num_nodes()-2;
        }

        inline uint32_t 
        get_num_edges()
        {
            return g_->get_num_edges_out();
        }

        // retrieve the xy coordinates of a node
        // @param graph_id: the id of the desired node
        // @return x: the x-coordinate of @param graph_id
        // @return y: the y-coordinate of @param graph_id
        inline void
        get_xy(uint32_t graph_id, int32_t& x, int32_t& y)
        { g_->get_xy(graph_id, x, y); }

        inline warthog::graph::node* 
        get_node(uint32_t id)
        {
            return g_->get_node(id);
        }

        inline uint32_t
        add_node(int32_t x, int32_t y, uint32_t ext_id = warthog::INF)
        {
            uint32_t graph_id = g_->add_node(x, y, ext_id);
            if(e_lab_index_)
            {
                if(graph_id >= e_lab_index_->size())
                {
                    e_lab_index_->resize(graph_id+1);
                }
            }
            return graph_id;
        }

        // edges can be assigned one of a fixed number of labels
        // (up to warthog::graph::corner_point_graph::NUM_LABELS)
        // this function adds edges and indexes them by label type.
        warthog::graph::edge_iter
        add_labeled_outgoing_edge(
                uint32_t node_id, 
                warthog::graph::edge e, 
                uint32_t label_id);

        // return the offset for the first neighbour of
        // node_id in direction d
        inline uint32_t
        labelled_edge_offset(uint32_t node_id, uint32_t label_id)
        {
            if(label_id >= NUM_LABELS)
            {
                return get_node(node_id)->out_degree();
            }
            return e_lab_index_->at(node_id).head_[label_id];
        }

        inline void set_verbose(bool verbose) 
        { 
            verbose_ = verbose; 
            g_->set_verbose(verbose);
        } 

        inline bool
        get_verbose() { return verbose_; }

        inline const char* 
        get_filename() { return g_->get_filename(); }

        inline warthog::gridmap* 
        get_gridmap() { return cpl_->get_map(); } 

        //inline bool
        //reserve(uint32_t new_cap) { return g_->reserve(new_cap); }

        virtual size_t
        mem();

        // insert the start and target locations into the graph.
        void
        insert( uint32_t start_grid_id, uint32_t target_grid_id);

        uint32_t 
        get_inserted_start_id() { return s_graph_id_; }

        uint32_t 
        get_inserted_target_id() { return t_graph_id_; }

        uint32_t 
        get_dummy_target_id() { return T_DUMMY_ID; }

        uint32_t 
        get_dummy_start_id() { return S_DUMMY_ID; }

        warthog::graph::xy_graph*
        get_xy_graph() { return g_.get(); }

        // post-hoc edge indexing
        // (essentially, index the edges of each node by their direction 
        // labels; allows faster identification of relevant edges during JPS)
        void
        build_edge_label_index();

    private:
        bool verbose_;
        std::shared_ptr<warthog::graph::xy_graph> g_;
        warthog::corner_point_locator* cpl_;
        
        // no copy ctor
        corner_point_graph(warthog::graph::corner_point_graph& other) 
            : S_DUMMY_ID(0), T_DUMMY_ID(0) { }

        // below the line: variables and data structures and functions for 
        // connecting the start and target nodes to the rest of the graph
        // ----------------------------------------------------------------
        uint32_t s_graph_id_, s_grid_id_;
        uint32_t t_graph_id_, t_grid_id_;
        uint32_t S_DUMMY_ID, T_DUMMY_ID; 

        std::vector<uint32_t> t_incoming_;
        std::vector<warthog::graph::edge_iter> t_incoming_iters_;

        // maps gridmap ids to graph ids
        std::unordered_map<uint32_t, uint32_t> id_map_;
        
        std::vector<label_index>* e_lab_index_;

        void
        construct();

        void
        insert_start(uint32_t start_grid_id, uint32_t target_grid_id);

        void
        insert_target(uint32_t start_grid_id, uint32_t target_grid_id);

        void
        uninsert();
};

}
}

#endif
