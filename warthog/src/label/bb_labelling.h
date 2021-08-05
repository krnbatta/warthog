#ifndef WARTHOG_LABEL_BB_LABELLING_H
#define WARTHOG_LABEL_BB_LABELLING_H

// label/bb_labelling.h
//
// For every outgoing edge of every node in a graph we store a rectangular
// bounding box. Inside the box can be found all nodes that are reached 
// optimally by a path whose first edge is the edge at hand.
//
// For theory see:
//
// [Wager & Willhalm, 2005, 
// Geometric Containers for Efficient Shortest Path Computation, 
// Journal of Experimental Algorithms, vol 10, pp 1-30]
//
// @author: dharabor
// @created: 2017-04-18
//

#include "flexible_astar.h"
#include "helpers.h"
#include "geom.h"
#include "xy_graph.h"
#include "search_node.h"
#include "solution.h"
#include "workload_manager.h"
#include "zero_heuristic.h"

#include <functional>

namespace warthog
{

namespace label
{

class bb_labelling
{
    public:
        virtual ~bb_labelling();

        // @param out: (out) the target stream to write to
        // @param first_id: the first node in the range to print
        // @param last_id: the end of the range to print (not inclusive)
        void
        print(std::ostream& out,
                uint32_t firstid=0, 
                uint32_t lastid=UINT32_MAX);

        inline std::vector<std::vector<warthog::geom::rectangle>>*
        get_raw_labels()
        {
            return labels_;
        }

        inline warthog::geom::rectangle&
        get_label(uint32_t node_id, uint32_t edge_id)
        {
            return labels_->at(node_id).at(edge_id);
        }

        static warthog::label::bb_labelling*
        load(const char* filename, warthog::graph::xy_graph* g);

        static bool
        load_bch_labels( const char* filename, 
                        warthog::graph::xy_graph* g,
                        std::vector<uint32_t>* lex_order, 
                        warthog::label::bb_labelling*& out_lab_fwd,
                        warthog::label::bb_labelling*& out_lab_bwd );
        
        template <typename t_expander>
        static warthog::label::bb_labelling*
        compute(warthog::graph::xy_graph* g, 
                std::function<t_expander*(void)>& fn_new_expander,
                warthog::util::workload_manager* workload)
        {
            if(g == 0) { return 0; } 
            std::cerr << "computing bb labels\n";

            struct bb_shared_data
            {
                std::function<t_expander*(void)> fn_new_expander_;
                warthog::label::bb_labelling* lab_;
                warthog::util::workload_manager* workload_;
            };

            // the actual precompute function
            void*(*thread_compute_fn)(void*) = [] (void* args_in) -> void*
            {
                warthog::helpers::thread_params* par = 
                    (warthog::helpers::thread_params*) args_in;
                bb_shared_data* shared = (bb_shared_data*)par->shared_;

                std::shared_ptr<t_expander> 
                    expander(shared->fn_new_expander_());
                warthog::zero_heuristic heuristic;
                warthog::pqueue_min open;

                warthog::flexible_astar<
                    warthog::zero_heuristic, 
                    t_expander, 
                    warthog::pqueue_min>
                         dijkstra(&heuristic, expander.get(), &open);

                // need to keep track of the first edge on the way to the 
                // current node (the solution is a bit hacky as we break the 
                // chain of backpointers to achieve this; it doesn't matter, 
                // we don't care about the path)
                std::function<void(warthog::search_node*)> relax_fn = 
                    [&expander] (warthog::search_node* n) -> void
                    {
                        // the start node and its children don't need their 
                        // parent pointers updated. for all other nodes the
                        // grandparent becomes the parent
                        uint32_t gp_id = 
                            expander->generate(n->get_parent())->get_parent();

                        if(gp_id != warthog::NODE_NONE)
                        {
                            if(expander->generate(gp_id)->get_parent()
                                    != warthog::NODE_NONE)
                            {
                                n->set_parent(gp_id);
                            }
                        }
                    };

                dijkstra.apply_on_relax(relax_fn);

                for(uint32_t i = 0; 
                        i < shared->workload_->num_flags_set(); i++)
                {
                    // skip nodes not in the workload
                    if(!shared->workload_->get_flag(i)) { continue; }

                    // source nodes are evenly divided among all threads;
                    // skip any source nodes not intended for current thread
                    if((i % par->max_threads_) != par->thread_id_) { continue; }

                    // process the source
                    uint32_t source_id = i;
                    uint32_t ext_source_id = 
                        shared->lab_->g_->to_external_id(source_id);
                    warthog::graph::node* source = 
                        shared->lab_->g_->get_node(source_id);

                    warthog::problem_instance pi(ext_source_id, warthog::INF);
                    warthog::solution sol;
                    dijkstra.get_path(pi, sol);
                    
                    // we need an easy way to convert between the ids of nodes
                    // adjacent to the source and the edge index to reach them
                    std::unordered_map<uint32_t, uint32_t> idmap;
                    uint32_t edge_idx = 0;
                    for(warthog::graph::edge_iter it = 
                            source->outgoing_begin(); 
                            it != source->outgoing_end(); it++)
                    {
                        idmap.insert( std::pair<uint32_t, uint32_t>(
                                    (*it).node_id_, edge_idx));
                        edge_idx++;
                    }

                    // allocate memory for each label of the source
                    shared->lab_->labels_->at(source_id).resize(
                            source->out_degree());

                    // compute the extent of each rectangle bounding box
                    std::function<void(warthog::search_node*)> bbox_fn = 
                        [shared, &source_id, &idmap, &expander]
                            (warthog::search_node* n) -> void
                    {
                        // skip the source node
                        // (it doesn't belong to any rectangle)
                        assert(n);
                        if(n->get_id() == source_id) { return; } 
                        assert(n->get_parent());

                        std::unordered_map<uint32_t, uint32_t>::iterator it;

                        // successors of the source are special
                        if(expander->generate(n->get_parent())->get_parent() 
                                == warthog::NODE_NONE)
                        { it = idmap.find(n->get_id()); }
                        else
                        // all the other nodes
                        { it = idmap.find(n->get_parent()); }

                        if(it == idmap.end())
                        {
                            // sanity check
                            std::cerr << "err; invalid first edge; "
                                 << " n id: "<< n->get_id() << " parent id: " 
                                 << n->get_parent() << std::endl;
                            exit(0);
                        }

                        // grow the rectangle
                        int32_t x, y;
                        shared->lab_->g_->get_xy(n->get_id(), x, y);
                        assert(x != warthog::INF && y != warthog::INF);
                        shared->lab_->labels_->at(source_id).at(
                                (*it).second).grow(x, y);
                        assert(shared->lab_->labels_->at(source_id).at(
                                    (*it).second).is_valid());
                    };
                    dijkstra.apply_to_closed(bbox_fn);
                    par->nprocessed_++;
                }
                return 0;
            };

            bb_shared_data shared;
            shared.lab_ = new warthog::label::bb_labelling(g);
            shared.fn_new_expander_ = fn_new_expander;
            shared.workload_ = workload;
            warthog::helpers::parallel_compute(
                    thread_compute_fn, &shared, workload->num_flags_set());
            return shared.lab_;
        }

    private:
        // creation via ::compute and ::load only, please
        bb_labelling(warthog::graph::xy_graph* g);
        warthog::graph::xy_graph* g_;
        std::vector<std::vector<warthog::geom::rectangle>>* labels_;

};

}

}

#endif

