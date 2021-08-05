#ifndef WARTHOG_LABEL_BBAF_LABELLING_H
#define WARTHOG_LABEL_BBAF_LABELLING_H

// label/bbaf_labelling.h
//
// Combined arcflags and bounding-box edge labelling.
//
// @author: dharabor
// @created: 2017-04-22
//

#include "flexible_astar.h"
#include "geom.h"
#include "helpers.h"
#include "xy_graph.h"
#include "search_node.h"
#include "workload_manager.h"
#include "zero_heuristic.h"

#include <functional>

namespace warthog
{

namespace label
{

class bbaf_label
{
    public:
        bbaf_label()
        {
            flags_ = 0;
        }

        // NB: shallow copy only
        bbaf_label(const bbaf_label& other)
        {
            flags_ = &*other.flags_;
            bbox_ = other.bbox_;
        }

        // NB: shallow copy only
        bbaf_label&
        operator=(const bbaf_label& other)
        {
            flags_ = &*other.flags_;
            bbox_ = other.bbox_;
            return *this;
        }


        uint8_t* flags_;
        warthog::geom::rectangle bbox_;
};

class bbaf_labelling
{
    public: 
        virtual ~bbaf_labelling();

        inline std::vector<uint32_t>*
        get_partitioning()
        {
            return part_;
        }

        inline std::vector<std::vector<bbaf_label>>*
        get_raw_labels()
        {
            return &labels_;
        }

        inline warthog::graph::xy_graph*
        get_graph() 
        { 
            return g_; 
        }

        inline bbaf_label&
        get_label(uint32_t node_id, uint32_t edge_id)
        {
            return labels_.at(node_id).at(edge_id);
        }

        inline void
        print_label(std::ostream& out, bbaf_label& label)
        {
            out << label.bbox_.x1 << " " << label.bbox_.y1 << " " 
                << label.bbox_.x2 << " " << label.bbox_.y2;
            for(uint32_t i = 0; i < bytes_per_af_label_; i++)
            {
                out << " " << (int)label.flags_[i];
            }
        }

        // @param out: (out) the target stream to write to
        // @param first_id: the first node in the range to print
        // @param last_id: the end of the range to print (not inclusive)
        void
        print(std::ostream& out, 
                uint32_t firstid=0, 
                uint32_t lastid=UINT32_MAX);

        static warthog::label::bbaf_labelling*
        load(const char* filename, warthog::graph::xy_graph* g, 
            std::vector<uint32_t>* partitioning);

        // load up a BBAF labelling for use with BCH
        // (bi-directional contraction hierarchies)
        //
        // there are two filters created in this case: one for the 
        // forward search and one for the backward search.
        // the forward labels contain all labels for outgoing up arcs and
        // every label is stored with the tail node of the correspinding arc
        // the backward labels contain all labels for outgoing down arcs
        // and every label is stored with the head node of the corresponding
        // arc.
        // 
        // @param filename: the file to load from
        // @param g: the graph associated with the labelling
        // @param partitioning: a disjoint partition of the nodes in @param g
        // @param lex_order: a lexical order of the nodes
        // @param out_lab_fwd: (output param) the fwd labels
        // @param out_lab_bwd: (output param) the bwd labels
        // @return: true if the function succeeds, false if it fails
        static bool
        load_bch_labels(
                const char* filename, warthog::graph::xy_graph* g,
                std::vector<uint32_t>* partitioning,
                std::vector<uint32_t>* lex_order, 
                warthog::label::bbaf_labelling*& out_lab_fwd,
                warthog::label::bbaf_labelling*& out_lab_bwd);

        
        // compute labels for all nodes specified by the given workload
        template <typename t_expander>
        static warthog::label::bbaf_labelling*
        compute(warthog::graph::xy_graph* g, std::vector<uint32_t>* part, 
                std::function<t_expander*(void)>& fn_new_expander,
                warthog::util::workload_manager* workload)
        {
            if(g == 0 || part  == 0) { return 0; } 
            std::cerr << "computing bbaf labels\n";

            // stuff that's shared between all worker threads
            struct bbaf_shared_data
            {
                std::function<t_expander*(void)> fn_new_expander_;
                warthog::label::bbaf_labelling* lab_;
                warthog::util::workload_manager* workload_;
            };

            void*(*thread_compute_fn)(void*) = [] (void* args_in) -> void*
            {
                warthog::helpers::thread_params* par = 
                    (warthog::helpers::thread_params*) args_in;
                bbaf_shared_data* shared = (bbaf_shared_data*)par->shared_;

                warthog::pqueue_min open;
                warthog::zero_heuristic heuristic;
                std::shared_ptr<t_expander> 
                    expander(shared->fn_new_expander_());

                warthog::flexible_astar<
                    warthog::zero_heuristic, 
                    t_expander,
                    warthog::pqueue_min>
                        dijkstra(&heuristic, expander.get(), &open);

                // need to keep track of the first edge on the way to the 
                // current node(the solution is a bit hacky as we break the 
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

                // run a dijkstra search from each node
                warthog::graph::xy_graph* g_ = shared->lab_->g_;
                for(uint32_t i = 0;
                        i < shared->workload_->num_flags_set(); i++)
                {
                    // skip nodes not in the workload
                    if(!shared->workload_->get_flag(i)) { continue; }

                    // source nodes are evenly divided among all threads;
                    // skip any source nodes not intended for current thread
                    if((i % par->max_threads_) != par->thread_id_) 
                    { continue; }

                    // process the source node (i.e. run a dijkstra search)
                    uint32_t source_id = i;
                    uint32_t ext_source_id = g_->to_external_id(source_id);
                    warthog::problem_instance pi(ext_source_id, warthog::INF);
                    warthog::solution sol;
                    dijkstra.get_path(pi, sol);

                    // now we analyse the closed list to compute arc flags
                    // but first, we need an easy way to convert between the 
                    // ids of nodes adjacent to the source and their 
                    // corresponding edge index
                    warthog::graph::node* source = g_->get_node(source_id);
                    std::unordered_map<uint32_t, uint32_t> idmap;
                    uint32_t edge_idx = 0;
                    for(warthog::graph::edge_iter it = 
                            source->outgoing_begin(); 
                            it != source->outgoing_end(); it++)
                    {
                        idmap.insert(
                            std::pair<uint32_t, uint32_t>
                                ((*it).node_id_, edge_idx));
                        edge_idx++;
                    }

                    // allocate memory for the labels of the source
                    for(uint32_t i = 0; i < source->out_degree(); i++)
                    {
                        bbaf_label label;
                        label.flags_= 
                            new uint8_t[shared->lab_->bytes_per_af_label_];
                        shared->lab_->labels_.at(source_id).push_back(label);
                    }


                    // now analyse the nodes on the closed list and label the 
                    // edges of the source node accordingly
                    std::function<void(warthog::search_node*)> fn_arcflags =
                    [shared, &source_id, &idmap]
                        (warthog::search_node* n) -> void
                    {
                        // skip edges from the source to itself
                        assert(n);
                        if(n->get_id() == source_id) { return; } 
                        assert(n->get_parent());

                        // label the edges of the source
                        // (TODO: make this stuff faster)
                        uint32_t part_id 
                            = shared->lab_->part_->at(n->get_id());
                        uint32_t e_idx  
                            = (*idmap.find(
                                n->get_parent() == source_id ? 
                                n->get_id() : 
                                n->get_parent())).second;
                        shared->lab_->labels_.at(source_id).at(
                                e_idx).flags_[part_id >> 3] |= 
                                    (1 << (part_id & 7));

                        int32_t x, y;
                        shared->lab_->g_->get_xy(n->get_id(), x, y);
                        assert(x != warthog::INF && y != warthog::INF);
                        shared->lab_->labels_.at(source_id).at(
                                e_idx).bbox_.grow(x, y);
                        assert(
                            shared->lab_->labels_.at(source_id).at(
                                e_idx).bbox_.is_valid());
                    };
                    dijkstra.apply_to_closed(fn_arcflags);
                    par->nprocessed_++;
                }
                return 0;
            };

            warthog::label::bbaf_labelling* lab = 
                new warthog::label::bbaf_labelling(g, part);
            bbaf_shared_data shared;
            shared.lab_ = lab;
            shared.fn_new_expander_ = fn_new_expander;
            shared.workload_ = workload;
            warthog::helpers::parallel_compute(thread_compute_fn, &shared, 
                    workload->num_flags_set());
            return lab;
        }

        inline size_t
        mem()
        {
            size_t retval = sizeof(this);
            for(uint32_t i = 0; i < labels_.size(); i++)
            {
                retval += 
                    sizeof(bbaf_label) *
                    (labels_.at(i).size() + bytes_per_af_label_);
            }
            return retval;
        }

    private:
        bbaf_labelling(
            warthog::graph::xy_graph* g, 
            std::vector<uint32_t>* partitioning);

        warthog::graph::xy_graph* g_;
        std::vector<uint32_t>* part_;

        std::vector<std::vector<bbaf_label>> labels_;
        uint32_t bytes_per_af_label_;
};

} // warthog::label::

} // warthog::

#endif
