#ifndef WARTHOG_DFS_LABELLING_H
#define WARTHOG_DFS_LABELLING_H

// label/dfs_labelling.h
//
// Collects a variety of different labels that we can compute for 
// the down-closure of a node. Intended for use with forward-driven 
// contraction hierarchies.
//
// Current edge labels:
//  - id range: specifying the range of ids in the down closure of each edge
//
//  - bounding box: a rectangular bounding box that contains every
//  node in the down closure of each edge
//  
//  - arc flags: a bitfield that indicates set-reachability info for every
//  node in the down closure of each edge
//
// @author: dharabor
// @created: 2017-12-06
// 

#include "bbaf_labelling.h"
#include "fch_expansion_policy.h"
#include "geom.h"
#include "solution.h"
#include "timer.h"
#include "forward.h"

#include <fstream>
#include <vector>
#include <cstdint>

namespace warthog
{

namespace label
{

struct id_range_label
{
    id_range_label() { left = INT32_MAX; right = INT32_MIN; } 

    id_range_label&
    operator=(const id_range_label& other)
    {
        left = other.left; right = other.right; 
        return *this;
    }

    void
    print(std::ostream& out)
    {
        out << "id-range " << left << " " << right;
    }

    void
    grow(int32_t val)
    {
        left = val < left  ? val : left;
        right = val > right ? val : right;
    }

    void
    grow(const id_range_label& other)
    {
        left = left < other.left ? left : other.left;
        right = right > other.right ? right : other.right;
    }

    inline bool
    contains(int32_t val)
    {
        return (right - val) + left >= left;
    }

    int32_t left, right;
};

std::istream&
operator>>(std::istream& in, warthog::label::id_range_label& label);

std::ostream& 
operator<<(std::ostream& out, warthog::label::id_range_label& label);


struct dfs_label
{
    typedef uint8_t T_FLAG;
    dfs_label(size_t arcflag_bytes)
    {
        for(uint32_t i = 0; i < arcflag_bytes; i++) { flags_.push_back(0); }
    }

    dfs_label&
    operator=(const dfs_label& other)
    {
        rank_ = other.rank_;
        ids_ = other.ids_;
        bbox_ = other.bbox_;
        for(uint32_t i = 0; i < other.flags_.size(); i++)
        { flags_[i] = other.flags_.at(i); }
        return *this;
    }

    void
    merge(const dfs_label& other)
    {
        rank_.grow(other.rank_);
        ids_.grow(other.ids_);
        bbox_.grow(other.bbox_);
        for(uint32_t i = 0; i < flags_.size(); i++)
        { flags_[i] |= other.flags_[i]; }
    }

    void
    print(std::ostream& out)
    {
        out << " dfs_label";
        rank_.print(out);
        ids_.print(out);
        bbox_.print(out);
        out << " arcflags ";
        for(uint32_t i = 0; i < flags_.size(); i++)
        {
            out << " " << flags_.at(i);
        }
    }

    id_range_label rank_;
    id_range_label ids_;
    std::vector<T_FLAG> flags_;
    warthog::geom::rectangle bbox_;
};

std::istream&
operator>>(std::istream& in, warthog::label::dfs_label& label);

std::ostream&
operator<<(std::ostream& out, warthog::label::dfs_label& label);

class dfs_labelling 
{
    friend std::ostream&
    operator<<(std::ostream& out, dfs_labelling& lab);

    friend std::istream&
    operator>>(std::istream& in, warthog::label::dfs_labelling& lab);

    public:

        ~dfs_labelling();

        inline std::vector<uint32_t>*
        get_partitioning()
        {
            return part_;
        }

        inline warthog::graph::xy_graph*
        get_graph() 
        { 
            return g_;
        }

        dfs_label&
        get_label(uint32_t node_id, uint32_t edge_idx)
        {
            assert(edge_idx < lab_->at(node_id).size());
            return lab_->at(node_id).at(edge_idx);
        }

        inline int32_t
        get_dfs_index(uint32_t graph_id) { return dfs_order_->at(graph_id); }

        inline size_t
        mem()
        {
            size_t retval = sizeof(this);
            for(uint32_t i = 0; i < lab_->size(); i++)
            {
                retval += (sizeof(dfs_label) + bytes_per_af_label_) 
                    * lab_->at(i).size();
            }
            retval += sizeof(int32_t) * dfs_order_->size();
            return retval;
        }

        static warthog::label::dfs_labelling*
        load(const char* filename, warthog::graph::xy_graph* g, 
            std::vector<uint32_t>* rank, std::vector<uint32_t>* part)
        {
            std::cerr << "loading dfs_labelling from file " 
                << filename << std::endl;
            std::ifstream ifs(filename, 
                    std::ios_base::in|std::ios_base::binary);

            if(!ifs.good())
            {
                std::cerr << "load failed (no such file?)" << std::endl;
                ifs.close();
                return 0;
            }

            warthog::label::dfs_labelling* lab = 
                new warthog::label::dfs_labelling(g, rank, part);

            ifs >> *lab;

            if(!ifs.good())
            {
                std::cerr << "load failed" << std::endl;
                delete lab;
                lab = 0;
            }

            ifs.close();
            return lab;
        }
        
        static void
        save(const char* filename, warthog::label::dfs_labelling& lab)
        {
            std::cerr << "writing labels to file " << filename << "\n";
            std::ofstream out(filename, 
                    std::ios_base::out|std::ios_base::binary);

            out << lab;

            if(!out.good())
            {
                std::cerr << "\nerror trying to write to file " 
                    << filename << std::endl;
            }
            out.close();
        }

        // compute labels for all nodes specified by the given workload
        static warthog::label::dfs_labelling*
        compute(warthog::graph::xy_graph* g, 
                std::vector<uint32_t>* part, 
                std::vector<uint32_t>* rank,
                warthog::util::workload_manager* workload)
        {
            warthog::timer t;
            t.start();

            if(g == 0 || rank == 0 || part  == 0) { return 0; } 

            std::function<fch_expansion_policy*(void)> expander_fn = 
            [g, rank] (void) -> fch_expansion_policy*
            { return new warthog::fch_expansion_policy(g, rank); };

            struct shared_data
            {
                std::function<fch_expansion_policy*(void)> fn_new_expander_;
                warthog::label::dfs_labelling* lab_;
                warthog::util::workload_manager* workload_;
                std::vector<uint32_t>* rank_;
            };

            // The actual precompute function. We construct a 
            // Dijkstra-based preprocessing to improve the labels for 
            // selected sets of nodes
            void*(*thread_compute_fn)(void*) = 
            [] (void* args_in) -> void*
            {
                warthog::helpers::thread_params* par = 
                    (warthog::helpers::thread_params*) args_in;
                shared_data* shared = (shared_data*) par->shared_;

                warthog::label::dfs_labelling* lab = shared->lab_;
                warthog::util::workload_manager* workload = shared->workload_;

                // variable used to track the node currently being processed
                uint32_t source_id;
                
                // alocate memory for the first moves
                std::vector<uint32_t> first_move(lab->g_->get_num_nodes());

                // callback function used to record the optimal first move 
                std::function<void(
                        warthog::search_node*, 
                        warthog::search_node*,
                        double, uint32_t)> on_generate_fn = 
                [ & ] (warthog::search_node* succ, warthog::search_node* from,
                            double edge_cost, uint32_t edge_id) -> void
                {
                    if(from == 0) { return; } // start node 

                    if(from->get_id() == source_id) // start node successors
                    { 
                        assert(edge_id < 
                        lab->g_->get_node(source_id)->out_degree());
                        first_move.at(succ->get_id()) = edge_id; 
                    }
                    else // all other nodes
                    {
                        uint32_t s_id = succ->get_id();
                        uint32_t f_id = from->get_id();
                        double alt_g = from->get_g() + edge_cost;
                        double g_val = 
                            succ->get_search_id() == from->get_search_id() ? 
                            succ->get_g() : warthog::INF; 

                        assert(first_move.at(f_id) < 
                        lab->g_->get_node(source_id)->out_degree());

                        //  update first move
                        if(alt_g < g_val) 
                        { first_move.at(s_id) = first_move.at(f_id); }
                    }
                };


                std::function<void(warthog::search_node*)> on_expand_fn =
                [&source_id, &first_move, lab]
                (warthog::search_node* current) -> void
                {
                    if(current->get_id() == source_id) { return; }

                    uint32_t node_id = current->get_id();
                    assert(node_id < first_move.size());

                    uint32_t edge_idx = first_move.at(node_id);
                    assert(edge_idx < lab->lab_->at(source_id).size());
                    dfs_label& s_lab = 
                        lab->lab_->at(source_id).at(edge_idx);

                    s_lab.rank_.grow(lab->rank_->at(node_id));
                    s_lab.ids_.grow(lab->dfs_order_->at(node_id));

                    int32_t x, y;
                    lab->g_->get_xy(node_id, x, y);
                    s_lab.bbox_.grow(x, y);

                    uint32_t s_part = lab->part_->at(node_id);
                    s_lab.flags_[s_part >> 3] |= (1 << (s_part & 7)); // div8, mod8
                    assert(s_lab.flags_[s_part >> 3] & (1 << (s_part & 7)));
                };

                warthog::zero_heuristic h;
                std::shared_ptr<warthog::fch_expansion_policy> 
                    expander(shared->fn_new_expander_());
                warthog::pqueue_min open;
                warthog::flexible_astar 
                    <warthog::zero_heuristic, 
                    warthog::fch_expansion_policy,
                    warthog::pqueue_min>
                        dijk(&h, expander.get(), &open);
                dijk.apply_on_generate(on_generate_fn);
                dijk.apply_on_expand(on_expand_fn);

                for(uint32_t i = 0; i < lab->g_->get_num_nodes(); i++)
                {
                    // skip any nodes not part of the precomputation workload
                    if(!workload->get_flag(i))
                    { continue; }

                    // source nodes are evenly divided among all threads;
                    // skip any source nodes not intended for current thread
                    if((i % par->max_threads_) != par->thread_id_) 
                    { continue; }

                    source_id = i;
                    uint32_t ext_source_id = 
                        lab->g_->to_external_id(source_id);
                    warthog::problem_instance problem(ext_source_id, 
                            warthog::INF);
                    //problem.verbose_ = true;
                    warthog::solution sol;
                    dijk.get_path(problem, sol);
                    par->nprocessed_++;
                }
                return 0;
            };

            warthog::label::dfs_labelling* lab = 
                new warthog::label::dfs_labelling(g, rank, part);

            shared_data shared;
            shared.fn_new_expander_ = expander_fn;
            shared.lab_ = lab;
            shared.workload_ = workload;
            shared.lab_ = lab;

            std::cerr << "computing dijkstra labels\n";
            warthog::helpers::parallel_compute(
                    thread_compute_fn, &shared, 
                    workload->num_flags_set());

            std::cerr << "computing dfs labels...\n";
            workload->set_all_flags_complement();
            lab->compute_dfs_labels(workload); // single threaded
            t.stop();

            std::cerr 
                << "total preproc time (seconds): "
                << t.elapsed_time_micro() / 1000000 << "\n";

            return lab;
        }

        // Computes a DFS post-order id for every node in a contraction
        // hierarchy (i.e. a top-down traversal)
        // @param id of the highest node in the contraction hierarchy
        static uint32_t
        compute_dfs_ids(
                warthog::graph::xy_graph* g, 
                std::vector<uint32_t>* rank, 
                std::vector<uint32_t>* dfs_ids)
        {
            // find the apex of the hierarchy
            uint32_t apex_id = 0;
            for(uint32_t i = 0; i < rank->size(); i++)
            { 
                if(rank->at(i) > rank->at(apex_id)) 
                { apex_id = i; } 
            }

            uint32_t next_id = 0;
            dfs_ids->resize(g->get_num_nodes(), INT32_MAX);
            std::function<void(uint32_t)> dfs_id_fn =
            [dfs_ids, rank, &next_id, &dfs_id_fn, g] (uint32_t source_id) 
            -> void
            {
                warthog::graph::node* source = g->get_node(source_id);
                warthog::graph::edge_iter begin = source->outgoing_begin();
                for( warthog::graph::edge_iter it = begin; 
                        it != source->outgoing_end();
                        it++)
                {
                    // skip up edges
                    if(rank->at(it->node_id_) > rank->at(source_id)) 
                    { continue; }

                    // recurse
                    if(dfs_ids->at(it->node_id_) == INT32_MAX)
                    { dfs_id_fn(it->node_id_); }
                }
                if(dfs_ids->at(source_id) == INT32_MAX)
                { dfs_ids->at(source_id) = next_id++; }
            };

            // gogogo
            dfs_id_fn(apex_id);
            return apex_id;
        }

    private:
        // only via ::compute or ::load please
        dfs_labelling(
            warthog::graph::xy_graph* g, 
            std::vector<uint32_t>* rank,
            std::vector<uint32_t>* partitioning);

        // DFS-based preprocessing computes labels for every edge 
        // @param contraction order of every node in the graph
        void 
        compute_dfs_labels(warthog::util::workload_manager* workload);

        warthog::graph::xy_graph* g_;
        std::vector<uint32_t>* rank_;
        std::vector<uint32_t>* part_;
        size_t bytes_per_af_label_;
        uint32_t apex_id_; 

        std::vector<uint32_t>* dfs_order_;
        std::vector< std::vector< dfs_label >>* lab_;
};

std::istream&
operator>>(std::istream& in, warthog::label::dfs_labelling& lab);

std::ostream&
operator<<(std::ostream& in, warthog::label::dfs_labelling& lab);


}


}

#endif

