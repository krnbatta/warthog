#include "dfs_labelling.h"
#include "xy_graph.h"

#include "search_node.h"
#include "zero_heuristic.h"
#include "flexible_astar.h"
#include "problem_instance.h"
#include "solution.h"
#include "graph_expansion_policy.h"

#include <algorithm>

std::istream&
warthog::label::operator>>(std::istream& in, 
        warthog::label::id_range_label& label)
{
    in.read((char*)(&label.left), sizeof(label.left));
    in.read((char*)(&label.right), sizeof(label.right));
    return in;
}

std::ostream& 
warthog::label::operator<<(std::ostream& out, 
        warthog::label::id_range_label& label)
{
    out.write((char*)(&label.left), sizeof(label.left));
    out.write((char*)(&label.right), sizeof(label.right));
    return out;
}

std::istream&
warthog::label::operator>>(std::istream& in, warthog::label::dfs_label& label)
{
    in >> label.rank_ >> label.ids_ >> label.bbox_;
    for(uint32_t i = 0; i < label.flags_.size(); i++)
    {   
        in.read( (char*)(&label.flags_.at(i)), 
                  sizeof(warthog::label::dfs_label::T_FLAG) );
    }
    return in;
}

std::ostream&
warthog::label::operator<<(std::ostream& out, 
        warthog::label::dfs_label& label)
{
    out << label.rank_ << label.ids_ << label.bbox_;
    for(uint32_t i = 0; i < label.flags_.size(); i++)
    { 
        out.write( (char*)(&label.flags_.at(i)), 
                    sizeof(warthog::label::dfs_label::T_FLAG) );
    }
    return out;
}

std::istream&
warthog::label::operator>>(std::istream& in, 
        warthog::label::dfs_labelling& lab)
{
    for(uint32_t n_id = 0; n_id < lab.g_->get_num_nodes(); n_id++)
    {
        warthog::graph::node* n = lab.g_->get_node(n_id);
        warthog::graph::edge_iter begin = n->outgoing_begin();
        warthog::graph::edge_iter end = n->outgoing_end();
        for(warthog::graph::edge_iter it = begin; it != end; it++)
        {
            in >> lab.lab_->at(n_id).at(it - begin);
            assert(in.good());

            if(!in.good())
            {
                std::cerr << "unexpected error while reading labels\n";
                std::cerr 
                    << "[debug info] node: " << n_id
                    << " out-edge-index: " << (it - begin) << "\n";
                return in;
            }
        }
    }
    return in;
}

std::ostream&
warthog::label::operator<<(std::ostream& out,
        warthog::label::dfs_labelling& lab)
{
    for(uint32_t n_id = 0; n_id < lab.g_->get_num_nodes(); n_id++)
    {
        warthog::graph::node* n = lab.g_->get_node(n_id);
        warthog::graph::edge_iter begin = n->outgoing_begin();
        warthog::graph::edge_iter end = n->outgoing_end();
        for(warthog::graph::edge_iter it = begin; it != end; it++)
        {
            out << lab.lab_->at(n_id).at(it - begin);
            if(!out.good())
            {
                std::cerr << "unexpected error while writing labels\n";
                std::cerr 
                    << "[debug info] node: " << n_id
                    << " out-edge-index: " << (it-begin) << "\n";
                return out;
            }
        }
    }
    return out;
}

warthog::label::dfs_labelling::dfs_labelling(
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* rank,
        std::vector<uint32_t>* partitioning)
    : g_(g), rank_(rank), part_(partitioning)
{
    dfs_order_ = new std::vector< uint32_t >();
    apex_id_ = compute_dfs_ids(g_, rank_, dfs_order_);
    
    // figure out how many bytes are required per label
    uint32_t max_id = *(std::max_element(part_->begin(), part_->end()));
    bytes_per_af_label_ = (max_id / 8) + !!(max_id % 8);

    // allocate memory for edge labels
    lab_ = new std::vector< std::vector < dfs_label > >();
    lab_->resize(g_->get_num_nodes());
    dfs_label dummy(bytes_per_af_label_);
    for(uint32_t n_id = 0; n_id < g_->get_num_nodes(); n_id++)
    {
        warthog::graph::node* n = this->g_->get_node(n_id);
        lab_->at(n_id).resize(n->out_degree(), dummy);
    }   
}

warthog::label::dfs_labelling::~dfs_labelling()
{ 
    delete lab_;
    delete dfs_order_;
}

void
warthog::label::dfs_labelling::compute_dfs_labels(
        warthog::util::workload_manager* workload)
{
    // Here we:
    // 1. perform a downwards dfs traversal of the CH from a given source node
    // 2. label the nodes in the subtree using a post-order visitation scheme
    // 3. store for every down edge a label that bounds the reachable subtree
    // 4. store for all nodes a label to bound its down closure
    std::vector< dfs_label > closure(
            this->g_->get_num_nodes(), dfs_label(bytes_per_af_label_));
    std::vector< uint8_t > recurse(this->g_->get_num_nodes(), 1);
    std::function<void(uint32_t)> label_fn = 
        [this, workload, &recurse, &closure, &label_fn] 
        (uint32_t source_id) -> void
        {
            dfs_label& s_lab = closure.at(source_id);
            warthog::graph::node* source = this->g_->get_node(source_id);
            warthog::graph::edge_iter begin = source->outgoing_begin();

            for( warthog::graph::edge_iter it = begin; 
                    it != source->outgoing_end();
                    it++)
            {
                // skip up edges
                if(this->rank_->at(it->node_id_) > this->rank_->at(source_id)) 
                { continue; }

                // DFS
                if(recurse.at(it->node_id_))
                { label_fn(it->node_id_); }

                // grow the label of the down edge at hand
                if(workload->get_flag(source_id))
                {
                    lab_->at(source_id).at(it - begin) = 
                        closure.at(it->node_id_);
                }
                s_lab.merge(lab_->at(source_id).at(it - begin));
            }
            recurse.at(source_id) = 0;

            s_lab.rank_.grow(this->rank_->at(source_id));
            s_lab.ids_.grow(this->dfs_order_->at(source_id));

            int32_t x, y;
            this->g_->get_xy(source_id, x, y);
            s_lab.bbox_.grow(x, y);

            uint32_t s_part = this->part_->at(source_id);
            s_lab.flags_[s_part >> 3] |= (1 << (s_part & 7)); // div8, mod8
            assert(s_lab.flags_[s_part >> 3] & (1 << (s_part & 7)));
        };

    // Here we:
    // 1. perform an upwards dfs traversal of the CH from a given source node
    // 2. store for every up edge a label that bounds the up-reachable subtree
    // 3. store for all nodes a label to bound its up-reachable closure
    std::vector< dfs_label > up_closure(
            this->g_->get_num_nodes(), dfs_label(bytes_per_af_label_));
    std::function<void(uint32_t)> up_label_fn = 
        [this, workload, &closure, &up_closure, &recurse, &up_label_fn] 
        (uint32_t source_id) -> void
        {
            warthog::graph::node* source = this->g_->get_node(source_id);
            warthog::graph::edge_iter begin = source->outgoing_begin();
            warthog::graph::edge_iter end = source->outgoing_end();
            dfs_label& s_lab = up_closure.at(source_id);

            // compute labels for each up edge by taking the union of:
            // 1. the up-closure of every up-edge
            // 2. the down-closure of every node in the up-closure
            for( warthog::graph::edge_iter it = begin; it != end; it++)
            {
                // skip down edges
                if(this->rank_->at(it->node_id_) < this->rank_->at(source_id)) 
                { continue; }

                // DFS
                if(recurse.at(it->node_id_))
                { up_label_fn(it->node_id_); }

                // grow the label for the up edge at hand
                if(workload->get_flag(source_id))
                {
                    dfs_label& e_lab = lab_->at(source_id).at(it-begin);
                    e_lab.merge(up_closure.at(it->node_id_)); 
                }

                // grow the up-reachable closure for the source node
                s_lab.merge(lab_->at(source_id).at(it-begin));
            }

            recurse.at(source_id) = 0;
            s_lab.merge(closure.at(source_id));
        };

    // Top-down DFS from the apex to compute down-edge labels
    label_fn(apex_id_);
    recurse.clear();
    recurse.assign(this->g_->get_num_nodes(), 1);

    // bottom-up DFS to compute up-edge labels
    for(uint32_t n_id = 0; n_id < this->g_->get_num_nodes(); n_id++)
    { 
        up_label_fn(n_id); 
    }
}

