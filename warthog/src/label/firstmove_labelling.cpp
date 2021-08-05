#include "fch_expansion_policy.h"
#include "firstmove_labelling.h"
#include "xy_graph.h"

#include <algorithm>

warthog::label::firstmove_labelling::firstmove_labelling(
        warthog::graph::xy_graph* g)
    : g_(g)
{
    lab_ = new std::vector< std::vector < warthog::label::fm_run > > ();
    lab_->resize(g_->get_num_nodes());
}

warthog::label::firstmove_labelling::~firstmove_labelling()
{ 
    delete lab_;
}

std::istream&
warthog::label::operator>>(std::istream& in, fm_run& the_run)
{
    in.read((char*)(&the_run.head_), 4);
    in.read((char*)(&the_run.label_), 1);
    return in ;
}

std::ostream&
warthog::label::operator<<(std::ostream& out, fm_run& the_run)
{
    out.write((char*)(&the_run.head_), 4);
    out.write((char*)(&the_run.label_), 1);
    return out;
}

std::istream&
warthog::label::operator>>(std::istream& in, 
        warthog::label::firstmove_labelling& lab)
{
    lab.lab_->resize(lab.g_->get_num_nodes());

    uint32_t num_rows;
    in.read((char*)(&num_rows), 4);

    for(uint32_t row = 0; row < num_rows; row++)
    {
        uint32_t row_id; 
        in.read((char*)(&row_id), 4);

        if(row_id > lab.g_->get_num_nodes())
        {
            std::cerr << "err; " << row_id 
                << " is aninvalid row identifier. aborting.\n";
            break;
        }

        if(lab.lab_->at(row_id).size() != 0)
        {
            std::cerr << "err; row id "<< row_id 
                << " appears more than once. aborting.\n";
            break;
        }

        uint32_t num_runs;
        in.read((char*)(&num_runs), 4);

        // read all the runs for the current row
        for(uint32_t i = 0; i < num_runs; i++)
        {
            fm_run tmp;
            in >> tmp;
            lab.lab_->at(row_id).push_back(tmp);

            assert(in.good());

            if(!in.good())
            {
                std::cerr << "err; while reading firstmove labels\n";
                std::cerr 
                    << "[debug info] row# " << row
                    << " row_id " << row_id 
                    << " run# " << i << " of " << num_runs 
                    << ". aborting.\n";
                return in;
            }
        }
    }
    return in;
}

std::ostream&
warthog::label::operator<<(std::ostream& out,
        warthog::label::firstmove_labelling& lab)
{
    uint32_t num_rows = 0;
    for(uint32_t n_id = 0; n_id < lab.g_->get_num_nodes(); n_id++)
    {
        if(lab.lab_->size() > 0) { num_rows++; }
    }
    out.write((char*)(&num_rows), 4);

    //uint32_t row_count = 0;
    //uint32_t run_count = 0;
    for(uint32_t row_id = 0; row_id < lab.g_->get_num_nodes(); row_id++)
    {
        if(lab.lab_->at(row_id).size() == 0) { continue; }
        out.write((char*)(&row_id), 4);

        uint32_t num_runs = lab.lab_->at(row_id).size();
        out.write((char*)(&num_runs), 4);

        for(uint32_t run = 0; run < num_runs; run++)
        {
            out << lab.lab_->at(row_id).at(run);
//            run_count++;
            if(!out.good())
            {
                std::cerr << "err; while writing labels\n";
                std::cerr 
                    << "[debug info] "
                    << " row_id " << row_id 
                    << " run# " << run 
                    << ". aborting.\n";
                return out;
            }
        }
//        row_count++;
    }
//    std::cerr 
//        << "wrote to disk " << row_count 
//        << " rows and "
//        << run_count << " runs \n";
    return out;
}

void
warthog::label::compute_fm_dfs_preorder(
        warthog::graph::xy_graph& g, 
        std::vector<uint32_t>& column_order)
{
    std::cerr << "warthog::label::compute_fm_dfs_preorder\n";
    typedef std::pair<uint32_t, uint32_t> dfs_pair;

    // dfs bookkeeping (we store some data to avoid cycles)
    std::vector<uint32_t> tmp(g.get_num_nodes(), INT32_MAX); 
    std::function<void(uint32_t)> dfs_preorder =
        [&column_order, &tmp, &g](uint32_t dfs_seed) -> void
    {
        std::vector<dfs_pair> dfs_stack;
        dfs_stack.push_back(dfs_pair(dfs_seed, 0));
        dfs_stack.reserve(g.get_num_nodes()*2);
        while(dfs_stack.size())
        {
            // current node in the dfs tree 
            dfs_pair dfs_node = dfs_stack.back();
            dfs_stack.pop_back();
            warthog::graph::node* n = g.get_node(dfs_node.first);

            //std::cerr << dfs_node.first << " " << dfs_node.second;

            // pre-order traversal
            assert(dfs_node.first != INT32_MAX);
            if(tmp.at(dfs_node.first) == INT32_MAX)
            { 
                tmp.at(dfs_node.first) = dfs_node.first;
                column_order.push_back(dfs_node.first); 
            }

            warthog::graph::edge_iter begin = n->outgoing_begin();
            warthog::graph::edge_iter left = begin + dfs_node.second;

            // right branch 
            for(uint32_t i = dfs_node.second+1; i < n->out_degree(); i++)
            {
                assert((begin+i)->node_id_ != INT32_MAX);
                if(tmp.at((begin+i)->node_id_) == INT32_MAX)
                {
                    //std::cerr << " right " << (begin+i)->node_id_;
                    dfs_stack.push_back(dfs_pair(dfs_node.first, i));
                    break;
                }
            }

            // left branch 
            if(left == n->outgoing_end()) { return; }
            assert(left->node_id_ != INT32_MAX);
            if(tmp.at(left->node_id_) == INT32_MAX)
            {
                //std::cerr << " left " << left->node_id_;
                dfs_stack.push_back(dfs_pair(left->node_id_, 0)); 
            }

            //std::cerr << "\n";
        }
    };

    // pick a random seed and run DFS; continue until every node is labelled
    while(true)
    {
        std::vector<uint32_t> seeds;
        for(uint32_t i = 0; i < tmp.size(); i++)
        {
            if(tmp.at(i) == INT32_MAX) { seeds.push_back(i); } 
        }
        if(!seeds.size()) { break; }

        uint32_t dfs_seed = seeds.at(rand() % seeds.size());
        dfs_preorder(dfs_seed);
    }
}

void
warthog::label::compute_fm_fch_dfs_preorder(
        warthog::graph::xy_graph& g, 
        std::vector<uint32_t>& fch_order,
        std::vector<uint32_t>& column_order)
{
    std::cerr << " warthog::label::compute_fm_fch_dfs_preorder\n";

    typedef std::pair<uint32_t, uint32_t> dfs_pair;

    // dfs bookkeeping 
    std::vector<bool> allow_up(g.get_num_nodes(), false);
    std::vector<uint32_t> tmp(g.get_num_nodes(), INT32_MAX); 

    std::function<void(uint32_t)> dfs_preorder =
        [&allow_up, &fch_order, &column_order, &tmp, &g](uint32_t dfs_seed) 
        -> void
    {
        std::vector<dfs_pair> dfs_stack;
        dfs_stack.push_back(dfs_pair(dfs_seed, 0));
        dfs_stack.reserve(g.get_num_nodes()*2);
        allow_up.at(dfs_seed) = true;
        while(dfs_stack.size())
        {
            // current node in the dfs tree 
            dfs_pair dfs_node = dfs_stack.back();
            dfs_stack.pop_back();
            warthog::graph::node* n = g.get_node(dfs_node.first);

            //std::cerr << dfs_node.first << " " << dfs_node.second;

            // pre-order traversal
            assert(dfs_node.first != INT32_MAX);
            if(tmp.at(dfs_node.first) == INT32_MAX)
            { 
                tmp.at(dfs_node.first) = dfs_node.first;
                column_order.push_back(dfs_node.first); 
            }

            warthog::graph::edge_iter begin = n->outgoing_begin();
            warthog::graph::edge_iter left = begin + dfs_node.second;

            // right branch 
            for(uint32_t i = dfs_node.second+1; i < n->out_degree(); i++)
            {
                assert((begin+i)->node_id_ != INT32_MAX);
                if(tmp.at((begin+i)->node_id_) == INT32_MAX)
                {
                    if(fch_order.at(dfs_node.first) > 
                        fch_order.at((begin+i)->node_id_))
                    {
                        allow_up.at((begin+i)->node_id_) = false;
                        dfs_stack.push_back(dfs_pair(dfs_node.first, i));
                        break;
                    }
                    else if(allow_up.at(dfs_node.first))
                    {
                        if(fch_order.at(dfs_node.first) < 
                            fch_order.at((begin+i)->node_id_))
                        {
                            allow_up.at((begin+i)->node_id_) = true;
                            dfs_stack.push_back(dfs_pair(dfs_node.first, i));
                            break;
                        }
                    }
                }
            }

            // left branch 
            if(left == n->outgoing_end()) { return; }
            assert(left->node_id_ != INT32_MAX);
            if(tmp.at(left->node_id_) == INT32_MAX)
            {
                if(fch_order.at(dfs_node.first) > 
                    fch_order.at(left->node_id_)) // going down
                {
                    allow_up.at(left->node_id_) = false;
                    dfs_stack.push_back(dfs_pair(left->node_id_, 0));
                }
                else if(allow_up.at(dfs_node.first)) 
                {
                    if(fch_order.at(dfs_node.first) < 
                        fch_order.at(left->node_id_)) // going up
                    {
                        allow_up.at(left->node_id_) = true;
                        dfs_stack.push_back(dfs_pair(left->node_id_, 0));
                    }
                }
            }
            //std::cerr << "\n";
        }
    };

    // pick a random seed and run DFS; continue until every node is labelled
    while(true)
    {
        std::vector<uint32_t> seeds;
        for(uint32_t i = 0; i < tmp.size(); i++)
        {
            if(tmp.at(i) == INT32_MAX) { seeds.push_back(i); } 
        }
        if(!seeds.size()) { break; }

        uint32_t dfs_seed = seeds.at(rand() % seeds.size());
        dfs_preorder(dfs_seed);
    }

}

void
warthog::label::compute_fm_fch_dijkstra_dfs_preorder(
        warthog::graph::xy_graph& g, 
        std::vector<uint32_t>& fch_order,
        std::vector<uint32_t>& column_order) 
{
    std::cerr << "warthog::label::compute_fm_fch_dijkstra_dfs_preorder\n";
    warthog::zero_heuristic heur;
    warthog::fch_expansion_policy exp(&g, &fch_order);
    warthog::pqueue_min open;

    warthog::flexible_astar<
        warthog::zero_heuristic, 
        warthog::fch_expansion_policy,
        warthog::pqueue_min>
            dijk(&heur, &exp, &open);

    uint32_t source_id;
    std::vector<uint32_t> s_row(g.get_num_nodes(), INT32_MAX);

    std::function<
        void(warthog::search_node*, warthog::search_node*, double, uint32_t)>  
            on_generate_fn = [&source_id, &s_row]
                (warthog::search_node* succ, warthog::search_node* from,
                 double edge_cost, uint32_t edge_id) -> void
    {
        if(from == 0) { return; } // start node 

        if(from->get_id() == source_id) // start node successors
        { 
            s_row.at(succ->get_id()) = edge_id;
        }
        else // all other nodes
        {
            uint32_t succ_id = succ->get_id();
            uint32_t from_id = from->get_id();
            double alt_g = from->get_g() + edge_cost;
            double g_val = 
                succ->get_search_id() == from->get_search_id() ? 
                succ->get_g() : warthog::INF; 

            //  update first move
            if(alt_g < g_val) 
            { 
                s_row.at(succ_id) = s_row.at(from_id);
            }
        }
    };

    dijk.apply_on_generate(on_generate_fn);
    source_id = rand() % g.get_num_nodes();
    uint32_t ext_source_id = g.to_external_id(source_id);
    warthog::problem_instance problem(ext_source_id, warthog::INF);
    warthog::solution sol;
    dijk.get_path(problem, sol);

    // dfs stuff
    typedef std::pair<uint32_t, uint32_t> dfs_pair;

    // dfs bookkeeping (we store some data to avoid cycles)
    std::vector<uint32_t> tmp(g.get_num_nodes(), INT32_MAX); 
    std::function<void(uint32_t)> dfs_preorder =
        [&source_id, &s_row, &column_order, &tmp, &g](uint32_t fm_id) -> void
    {
        std::vector<dfs_pair> dfs_stack;
        dfs_stack.push_back(dfs_pair(source_id, fm_id));
        dfs_stack.reserve(g.get_num_nodes()*2);
        while(dfs_stack.size())
        {
            // current node in the dfs tree 
            dfs_pair dfs_node = dfs_stack.back();
            dfs_stack.pop_back();
            warthog::graph::node* n = g.get_node(dfs_node.first);

            //std::cerr << dfs_node.first << " " << dfs_node.second;

            // pre-order traversal
            if(s_row.at(dfs_node.first) != fm_id) { continue; }

            assert(dfs_node.first != INT32_MAX);
            if(tmp.at(dfs_node.first) == INT32_MAX)
            { 
                tmp.at(dfs_node.first) = dfs_node.first;
                column_order.push_back(dfs_node.first); 
            }


            warthog::graph::edge_iter begin = n->outgoing_begin();
            if((begin+dfs_node.second) == n->outgoing_end()) { return; }

            // right branch 
            for(uint32_t i = dfs_node.second+1; i < n->out_degree(); i++)
            {
                assert((begin+i)->node_id_ != UINT32_MAX);
                if(tmp.at((begin+i)->node_id_) == INT32_MAX)
                {
                    //std::cerr << " right " << (begin+i)->node_id_;
                    if(s_row.at((begin+i)->node_id_) == fm_id)
                    {
                        dfs_stack.push_back(dfs_pair(dfs_node.first, i));
                        break;
                    }
                }
            }

            // left branch 
            if(tmp.at((begin+dfs_node.second)->node_id_) == INT32_MAX)
            {
                dfs_stack.push_back(
                    dfs_pair((begin+dfs_node.second)->node_id_, 0));
            }
            //std::cerr << "\n";
        }
    };

    // find the apex of the hierarchy
    source_id = 0;
    for(uint32_t i = 0; i < fch_order.size(); i++)
    { 
        if(fch_order.at(i) > fch_order.at(source_id)) 
        { source_id = i; } 
    }
    warthog::graph::node* source = g.get_node(source_id);
    for(uint32_t i = 0; i < source->out_degree(); i++)
    {
        dfs_preorder(i);
    }

    // add all the remaining, unreachable nodes
    for(uint32_t i = 0; i < tmp.size(); i++)
    {
        if(tmp.at(i) == INT32_MAX) { column_order.push_back(i); }
    }
    assert(column_order.size() == g.get_num_nodes());
}
