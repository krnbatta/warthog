#include "contraction.h"
#include "fch_expansion_policy.h"
#include "flexible_astar.h"
#include "xy_graph.h"
#include "problem_instance.h"
#include "search_node.h"
#include "solution.h"
#include "zero_heuristic.h"
#include <algorithm>
#include <stack>
#include <vector>

void
warthog::ch::make_input_order(warthog::graph::xy_graph& g, std::vector<uint32_t>& order)
{
    order.clear();
    order.reserve(g.get_num_nodes());
    for(uint32_t i=0; i < g.get_num_nodes(); i++)
    {
        order.push_back(i);
    }
}

void 
warthog::ch::write_node_order(const char* filename, std::vector<uint32_t>& order)
{
    std::ofstream ofs(filename, std::ios_base::out | std::ios_base::trunc);
    if(ofs.good())
    {
        ofs << "# node ids, in order of contraction\n";
        for(uint32_t i = 0; i < order.size(); i++)
        {
            ofs << order.at(i) << "\n";
        }
    }
    else
    {
        std::cerr << "err; cannot write to file " << filename << std::endl;
    }
    ofs.close();
}

bool
warthog::ch::load_node_order(const char* filename, 
        std::vector<uint32_t>& order, bool lex_order)
{
    order.clear();
    std::ifstream ifs(filename, std::ios_base::in);
    if(!ifs.good())
    {
        std::cerr << "\nerror trying to load node order from file " 
            << filename << std::endl;
        ifs.close();
        return false;
    }

    while(true)
    {
        // skip comment lines
        while(ifs.peek() == '#')
        {
            while(ifs.get() != '\n');
        }

        uint32_t tmp;
        ifs >> tmp;
        if(!ifs.good()) { break; }
        order.push_back(tmp);
    }
    ifs.close();

    if(lex_order)
    {
        warthog::ch::value_index_swap_dimacs(order);
    }
    return true;
}

typedef std::set<uint32_t>::iterator set_iter;

void
warthog::ch::compute_closure(uint32_t source, 
        warthog::graph::xy_graph* g, std::set<uint32_t>* closure, 
        uint32_t maxdepth)
{
    std::stack<std::pair<uint32_t, uint32_t>> stack; // stack of node ids
    stack.push(std::pair<uint32_t, uint32_t>(source, 0));
    while(stack.size() != 0)
    {
        // add top element to the closure
        std::pair<uint32_t, uint32_t> top = stack.top();
        uint32_t next_id = top.first;
        uint32_t depth = top.second;
        std::pair<std::set<uint32_t>::iterator, bool> ret 
            = closure->insert(next_id);
        stack.pop();
        if(ret.second == false) 
        { 
            continue;  // already in the closure
        }

        if(depth >= maxdepth)
        { 
            continue; // limited depth closure
        }

        // add all outgoing neighbours to the stack
        warthog::graph::node* n = g->get_node(next_id);
        for( warthog::graph::edge_iter it = n->outgoing_begin(); 
                it != n->outgoing_end(); 
                it++)
        {
            stack.push(std::pair<uint32_t, uint32_t>((*it).node_id_, depth+1));
        }
    }
}

void
warthog::ch::compute_down_closure(uint32_t source, 
        warthog::graph::xy_graph* g, std::vector<uint32_t>* rank,
        std::set<uint32_t>* closure)
{
    closure->insert(source);

    std::stack<uint32_t> stack; 
    stack.push(source);
    while(stack.size() != 0)
    {
        uint32_t top_id = stack.top();
        stack.pop();

        warthog::graph::node* top = g->get_node(top_id);
        for( warthog::graph::edge_iter it = top->outgoing_begin(); 
                it != top->outgoing_end(); 
                it++)
        {
            uint32_t next_id = it->node_id_; 
            if(rank->at(next_id) < rank->at(top_id)) 
            { 
                if(closure->find(next_id) == closure->end())
                {
                    stack.push(next_id); 
                    closure->insert(next_id);
                }
            }
        }
    }
}

void
warthog::ch::value_index_swap_dimacs(
        std::vector<uint32_t>& vec)
{
    // re-maps @param vec s.t. for each x and i
    // v[i] = x becomes v[x] = i
    std::vector<uint32_t> tmp;
    tmp.resize(vec.size());
    for(uint32_t i = 0; i < vec.size(); i++)
    {
        tmp.at(i) = vec.at(i);
    }

    assert( (*std::min_element(tmp.begin(), tmp.end())) == 0);
    assert( (*std::max_element(tmp.begin(), tmp.end())) == vec.size()-1);
    for(uint32_t i = 0; i < tmp.size(); i++)
    {
        vec.at(tmp.at(i)) = i;
    }
}

void
warthog::ch::partition_greedy_bottom_up(
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* rank,
        uint32_t nparts,
        std::vector<uint32_t>* part)
{

}

void
warthog::ch::unpack(uint32_t from_id,
        warthog::graph::edge_iter it_e,
        warthog::graph::xy_graph* g,
        std::set<uint32_t>& intermediate)
{
    warthog::graph::node* from = g->get_node(from_id);
    assert(it_e >= from->outgoing_begin() && it_e < from->outgoing_end());

    warthog::graph::edge* e_ft = &*it_e;
    uint32_t to_id = e_ft->node_id_;

    for(warthog::graph::edge_iter it = from->outgoing_begin(); 
            it < from->outgoing_end(); it++)
    {
        warthog::graph::node* succ = g->get_node((*it).node_id_);
        warthog::graph::edge_iter it_e_succ = succ->outgoing_begin();
        while(true)
        {
            it_e_succ = succ->find_edge(to_id, it_e_succ);
            if(it_e_succ == succ->outgoing_end()) { break; }
            assert( it_e_succ >= succ->outgoing_begin() && 
                    it_e_succ <= succ->outgoing_end());

            warthog::graph::edge* e_st = &*it_e_succ;
            if(((*it).wt_ + e_st->wt_) == e_ft->wt_) { break; }
            it_e_succ++;
        }
        if(it_e_succ == succ->outgoing_end()) { continue; } 

        intermediate.insert(from_id);
        intermediate.insert((*it).node_id_);

        // recursively unpack the two edges being represented by
        // the single shortcut edge (from_id, to_id)
        unpack(from_id, it, g, intermediate);

        succ = g->get_node((*it).node_id_);
        unpack((*it).node_id_, it_e_succ, g, intermediate);
        break;
    }
}

void
warthog::ch::unpack_and_list_edges(warthog::graph::edge* scut,
        uint32_t scut_tail_id, warthog::graph::xy_graph* g,
        std::vector<warthog::graph::edge*>& unpacked, bool recurse)
{
    warthog::graph::node* from = g->get_node(scut_tail_id);

    // we want to unpack each shortcut edge and find the 
    // two underlying edges that the shortcut is bypassing
    for(warthog::graph::edge_iter it_e1 = from->outgoing_begin(); 
            it_e1 < from->outgoing_end(); it_e1++)
    {
        warthog::graph::node* nei = g->get_node(it_e1->node_id_);
        for(warthog::graph::edge_iter it_e2 = nei->outgoing_begin();
            it_e2 != nei->outgoing_end(); it_e2++)
        {
            if( (it_e2->node_id_ == scut->node_id_) &&
                ((it_e1->wt_ + it_e2->wt_) == scut->wt_) )
            { 
                unpacked.push_back(&*it_e1);
                unpacked.push_back(&*it_e2);
                if(recurse)
                {
                    unpack_and_list_edges(it_e1, scut_tail_id, g, unpacked);
                    unpack_and_list_edges(it_e2,it_e1->node_id_,g,unpacked);
                }
                break; 
            }
        }
    }
}

void
warthog::ch::optimise_graph_for_bch(warthog::graph::xy_graph* g,
        std::vector<uint32_t>* rank)
{
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);
        uint32_t n_rank = rank->at(i);

//        std::cerr << "before: node "<< i << "; in_deg="<<n->in_degree()
//                  << "; out_deg="<<n->out_degree() << "\n";

        // delete all incoming edges except reverse down edges
        for(uint32_t j = 0; j < n->in_degree(); j++)
        {
            warthog::graph::edge_iter e_iter = (n->incoming_begin()+j);
            uint32_t in_rank = rank->at((*e_iter).node_id_);
            if(in_rank < n_rank)
            {
                n->del_incoming(e_iter);
                j--;
            }
        }

        // delete all outgoing edges except those going up
        for(uint32_t j = 0; j < n->out_degree(); j++)
        {
            warthog::graph::edge_iter e_iter = (n->outgoing_begin()+j);
            uint32_t out_rank = rank->at((*e_iter).node_id_);
            if(out_rank < n_rank)
            {
                n->del_outgoing(e_iter);
                j--;
            }
        }
//        std::cerr << "after: node "<< i << "; in_deg="<<n->in_degree()
//                  << "; out_deg="<<n->out_degree() << "\n";
    }
}

void
warthog::ch::optimise_graph_for_bch_v2(
        warthog::graph::xy_graph* g, std::vector<uint32_t>* rank)
{
    //fch_sort_successors(g, rank);

    // create an new graph with all the same nodes but no edges
    warthog::graph::xy_graph bch_g;
    bch_g.capacity(g->get_num_nodes());
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        int32_t node_x, node_y;
        g->get_xy(i, node_x, node_y);
        bch_g.add_node(node_x, node_y, g->to_external_id(i));
    }

    // add edges, optimising the layout for BCH
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);
        uint32_t n_rank = rank->at(i);

        // delete all outgoing edges except those going up
        for(uint32_t j = 0; j < n->out_degree(); j++)
        {
            warthog::graph::edge_iter e_iter = (n->outgoing_begin()+j);
            uint32_t out_rank = rank->at((*e_iter).node_id_);

            // outgoing-down-eddges become incoming-up-edges and get
            // stored with the head node
            if(out_rank < n_rank)
            {
                warthog::graph::edge rev_up_e = *e_iter;
                uint32_t head_id = (*e_iter).node_id_;
                rev_up_e.node_id_ = i; 
                bch_g.get_node(head_id)->add_incoming(rev_up_e);
            }
            // outgoing up 
            else
            {
                bch_g.get_node(i)->add_outgoing(*e_iter);
            }
        }
    }
    *g = std::move(bch_g);
}

warthog::graph::xy_graph* 
warthog::ch::load_contraction_hierarchy_and_optimise_for_fch( 
        const char* gr_file, 
        const char* co_file, 
        std::vector<uint32_t>* rank, 
        bool reverse_arcs, 
        bool store_incoming_edges, 
        bool enforce_euclidean)
{
    warthog::graph::xy_graph* g = new warthog::graph::xy_graph();
    if(!g->load_from_dimacs(gr_file, co_file, reverse_arcs, 
                store_incoming_edges, enforce_euclidean))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        delete g;
        return 0;
    }

    // sort the edges of the graph by rank and in descending order
    warthog::ch::fch_sort_successors(g, rank);
    return g;
}


void
warthog::ch::fch_sort_successors(
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* rank)
{
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);
        std::sort(n->outgoing_begin(), n->outgoing_end(), 
                [rank] 
                (warthog::graph::edge& first,
                 warthog::graph::edge& second) -> bool
                {
                    uint32_t f_rank = rank->at(first.node_id_);
                    uint32_t s_rank = rank->at(second.node_id_);
                    return f_rank > s_rank;
                } );
    }
}

// pruning rule: up-then-down < up
// related rules that don't work:
// down-up < down: fails because the two down frontiers may not ever meet
// up-down < down: seldom works; up usually goes far away and down is local
void
warthog::ch::sod_pruning(
        warthog::graph::xy_graph* g, std::vector<uint32_t>* rank)
{   
    //std::vector<double> cost(g->get_num_nodes(), DBL_MAX);
    //std::vector<double> from(g->get_num_nodes(), g->get_num_nodes());

    //uint32_t source = 0;
    //std::function<void(uint32_t, uint32_t, double, uint32_t)> dfs_fn =
    //    [g, rank, &cost, &from, &dfs_fn, source] 
    //    (uint32_t current, uint32_t up_hops, double sum, uint32_t min_lvl) 
    //        -> void
    //    {
    //        warthog::graph::node* n = g->get_node(current);
    //        warthog::graph::edge_iter it = n->outgoing_begin();
    //        uint32_t n_rank = rank->at(current);
    //        for( ; it != n->outgoing_end(); it++)
    //        {
    //            double dfs_cost = sum + it->wt_;
    //            if(from.at(it->node_id_) != source)
    //            { 
    //                from.at(it->node_id_) = source;
    //                cost.at(it->node_id_) = dfs_cost;
    //            }
    //            else if((dfs_cost + 0.0000001) < cost.at(it->node_id_))
    //            { 
    //                cost.at(it->node_id_) = dfs_cost; 
    //            }
    //            else continue;

    //            uint32_t nei_rank = rank->at(it->node_id_);
    //            if(nei_rank > n_rank)
    //            {
    //                if(up_hops) 
    //                { 
    //                    dfs_fn(it->node_id_, up_hops-1, dfs_cost, min_lvl);
    //                }
    //            }
    //            else
    //            {
    //                if(nei_rank >= min_lvl)
    //                {
    //                    dfs_fn(it->node_id_, 0, dfs_cost, min_lvl);
    //                }
    //            }
    //        }
    //    };
    //    
    //uint32_t up_hops = 2;
    uint32_t stall_rm = 0;
    for(uint32_t source = 0; source < g->get_num_nodes(); source++)
    {
            warthog::graph::node* s = g->get_node(source);
            warthog::graph::edge_iter it1 = s->outgoing_begin();
            warthog::graph::edge_iter it2 = it1+1;
            std::set<warthog::graph::edge_iter> stalled;
            for( ; it1 != s->outgoing_end(); it1++)
            {
                if(rank->at(it1->node_id_) < rank->at(source)) {continue;}

                for( ; it2 != s->outgoing_end(); it2++)
                {
                    if(rank->at(it2->node_id_) < rank->at(source)) {continue;}
                    warthog::graph::edge_iter high, low;
                    if(rank->at(it1->node_id_) < rank->at(it2->node_id_))
                    {
                        high = it2;
                        low = it1;
                    }
                    else
                    {
                        high = it1;
                        low = it2;
                    }

                    warthog::graph::node* high_node = 
                        g->get_node(high->node_id_);
                    warthog::graph::edge_iter stall_it = 
                        high_node->find_edge(low->node_id_);
                    if(stall_it != high_node->outgoing_end() &&
                        (high->wt_ + stall_it->wt_) < low->wt_)
                    {
                        s->del_outgoing(low);
                        it1--;
                        it2--;
                        stall_rm++;
                    }
                }
            }
    }
    std::cerr << "stall_rm total " << stall_rm << std::endl;

    //uint32_t up_hops = 2;
    //for(source = 0; source < g->get_num_nodes(); source++)
    //{
    //        warthog::graph::node* n = g->get_node(source);
    //        warthog::graph::edge_iter it = n->outgoing_begin();
    //        uint32_t min_lvl = UINT32_MAX;
    //        for( ; it != n->outgoing_end(); it++)
    //        {
    //            if(rank->at(it->node_id_) < min_lvl) 
    //            { 
    //                min_lvl = rank->at(it->node_id_);
    //            }
    //            cost.at(it->node_id_) = DBL_MAX;
    //        }

    //        dfs_fn(source, 1, 0, min_lvl);
    //        for( it = n->outgoing_begin(); it != n->outgoing_end(); it++)
    //        {
    //            if((cost.at(it->node_id_) + 0.0000001)< it->wt_)
    //            { 
    //                crap++;
    //            }
    //        }
    //}
}
