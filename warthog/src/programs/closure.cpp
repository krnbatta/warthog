#include "contraction.h"
#include "xy_graph.h"
#include <set>

int
main(int argv, char** argc)
{   
    if(argv < 4)
    {
        std::cout << "Usage: ./closure [gr file] [co file] [source id]"
            << " [optional exclude ids] " << std::endl;
        return 0;
    }

    warthog::graph::xy_graph g;
    g.load_from_dimacs(argc[1], argc[2]);

    std::set<uint32_t> closure;
    uint32_t node_id = atoi(argc[3]);

    if(argv > 4)
    {
        for(int i = 4; i < argv; i++)
        {
            std::cout << "ignore " << (atoi(argc[i])) << std::endl;
            closure.insert(atoi(argc[i]));
        }
    }
    warthog::ch::compute_closure(node_id, &g, &closure, 1);

    std::cout << "closure: \n";
    for(std::set<uint32_t>::iterator it = closure.begin(); 
            it != closure.end(); it++)
    {
        std::cout << (*it) << " ";
    }
    std::cout << std::endl;
    return 0;
}

//void
//stats()
//{
//    std::string grfile = cfg.get_param_value("input");
//    std::string cofile = cfg.get_param_value("input");
//    std::string orderfile = cfg.get_param_value("input");
//
//    if( grfile == "" || cofile == "" || orderfile == "")
//    {
//        std::cerr << "err; insufficient input parameters."
//                  << " required, in order:\n"
//                  << " --input [gr file] [co file]"
//                  << " [node ordering file]\n";
//        return;
//    }
//
//    // load up the graph
//    warthog::graph::xy_graph g;
//    if(!g.load_dimacs(grfile.c_str(), cofile.c_str(), false, true))
//    {
//        std::cerr << "err; could not load gr or co input files (one or both)\n";
//        return;
//    }
//
//    // load up the node ordering
//    std::vector<uint32_t> order;
//    if(!warthog::ch::load_node_order(orderfile.c_str(), order, false))
//    {
//        std::cerr << "err; could not load node order input file\n";
//        return;
//    }
//
//    std::vector<uint32_t> rank;
//    if(!warthog::ch::load_node_order(orderfile.c_str(), rank, true))
//    {
//        std::cerr << "err; could not load node order input file\n";
//        return;
//    }
//
//    if(order.size() != g.get_num_nodes())
//    {
//        std::cerr 
//            << "err; partial contraction ordering not supported\n";
//    }
//
//    int32_t id = 0;
//    uint32_t count = 0;
//    std::vector< int32_t > recurse(g.get_num_nodes(), id);
//    std::function<void(uint32_t)> dn_closure_fn = 
//        [&recurse, &dn_closure_fn, &g, &rank, &count, &id]
//        (uint32_t source_id) -> void
//        {
//            warthog::graph::node* source = g.get_node(source_id);
//            warthog::graph::edge_iter begin = source->outgoing_begin();
//
//            for( warthog::graph::edge_iter it = begin; 
//                    it != source->outgoing_end();
//                    it++)
//            {
//                // skip up edges
//                if(rank.at(it->node_id_) > rank.at(source_id)) 
//                { continue; }
//
//                if(recurse.at(it->node_id_)  <= id)
//                { dn_closure_fn(it->node_id_); }
//            }
//            recurse.at(source_id) = id+1;
//            count++;
//        };
//
//    //std::vector< int32_t > dn_closure(g.get_num_nodes(), 0);
//    std::set< uint32_t > dn_closure;
//    for(uint32_t i = 0; i < g.get_num_nodes(); i++)
//    {
//        dn_closure.clear();
//        //recurse.assign(g.get_num_nodes(), INT32_MAX);
//        warthog::ch::compute_down_closure(i, &g, &rank, &dn_closure);
////        dn_closure.at(i) = count;
//
////        count = 0;
////        dn_closure_fn(i);
////        dn_closure.at(i) = count;
////        id++;
//        std::cerr << i << "\r";
//    }
//
//    std::cout << "rank\tup_deg\tdn_deg\tmax_level\tndown\n";
//    for(uint32_t i = 0; i < order.size(); i++)
//    {
//        int32_t up_degree = 0;
//        int32_t dn_degree = 0;
//        int32_t max_level = 0;
//        warthog::graph::node* n = g.get_node(order.at(i));
//        warthog::graph::edge_iter begin = n->outgoing_begin();
//        warthog::graph::edge_iter end = n->outgoing_end();
//
//        for(warthog::graph::edge_iter it = begin; it != end; it++)
//        {
//            if(rank.at(it->node_id_) > i)
//            {
//                up_degree++;
//            }
//            else
//            {
//                dn_degree++;
//            }
//            max_level = std::max<int32_t>(max_level, rank.at(it->node_id_));
//        }
//        std::cout << i 
//            << "\t" << up_degree 
//            << "\t" << dn_degree 
//            << "\t" << max_level  
//            << "\t" << dn_closure.size()
//            << "\n";
//    }
//}
