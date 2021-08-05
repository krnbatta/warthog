// Check if each arc of a given DIMACS input graph is at least as long as the
// Euclidean distance between its endpoints. Each arc that does not
// satisfy this criteria has its weight set to the Euclidean distance
// between its endpoints. The result is dumped to the console
// (even if nothing is modified).
//
// @author: dharabor
// @created: 2018-05-04

#include "xy_graph.h"

#include <iostream>

int
main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr 
            << argv[0] << " "
            << "[DIMACS graph file] " 
            << "[DIMACS coordinates file] "
            << std::endl;
        return EINVAL;
    }
    
    warthog::graph::xy_graph g;
    g.load_from_dimacs(argv[1], argv[2]);
    g.print_dimacs_gr(std::cout, 0, g.get_num_nodes());
}
