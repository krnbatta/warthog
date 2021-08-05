#ifndef WARTHOG_SOLUTION_H
#define WARTHOG_SOLUTION_H

// a wrapper for solutions found by algorithms implemented
// in the warthog library
//
// @author: dharabor
// @created: 2017-05-03
//

#include <vector>

namespace warthog
{

class solution
{
    public:
        solution() : 
            sum_of_edge_costs_(warthog::INF), 
            time_elapsed_nano_(0),
            nodes_expanded_(0), 
            nodes_inserted_(0), 
            nodes_updated_(0),
            nodes_touched_(0)
        { }

        solution(const solution& other) :   
            sum_of_edge_costs_(other.sum_of_edge_costs_), 
            time_elapsed_nano_(other.time_elapsed_nano_), 
            nodes_expanded_(other.nodes_expanded_),
            nodes_inserted_(other.nodes_inserted_),
            nodes_updated_(other.nodes_updated_), 
            nodes_touched_(other.nodes_updated_),
            path_(other.path_)
        { }

        inline void
        print(std::ostream& out)
        {
            print_metrics(out);
            out << std::endl;
            print_path(out);
            out << std::endl;
        }

        inline void
        print_metrics(std::ostream& out)
        {
            out 
                << "sum_of_edge_costs=" << sum_of_edge_costs_ 
                << "time_elapsed_nano=" << time_elapsed_nano_ 
                << std::endl
                << "nodes expanded=" << nodes_expanded_ 
                << "inserted=" << nodes_inserted_ 
                << "updated=" << nodes_updated_ 
                << "touched= " << nodes_touched_;
        }

        inline void
        print_path(std::ostream& out)
        {
            out << "path=";
           for(auto &id : path_) { out << id << " "; }
        }

        void
        reset()
        {
            sum_of_edge_costs_ = warthog::INF;
            time_elapsed_nano_ = 0;
            nodes_expanded_ = 0; 
            nodes_inserted_ = 0; 
            nodes_updated_ = 0;
            nodes_touched_ = 0;
            path_.clear();
        }

        // metrics
        double sum_of_edge_costs_;
        double time_elapsed_nano_;
        uint32_t nodes_expanded_;
        uint32_t nodes_inserted_;
        uint32_t nodes_updated_;
        uint32_t nodes_touched_;

        // the actual solution
        std::vector<uint32_t> path_;

};

}

#endif
