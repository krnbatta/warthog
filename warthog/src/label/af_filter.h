#ifndef WARTHOG_AF_FILTER_H
#define WARTHOG_AF_FILTER_H

// af_filter.h
//
// A search filter for use with an arcflag labelling. The idea is to filter
// (== prune) any edges which cannot possibly appear on the optimal path 
// to a specified target node.
//
// The filter requires a labelling 
//
// For theoretical details relating to the basic arcflags technique see:
//
// [Fast Point-to-Point Shortest Path Computations with Arc-Flags,
//  Ekkehard Ko ̈hler, Rolf H. Mo ̈hring, and Heiko Schilling,
//  In The Shortest Path Problem: Ninth DIMACS Implementation Challenge, 
//  Edited by Demetrescu, Camil and Goldberg, Andrew V. and Johnson, David S 
//  pp 41-72, American Mathematical Society, 2009]
//
// @author: dharabor
// @created: 2016-08-16
//

#include "af_labelling.h"
#include "forward.h"

#include <vector>
#include <cassert>
#include <cstdint>
#include <iostream>

namespace warthog
{

class af_filter
{
    public:
        af_filter(warthog::label::af_labelling* lab)
        {
            t_byte_ = 0;
            t_bitmask_ = 0;
            lab_ = lab;
        }

        virtual ~af_filter() { } 

        // Check if the label of a given edge matches the filter target
        // @param node_id, @param edge_idx specify the ith edge of node n.
        // The idea is to prune all edges that do not match the target.
        //
        // @return true if the edge label DOES NOT MATCH and false otherwise
        inline bool 
        filter(uint32_t node_id, uint32_t edge_idx)
        {
            uint8_t* label = lab_->get_label(node_id, edge_idx);
            return (label[t_byte_] & t_bitmask_) == 0;
        }

        inline void
        set_target(uint32_t target_id) 
        { 
            uint32_t t_part = lab_->get_partitioning()->at(target_id);
            t_byte_ = t_part >> 3;
            t_bitmask_ = 1 << (t_part & 7);
            assert(t_byte_ < lab_->get_label_size());
        }

        inline warthog::label::af_labelling*
        get_labelling() { return lab_; }

    private:    
        warthog::label::af_labelling* lab_;

        // we cache which bit corresponds to the target partition
        uint32_t t_byte_;
        uint32_t t_bitmask_;
};

}

#endif
