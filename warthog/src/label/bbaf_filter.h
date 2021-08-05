#ifndef WARTHOG_BBAF_FILTER_H
#define WARTHOG_BBAF_FILTER_H

// bbaf_filter.h
//
// combined arcflags and bounding box filter
//
// @author: dharabor
// @created: 2016-08-25
//

#include <vector>
#include <cassert>
#include <cstdint>
#include <iostream>

#include "bbaf_labelling.h"
#include "forward.h"
#include "geom.h"

namespace warthog
{

class bbaf_filter
{

    public:
        // @param g: the search graph
        // @param part: a partitioning of the graph nodes
        bbaf_filter(warthog::label::bbaf_labelling* lab)
        {
            lab_ = lab;
            t_byte_ = 0;
            t_bitmask_ = 0;
            tx_ = ty_ = 0;
        }

        virtual ~bbaf_filter() { } 

        // return true if the ith edge of @param node_id
        // (as specified by @param edge_index) cannot possibly 
        // appear on any optimal path to the current target;
        // return false otherwise.
        inline bool 
        filter(uint32_t node_id, uint32_t edge_idx)
        {
            warthog::label::bbaf_label& label 
                = lab_->get_label(node_id, edge_idx);
            bool retval = (label.flags_[t_byte_] & t_bitmask_);
            retval = retval && label.bbox_.contains(tx_, ty_);
            return !retval; 
        }

        inline void
        set_target(uint32_t goalid)
        {
            uint32_t t_part = lab_->get_partitioning()->at(goalid);
            t_byte_ = t_part >> 3;
            t_bitmask_ = 1 << (t_part & 7);
            lab_->get_graph()->get_xy(goalid, tx_, ty_);
        }

    private:    
        // values to quickly extract the flag bit for the target at hand
        uint32_t t_byte_;
        uint32_t t_bitmask_;
        int32_t tx_, ty_;
        warthog::label::bbaf_labelling* lab_;
};

}

#endif

