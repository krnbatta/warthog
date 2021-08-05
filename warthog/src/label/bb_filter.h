#ifndef WARTHOG_BB_FILTER_H
#define WARTHOG_BB_FILTER_H

// contraction/bb_filter.h
//
// A node filter making use of rectangular geometric 
// containers (i.e. bounding boxes)
//
// For theory see:
//
// [Wager & Willhalm, 2005, Geometric Containers for 
// Efficient Shortest Path Computation, 
// Journal of Experimental Algorithms, vol 10, pp 1-30]
//
// @author: dharabor
// @created: 2016-08-02
//

#include "bb_labelling.h"
#include "forward.h"
#include "geom.h"
#include <vector>

namespace warthog
{

class bb_filter
{

    public:
        bb_filter(warthog::label::bb_labelling* bbl)
        {
            bbl_ = bbl;
            tx_ = ty_ = warthog::INF;
        }

        ~bb_filter() { }

        inline void
        set_target_xy(int32_t tx, int32_t ty) { tx_ = tx; ty_ = ty; }

        // return true if the bounding box of of the specified edge DOES NOT
        // contain the target, and return false otherwise. 
        inline bool 
        filter(uint32_t node_id, uint32_t edge_id)
        {
            warthog::geom::rectangle rect = bbl_->get_label(node_id, edge_id);
            if(rect.contains(tx_, ty_))
            {
                return false;
            }
            return true;
        }

    private:
        int32_t tx_, ty_;
        warthog::label::bb_labelling* bbl_;

};

}

#endif

