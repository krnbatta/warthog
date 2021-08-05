#ifndef WARTHOG_BITFIELD_FILTER_H
#define WARTHOG_BITFIELD_FILTER_H

// search/bitfield_filter.h
//
// Sometimes during search it is desriable to ignore certain nodes that
// are identified bitfield as not useful for any query. This filter helps to 
// achieve the goal by keeping track of a single bit for each node in a 
// discrete graph. During search the filter can be queried about the 
// bit-state of each node and those nodes whose bit is set can be ignored 
// (not generated or expanded).
// 
// @author: dharabor
// @created: 2016-07-19

#include "constants.h"
#include <cstdint>
#include <cstring>

namespace warthog
{

class bitfield_filter 
{
    public:
        bitfield_filter(uint32_t num_nodes);
        virtual ~bitfield_filter();

        // returns true if the successor node specified by @param edge_idx
        // is being filtered. if the successor is not filtered, returns false
        bool
        filter(uint32_t node_id, uint32_t edge_idx);

        void
        set_flag_true(uint32_t node_id);

        void
        set_flag_false(uint32_t node_id);

        bool
        get_flag(uint32_t node_id);

        // clear all filter flags
        void
        reset_filter();

        // not used by this filter
        inline void
        set_target(uint32_t) { }

        inline size_t
        mem()
        {
            return 
                sizeof(*filter_)*filter_sz_ +
                sizeof(this);
        }

    private:
        warthog::dbword* filter_;
        uint32_t filter_sz_;
};

}

#endif
