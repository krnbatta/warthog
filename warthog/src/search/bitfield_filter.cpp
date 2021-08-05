#include "bitfield_filter.h"
#include "search_node.h"
#include <cassert>

warthog::bitfield_filter::bitfield_filter(uint32_t num_elements) 
{
    filter_sz_ = (num_elements >> warthog::LOG2_DBWORD_BITS)+1;
    filter_ = new warthog::dbword[filter_sz_];
    reset_filter();
}

warthog::bitfield_filter::~bitfield_filter()
{
    delete [] filter_;
}

void 
warthog::bitfield_filter::set_flag_true(uint32_t node_id)
{
    int index = node_id >> warthog::LOG2_DBWORD_BITS;
    int pos = node_id & DBWORD_BITS_MASK;
    filter_[index] |= (1 << pos);
}

void
warthog::bitfield_filter::set_flag_false(uint32_t node_id)
{
    int index = node_id >> warthog::LOG2_DBWORD_BITS;
    int pos = node_id & DBWORD_BITS_MASK;
    filter_[index] &= ~(1 << pos);
}

void 
warthog::bitfield_filter::reset_filter()
{
    for(uint32_t i = 0; i < filter_sz_; i++)
    {
        filter_[i] = 0;
    }
}

bool
warthog::bitfield_filter::filter(uint32_t node_id, uint32_t edge_idx)
{
    return get_flag(node_id);
}

bool
warthog::bitfield_filter::get_flag(uint32_t id) 
{
    int index = id >> warthog::LOG2_DBWORD_BITS;
    int pos = id & DBWORD_BITS_MASK;
    return filter_[index] & (1 << pos);
}

