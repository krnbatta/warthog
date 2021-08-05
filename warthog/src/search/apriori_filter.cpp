#include "apriori_filter.h"
#include "search_node.h"
#include <cassert>

warthog::apriori_filter::apriori_filter(uint32_t num_elements) 
{
    filter_sz_ = num_elements;
    filter_ = new uint8_t[filter_sz_];
    reset_filter();
}

warthog::apriori_filter::~apriori_filter()
{
    delete [] filter_;
}

void 
warthog::apriori_filter::set_flag_true(uint32_t node_id)
{
    filter_[node_id] = true;
}

void
warthog::apriori_filter::set_flag_false(uint32_t node_id)
{
    filter_[node_id] = false;
}

void 
warthog::apriori_filter::reset_filter()
{
    for(uint32_t i = 0; i < filter_sz_; i++)
    {
        filter_[i] = false;
    }
}

bool
warthog::apriori_filter::filter(uint32_t node_id, uint32_t edge_idx)
{
    return get_flag(node_id);
}

bool
warthog::apriori_filter::get_flag(uint32_t node_id) 
{
    return filter_[node_id];
}

