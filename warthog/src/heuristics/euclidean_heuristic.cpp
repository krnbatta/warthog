#include "euclidean_heuristic.h"
#include "xy_graph.h"

warthog::euclidean_heuristic::euclidean_heuristic(warthog::graph::xy_graph* g) 
{ 
    g_ = g;
    hscale_ = 1; 
}

warthog::euclidean_heuristic::~euclidean_heuristic() 
{ }

double
warthog::euclidean_heuristic::h(uint32_t id, uint32_t id2)
{
    int32_t x, x2;
    int32_t y, y2;
    g_->get_xy(id, x, y);
    g_->get_xy(id2, x2, y2);
    return this->h(x, y, x2, y2);
}

double
warthog::euclidean_heuristic::h(int32_t x, int32_t y, int32_t x2, int32_t y2)
{
    // NB: precision loss when warthog::cost_t is an integer
    double dx = x-x2;
    double dy = y-y2;
    return sqrt(dx*dx + dy*dy) * hscale_;
}

void
warthog::euclidean_heuristic::set_hscale(double hscale)
{
    if(hscale > 0)
    {
        hscale_ = hscale;
    }
}

double
warthog::euclidean_heuristic::get_hscale() 
{ 
    return hscale_; 
}

size_t
warthog::euclidean_heuristic::mem() 
{ 
    return sizeof(this); 
}

