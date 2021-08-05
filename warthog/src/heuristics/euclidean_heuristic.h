#ifndef WARTHOG_EUCLIDEAN_HEURISTIC_H
#define WARTHOG_EUCLIDEAN_HEURISTIC_H

// euclidean_heuristic.h
//
// Straight-line heuristic for measuring distances in the plane.
//
// @author: dharabor
// @created: 2016-02-11
//
//

#include "constants.h"
#include "forward.h"

namespace warthog
{

typedef void (*xyFn)(uint32_t id, int32_t& x, int32_t& y);
class euclidean_heuristic
{
    public:
        euclidean_heuristic(warthog::graph::xy_graph* g);
        ~euclidean_heuristic();

        double
        h(uint32_t id, uint32_t id2);

		double
		h(int32_t x, int32_t y, int32_t x2, int32_t y2);

        void
        set_hscale(double hscale);

        double
        get_hscale();

        size_t
        mem(); 

	private:
        warthog::graph::xy_graph* g_;
        double hscale_;

};

}

#endif

