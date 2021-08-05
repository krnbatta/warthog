#ifndef WARTHOG_MANHATTAN_HEURISTIC_H
#define WARTHOG_MANHATTAN_HEURISTIC_H

// manhattan_heuristic.h
//
// @author: dharabor
// @created: 21/08/2012
//

#include "constants.h"
#include "helpers.h"

#include <cstdlib>

namespace warthog
{

class manhattan_heuristic
{
	public:
		manhattan_heuristic(unsigned int mapwidth, unsigned int mapheight)
		 : mapwidth_(mapwidth) 
        {
            uint32_t bitwidth_map = 
                32 - __builtin_clz(mapwidth*mapheight);
            id_mask_ = (1 << bitwidth_map)-1;
        }
		~manhattan_heuristic() {}

		inline double
		h(int32_t x, int32_t y, int32_t x2, int32_t y2)
		{
            // NB: precision loss when double is an integer
			return (abs(x-x2) + abs(y-y2));
		}

		inline double
		h(unsigned int id, unsigned int id2)
		{
            id = id & id_mask_;
            id2 = id2 & id_mask_;

			int32_t x, x2;
			int32_t y, y2;
			warthog::helpers::index_to_xy(id, mapwidth_, x, y);
			warthog::helpers::index_to_xy(id2,mapwidth_, x2, y2);
			return this->h(x, y, x2, y2);
		}

        size_t
        mem() { return sizeof(this); }

	private:
		unsigned int mapwidth_;
        uint32_t id_mask_;
};

}

#endif

