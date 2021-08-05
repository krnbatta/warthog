#ifndef WARTHOG_JPS_WGM_H
#define WARTHOG_JPS_WGM_H

// jps_wgm.h
//
// This file contains common definitions required for
// various classes that facilitate Jump Point Search on weighted gridmaps.
//
// @author: dharabor
// @created: 2014-09-25
//

#include "constants.h"

namespace warthog
{

namespace jps
{

// Analogous to warthog::jps::compute_natural but specialised for weighted
// grid maps.
uint32_t 
compute_natural_wgm(warthog::jps::direction d, warthog::dbword tiles[9]);

// Analogous to warthog::jps::compute_forced but specialised for weighted 
// grid maps.
uint32_t 
compute_forced_wgm(warthog::jps::direction d, warthog::dbword tiles[9]);

// Additional forced neighbour checks for weighted gridmaps.
// This version is based on terrain type: any tile whose terrain 
// differs from the current node is considered forced.
uint32_t 
compute_forced_wgm_terrain(warthog::jps::direction d, 
        warthog::dbword tiles[9]);

// Computes successors (forced \union natural) of a node (x, y)
// This function is specialised for weighted-cost grid maps.
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the 3x3 square of cells having (x, y) at its centre.
//
// @return an integer representing the set of forced directions.
// Each of the first 8 bits of the returned value, when set, correspond to a
// direction, as defined in warthog::jps::direction
inline uint32_t
compute_successors_wgm(warthog::jps::direction d, warthog::dbword tiles[9])
{
	return warthog::jps::compute_forced_wgm(d, tiles) |
	   	warthog::jps::compute_natural_wgm(d, tiles);
}
}

}


#endif

