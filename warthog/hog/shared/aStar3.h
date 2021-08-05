/*
 * $Id: aStar3.h,v 1.2 2006/09/18 06:19:31 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Nathan Sturtevant on 9/29/04.
 *  Copyright 2004 Nathan Sturtevant. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef ASTAROld_H
#define ASTAROld_H

#include "searchAlgorithm.h"
#include "heap.h"
#include <stdexcept>

// this is a "classic" implementation of A*
// it is not particularly optimized, it is more of an example of how an
// algorithm can be coded in this framework

/**
* A sample A* implementation.
 */

class aStarOld : public searchAlgorithm {
	
public:
	aStarOld(double _w = 1.0, bool _doPathDraw = true);
	virtual ~aStarOld() {}
	virtual path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *getName() { return aStarName; }
	void drawPath(bool _doPathDraw) { doPathDraw = _doPathDraw; }
	void setDefaultEdgeWeight(double newwh) { wh = newwh; }
	double getDefaultEdgeWeight() { return wh; }
	void setGraphAbstraction(graphAbstraction *aMap) { map = aMap; }
	graphAbstraction* getGraphAbstraction() { return map; }

	virtual double h(node* a, node* b) throw(std::invalid_argument);
	
protected:
	virtual void relaxEdge(heap *nodeHeap, graph *g, edge *e, int source, int nextNode, node *to);
	virtual path *extractBestPath(graph *g, unsigned int current);

private:
	graphAbstraction *map;
	double wh;
	char aStarName[128];
	bool doPathDraw;
};

#endif
