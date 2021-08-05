/*
 * $Id: mapAbstraction.h,v 1.15 2006/11/01 23:02:12 nathanst Exp $
 *
 *  mapAbstraction.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "map.h"
#include "graphAbstraction.h"
#include "glUtil.h"
#include "constants.h"

#ifndef MAPABSTRACTION_H
#define MAPABSTRACTION_H

/**
* This class is designed as an interface to be added onto any type of graphAbstraction
 * to support a few extra functionalities that mapabstractions should have.
 */

class INodeFactory;
class IEdgeFactory;

class mapAbstraction : public graphAbstraction {
public:
	mapAbstraction(Map *_m, bool diag, bool corners) :
		m(_m), allow_diagonals_(diag), cut_corners_(corners), levelDraw(0) {}

	virtual ~mapAbstraction();
	/** return a new abstraction map of the same type as this map abstraction */
	virtual mapAbstraction *clone(Map *) = 0;
	node *getNodeFromMap(int x, int y, tCorner c = kNone) { return abstractions[0]->getNode(m->getNodeNum(x, y, c)); }
	void getTileFromNode(node *n, int &x, int &y);
	void getRandomTileFromNode(node *n, int &x, int &y);

	/** Given a location (recVec) return the tile under that location */
	void getTileUnderLoc(int &x, int &y, const recVec &);

	Map* getMap() { return m; }
	
	virtual double h(node *a, node *b);
	bool getAllowDiagonals() { return this->allow_diagonals_; }
	bool getCutCorners() { return this->cut_corners_; }
	
	double octileDistance(double,double,double,double);
	
	// display functions
	virtual void openGLDraw();
	void toggleDrawAbstraction(int which);
	void clearMarkedNodes();
	recVec getNodeLoc(node *n);
	void clearColours(); // stop visualising touched and expanded nodes

private:
		
	void drawLevelConnections(node *n);
	void drawGraph(graph *g);
	
	Map *m;
	bool allow_diagonals_;
	bool cut_corners_;
	unsigned long levelDraw;
};

// methods for creating graph objects from Map objects
graph* getMapGraph(Map* m, INodeFactory* nf, IEdgeFactory* ef, bool allowDiagonals, bool cutCorners=true);
graph *getMapGraph(Map *m);

// support methods for getMapGraph
void addMapEdges(Map *m, graph *g, IEdgeFactory* ef, int x, int y, bool allowDiagonals, bool cutCorners);
graph* makeMapNodes(Map* m, INodeFactory* nf);

#endif

