/*
 * $Id: aStar.h,v 1.8 2007/03/07 21:58:08 nathanst Exp $
 *
 *  aStar3.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/22/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
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


#ifndef ASTAR3_H
#define ASTAR3_H

#include "searchAlgorithm.h"
#include "path.h"
#include "graphAbstraction.h"
#include "fpUtil.h"
#include <ext/hash_map>
#include "heap2.h"

namespace AStar3Util
{
	class SearchNode {
public:
		SearchNode(double _fCost=0, double _gCost=0, edge *_e=0, node *curr=0, node *prev=0)
		:fCost(_fCost), gCost(_gCost), e(_e), currNode(curr), prevNode(prev) {}
		
		double fCost;
		double gCost;
//		double steps;
		edge *e;
		node *currNode;
		node *prevNode;
	};
	
	
	struct NodeEqual {
		bool operator()(const node *i1, const node *i2)
		{ return (i1->getUniqueID() == i2->getUniqueID()); } };
	
	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{ return (i1.currNode->getUniqueID() == i2.currNode->getUniqueID()); } };
	
	struct SearchNodeCompare {
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} };
	
	struct NodeHash {
		size_t operator()(const node *x) const
		{ return (size_t)(x->getUniqueID()); }
	};
	
	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode->getUniqueID()); }
	};
	
	typedef heap2<AStar3Util::SearchNode, AStar3Util::SearchNodeHash,
		AStar3Util::SearchNodeEqual, AStar3Util::SearchNodeCompare> PQueue;
	
	typedef __gnu_cxx::hash_map<node*, AStar3Util::SearchNode,
		AStar3Util::NodeHash, AStar3Util::NodeEqual > NodeLookupTable;
	
	typedef __gnu_cxx::hash_map<node*, bool,
		AStar3Util::NodeHash, AStar3Util::NodeEqual > Corridor;
}

class aStar : public searchAlgorithm {
public:
	aStar() {}
	virtual ~aStar() {}
	path *getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *getName();
	
	double getHVal(node *whence);
	void setCorridor(path *corridor, int width);
	
	void printStats();
	long getNodesExpanded() { return nodesExpanded; }
	long getNodesTouched() { return nodesTouched; }
	void resetNodeCount() { nodesExpanded = nodesTouched = 0; }
	int getMemoryUsage();
private:
	//	long nodesTouched, nodesExpanded;
	inline node *ABSNode(node *n) { return abstr->getNthParent(n, absLevel); }
	path *getPathToNode(node *target);
	path *extractPathToStart(graph *g, node *n);
	node *getNextNode();
	void updateWeight(node *currOpenNode, node *neighbor, edge *e);
	void addToOpenList(node *currOpenNode, node *neighbor, edge *e);
	bool nodeInCorridor(node *n);
	void addNeighborsToCorridor(graph *g, node *n, int windowSize);
	void buildCorridor(path *p, int windowSize);
	double internalHeuristic(node *from, node *to);
	AStar3Util::PQueue openQueue;
	AStar3Util::NodeLookupTable openList, closedList;
	node *goal, *start;
	graph *g;
	graphAbstraction *abstr;
	AStar3Util::Corridor eligibleNodes;
	int absLevel;
	//	AStarHeuristic *abstraction;
};

#endif
