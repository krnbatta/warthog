/*
 * $Id: aStar.cpp,v 1.10 2006/10/18 23:52:25 nathanst Exp $
 *
 *  aStar3.cpp
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

#include "aStar.h"
#include "float.h"

using namespace AStar3Util;
static const bool verbose = false;
//const int gMaxAbstraction = 0;

inline double max(double a, double b)
{
	if (fgreater(a, b))
		return a;
	return b;
}

const char *aStar::getName()
{
	static char name[32];
	sprintf(name, "aStar[]");
	return name;
}


//aStar::aStar(graphAbstraction *_abstr, node *_start, node *_goal,
//															 path *corridor, int corridorWidth, int _absLevel)
path *aStar::getPath(graphAbstraction *aMap, node *from, node *to, reservationProvider *)
{
	assert(openList.size() == 0);
	assert(openQueue.size() == 0);
	assert(closedList.size() == 0);
	nodesTouched = nodesExpanded = 0;
	abstr = aMap;
	start = from;
	goal = to;
	if ((from == 0) || (to == 0) || (from == to) || (!aMap->pathable(from, to)))
		return 0;
	g = abstr->getAbstractGraph(start);
	
	SearchNode first(internalHeuristic(goal, start), 0, 0, start, start);
	openQueue.add(first);
	openList[start] = first;
		
	return getPathToNode(goal);
	//printf("##Initial nodes expanded: %d, touched: %d\n", nodesExpanded, nodesTouched);
}

void aStar::setCorridor(path *corridor, int width)
{
	eligibleNodes.resize(0);
	buildCorridor(corridor, width);
}

path *aStar::getPathToNode(node *target)
{
	node *currentOpenNode = 0;
	while ((openList.size() > 0) && (currentOpenNode != target))
	{
		// get top of queue
		currentOpenNode = getNextNode();
		if (currentOpenNode == target)
			break;
		assert(openList.size() == openQueue.size());
		if (currentOpenNode == 0)
			printf("Oh no! The current open node is NULL\n");
		edge_iterator ei = currentOpenNode->getEdgeIter();
		
		// iterate over all the children
		for (edge *e = currentOpenNode->edgeIterNext(ei); e; e = currentOpenNode->edgeIterNext(ei))
		{
			nodesTouched++;
			unsigned int which;
			if ((which = e->getFrom()) == currentOpenNode->getNum()) which = e->getTo();
			node *neighbor = g->getNode(which);
			assert(neighbor != 0);
			
			if (closedList.find(neighbor) != closedList.end())
			{
				if (verbose) { printf("skipping node %d\n", neighbor->getNum()); }
				continue;
			}
			else if (!nodeInCorridor(neighbor))
			{
				if (verbose) { printf("node %d not in corridor\n", neighbor->getNum()); }
			}
			else if (openList.find(neighbor) != openList.end())
			{
				if (verbose) { printf("updating node %d\n", neighbor->getNum()); }
				updateWeight(currentOpenNode, neighbor, e);
				assert(openList.size() == openQueue.size());
			}
			else {
				if (verbose) { printf("addinging node %d\n", neighbor->getNum()); }
				addToOpenList(currentOpenNode, neighbor, e);
				assert(openList.size() == openQueue.size());
			}
		}
	}
	if (currentOpenNode == target)
	{
		//		if (abstr->getAbstractionLevel(whence) == 0)
		path *p = extractPathToStart(g, currentOpenNode);
		closedList.clear();
		openList.clear();
		openQueue.reset();
		return p;
	}
	closedList.clear();
	openList.clear();
	openQueue.reset();
	return 0; // no path found!
}

node *aStar::getNextNode()
{
	nodesExpanded++;
	node *next;
	SearchNode it = openQueue.remove();
	next = it.currNode;
	//openQueue.pop();
	if (openList.find(next) == openList.end())
	{
		printf("Error; next node in openQueue not in openList!\n");
	}
	openList.erase(openList.find(next));
	closedList[next] = it;
	if (openList.size() != openQueue.size())
	{
		printf("openList size %u, openQueue size %u\n", (unsigned int)openList.size(),
					 (unsigned int)openQueue.size());
		printf("Opening node %d (f = %1.2f, g = %1.2f. [h=%1.2f])\n",
					 next->getNum(), it.fCost, it.gCost, abstr->h(next, goal));
		assert(openList.size() == openQueue.size());
	}
	return next;
}

void aStar::updateWeight(node *currOpenNode, node *neighbor, edge *e)
{
	SearchNode prev = openList[neighbor];
	SearchNode alt = closedList[currOpenNode];
	double altCost = alt.gCost+e->getWeight()+(prev.fCost-prev.gCost);
	if (fgreater(prev.fCost, altCost))
	{
		//prev.steps = alt.steps+1;
		prev.fCost = altCost;
		prev.gCost = alt.gCost+e->getWeight();
		prev.prevNode = currOpenNode;
		prev.e = e;
		// reset neighbor in queue
		//openQueue.erase(openQueue.find(neighbor));
		//openQueue.push(prev);
		openQueue.decreaseKey(prev);
		openList[neighbor] = prev;
	}
}

void aStar::addToOpenList(node *currOpenNode, node *neighbor, edge *e)
{
	SearchNode n(closedList[currOpenNode].gCost+e->getWeight()+internalHeuristic(neighbor, goal),
							 closedList[currOpenNode].gCost+e->getWeight(),
							 e,
							 neighbor, currOpenNode/*, closedList[currOpenNode].steps+1*/);
	if (verbose) 
	{ printf("Adding %u to openQueue, old size %u, ", neighbor->getNum(), openQueue.size()); }
	openQueue.add(n);

	if (verbose)
	{ 
		printf("New size %u\n", openQueue.size());
		printf("Adding %u to openList, old size %u, ", (unsigned int)neighbor->getNum(),
					 (unsigned int)openList.size());
	}
	openList[neighbor] = n;
	if (verbose) 
	{
		printf("New size %u\n", (unsigned int)openList.size());
		if (openList.find(neighbor) == openList.end())
			printf("!!Hey; can't lookup the item I just inserted!\n");
		assert(openList.find(neighbor) != openList.end());
	}
}

path *aStar::extractPathToStart(graph *_g, node *goalNode)
{
	path *p = 0;
	SearchNode n = closedList[goalNode];
	do {
		if (n.currNode && n.prevNode)
			_g->findEdge(n.currNode->getNum(), n.prevNode->getNum())->setMarked(true);
		p = new path(n.currNode, p);
		n = closedList[n.prevNode];
	} while (n.currNode != n.prevNode);
	p = new path(n.currNode, p);
	return p;
}

void aStar::printStats()
{
	printf("%u items in closed list\n", (unsigned int)closedList.size());
	printf("%u items in open list\n", (unsigned int)openList.size());
	printf("%u items in queue\n", (unsigned int)openQueue.size());
}

int aStar::getMemoryUsage()
{
	return closedList.size()+openList.size();
}

void aStar::buildCorridor(path *p, int windowSize)
{
	//	Corridor eligibleNodes;
	for (path *t = p; t; t = t->next)
		addNeighborsToCorridor(abstr->getAbstractGraph(t->n), t->n, windowSize);
}

void aStar::addNeighborsToCorridor(graph *_g, node *n, int windowSize)
{
	// if node is already in corridor, stop
	//	if (eligibleNodes.find(n) != eligibleNodes.end())
	//		return;
	eligibleNodes[n] = true;
	edge_iterator ei = n->getEdgeIter();
	
	// iterate over all the neighbors
	for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
	{
		unsigned int which;
		if ((which = e->getFrom()) == n->getNum()) which = e->getTo();
		
		node *neighbor = _g->getNode(which);
		if (windowSize)
			addNeighborsToCorridor(_g, neighbor, windowSize-1);
		else
			eligibleNodes[neighbor] = true;
	}
}

bool aStar::nodeInCorridor(node *n)
{
	return ((eligibleNodes.size() == 0) ||
					(eligibleNodes.find(abstr->getParent(n)) != eligibleNodes.end()));
}

double aStar::internalHeuristic(node *from, node *to)
{
	//	if ((abstr->getAbstractionLevel(from) >= gMaxAbstraction) || (abstraction == 0))
		return abstr->h(from, to);
	//	return abstraction->getHVal(abstr->getParent(from));
}
