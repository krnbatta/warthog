#pragma once
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <unordered_map>
#include "point.h"
#include "searchnode.h"

typedef std::unordered_map<std::string, std::string> map1;
typedef std::unordered_map<std::string, map1> map2;
typedef std::unordered_map<std::string, double> map3;
typedef std::unordered_map<std::string, map3> map4;

class Debugger
{
private:
	int eventCount = 0;
	map1 ids;
	map1 parentIds;
	map2 nodeVariables;
	map4 nodeValues;
	int global_id = 1;
	int firstExpanded = 0;
	std::stringstream outputString;
	std::stringstream nodeString;
	std::stringstream eventString;

public:
    Debugger();

    void AddStartEvent(std::string startId,std::string endId);

	void AddEndEvent(std::string endId);

	std::string AddStartNode(polyanya::Point root);
	void AddStartEvent(polyanya::Point start, polyanya::Point goal);
	std::string AddEndNode(polyanya::Point root);
	void AddEndEvent(polyanya::SearchNodePtr node, polyanya::Point root);
	std::string GetId(polyanya::SearchNodePtr nodePtr);
	std::string GetId(polyanya::Point point);
	std::string GetEventString(std::string eventType, std::string id, std::string parentId, std::unordered_map<std::string,std::string> variables, double g, double f);

	void AddGenerateEvent(polyanya::Point root, polyanya::SearchNodePtr nodePtr);


	void AddExpandingEvent(polyanya::SearchNodePtr nodePtr);

	void AddClosingEvent(polyanya::SearchNodePtr nodePtr);

  void printToDebugFile();

};
