#include "Debug.h"
#include <algorithm>
#include <unordered_map>
#include "point.h"
#include "searchnode.h"

using namespace std;

Debugger::Debugger(){
	outputString << "{";
	nodeString << "\"nodeStructure\": [{\"type\": \"circle\", \"variables\": {\"cx\": \"cx\", \"cy\": \"cy\"}, \"persisted\": true, \"drawPath\": true},{\"type\": \"line\", \"variables\": {\"x1\": \"x1\", \"y1\": \"y1\", \"x2\": \"x2\", \"y2\": \"y2\"}, \"persisted\": false},{\"type\": \"line\", \"variables\": {\"x1\": \"cx\", \"y1\": \"cy\", \"x2\": \"x1\", \"y2\": \"y1\"}, \"persisted\": false},{\"type\": \"line\", \"variables\": {\"x1\": \"cx\", \"y1\": \"cy\", \"x2\": \"x2\", \"y2\": \"y2\"}, \"persisted\": false}, {\"type\": \"polygon\", \"variables\": {\"points\": [\"x1\", \"y1\", \"x2\", \"y2\", \"cx\", \"cy\"]}, \"persisted\": false}]";
	eventString << "\"eventList\":[";
}

std::string Debugger::AddStartNode(polyanya::Point root){
	std::string id = GetId(root);

	std::unordered_map<std::string, std::string> variables;
	variables["cx"] = to_string(root.x);
	variables["cy"] = to_string(root.y);
	variables["x1"] = to_string(root.x);
	variables["y1"] = to_string(root.y);
	variables["x2"] = to_string(root.x);
	variables["y2"] = to_string(root.y);

	nodeVariables[id] = variables;

	return "{\"type\": \"source\", \"id\":" + id + ", \"variables\": {" +
    "\"cx\": " + to_string(root.x) + ", \"cy\": " + to_string(root.y) + ", " +
    "\"x1\": " + to_string(root.x) + ", \"y1\": " + to_string(root.y) + ", " +
    "\"x2\": " + to_string(root.x) + ", \"y2\": " + to_string(root.y) + "}}";
}

std::string Debugger::AddEndNode(polyanya::Point root){
	std::string id = GetId(root);

	std::unordered_map<std::string, std::string> variables;
	variables["cx"] = to_string(root.x);
	variables["cy"] = to_string(root.y);
	variables["x1"] = to_string(root.x);
	variables["y1"] = to_string(root.y);
	variables["x2"] = to_string(root.x);
	variables["y2"] = to_string(root.y);

	nodeVariables[id] = variables;

	return "{\"type\": \"destination\", \"id\":" + id + ", \"variables\": {" +
    "\"cx\": " + to_string(root.x) + ", \"cy\": " + to_string(root.y) + ", " +
    "\"x1\": " + to_string(root.x) + ", \"y1\": " + to_string(root.y) + ", " +
    "\"x2\": " + to_string(root.x) + ", \"y2\": " + to_string(root.y) + "}}";
}

void Debugger::AddStartEvent(polyanya::Point start, polyanya::Point goal){
	eventString << AddStartNode(start) << ", " << AddEndNode(goal) << ", ";
	std::unordered_map<std::string, std::string> variables;
	variables["cx"] = to_string(start.x);
	variables["cy"] = to_string(start.y);
	variables["x1"] = to_string(start.x);
	variables["y1"] = to_string(start.y);
	variables["x2"] = to_string(start.x);
	variables["y2"] = to_string(start.y);

	std::string id = GetId(start);

	nodeVariables[id] = variables;

	eventString << GetEventString("generating", id, "null", variables, 0.0, 0.0) << ", ";
	eventString << GetEventString("expanding", id, "null", variables, 0.0, 0.0) << ", ";
}

void Debugger::AddEndEvent(polyanya::SearchNodePtr node, polyanya::Point root){
	std::unordered_map<std::string, std::string> variables;
	variables["cx"] = to_string(root.x);
	variables["cy"] = to_string(root.y);
	polyanya::Point left = node -> left;
	polyanya::Point right = node -> right;
	variables["x1"] = to_string(left.x);
	variables["y1"] = to_string(left.y);
	variables["x2"] = to_string(right.x);
	variables["y2"] = to_string(right.y);

	std::string id = "2";

	nodeVariables[id] = variables;

	eventString << GetEventString("end", id, GetId(node->parent), variables, node->g, node->f);
}

std::string Debugger::GetId(polyanya::Point point){
	std::stringstream pointId;
	if(ids.find(point.toStr()) == ids.end()){
		ids[point.toStr()] = to_string(global_id++);
	}
	return ids[point.toStr()];
}

std::string Debugger::GetId(polyanya::SearchNodePtr nodePtr){
	if(ids.find(nodePtr -> toStr()) == ids.end()){
		ids[nodePtr -> toStr()] = to_string(global_id++);
	}
	return ids[nodePtr -> toStr()];
}

std::string Debugger::GetEventString(std::string eventType, std::string id, std::string parentId, std::unordered_map<std::string,std::string> variables, double g, double f){
	std::string eventStr = "";
	eventStr += "{\"id\":\""+id+"\",\"pId\":\""+parentId+"\",\"type\":\""+eventType+"\",\"variables\":{";

	for (auto entry : variables) {
		std::string key = entry.first;
		std::string value = entry.second;
		eventStr += "\"" + key + "\": " + value + ",";
	}

	eventStr = eventStr.substr(0,eventStr.length()-1);
	eventStr += "},\"g\":"+to_string(g)+",\"f\":"+to_string(f)+"}";
	return eventStr;
}

void Debugger::AddGenerateEvent(polyanya::Point root, polyanya::SearchNodePtr nodePtr){

	std::unordered_map<std::string, std::string> variables;
	std::unordered_map<std::string, double> values;
	variables["cx"] = to_string(root.x);
	variables["cy"] = to_string(root.y);
	polyanya::Point left = nodePtr -> left;
	polyanya::Point right = nodePtr -> right;
	variables["x1"] = to_string(left.x);
	variables["y1"] = to_string(left.y);
	variables["x2"] = to_string(right.x);
	variables["y2"] = to_string(right.y);
	double f = nodePtr->f;
	double g = nodePtr->g;
	values["f"] = f;
	values["g"] = g;
	std::string id = GetId(nodePtr);
	std::string parentId;
	if(!firstExpanded){
		parentId = "1";
	}
	else{
		parentId = GetId(nodePtr->parent);
	}
	parentIds[id] = parentId;
	nodeVariables[id] = variables;
	nodeValues[id] = values;
	eventString << GetEventString("generating", id, parentId, variables, g, f) + ", ";
}

void Debugger::AddExpandingEvent(polyanya::SearchNodePtr nodePtr){
	if(!firstExpanded){
			eventString << GetEventString("closing", "1", "null", nodeVariables["1"], 0.0, 0.0) << ", ";
			firstExpanded = 1;
	}
	std::string id = GetId(nodePtr);
	std::unordered_map<std::string, std::string> variables = nodeVariables[id];
	std::string parentId = parentIds[id];
	std::unordered_map<std::string, double> values = nodeValues[id];
	double g = values["g"];
	double f = values["f"];
	eventString << GetEventString("expanding", id, parentId, variables, g, f) + ", ";
}

void Debugger::AddClosingEvent(polyanya::SearchNodePtr nodePtr){
	std::string id = GetId(nodePtr);
	std::unordered_map<std::string, std::string> variables = nodeVariables[id];
	std::string parentId = parentIds[id];
	std::unordered_map<std::string, double> values = nodeValues[id];
	double g = values["g"];
	double f = values["f"];
	eventString << GetEventString("closing", id, parentId, variables, g, f) + ", ";
}

void Debugger::printToDebugFile()
{
	std::ofstream outputfile; //open and save the file

		std::time_t t = time(0);
		struct tm * now = localtime(&t);
		std::string strFileName = "Debug:";
		strFileName.append(asctime(now));
		strFileName.append(".json");
		outputfile.open(strFileName);
		outputString << nodeString.str() << ",\n" << eventString.str() << "]}";

		std::string data = outputString.str();
		outputfile << data;
		outputfile.close();

		outputfile.open("ids");
		std::stringstream idStream;
		for (auto entry : ids) {
			std::string key = entry.first;
			std::string value = entry.second;
			idStream << key << " => " << value << "\n";
		}
		outputfile << idStream.str();

		outputfile.close();
}
