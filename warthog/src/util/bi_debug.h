#ifndef WARTHOG_BIDEBUG
#define WARTHOG_BIDEBUG
#include <iostream>
#include <sstream>
#include <fstream>
#include <ctime>
#include "gridmap.h"
#include "search_node.h"

class Bidebugger
{
private:
	std::string typeStrings[6] = {"start","expanding","generating","updating","closing","end"};
	std::stringstream saveData;
public:
    Bidebugger(){
			saveData << "{\"nodeStructure\": [{\"type\": \"circle\", \"variables\": {\"cx\": \"x\", \"cy\": \"y\"}, \"persisted\": true, \"drawPath\": true},{\"type\": \"line\", \"variables\": {\"x1\": \"parent:x\", \"y1\": \"parent:y\", \"x2\": \"x\", \"y2\": \"y\"}, \"persisted\": true}], ";

		saveData << "\"eventList\" : [ \n";

    }

	void addSource(warthog::search_node* node, std::string id, int32_t x, int32_t y, std::string pId){
		saveData << "{\"type\":\"source\","
				 << "\"id\":\"" << id << "\",\"variables\":{\"x\":" << x << ",\"y\":" << y << "}"
				 << "},\n";
	}
	void addDestination(warthog::search_node* node, std::string id, int32_t x, int32_t y, std::string pId){
		saveData << "{\"type\":\"destination\","
				 << "\"id\":\"" << id << "\",\"variables\":{\"x\":" << x << ",\"y\":" << y << "}"
				 << "}";
	}

	void endSearch(warthog::search_node* node, std::string id, int32_t x, int32_t y, std::string pId)
	{
			saveData<<",\n";

			saveData << "{\"type\":\"end\","
					 << "\"id\":\"" << id << "\",\"variables\":{\"x\":" << x << ",\"y\":" << y << "}";
		 saveData << "," << "\"g\":" << node->get_g() / (double)warthog::ONE << ","
	 				 << "\"f\":" << node->get_f() / (double)warthog::ONE;

 			if(node->get_parent() != -1)
 			{
 				saveData << "," << "\"pId\":\""<< pId << "\"";
 			}
 			else
 			{
 				saveData <<  "," << "\"pId\":null";
 			}
		 		// }
		 		saveData << "}";
	}
	void AddEvent(std::string type, warthog::search_node* node, std::string id, int32_t x, int32_t y, std::string pId)
	{
		saveData<<",\n";
		saveData << "{\"type\":\"" + type + "\","
				 << "\"id\":\"" << id << "\"";

		// if(type == generating || type == updating)
		// {
		// 	if(type == generating)
		// 	{
				saveData << ",\"variables\":{\"x\":" << x << ",\"y\":" << y << "}";
			// }


			saveData << "," << "\"g\":" << node->get_g() / (double)warthog::ONE << ","
				 << "\"f\":" << node->get_f() / (double)warthog::ONE;

			if(node->get_parent() != -1)
			{
				saveData << "," << "\"pId\":\""<< pId << "\"";
			}
			else
			{
				saveData <<  "," << "\"pId\":null";
			}
		// }
		saveData << "}";
	}

    void printToDebugFile()
	{
		std::ofstream outputfile; //open and save the file

		std::time_t t = time(0);
		struct tm * now = localtime(&t);
		std::string strFileName = "Debug:";
		strFileName.append(asctime(now));
		strFileName.append(".json");
		outputfile.open(strFileName);
		saveData << "\n]}"; //Add terminating curley bracket for Json

		std::string data = saveData.str();
		outputfile << data;//data.substr(0,data.length()-3);
		outputfile.close();
	}
};
#endif
