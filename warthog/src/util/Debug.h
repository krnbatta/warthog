#ifndef WARTHOG_DEBUG
#define WARTHOG_DEBUG
#include <iostream>
#include <sstream>
#include <fstream>
#include <ctime>
#include "gridmap.h"
#include "search_node.h"

enum EventType
{
	start,
	expanding,
	generating,
	updating,
	closing,
	end
};
template <class E> // E is expander
class Debugger
{
private:
	int eventCount = 0;
	std::string typeStrings[6] = {"start","expanding","generating","updating","closing","end"};
	std::stringstream saveData;
	E* expander_;
	uint32_t width;
	int32_t end_x, end_y;
public:
    Debugger(){
		saveData << "{\"nodeStructure\":[{\"type\": \"rectangle\", \"variables\": {\"x\": \"x\", \"y\": \"y\"}, \"persisted\": true, \"drawPath\": true}], " ;
		saveData << "\"eventList\" : [ \n";

    }

	Debugger(E* expander)
	{
		expander_ = expander;
		saveData << "{\"nodeStructure\": [{\"type\": \"rectangle\", \"variables\": {\"x\": \"x\", \"y\": \"y\"}, \"persisted\": true, \"drawPath\": true}],\n " ;
		saveData << "\"eventList\":[ \n";
    //     width = getMapWidth(expander_->getMapFileName());
		// std::cerr << " : " << getMapWidth(expander_->getMapFileName()) << std::endl;
    }
    void startSearch(uint32_t start,uint32_t end)
	{
	int32_t x,y;
		if(eventCount != 0)
		{
			saveData <<", \n";
			expander_-> get_xy(start,x,y);

			saveData << "{\"type\":\"source\","
					 << "\"id\":" << start << ",\"variables\":{\"x\":" << x << ",\"y\":" << y << "}"
					 << "},\n";
		}
	  	expander_-> get_xy(end,x,y);

			end_x = x;
			end_y = y;

		  saveData << "{\"type\":\"destination\","
					 << "\"id\":" << end << ",\"variables\":{\"x\":" << x << ",\"y\":" << y << "}"
					 << "}";

		eventCount++;
	}

	void endSearch(uint32_t end)
	{
		if(eventCount != 0)
		{
			saveData<<",\n";

			saveData << "{\"type\":\"end\","
					 << "\"id\":" << end << ",\"variables\":{\"x\":" << end_x << ",\"y\":" << end_y << "}"
					 << "}";
		}
	}
	uint32_t realId(uint32_t paddedId)
	{
	    uint32_t x,y;
		expander_-> unpadCoordinate(paddedId,x,y);
		return y*width + x;
	}
	void AddEvent(EventType type, warthog::search_node* node)
	{
		int32_t x,y;
		expander_-> get_xy(node->get_id(),x,y);
		if(eventCount != 0)
		{
			saveData<<",\n";
		}
		saveData << "{\"type\":\"" + typeStrings[type] + "\","
				 << "\"id\":" << node->get_id();

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
				saveData << "," << "\"pId\":"<< node->get_parent();
			}
			else
			{
				saveData <<  "," << "\"pId\":null";
			}
		// }
		saveData << "}";
		eventCount++;
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
    uint32_t getMapWidth(const char* fileName)
    {
        uint32_t fileWidth;
        std::string line;
		std::ifstream file;
		std::stringstream widthStream;
		file.open(fileName);
		for(int i = 0 ; i < 3; i++)
		{
			getline(file,line);
			if(i == 2)
				widthStream << line.substr(6,line.size());
		}
		file.close();
		widthStream >> fileWidth;
		return fileWidth;
    }
	std::string getMapString(const char* fileName)
	{
		std::string line;
		std::ifstream file;
		std::stringstream mapString;
		file.open(fileName);
		for(int i = 0 ; i < 4; i++)
		{
			getline(file,line);
			if(i == 1)
				mapString << "{ \"mHeight\" : "<<line.substr(7,line.size()) <<" , ";
			if(i == 2)
				mapString << " \"mWidth\" : " << line.substr(6,line.size()) << " , ";
		}
		mapString << " \"mapData\" : \"";
		while( getline( file, line))
		{
			if(line[line.size()-1] == '\n')
				mapString << line.substr(0,line.size());
			else
				mapString << line;
		}
		mapString << "\" }";
		file.close();
		return mapString.str();
	}
};
#endif
