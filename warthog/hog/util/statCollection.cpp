/*
 * $Id: statCollection.cpp,v 1.24 2006/11/07 20:52:26 nathanst Exp $
 *
 *  statCollection.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/1/05.
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

#include "statCollection.h"
#include <cstdio>
#include <cstring>


// typedef union { double fval; long lval; } statValue;

//class stat {
//public:
//	int category, owner;
//	statValue value;
//};

//std::vector<char *> categories;
//std::vector<char *> owners;
//std::vector<stat> stats;

statCollection::statCollection() :categories(), owners(), stats() { printOutput = false; } statCollection::~statCollection() { for (unsigned int x = 0; x < categories.size(); x++) { delete [] categories[x]; categories[x] = 0; } for (unsigned int x = 0; x < owners.size(); x++) { delete [] owners[x]; owners[x] = 0; } for (unsigned int x = 0; x < excludeFilters.size(); x++) { delete [] excludeFilters[x]; excludeFilters[x] = 0; } for (unsigned int x = 0; x < includeFilters.size(); x++) { delete [] includeFilters[x]; includeFilters[x] = 0; } } /** * Add a new stat entry for the given category, owner and value.  */ void statCollection::addStat(const char *category, const char *owner, double value) { if (!passFilter(category)) return; int catID, ownerID; catID = addCategory(category); ownerID = addOwner(owner); stats.resize(stats.size()+1);
	stats[stats.size()-1].category = catID;
	stats[stats.size()-1].owner = ownerID;
	stats[stats.size()-1].value.fval = value;
	stats[stats.size()-1].sType = floatStored;
	
	if (printOutput)
		printf("%s\t%s\t%1.2f\n", category, owner, value);
}

/**
* Add a new stat entry for the given category, owner and value.
 */
void statCollection::addStat(const char *category, const char *owner, long value)
{
	if (!passFilter(category))
		return;
	int catID, ownerID;
	catID = addCategory(category);
	ownerID = addOwner(owner);
	stats.resize(stats.size()+1);
	stats[stats.size()-1].category = catID;
	stats[stats.size()-1].owner = ownerID;
	stats[stats.size()-1].value.lval = value;
	stats[stats.size()-1].sType = longStored;
	if (printOutput)
		printf("%s\t%s\t%ld\n", category, owner, value);
}

/**
* Given stats for the category and owner, find an existing stat (chronologically
																 * backwards search) with the same category and owner, and add this stat to the existing
 * value. If the stat doesn't exist with the same category and owner doesn't exist,
 * a new one will be initialized with the given value.
 */
void statCollection::sumStat(const char *category, const char *owner, double value)
{
	statValue *sv = getLastStat(category, owner);
	if (sv)
	{
//		if (sv->sType != floatStored)
//			printf("Warning: Adding double to value previous stored as long\n");
		sv->fval += value;
		if (printOutput)
			printf("%s\t%s\t%1.2f\n", category, owner, sv->fval);
	}
	else
		addStat(category, owner, value);
}

/**
* Given stats for the category and owner, find an existing stat (chronologically
																 * backwards search) with the same category and owner, and add this stat to the existing
 * value. If the stat doesn't exist with the same category and owner doesn't exist,
 * a new one will be initialized with the given value.
 */
void statCollection::sumStat(const char *category, const char *owner, long value)
{
	statValue *sv = getLastStat(category, owner);
	if (sv)
	{
//		if (sv->sType != floatStored)
//			printf("Warning: Adding long to value previous stored as double\n");
		sv->lval += value;
		if (printOutput)
			printf("%s\t%s\t%ld\n", category, owner, sv->lval);
	}
	else
		addStat(category, owner, value);
}

/**
* Remove all stat entries from the collection.
 */
void statCollection::clearAllStats()
{
	stats.resize(0);
}

//void statCollection::clearOwnerStats(char *owner);
//void statCollection::clearCategoryStats(char *category);
/**
* The number of stats collected so far
 */
int statCollection::getNumStats() const
{
	return (int)stats.size();
}

int statCollection::getNumOwners() const
{
	return (int)owners.size();
}

/**
* Return the nth stat which has been collected.
 */
const stat *statCollection::getStatNum(int which) const
{
	return &stats[which];
}

/**
* Given a category ID, return the text description.
 */
const char *statCollection::lookupCategoryID(int id) const
{
	return categories[id];
}

/**
* Given a owner ID, return the text description.
 */
const char *statCollection::lookupOwnerID(int id) const
{
	return owners[id];
}

/**
* Adding a filter will cause only stats of the given category to be
 * collected. All other stats added will be ignored. As many categories
 * can be added as needed.
 */
void statCollection::addFilter(char *category)
{
	addIncludeFilter(category);
}

void statCollection::addIncludeFilter(char *category) // include only added categories
{
	char *str = new char [strlen(category)+1];
	strcpy(str, category);
	includeFilters.push_back(str);
}

void statCollection::addExcludeFilter(char *category) // exclude only added categories
{
	char *str = new char [strlen(category)+1];
	strcpy(str, category);
	excludeFilters.push_back(str);
}

/**
* Clear any filters being used for stat entry.
 */
void statCollection::clearFilters()
{
	for (unsigned int x = 0; x < excludeFilters.size(); x++)
	{
			delete [] excludeFilters[x];
			excludeFilters[x] = 0;
	}
	for (unsigned int x = 0; x < includeFilters.size(); x++)
	{
			delete [] includeFilters[x];
			includeFilters[x] = 0;
	}
	excludeFilters.resize(0);
	includeFilters.resize(0);
}

/**
* Given a category, look up the ID. O(# categories) operation. If not found, returns -1.
 */
int statCollection::lookupCategory(const char *category) const
{
	for (unsigned int x = 0; x < categories.size(); x++)
		if (strcmp(category, categories[x]) == 0)
			return x;
	return -1;
}


/**
* Add a new category to the category list. If the category exists, returns the id.
 * Otherwise creates the category and returns the id.
 */
int statCollection::addCategory(const char *category)
{
	int id = lookupCategory(category);
	if (id != -1)
		return id;
	char *str = new char [strlen(category)+1];
	strcpy(str, category);
	categories.push_back(str);
	return categories.size()-1;
}

/**
* Given an owner, look up the ID. O(# owners) operation. If not found, returns -1.
 */
int statCollection::lookupOwner(const char *owner) const
{
	for (unsigned int x = 0; x < owners.size(); x++)
		if (strcmp(owner, owners[x]) == 0)
			return x;
	return -1;
}

/**
* Add a new owner to the owner list. If the owner exists, returns the id.
 * Otherwise creates the owner and returns the id.
 */
int statCollection::addOwner(const char *owner)
{
	int id = lookupOwner(owner);
	if (id != -1)
		return id;
	char *str = new char [strlen(owner)+1];
	strcpy(str, owner);
	owners.push_back(str);
	return owners.size()-1;
}

/**
* Find the last stat entered that matches the category and owner. Returns
 * copy of stat entry. Returns true if a stat was found and false otherwise.
 */
bool statCollection::lookupStat(const char *category, const char *owner, statValue &v) const
{
	if (!passFilter(category))
	{
		return false;
	}
	int catID, ownerID;
	catID = lookupCategory(category);
	ownerID = lookupOwner(owner);
	for (int x = (int)stats.size()-1; x >= 0; x--)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
		{
			v = stats[x].value;
			return true;
		}
	}
	return false;
}

bool statCollection::lookupStat(unsigned int index, statValue &v) const
{
	if (index < stats.size())
	{
		v = stats[index].value;
		return true;
	}
	return false;
}

/**
* Find the last stat entered that matches the category and owner. Returns pointer
 * to entry.
 */
statValue *statCollection::getLastStat(const char *category, const char *owner)
{
	if (!passFilter(category))
		return 0;
	int catID, ownerID;
	catID = lookupCategory(category);
	ownerID = lookupOwner(owner);
	for (int x = (int)stats.size()-1; x >= 0; x--)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
			return &stats[x].value;
	}
	return 0;
}

/**
* Check to see if the category stats should be saved.
 */
bool statCollection::passFilter(const char *category) const
{
	if ((includeFilters.size() == 0) && (excludeFilters.size() == 0))
		return true;

	for (unsigned int x = 0; x < excludeFilters.size(); x++)
		if (strcmp(excludeFilters[x], category) == 0)
			return false;

	if (includeFilters.size() == 0)
		return true;

	for (unsigned int x = 0; x < includeFilters.size(); x++)
		if (strcmp(includeFilters[x], category) == 0)
			return true;

	return false;
}

/**
* Find the next stat entry that matches the given category and owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int statCollection::findNextStat(const char *category, const char *owner, int startIndex) const
{
	// Check if the category has been registered
	if (!passFilter(category))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to zero
	if (startIndex < 0 || startIndex > (int)stats.size()-1)
		startIndex = 0;
	
	int catID, ownerID;
	catID = lookupCategory(category);
	ownerID = lookupOwner(owner);
	
	for (int x = startIndex; x < (int)stats.size(); x++)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
			return x;
	}
	
	return -1;     
}

/**
* Find the previous stat entry that matches the given category and owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int statCollection::findPrevStat(const char *category, const char *owner, int startIndex) const
{
	// Check if the category has been registered
	if (!passFilter(category))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to the last
	// index
	if (startIndex < 0 || startIndex > (int)stats.size() - 1)
		startIndex = (int) stats.size() -1;
	
	int catID, ownerID;
	catID = lookupCategory(category);
	ownerID = lookupOwner(owner);
	
	for (int x = startIndex; x > 0; x--)
	{
		if ((stats[x].category == catID) && (stats[x].owner == ownerID))
			return x;
	}
	
	return -1;
}

/**
* Find the next stat entry that matches the category if findCategory is true, otherwise
 * look to match the owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int statCollection::findNextStat(const char *what, bool findCategory, int startIndex) const
{
	// Check if the category has been registered
	if (findCategory && (!passFilter(what)))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to zero
	if (startIndex < 0 || startIndex > (int)stats.size()-1)
		startIndex = 0;
	
	int ID;
	if (findCategory)
		ID = lookupCategory(what);
	else
		ID = lookupOwner(what);
	
	for (int x = startIndex; x < (int)stats.size(); x++)
	{
		if ((findCategory && (stats[x].category == ID)) ||
				(!findCategory && (stats[x].owner == ID)))
			return x;
	}
	
	return -1;     
}

/**
* Find the next stat entry that matches the category if findCategory is true, otherwise
 * look to match the owner name. If error occurs,
 * return -1; otherwise, return the found index.
 */
int statCollection::findPrevStat(const char *what, bool findCategory, int startIndex) const
{
	// Check if the category has been registered
	if (findCategory && (!passFilter(what)))
		return -1;
	
	// If the given startIndex is out of range of the stats table size, reset it to the last
	// index
	if (startIndex < 0 || startIndex > (int)stats.size() - 1)
		startIndex = (int) stats.size() -1;
	
	int ID;
	if (findCategory)
		ID = lookupCategory(what);
	else
		ID = lookupOwner(what);
	
	for (int x = startIndex; x > 0; x--)
	{
		if ((findCategory && (stats[x].category == ID)) ||
				(!findCategory && (stats[x].owner == ID)))
			return x;
	}
	
	return -1;
}


/*
 * Print the Stats Table for debugging purpose for now.
 */
void statCollection::printStatsTable() const
{
	printf("Stats Table:\n----------------------------\n");
	
	for (int x = 0; x < (int)stats.size(); x++)
		if (stats[x].sType == floatStored)
			printf("%d \t%s \t%s \t%e\n", x, categories[stats[x].category],
						 owners[stats[x].owner], stats[x].value.fval);
		else
			printf("%d \t%s \t%s \t%ld\n", x, categories[stats[x].category],
						 owners[stats[x].owner],stats[x].value.lval);
}
