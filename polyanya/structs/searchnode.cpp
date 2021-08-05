#include "searchnode.h"

std::string polyanya::getSearchNodeId(polyanya::SearchNode* node)
{
	std::stringstream nodeId;
	nodeId << node->root << node->left << node->right;
	return nodeId.str();
}
