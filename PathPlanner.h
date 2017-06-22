/*
 * PathPlanner.h
 *
 *  Created on: Jun 17, 2017
 *      Author: user
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <HamsterAPIClientCPP/Hamster.h>
#include <queue>
#include "Node.h"

using namespace HamsterAPI;
using namespace std;

typedef vector<pair<int,int> > Path;

class PathPlanner {
private:
	OccupancyGrid& grid;
	int startRow, startCol;
	vector<vector<Node*> > mat;

	void buildGraph();
	vector<Node*> getSuccessors(Node* node);
	bool areNodesEqual(Node* n1, Node* n2);
	double calculateHValue(Node* source, Node* dest);
	Path reconstructPath(Node* dest);

public:
	PathPlanner(OccupancyGrid& grid, int startRow, int startCol);
	Path computeShortestPath(int destRow, int destCol);
	virtual ~PathPlanner();
};

#endif /* PATHPLANNER_H_ */
