/*
 * RobotManager.cpp
 *
 *  Created on: Jul 22, 2017
 *      Author: user
 */

#include <iostream>
#include "../headers/RobotManager.h"

using namespace std;

RobotManager::RobotManager(Map* mMap)
{
	this->map = mMap;
	this->mStartX = Configuration::Instance()->start().x;
	this->mStartY = Configuration::Instance()->start().y;
	this->mGoalX = Configuration::Instance()->goal().x;
	this->mGoalY = Configuration::Instance()->goal().y;
}

void RobotManager::Start()
{
	// Paint the start position in blue
	map->paintCell(470 , 437, 0,0,255);

	// Paint the goal position in green
	map->paintCell(472 , 470, 0,255,0);

	PathPlanner pln(*(map->blownGrid), 470, 437);
	Path path = pln.computeShortestPath(472, 470);

	// Paint the path in red
	for (int i = 0; i < path.size(); ++i) {

		map->paintCell(path[path.size() - 1 - i].first,
					  path[path.size() - 1 - i].second,
					  255,0,0);
	}

	Path smoothPath = getWaypoints(path);

	// Paint the path in blue
	for (int i = 0; i < smoothPath.size(); ++i) {

		map->paintCell(smoothPath[smoothPath.size() - 1 - i].first,
				smoothPath[smoothPath.size() - 1 - i].second,
					  0,0,255);
	}

//	// Printing
//			for (int i = 0; i < smoothPath.size(); ++i) {
//				cout << "(" << smoothPath[smoothPath.size() - 1 - i].first << ", " << smoothPath[smoothPath.size() - 1 - i].second << ") -> ";
//			}
}

Path RobotManager::getWaypoints(Path path) {

	// Smoothen the result and remove unnecessary waypoints
	int dx = 0;
	int dy = 0;
	Path smooth;

	// If we have a path, make it smooth
	if (path.size() > 0)
	{
		pair<int,int> last = path[0];

		for (int i = 1; i < path.size(); i++)
		{
			// Calculate the new deltas
			int dxNew = path[i].first - last.first;
			int dyNew = path[i].second - last.second;

			// If the deltas have changed, push the last into the vector
			if (dx != dxNew || dy != dyNew)
			{
				dx = dxNew;
				dy = dyNew;
				smooth.push_back(last);
			}

			last = path[i];
		}

		// Push last to the smooth result to finish it
		smooth.push_back(last);
	}

	return smooth;
}



