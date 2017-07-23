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
}



