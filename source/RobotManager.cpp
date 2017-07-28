/*
 * RobotManager.cpp
 *
 *  Created on: Jul 22, 2017
 *      Author: user
 */

#include <iostream>
#include "../headers/RobotManager.h"

using namespace std;

RobotManager::RobotManager(HamsterAPI::Hamster* robot, Map* mMap)
{
	this->map = mMap;
	this->mStartX = Configuration::Instance()->start().x;
	this->mStartY = Configuration::Instance()->start().y;
	this->mGoalX = Configuration::Instance()->goal().x;
	this->mGoalY = Configuration::Instance()->goal().y;
	this->mWaypointManager = new WaypointManager(robot, map);
}

//void RobotManager::Start()
//{
//	// Paint the start position in blue
//	map->paintCell(470 , 437, 0,0,255);
//
//	// Paint the goal position in green
//	map->paintCell(472 , 470, 0,255,0);
//
//	PathPlanner pln(*(map->blownGrid), 470, 437);
//	Path path = pln.computeShortestPath(472, 470);
//
//	// Paint the path in red
//	for (int i = 0; i < path.size(); ++i) {
//
//		map->paintCell(path[path.size() - 1 - i].first,
//					  path[path.size() - 1 - i].second,
//					  255,0,0);
//	}
//
//	Path smoothPath = getWaypoints(path);
//
//	// Printing
//	for (int i = 0; i < smoothPath.size(); ++i) {
//		cout << "(" << smoothPath[smoothPath.size() - 1 - i].first << ", " << smoothPath[smoothPath.size() - 1 - i].second << ") -> ";
//	}
//
//	// Paint the path in blue
//	for (int i = 0; i < smoothPath.size(); ++i) {
//
//		map->paintCell(smoothPath[smoothPath.size() - 1 - i].first,
//				smoothPath[smoothPath.size() - 1 - i].second,
//					  0,0,255);
//	}
//}

void RobotManager::Start()
{
	this->mWaypointManager->SetDestination(this->mStartX, this->mStartY, this->mGoalX, this->mGoalY);

	// Start the execution of the robot
	while (true)
	{
		// Read values from the robot
//		this->mRobot->Read();

		// Update the particle manager and get the best particle
		Particle* best = Particle(robot->getPose()->getX(), robot->getPose()->getY(), robot->getPose()->getAngle());
//				this->mParticleManager->Update(this->mRobot, this->mMap);

		// Update the waypoint manager
		this->mWaypointManager->Update(best);
	}
}



