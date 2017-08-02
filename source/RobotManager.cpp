/*
 * RobotManager.cpp
 *
 *  Created on: Jul 22, 2017
 *      Author: user
 */

#include <iostream>
#include "../headers/RobotManager.h"
#include <HamsterAPIClientCPP/Hamster.h>

using namespace std;

RobotManager::RobotManager(HamsterAPI::Hamster* robot, Map* mMap)
{
	this->robot = robot;
	this->map = mMap;
	this->mStartX = ConfigurationManager::Instance()->start().x;
	this->mStartY = ConfigurationManager::Instance()->start().y;
	this->mGoalX = ConfigurationManager::Instance()->goal().x;
	this->mGoalY = ConfigurationManager::Instance()->goal().y;
	this->mWaypointManager = new WaypointManager(robot, map);
	this->mParticleManager = new ParticleManager(map);
}

void RobotManager::Start()
{
	this->mWaypointManager->SetDestination(this->mStartX, this->mStartY, this->mGoalX, this->mGoalY);

	//	Paint the start position in blue
	map->paintCell(mStartY , mStartX, 0,0,255);

	// Paint the goal position in green
	map->paintCell(mGoalY , mGoalX, 0,255,0);

	// Setting initial deltaX and delta Y
	int deltaX = 0;
	int deltaY = 0;
	int deltaYaw = 0;

	int currX = this->mStartX;
	int currY = this->mStartY;
	int currYaw = ConfigurationManager::Instance()->start().yaw;

	// Start the execution of the robot
	while (robot->isConnected())
	{
		this->map->show();

		// Update the particle manager and get the best particle
		Particle* best = this->mParticleManager->Update(this->robot, this->map, deltaX, deltaY, deltaYaw);
//		Particle* best = NULL;

		if (best)
		{
			map->paintCell(best->getY(), best->getX(), 0, 180, 0);
//			deltaX = abs(best->mX - currX);
//			deltaY = abs(best->mY - currY);
//			deltaYaw = abs(best->getYaw() - currYaw);

			deltaX = currX - best->getX();
			deltaY = currY - best->getY();
			deltaYaw = currYaw - best->getYaw();

			currX = best->mX;
			currY = best->mY;
			currYaw = best->getYaw();
			cout << "deltaX=" << deltaX << ", deltaY=" << deltaY << endl;
//			cout << "best=" << best->getX() << ", =" << best->getY() << endl;

//			cout << "I have best!! " << endl;
			// Update the waypoint manager
			this->mWaypointManager->Update(best, &deltaX, &deltaY, &deltaYaw);
			sleep(0.2);
		} else {
			deltaX = 0;
			deltaY = 0;
			deltaYaw = 0;
		}
	}
}



