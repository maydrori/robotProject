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
	int currYaw = 0;

	// Start the execution of the robot
	while (robot->isConnected())
	{
		this->map->show();

//		 Update the particle manager and get the best particle
		Particle* best = this->mParticleManager->Update(this->robot, this->map, deltaX, deltaY, deltaYaw);

		if (best)
		{
			map->paintCell(best->getX(), best->getY(), 0, 180, 0);
			deltaX = best->mX - currX;
			deltaY = best->mY - currY;
			deltaYaw = best->getYaw() - currYaw;

			currX = best->mX;
			currY = best->mY;
			currYaw = best->getYaw();
			cout << "deltaX=" << deltaX << ", deltaY=" << deltaY;
//			cout << "currX=" << currX << ", currY=" << currY << endl;

//			cout << "I have best!! " << endl;
			// Update the waypoint manager
			this->mWaypointManager->Update(best, &deltaX, &deltaY, &deltaYaw);
		}

		sleep(0.5);
	}
}



