/*
 * RobotManager.cpp
 *
 *  Created on: Jul 22, 2017
 *      Author: user
 */

#include <iostream>
#include "../headers/RobotManager.h"
#include <HamsterAPIClientCPP/Hamster.h>
#include <thread>

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
	this->pose = NULL;
}

void RobotManager::printRobotPoseInterval() {
	thread th([&]() {
		while(true) {
			std::this_thread::__sleep_for(chrono::seconds(3), chrono::nanoseconds(0));
				if (pose) {
					cout << "Robot position: (" <<
						pose->getX() << "," <<
						pose->getY() << "," <<
						pose->getYaw() << ")" << endl;
				}
		}
	});
	th.detach();
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

	printRobotPoseInterval();

	// Start the execution of the robot
	while (robot->isConnected())
	{
		this->map->show();

		// Get the current position of the robot
		Pose robotPose = robot->getPose();

		// Calculate the pose on the grid
		int pixelX = (robotPose.getX() / map->blownGrid.getResolution()) + (map->getWidth() / 2) - 42;
		int pixelY = (map->getHeight() / 2) - (robotPose.getY() / map->blownGrid.getResolution()) - 42;

		// Update the particle manager and get the best particle
		pose = new Particle(pixelX, pixelY, robotPose.getHeading());
//		Particle* best = this->mParticleManager->Update(this->robot, this->map, deltaX, deltaY, deltaYaw);

		if (pose)
		{
			map->paintCell(pose->getY(), pose->getX(), 0, 180, 0);

			deltaX = currX - pose->getX();
			deltaY = currY - pose->getY();
			deltaYaw = currYaw - pose->getYaw();

			currX = pose->getX();
			currY = pose->getY();
			currYaw = pose->getYaw();

			// Update the waypoint manager
			this->mWaypointManager->Update(pose);

			sleep(0.2);
		} else {
			deltaX = 0;
			deltaY = 0;
			deltaYaw = 0;
		}
	}
}







