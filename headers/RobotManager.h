/*
 * RobotManager.h
 *
 *  Created on: Jul 22, 2017
 *      Author: user
 */

#ifndef HEADERS_ROBOTMANAGER_H_
#define HEADERS_ROBOTMANAGER_H_

#include "Map.h"
#include "PathPlanner.h"
#include "WaypointManager.h"
#include "ParticleManager.h"

class RobotManager
{
	private:
		HamsterAPI::Hamster* robot;
		Map* map;
		int mStartX;
		int mStartY;
		int mGoalX;
		int mGoalY;

	public:
		WaypointManager* mWaypointManager;
		ParticleManager* mParticleManager;
		RobotManager(HamsterAPI::Hamster* robot, Map* map);
		void Start();
};

#endif /* HEADERS_ROBOTMANAGER_H_ */
