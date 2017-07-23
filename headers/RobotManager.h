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

class RobotManager
{
	private:
		Map* map;
		int mStartX;
		int mStartY;
		int mGoalX;
		int mGoalY;

	public:
		RobotManager(Map* map);
		void Start();
};

#endif /* HEADERS_ROBOTMANAGER_H_ */
