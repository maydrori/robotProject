#ifndef __WAYPOINT_MANAGER_H__
#define __WAYPOINT_MANAGER_H__

#define ROBOT_REACHED_WAYPOINT_RADIUS 2

#include <stack>
#include "Map.h"
#include "Node.h"
#include "Behaviour.h"
#include "Dance.h"
#include "DriveToWaypoint.h"
#include "StandInPosition.h"
#include "PathPlanner.h"

class WaypointManager
{
	private:
		stack<Node*> mPaths;
		HamsterAPI::Hamster* mRobot;
		Map* mMap;
//		Graph* mGraph;
		Node* mCurrentTarget;
		Behaviour* mBehaviour;

		void NextTarget(bool bHappy);
		void SetBehaviour(Behaviour* behaviour);

	public:
		WaypointManager(HamsterAPI::Hamster* robot, Map* map);

		void SetDestination(int nStartX, int nStartY, int nGoalX, int nGoalY);
		void Update(Particle* best);
		Path getWaypoints(Path path);
};

#endif
