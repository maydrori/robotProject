#include "../headers/WaypointManager.h"
#include <iostream>
using namespace std;

WaypointManager::WaypointManager(HamsterAPI::Hamster* robot, Map* map)
{
	this->mRobot = robot;
	this->mMap = map;
//	this->mGraph = graph;
	this->mCurrentTarget = NULL;
	this->mBehaviour = new StandInPosition(robot);
}

void WaypointManager::SetDestination(int nStartX, int nStartY, int nGoalX, int nGoalY)
{
	// Clear the stack
	while (!this->mPaths.empty())
	{
		this->mPaths.pop();
	}

	// Halt robot while we're computing stuff
	this->mRobot->SetSpeed(0, 0);

	// Perform A*
//	vector<Node*> path = this->mGraph->CalculatePath(nStartX, nStartY, nGoalX, nGoalY);
	PathPlanner pln(*(map->blownGrid), 470, 437);
	Path path = getWaypoints(pln.computeShortestPath(472, 470));

	this->mPaths.push(NULL);

	// Move the elements into the stack
	for (unsigned i = 0; i < path.size(); i++)
	{
		if (path[i] != NULL)
		{
			this->mPaths.push(path[i]);
		}
	}

	this->NextTarget(false);
}

Path WaypointManager::getWaypoints(Path path) {

	// Smoothen the result and remove unnecessary waypoints
	int dx = 0;
	int dy = 0;
	Path smooth;

	// If we have a path, make it smooth
	if (path.size() > 0)
	{
		pair<int,int> last = path[0];
		int counter = 0;

		for (int i = 1; i < path.size(); i++)
		{
			// Calculate the new deltas
			int dxNew = path[i].first - last.first;
			int dyNew = path[i].second - last.second;

			// If the deltas have changed, push the last into the vector
			if (dx != dxNew || dy != dyNew || counter >= 10)
			{
				counter = 0;

				dx = dxNew;
				dy = dyNew;
				smooth.push_back(last);
			}
			else ++counter;

			last = path[i];
		}

		// Push last to the smooth result to finish it
		smooth.push_back(last);
	}

	return smooth;
}

void WaypointManager::Update(Particle* best)
{
	if (this->mCurrentTarget != NULL)
	{
		// Translate particle position to grid
		double fMapToGrid = Configuration::Instance()->gridResolution() / Configuration::Instance()->mapResolution();
		int nX = best->x() / fMapToGrid;
		int nY = best->y() / fMapToGrid;

		// Calculate deltas
		int dx = abs(this->mCurrentTarget->x() - nX);
		int dy = abs(this->mCurrentTarget->y() - nY);

		int nAllowedRadius = ROBOT_REACHED_WAYPOINT_RADIUS;

		// If the current waypoint is the last waypoint, the robot must go to it exactly
		if (this->mPaths.size() == 0 || this->mPaths.top() == NULL)
		{
			nAllowedRadius = 0;
		}

		// Check if robot reached target cell in grid
		if (dx <= nAllowedRadius && dy <= nAllowedRadius)
		{
			this->NextTarget(true);
		}
	}

	this->mBehaviour->Action(best);
}

void WaypointManager::NextTarget(bool bHappy)
{
	Node* next = NULL;

	// If there are places to be, we shall go to the next place
	if (!this->mPaths.empty())
	{
		next = this->mPaths.top();
		this->mPaths.pop();
	}

	// If there are no places to be (or we've reached the end)
	if (next == NULL)
	{
		if (!bHappy)
		{
			// Stop the robot
			this->SetBehaviour(new StandInPosition(this->mRobot));
		}
		else
		{
			// The robot is happy
			this->SetBehaviour(new Dance(this->mRobot));
		}
	}
	else
	{
		double fResolution = Configuration::Instance()->gridResolution() / Configuration::Instance()->mapResolution();
		int x = next->x() * fResolution;
		int y = next->y() * fResolution;
		this->SetBehaviour(new DriveToWaypoint(this->mRobot, x, y));
	}

	this->mCurrentTarget = next;
}

void WaypointManager::SetBehaviour(Behaviour* behaviour)
{
	if (this->mBehaviour != NULL)
	{
		delete this->mBehaviour;
	}

	this->mBehaviour = behaviour;
}
