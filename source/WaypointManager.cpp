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
	this->mRobot->sendSpeed(0, 0);

	// Perform A*
//	vector<Node*> path = this->mGraph->CalculatePath(nStartX, nStartY, nGoalX, nGoalY);
	PathPlanner pln(*(mMap->blownGrid), nStartY, nStartX);
	Path allPath = pln.computeShortestPath(nGoalY, nGoalX);
	Path path = getWaypoints(allPath);
cout << "finish a* and way points = " << path.size() << endl;

	// Paint the all path in red
	for (int i = 0; i < allPath.size(); ++i) {

		this->mMap->paintCell(allPath[allPath.size() - 1 - i].first,
				allPath[allPath.size() - 1 - i].second,
					  255,0,0);
	}

	// Paint the smooth points path in blue
	for (int i = 0; i < path.size(); ++i) {

		this->mMap->paintCell(path[path.size() - 1 - i].first,
				path[path.size() - 1 - i].second,
					  0,0,255);
	}

	this->mPaths.push(NULL);

	// Move the elements into the stack
	for (unsigned i = 0; i < path.size(); i++)
	{
		Node* node = new Node();
		node->row = path[i].first;
		node->col = path[i].second;
		this->mPaths.push(node);
	}

	this->NextTarget(false);
}

Path WaypointManager::getWaypoints(Path path) {

	// Smoothen the result and remove unnecessary waypoints
	int dRow = 0;
	int dCol = 0;
	Path smooth;

	// If we have a path, make it smooth
	if (path.size() > 0)
	{
		pair<int,int> last = path[0];
		int counter = 0;

		for (int i = 1; i < path.size(); i++)
		{
			// Calculate the new deltas
			int dRowNew = path[i].first - last.first;
			int dColNew = path[i].second - last.second;

			// If the deltas have changed, push the last into the vector
			if (dRow != dRowNew || dCol != dColNew || counter >= 10)
			{
				counter = 0;

				dRow = dRowNew;
				dCol = dColNew;
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
//		double fMapToGrid = Configuration::Instance()->gridResolution() / Configuration::Instance()->mapResolution();
//		int nX = best->x() / fMapToGrid;
//		int nY = best->y() / fMapToGrid;

		// Calculate deltas
		int dRow = abs(this->mCurrentTarget->row - best->getY());
		int dCol = abs(this->mCurrentTarget->col - best->getX());
		cout << "dRow=" << this->mCurrentTarget->row << "," << best->getY() << endl;
		cout << "dCOl=" << this->mCurrentTarget->col << "," << best->getX() << endl;

		int nAllowedRadius = ROBOT_REACHED_WAYPOINT_RADIUS;

		// If the current waypoint is the last waypoint, the robot must go to it exactly
		if (this->mPaths.size() == 0 || this->mPaths.top() == NULL)
		{
			nAllowedRadius = 0;
		}

		// Check if robot reached target cell in grid
		if (dRow <= nAllowedRadius && dCol <= nAllowedRadius)
		{
			this->NextTarget(true);
		}
	}

//	cout << "going to behave" << endl;
	this->mBehaviour->Action(best);
}

void WaypointManager::NextTarget(bool bHappy)
{
	cout << "NextTarget" << endl;
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
			cout << "StandInPosition" << endl;
			// Stop the robot
			this->SetBehaviour(new StandInPosition(this->mRobot));
		}
		else
		{
			cout << "Dance" << endl;
			// The robot is happy
			this->SetBehaviour(new Dance(this->mRobot));
		}
	}
	else
	{
		cout << "DriveToWayPoint" << endl;
//		double fResolution = Configuration::Instance()->gridResolution() / Configuration::Instance()->mapResolution();
//		int x = next->x() * fResolution;
//		int y = next->y() * fResolution;
		this->SetBehaviour(new DriveToWaypoint(this->mRobot, next->col, next->row));
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
