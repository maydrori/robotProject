#include "../headers/DriveToWaypoint.h"
#include <iostream>

#define sign(a) ((a) > 0 ? 1 : -1)

DriveToWaypoint::DriveToWaypoint(HamsterAPI::Hamster* robot, int nDstX, int nDstY)
	: Behaviour(robot)
{
	this->mDstX = nDstX;
	this->mDstY = nDstY;
}

void DriveToWaypoint::Action(Particle* best)
{
	double fAngleToWaypoint = this->GetAngleToWaypoint(best);

	// If the robot is facing towards the waypoint, drive straight
	if (this->IsFacingDirection(fAngleToWaypoint, best))
	{
		this->mRobot->sendSpeed(MOVEMENT_SPEED, 0);
	}
	else
	{
		// Get the angle we need to turn to
		double fCheapestAngle = this->GetCheapestAngleToTurn(fAngleToWaypoint, best);
		double fAbsAngle = abs(fCheapestAngle);

		// If the angle is positive, we need to turn left
		// If the angle is negative, we need to turn right
		// Get the multiplier which tells us which way to turn (multiply by robot's angular speed)
		int nTurnMultiplier = sign(fCheapestAngle);

		this->mRobot->sendSpeed(MOVEMENT_SPEED_WHILE_TURNING, fAbsAngle * nTurnMultiplier);
	}
}

double DriveToWaypoint::GetCheapestAngleToTurn(double fTowardsAngle, Particle* best)
{
	// Get the angles
	double fRobotAngle = best->getYaw();

	// Calculate turn cost for left turn and right turn
	double fLeft = fTowardsAngle - fRobotAngle;
	double fRight = fRobotAngle - fTowardsAngle;

	// Fix angles

	if (fLeft < 0)
	{
		fLeft += 360;
	}

	if (fRight < 0)
	{
		fRight += 360;
	}

	// Check which direction is better
	if (fLeft < fRight)
	{
		return (+fLeft);
	}
	else
	{
		return (-fRight);
	}
}

bool DriveToWaypoint::IsFacingDirection(double fAngle, Particle* best)
{
	// Check if robot's yaw is equal to (around) the calculated angle
	if (abs(fAngle - best->getYaw()) <= ANGLE_COMPARISON_ACCEPTABLE_NOISE)
	{
		return (true);
	}
	else
	{
		return (false);
	}
}

double DriveToWaypoint::GetAngleToWaypoint(Particle* best)
{
	// Calculate delta from waypoint to robot
	int dy = best->getY() - this->mDstY;
	int dx = this->mDstX - best->getX();

	// Calculate the angle between robot to waypoint
	double fAngleToWaypoint = toDeg(atan2(dy, dx));

	if (fAngleToWaypoint < 0)
	{
		fAngleToWaypoint += 360;
	}

	return (fAngleToWaypoint);
}
