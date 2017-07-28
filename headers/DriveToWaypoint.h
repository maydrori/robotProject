#ifndef __DRIVE_TO_WAYPOINT_H__
#define __DRIVE_TO_WAYPOINT_H__

#include "Behaviour.h"

#define ANGLE_COMPARISON_ACCEPTABLE_NOISE 	5	// must turn if not in this range
#define MOVE_WHILE_TURNING_ANGLE_THRESHOLD 	25	// can walk while turning if angle is in this range
#define MOVEMENT_SPEED 				 		0.3	// default movement speed (when walking straight)
#define TURNING_SPEED  				 		0.3	// turning speed
#define MOVEMENT_SPEED_WHILE_TURNING 		0.2	// movement speed while turning

class DriveToWaypoint : public Behaviour
{
	private:
		int mDstX;
		int mDstY;

		double GetCheapestAngleToTurn(double fTowardsAngle, Particle* best);
		bool IsFacingDirection(double fAngle, Particle* best);
		double GetAngleToWaypoint(Particle* best);
	public:
		DriveToWaypoint(HamsterAPI::Hamster* robot, int nDstX, int nDstY);

		void Action(Particle* best);
};

#endif
