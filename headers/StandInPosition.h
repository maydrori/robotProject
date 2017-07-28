#ifndef __STAND_IN_POSITION_H__
#define __STAND_IN_POSITION_H__

#include "Behaviour.h"

class StandInPosition : public Behaviour
{
	public:
		StandInPosition(Robot* robot) : Behaviour(robot) { }

		void Action(Particle* best)
		{
			this->mRobot->SetSpeed(0, 0);
		}
};

#endif
