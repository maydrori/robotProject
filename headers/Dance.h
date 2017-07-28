#ifndef __DANCE_H__
#define __DANCE_H__

#include "Behaviour.h"

class Dance : public Behaviour
{
	public:
		Dance(HamsterAPI::Hamster* robot) : Behaviour(robot) { }

		void Action(Particle* best)
		{
			this->mRobot->SetSpeed(0, 5);
		}
};

#endif
