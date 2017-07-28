#ifndef __BEHAVIOUR_H__
#define __BEHAVIOUR_H__

#include "Particle.h"
#include <HamsterAPIClientCPP/Hamster.h>

class Behaviour
{
	protected:
	HamsterAPI::Hamster* mRobot;

	public:
		Behaviour(HamsterAPI::Hamster* robot) : mRobot(robot) { }
		virtual ~Behaviour() { this->mRobot = NULL; }

		virtual void Action(Particle* best) = 0;
};

#endif
