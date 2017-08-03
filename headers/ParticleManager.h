#ifndef __PARTICLE_MANAGER_H__
#define __PARTICLE_MANAGER_H__

#include <stack>
#include "Particle.h"
#include "Map.h"
#include <HamsterAPIClientCPP/Hamster.h>

#define PARTICLE_REMOVAL_THRESHOLD 			0.4
#define PARTICLE_STRONG_SIGNAL_THRESHOLD	0.9
#define PARTICLE_INIT_COUNT					100
#define MAX_PARTICLES 						300
#define PARTICLE_DUPLICATION_COUNT			10

using namespace HamsterAPI;
using namespace std;

class ParticleManager
{
	private:
		Particle* mCurrentBest;
		stack<Particle*> mParticles;
		void CreateRandomParticles(Map* map, Particle* p);

	public:
		ParticleManager(Map* map);
		virtual ~ParticleManager();

		Particle* Update(HamsterAPI::Hamster* robot, Map* map, int deltaX, int deltaY, int deltaYaw);
};

#endif
