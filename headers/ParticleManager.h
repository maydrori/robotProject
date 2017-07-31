#ifndef __PARTICLE_MANAGER_H__
#define __PARTICLE_MANAGER_H__

#include <stack>
#include "Particle.h"
#include "Map.h"
#include <HamsterAPIClientCPP/Hamster.h>

#define PARTICLE_REMOVAL_THRESHOLD 			0.3
#define PARTICLE_STRONG_SIGNAL_THRESHOLD	0.7
#define PARTICLE_INIT_COUNT					100
#define MAX_PARTICLES 						200
#define PARTICLE_DUPLICATION_COUNT			10

using namespace HamsterAPI;
using namespace std;

class ParticleManager
{
	private:
		stack<Particle*> mParticles;

	public:
		ParticleManager(Map* map);
		virtual ~ParticleManager();

		Particle* Update(HamsterAPI::Hamster* robot, Map* map, int deltaX, int deltaY, int deltaYaw);
		void ResampleParticles(Map* map);
		void CreateRandomParticle(Map* map);
};

#endif
