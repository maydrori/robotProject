#include "../headers/ParticleManager.h"
#include <algorithm>

ParticleManager::ParticleManager(Map* map)
{
	Particle* start = new Particle(ConfigurationManager::Instance()->start().x, ConfigurationManager::Instance()->start().y, ConfigurationManager::Instance()->start().yaw);
	this->mCurrentBest = start;
	this->mParticles.push(start);

	this->CreateRandomParticles(map, start);
}

void ParticleManager::CreateRandomParticles(Map* map, Particle* p)
{
	// Create more like it
	while (mParticles.size() < MAX_PARTICLES)
	{
		this->mParticles.push(p->RandomCloseParticle(map));
	}
}

ParticleManager::~ParticleManager() {

}

bool ParticleCompareBeliefs(Particle* a, Particle* b)
{
	return (a->belief() > b->belief());
}

Particle* ParticleManager::Update(HamsterAPI::Hamster* robot, Map* map, int deltaX, int deltaY, int deltaYaw)
{
	vector<Particle*> remaining;
	Particle* best;

	HamsterAPI::LidarScan scan = robot->getLidarScan();

	while (this->mParticles.size() > 0)
	{
		// Get the current particle
		Particle* current = this->mParticles.top();
		this->mParticles.pop();

		// Update it
		current->Update(scan, map, deltaX, deltaY, deltaYaw);

		// Check deletion conditions
		// * removal threshold
		// * outside of the map
		// * on an occupied cell
		if (current->belief() < PARTICLE_REMOVAL_THRESHOLD ||
			current->getY() < 0 || current->getY() >= map->getHeight() ||
			current->getX() < 0 || current->getX() >= map->getWidth() ||
			map->getCell(current->getY(), current->getX()) != CELL_FREE )
		{
			delete current;
			continue;
		}
		else
		{
			remaining.push_back(current);
		}
	}

	// If nothing remains then we need to resample (which is very bad)
	if (remaining.size() == 0)
	{
		this->CreateRandomParticles(map, mCurrentBest);
		return NULL;
	}

	// Sort the remaining particles vector by highest priority
	std::sort(remaining.begin(), remaining.end(), ParticleCompareBeliefs);

	// The best particle should be the first particle in the vector
	best = remaining.front();

	// Loop through the remaining particles and add them to the particle stack
	for (int i = 0; i < remaining.size() && this->mParticles.size() < MAX_PARTICLES; i++)
	{
		// If the current particle is strongly believable, then it should give birth to more like it
		if (remaining[i]->belief() >= PARTICLE_STRONG_SIGNAL_THRESHOLD)
		{
			for (int birth = 0; birth < PARTICLE_DUPLICATION_COUNT && this->mParticles.size() < MAX_PARTICLES; birth++)
			{
				this->mParticles.push(remaining[i]->RandomCloseParticle(map));
			}
		}
	}

	// In case we're low on particles, give birth to more like the best one
	// This should increase the chance to get the location and reduce the chance of resampling
	while (this->mParticles.size() < MAX_PARTICLES)
	{
		this->mParticles.push(best->RandomCloseParticle(map));
	}

	// Saving current best
	this->mCurrentBest = best;

	return (best);
}

