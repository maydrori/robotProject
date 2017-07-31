#include "../headers/ParticleManager.h"
#include <algorithm>

ParticleManager::ParticleManager(Map* map)
{
	// Create a particle according to start position
	Particle* start = new Particle(ConfigurationManager::Instance()->start().x, ConfigurationManager::Instance()->start().y, ConfigurationManager::Instance()->start().yaw);
	this->mParticles.push(start);

	// Create more like it
	while (this->mParticles.size() < MAX_PARTICLES)
	{
		this->mParticles.push(start->RandomCloseParticle(map));
	}
}

ParticleManager::~ParticleManager() {

}

void ParticleManager::ResampleParticles(Map* map)
{
	// Create maximum random particles
	// This increases the chance for a hit when we're out of ideas
	while (this->mParticles.size() < MAX_PARTICLES)
	{
		this->CreateRandomParticle(map);
	}
}

bool ParticleCompareBeliefs(Particle* a, Particle* b)
{
	return (a->belief() > b->belief());
}

Particle* ParticleManager::Update(HamsterAPI::Hamster* robot, Map* map, int deltaX, int deltaY, int deltaYaw)
{
//	cout << "Starting to update paritcle" <<  endl;
	vector<Particle*> remaining;
	Particle* best;

	while (this->mParticles.size() > 0)
	{
		// Get the current particle
		Particle* current = this->mParticles.top();
		this->mParticles.pop();

		// Update it
		current->Update(robot, map, deltaX, deltaY, deltaYaw);

		// Check deletion conditions
		// * removal threshold
		// * outside of the map
		// * on an occupied cell
		if (current->belief() < PARTICLE_REMOVAL_THRESHOLD ||
			current->getY() < 0 || current->getY() >= map->getHeight() ||
			current->getX() < 0 || current->getX() >= map->getWidth() ||
			map->getCell(current->getY(), current->getX()) == CELL_OCCUPIED)
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
		// TODO: Decide how to handle this shit
//		cout << "Resampling!";
		this->ResampleParticles(map);
		return NULL;
	}

//	cout << "Sorting" << endl;

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

	return (best);
}

void ParticleManager::CreateRandomParticle(Map* map)
{
	int nX;
	int nY;
	double nYaw;

	do
	{
		nX = rand() % map->getWidth();
		nY = rand() % map->getHeight();
	}
	while (map->getCell(nY, nX) == CELL_OCCUPIED);

	nYaw = (rand() % 3600) / 10;

	this->mParticles.push(new Particle(nX, nY, nYaw));
}
