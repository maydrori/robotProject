#include "../headers/ParticleManager.h"
#include <algorithm>

ParticleManager::ParticleManager(Map* map)
{
//	CreateRandomParticle(map);
	Particle* start = new Particle(ConfigurationManager::Instance()->start().x, ConfigurationManager::Instance()->start().y, ConfigurationManager::Instance()->start().yaw);
	this->mCurrentBest = start;
	this->init(map, start);
//	ResampleParticles(map);
}

void ParticleManager::init(Map* map, Particle* p)
{
	// Create a particle according to start position
//	Particle* start = new Particle(ConfigurationManager::Instance()->start().x, ConfigurationManager::Instance()->start().y, ConfigurationManager::Instance()->start().yaw);
//	this->mParticles = stack<Particle*>();

	// Create more like it
	while (mParticles.size() < MAX_PARTICLES)
	{
		Particle* pNew = p->RandomCloseParticle(map);
//		map->paintCell(pNew->getY(), pNew->getX(), 0, 100, 0);
		this->mParticles.push(pNew);
	}

//	this->mParticles.push(p);
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
	vector<Particle*> remaining;
	Particle* best;

	HamsterAPI::LidarScan scan = robot->getLidarScan();

	while (this->mParticles.size() > 0)
	{
		// Get the current particle
		Particle* current = this->mParticles.top();
		this->mParticles.pop();

		HamsterAPI::Cell cellBefore = map->getCell(current->getY(), current->getX());

		if (cellBefore != HamsterAPI::CELL_FREE) {
			delete current;
			continue;
		}

		// Update it
		current->Update(scan, map, deltaX, deltaY, deltaYaw);

//		cout << "Belief: " << current->belief() << endl;

		HamsterAPI::Cell cell = map->getCell(current->getY(), current->getX());
		// Check deletion conditions
		// * removal threshold
		// * outside of the map
		// * on an occupied cell
		if (current->getY() < 0 || current->getY() >= map->getHeight() ||
			current->getX() < 0 || current->getX() >= map->getWidth() ||
			cell != CELL_FREE )
		{
			delete current;
			continue;
		}
		else if (current->belief() < PARTICLE_REMOVAL_THRESHOLD) {
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
		cout << "Resampling!" << endl;
//		this->init(map, mCurrentBest);
		this->ResampleParticles(map);
		return NULL;
	}

//	cout << "Sorting" << endl;

	// Sort the remaining particles vector by highest priority
	std::sort(remaining.begin(), remaining.end(), ParticleCompareBeliefs);

	// The best particle should be the first particle in the vector
	best = remaining.front();

	cout << "best belief: " << best->belief() << " worst belief: " << remaining.back()->belief() << endl;

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
	mCurrentBest = best;

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
		nYaw = rand() % 360;
	}
	while (map->getCell(nY, nX) != HamsterAPI::CELL_FREE);

	this->mParticles.push(new Particle(nX, nY, nYaw));
}
