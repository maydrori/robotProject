#include "../headers/Particle.h"
#include <math.h>
#include <iostream>

using namespace std;

Particle::Particle(int x, int y, double yaw)
	: mX(x), mY(y), mYaw(yaw), mBelief(1.0)
{
}

Particle::~Particle()
{
}

double Particle::ProbByMove(int dx, int dy, double dyaw)
{
	// Calculate scores based on yaw and distance
	double distance = sqrt(pow(dx, 2) + pow(dy, 2));
	double distScore = pow(DISTANCE_SCORE_MODIFIER, distance);
	double yawScore = 1 - (abs(dyaw) / 180);

#ifdef TEST_MODE
	cout << "PBM:[";
	cout << "dist: " << distScore <<"|yawScore: " << yawScore << "]";
#endif

	if (distScore < yawScore)
	{
		return (distScore);
	}
	else
	{
		return (yawScore);
	}
}

double Particle::ProbByScan(HamsterAPI::LidarScan scan, Map* map)
{
	int mapResolution = ConfigurationManager::Instance()->mapResolution();
	int hits = 0;
	int misses = 0;

	for (int i = 0; i < scan.getScanSize(); i++) {
		double angle = scan.getScanAngleIncrement() * i * DEG2RAD;

		if (scan.getDistance(i) < scan.getMaxRange() - 0.001) {

			// Obstacle distance count
			double obsX = this->mX + scan.getDistance(i) * cos(angle + 90*DEG2RAD + this->mYaw * DEG2RAD);
			double obsY = this->mY + scan.getDistance(i) * sin(angle + 90*DEG2RAD + this->mYaw * DEG2RAD);

			int pixelY = (double)(map->getHeight() / 2) - obsY / mapResolution;
			int pixelX = obsX / mapResolution + (double)(map->getWidth() / 2);

			if (pixelY >= 0 && pixelX >= 0 && pixelY <= map->getHeight() && pixelX <= map->getWidth() &&
					map->getCell(pixelY, pixelX) == HamsterAPI::CELL_OCCUPIED) {
				hits++;
			} else {
				misses++;
			}
		}
	}
	return (float)hits / (hits + misses);
}

void Particle::Update(HamsterAPI::LidarScan scan, Map* map, int deltaX, int deltaY, int deltaYaw)
{
	// lets say it doesnt cross borders
	double root = sqrt(deltaX * deltaX + deltaY * deltaY);
	this->mX += root * cos((double)(this->mYaw * DEG2RAD));
	this->mY += root * sin((double)(this->mYaw  * DEG2RAD));
	this->mYaw = fmod((double)(this->mYaw + deltaYaw), 360.0);

	//double move = this->ProbByMove(deltaX, deltaY, deltaYaw);
	double measures = this->ProbByScan(scan, map);
	mes = measures;
	//mov = move;
	this->last = last;
	this->mBelief = measures;

	if (this->mBelief > 1)
	{
		this->mBelief = 1;
	}

#ifdef TEST_MODE
	cout << "P:[" << mX << ", " << mY << ", " << mYaw << "] " <<
			"R:[" << robot->x() << ", " << robot->y() << ", " << robot->yaw() << "]" <<
			"Belief: " << mBelief <<
			"(move: " << move << ")(measures: " << measures << ")(last: " << last << ")" << endl;
#endif
}

Particle* Particle::RandomCloseParticle(Map* map)
{
	int nX = 0;
	int nY = 0;
	double nYaw = this->mYaw + (((rand() % (PARTICLE_CREATE_YAW_RANGE * 2)) - PARTICLE_CREATE_YAW_RANGE) / (double)10);

	// Generate a random particle around the current particle (within PARTICLE_CREATE_IN_RADIUS)
	int radius = PARTICLE_CREATE_IN_RADIUS;
	if (radius < 2) radius = 2;
	do
	{
		int rnd1 = rand();
		int rnd2 = rand();
		// Generate (x,y) in range (-PARTICLE_CREATE_IN_RADIUS, +PARTICLE_CREATE_IN_RADIUS)
		nX = this->mX + (rnd1 % (radius * 2)) - radius;
		nY = this->mY + (rnd2 % (radius * 2)) - radius;
	}
	while ((nX >= 0 && nY >= 0 && nX < map->getWidth() && nY < map->getHeight() && map->getCell(nY, nX) == HamsterAPI::CELL_OCCUPIED));

	Particle* random = new Particle(nX, nY, nYaw);
	random->mBelief = this->mBelief;
//	map->paintCell(random->mY, random->mX, 0, 100, 0);
	return (random);
}
