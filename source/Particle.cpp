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

	double hits = 0.0;
	double misses = 0.0;
	int scanSize = scan.getScanSize();
	double total = 0.0;
	for (int i = 0; i < scanSize; ++i)
	{
		int disatnce = scan.getDistance(i);
		int maxRange = scan.getMaxRange();

		if (disatnce < maxRange - 0.001)
		{
			double mapRes = ConfigurationManager::Instance()->mapResolution();
			double radian = ((((int)this->mYaw) + 180 + i) % 360) * M_PI / 180.0;
			int newX = this->mX + disatnce * cos(radian) / mapRes;
			int newY = this->mY + disatnce * sin(radian) / mapRes;

			if (newX >= 0 && newY >= 0 && newX <= map->getWidth() && newY <= map->getHeight()
					&& map->getCell(newY, newX) == HamsterAPI::CELL_OCCUPIED)
			{
				hits++;
			} else {
				misses++;
			}

			total++;
		}
	}
	return ((double) hits/ (total));
}

void Particle::Update(HamsterAPI::LidarScan scan, Map* map, int deltaX, int deltaY, int deltaYaw)
{
	// lets say it doesnt cross borders
	this->mX += deltaX;
	this->mY += deltaY;
	this->mYaw += deltaYaw;

	if (this->mYaw < 0)
	{
		this->mYaw += 360;
	}

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
	int tryCounter = 0;
	do
	{
		tryCounter++;

		int rnd1 = rand();
		int rnd2 = rand();
		// Generate (x,y) in range (-PARTICLE_CREATE_IN_RADIUS, +PARTICLE_CREATE_IN_RADIUS)
		nX = this->mX + (rnd1 % (radius * 2)) - radius;
		nY = this->mY + (rnd2 % (radius * 2)) - radius;

//		// If there were NUM_OF_RADIUS_TRIES tries withous success we increase the radius
//		if (tryCounter % NUM_OF_RADIUS_TRIES == 0 ) {
//			radius++;
//		}
	}
	while ((!((nX >= 0 && nY >= 0 && nX < map->getWidth() && nY < map->getHeight())))||
			(nX >= 0 && nY >= 0 && nX < map->getWidth() && nY < map->getHeight() && map->getCell(nY, nX) == HamsterAPI::CELL_OCCUPIED));

	Particle* random = new Particle(nX, nY, nYaw);
	random->mBelief = this->mBelief;
	return (random);
}
