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

bool Particle::NeighboursOccupied(OccupancyGrid* grid, int x, int y, int level=1)
{
	return (grid->getCell(x+level, y) == HamsterAPI::CELL_OCCUPIED)||
			(grid->getCell(x, y+level) == HamsterAPI::CELL_OCCUPIED)||
			(grid->getCell(x+level, y+level) == HamsterAPI::CELL_OCCUPIED)||
			(grid->getCell(x-level, y-level) == HamsterAPI::CELL_OCCUPIED)||
			(grid->getCell(x, y-level) == HamsterAPI::CELL_OCCUPIED)||
			(grid->getCell(x-level, y) == HamsterAPI::CELL_OCCUPIED)||
			(grid->getCell(x+level, y-level) == HamsterAPI::CELL_OCCUPIED)||
			(grid->getCell(x-level, y+level) == HamsterAPI::CELL_OCCUPIED);
}

double Particle::ProbByScan(HamsterAPI::LidarScan scan, Map* map)
{
	double hits = 0;
	double misses = 0.0;
	int scanSize = scan.getScanSize();

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

//			cout << "Map resolution: " << mapRes << endl;
//			cout << "this->mYaw: " << this->mYaw << endl;
//			cout << "this->mX: " << this->mX << endl;
//			cout << "this->mY: " << this->mY << endl;
//			cout << "newX: " << newX << endl;
//			cout << "newY: " << newY << endl;

			if (map->blownGrid->getCell(newX, newY) == HamsterAPI::CELL_OCCUPIED)
			{
				hits++;
			} else {
				misses++;
			}
		}
	}
	return ((double) hits/ (hits+ misses));
}

void Particle::Update(HamsterAPI::Hamster* robot, Map* map, int deltaX, int deltaY, int deltaYaw)
{
	this->mX += deltaX;
	this->mY += deltaY;
	this->mYaw += deltaYaw;

	if (this->mYaw < 0)
	{
		this->mYaw += 360;
	}

	//double move = this->ProbByMove(deltaX, deltaY, deltaYaw);
	double measures = this->ProbByScan(robot->getLidarScan(), map);
	double last = this->mBelief;
	mes = measures;
	//mov = move;
	this->last = last;
	this->mBelief = measures * last * NORMALIZATION_FACTOR;

	if (this->mBelief > 0)
	{
//		cout << "Move: " << move << endl;
//		cout << "Measures: " << measures << endl;
//		cout << "Last: " << last << endl;
//		cout << "Normaliztion Factor: " << NORMALIZATION_FACTOR << endl;
//		cout << "Belief: " << this->mBelief << endl;
	}

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

		// If there were NUM_OF_RADIUS_TRIES tries withous success we increase the radius
		if (tryCounter % NUM_OF_RADIUS_TRIES == 0) {
			radius++;
		}
	}
	while (nX >= 0 && nY >= 0 && nX < map->blownGrid->getWidth() && nY < map->blownGrid->getHeight() && map->blownGrid->getCell(nY, nX) == HamsterAPI::CELL_OCCUPIED);

	Particle* random = new Particle(nX, nY, nYaw);
	random->mBelief = this->mBelief;\
	return (random);
}
