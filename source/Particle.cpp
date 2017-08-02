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

double Particle::ProbByScan(HamsterAPI::LidarScan scan, Map* map)
{
	double mapResolution = map->blownGrid.getResolution();
	int hits = 0;
	int misses = 0;

	for (int i = 0; i < scan.getScanSize(); i++) {

		double angle = scan.getScanAngleIncrement() * i * DEG2RAD;

		if (scan.getDistance(i) < scan.getMaxRange() - 0.001) {

			// Obstacle distance count
			int pixelX = mX + scan.getDistance(i) / mapResolution * cos( 180*DEG2RAD + this->mYaw * DEG2RAD - angle);
			int pixelY = mY + scan.getDistance(i) / mapResolution * sin( 180*DEG2RAD + this->mYaw * DEG2RAD - angle);

			if (pixelY >= 0 && pixelX >= 0 && pixelY <= map->getHeight() && pixelX <= map->getWidth() &&
					map->blownGrid.getCell(pixelY, pixelX) == HamsterAPI::CELL_OCCUPIED) {
				hits++;
			} else {
				misses++;
			}
		}
	}

//	cout <<"hits="<<hits<<"misses="<<misses<<" = " <<((float)hits / (hits + misses))<<endl;
	return (float)hits / (hits + misses);
}

void Particle::Update(HamsterAPI::LidarScan scan, Map* map, int deltaX, int deltaY, int deltaYaw)
{
	// lets say it doesnt cross borders
//	double root = sqrt(deltaX * deltaX + deltaY * deltaY);
//	this->mX += root * cos((double)(this->mYaw * DEG2RAD));
//	this->mY += root * sin((double)(this->mYaw  * DEG2RAD));
//	this->mYaw = fmod((double)(this->mYaw + deltaYaw), 360.0);

	this->mX += deltaX;
	this->mY += deltaY;
	this->mYaw += deltaYaw;

	double measures = this->ProbByScan(scan, map);
	mes = measures;

	this->last = last;
	this->mBelief = measures;

	if (this->mBelief > 1)
	{
		this->mBelief = 1;
	}
}

Particle* Particle::RandomCloseParticle(Map* map)
{
	int nX = 0;
	int nY = 0;
	double nYaw = this->mYaw + (((rand() % (PARTICLE_CREATE_YAW_RANGE * 2)) - PARTICLE_CREATE_YAW_RANGE) / (double)10);

	// Generate a random particle around the current particle (within PARTICLE_CREATE_IN_RADIUS)
	int radius = PARTICLE_CREATE_IN_RADIUS;
	int mapWidth = map->getWidth();
	int mapHeight = map->getHeight();
	if (radius < 2) radius = 2;
	do
	{
		int rnd1 = rand();
		int rnd2 = rand();
		// Generate (x,y) in range (-PARTICLE_CREATE_IN_RADIUS, +PARTICLE_CREATE_IN_RADIUS)
		nX = this->mX + ((rnd1 % (radius * 2))) - radius;
		nY = this->mY + ((rnd2 % (radius * 2))) - radius;
	} while ((nX < this->mX-radius || nY < this->mY-radius) ||
			(nX > this->mX+radius) || (nY > this->mY+radius) ||
			(map->getCell(nY, nX) == HamsterAPI::CELL_OCCUPIED));

	Particle* random = new Particle(nX, nY, nYaw);
	random->mBelief = this->mBelief;
//	map->paintCell(nY, nX, 0, 100, 0);
	return (random);
}
