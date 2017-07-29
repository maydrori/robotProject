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

			if (map->blownGrid->getCell(newX, newY) == HamsterAPI::CELL_OCCUPIED)
			{
				hits++;
			}
			else if (this->NeighboursOccupied(map->blownGrid, newX, newY)) {
				hits += 0.6;
			}
		}
	}

	return (hits / 360);
//	int nCorrectReadings = 0;
//	int nTotalReadings = 0;
//	LidarScan scan = robot->getLidarScan();
//	double fMapResolution = ConfigurationManager::Instance()->mapResolution();
//	int fMaxLaserDistPx = floor(robot->maxLaserRange() * 100 / fMapResolution);
//	int nMapWidth = map->mapWidth();
//	int nMapHeight = map->mapHeight();
//
//	// Loop through all the angles
//	for (int i = scan.getMinScanAngle(); i <= scan.getMaxScanAngle(); i++)
//	{
//		// Get the index of the laser reading in the current angle
//		int nCurrReadingIdx = this->DegToIndex(i, 240, nLaserCount);
//
//		// Get the reading from the laser
//		double fLaserReading = robot->laser(nCurrReadingIdx);
//
//		// Get the actual angle on the map in the direction of the laser
//		// TODO: Check if yaw is radian or degree
//		double fPointDirection = this->mYaw + i;
//
//		// Get the X and Y modifiers to test the map
//		double fModX = +cos(toRad(fPointDirection));
//		double fModY = -sin(toRad(fPointDirection));
//
//		/*
//		cout << "Reading in angle: " << i << endl
//			 << "Reading from index: " << nCurrReadingIdx << endl
//			 << "Laser reading: " << fLaserReading << endl
//			 << "Laser reading in pixels: " << fLaserReadingPx << endl
//			 << "Direction of the reading (deg angle): " << fPointDirection << endl
//			 << "X/Y mods: (" << fModX << ", " << fModY << ")" << endl;
//		 */
//
//		bool bFoundObstacle = false;
//		int nObstacleX;
//		int nObstacleY;
//
//		// Loop until we find an obstacle in the direction of the laser
//		for (int nParticleDist = 1; nParticleDist < fMaxLaserDistPx && !bFoundObstacle; nParticleDist++)
//		{
//			// Calculate X and Y for the cell
//			int nCurrX = this->mX + nParticleDist * fModX;
//			int nCurrY = this->mY + nParticleDist * fModY;
//
//			// Map boundary checks ahead
//			// If we hit a map boundary, then we consider it as an obstacle at the boundary point
//
//			// Check X boundaries
//			if (nCurrX < 0)
//			{
//				nCurrX = 0;
//				bFoundObstacle = true;
//			}
//			else if (nCurrX >= nMapWidth)
//			{
//				nCurrX = nMapWidth - 1;
//				bFoundObstacle = true;
//			}
//
//			// Check Y boundaries
//			if (nCurrY < 0)
//			{
//				nCurrY = 0;
//				bFoundObstacle = true;
//			}
//			else if (nCurrY >= nMapHeight)
//			{
//				nCurrY = nMapHeight - 1;
//				bFoundObstacle = true;
//			}
//
//			// Check if the cell is occupied
//			if (map->map()[nCurrY][nCurrX] == OCCUPIED_CELL)
//			{
//				bFoundObstacle = true;
//			}
//
//			// Check if we found an obstacle (either OCCUPIED_CELL hit or boundary hit)
//			if (bFoundObstacle)
//			{
//				nObstacleX = nCurrX;
//				nObstacleY = nCurrY;
//			}
//		}
//
//		double fDistToObstacle;
//
//		// If an obstacle was found, then calculate the distance to it.
//		// If an obstacle was not found, use the maximum laser distance
//		if (bFoundObstacle)
//		{
//			fDistToObstacle = sqrt(pow(this->mX - nObstacleX, 2) + pow(this->mY - nObstacleY, 2));
//			/*
//			cout << "Found obstacle at " << nObstacleX << " , " << nObstacleY << endl;
//			cout << "Dist: " << fDistToObstacle << endl
//				 << "Dist in meters: " << fDistToObstacle * fMapResolution / 100 << endl;
//			 */
//		}
//		else
//		{
//			fDistToObstacle = fMaxLaserDistPx;
//		}
//
//		// Convert pixels distance to meters
//		fDistToObstacle = fDistToObstacle / 100 * fMapResolution;
//
//		// Calculate delta between laser reading and calculated distance from obstacle
//		double fDelta = abs(fDistToObstacle - fLaserReading);
//
//		// If the delta is lower than or equal to our defined acceptable noise, we can mark this reading as correct
//		if (fDelta <= PARTICLE_LASER_COMPARE_NOISE)
//		{
//			nCorrectReadings++;
//		}
//
//		nTotalReadings++;
//	}
//
//	return ((double)nCorrectReadings / nTotalReadings);
}

void Particle::Update(HamsterAPI::Hamster* robot, Map* map)
{
	int dx = robot->getPose().getX();
	int dy = robot->getPose().getY();
	double dyaw = robot->getPose().getHeading();

	this->mX += dx;
	this->mY += dy;
	this->mYaw += dyaw;

	if (this->mYaw < 0)
	{
		this->mYaw += 360;
	}

	double move = this->ProbByMove(dx, dy, dyaw);
	double measures = this->ProbByScan(robot->getLidarScan(), map);
	double last = this->mBelief;
	mes = measures;
	mov = move;
	this->last = last;

	this->mBelief = move * measures * last * NORMALIZATION_FACTOR;

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
	do
	{
		// Generate (x,y) in range (-PARTICLE_CREATE_IN_RADIUS, +PARTICLE_CREATE_IN_RADIUS)
		nX = this->mX + (rand() % (PARTICLE_CREATE_IN_RADIUS * 2)) - PARTICLE_CREATE_IN_RADIUS;
		nY = this->mY + (rand() % (PARTICLE_CREATE_IN_RADIUS * 2)) - PARTICLE_CREATE_IN_RADIUS;
	}
	while (nX >= 0 && nY >= 0 && nX < map->blownGrid->getWidth() && nY < map->blownGrid->getHeight() && map->blownGrid->getCell(nX, nY) == HamsterAPI::CELL_OCCUPIED);

	Particle* random = new Particle(nX, nY, nYaw);
	random->mBelief = this->mBelief;

	return (random);
}
