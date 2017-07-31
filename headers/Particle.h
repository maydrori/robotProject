#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "Map.h"

#define PARTICLE_CREATE_IN_RADIUS 	 2
#define NUM_OF_RADIUS_TRIES			 20
#define PARTICLE_CREATE_YAW_RANGE	 30 // = 5.0
#define toRad(a) 					 (((a) / 180) * M_PI)
#define toDeg(a) 					 (((a) / M_PI) * 180)
#define PARTICLE_LASER_COMPARE_NOISE 0.25
#define NORMALIZATION_FACTOR 		 1.15
#define DISTANCE_SCORE_MODIFIER		 0.97

class Particle
{
	private:
		double mYaw;
		double mBelief;

		bool NeighboursOccupied(OccupancyGrid* grid, int x, int y, int level);

	public:
		int mX;
		int mY;
		double mes;
		double mov;
		double last;

		Particle(int x, int y, double yaw);
		virtual ~Particle();
		double ProbByMove(int dx, int dy, double dyaw);
		double ProbByScan(HamsterAPI::LidarScan scan, Map* map);
		void Update(HamsterAPI::LidarScan scan, Map* map, int deltaX, int deltaY, int deltaYaw);
		Particle* RandomCloseParticle(Map* map);

		int DegToIndex(double deg, int angleRange, int laserCount)
		{
			// 0.36
			double anglesPerLaser = ((double)angleRange / laserCount);

			// 120
			int anglesMidPoint = angleRange / 2;

			// (deg + 120) / 0.36
			return (deg + anglesMidPoint) / anglesPerLaser;
		}

		double IndexToDeg(int index, int angleRange, int laserCount)
		{
			// 0.36
			double anglesPerLaser = ((double)angleRange / laserCount);

			// 120
			int anglesMidPoint = angleRange / 2;

			// index * 0.36 - 120
			return index * anglesPerLaser - anglesMidPoint;
		}

		double belief()
		{
			return (this->mBelief);
		}

		int getX()
		{
			return (this->mX);
		}

		int getY()
		{
			return (this->mY);
		}

		double getYaw()
		{
			return (this->mYaw);
		}
};

#endif
