#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "Map.h"

#define PARTICLE_CREATE_IN_RADIUS 	 3
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
		int mX;
		int mY;
		double mBelief;
		bool NeighboursOccupied(OccupancyGrid* grid, int x, int y, int level);

	public:
		Particle(int x, int y, double yaw);
		virtual ~Particle();
		double ProbByScan(HamsterAPI::LidarScan scan, Map* map);
		void Update(HamsterAPI::LidarScan scan, Map* map, int deltaX, int deltaY, int deltaYaw);
		Particle* RandomCloseParticle(Map* map);

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
