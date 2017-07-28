#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "Map.h"

#define PARTICLE_CREATE_IN_RADIUS 	 3
#define PARTICLE_CREATE_YAW_RANGE	 30 // = 5.0
#define toRad(a) 					 (((a) / 180) * M_PI)
#define toDeg(a) 					 (((a) / M_PI) * 180)
#define PARTICLE_LASER_COMPARE_NOISE 0.25
#define NORMALIZATION_FACTOR 		 1.15
#define DISTANCE_SCORE_MODIFIER		 0.97

class Particle
{
	private:
		int mX;
		int mY;
		double mYaw;
		double mBelief;

	public:
		double mes;
		double mov;
		double last;

		Particle(int x, int y, double yaw);
		virtual ~Particle();
//		double ProbByMove(int dx, int dy, double dyaw);
//		double ProbByMeasures(Robot* robot, Map* map);
//		void Update(Robot* robot, Map* map);
//		Particle* RandomCloseParticle(Map* map);
//
//		int DegToIndex(double deg, int angleRange, int laserCount)
//		{
//			// 0.36
//			double anglesPerLaser = ((double)angleRange / laserCount);
//
//			// 120
//			int anglesMidPoint = angleRange / 2;
//
//			// (deg + 120) / 0.36
//			return (deg + anglesMidPoint) / anglesPerLaser;
//		}
//
//		double IndexToDeg(int index, int angleRange, int laserCount)
//		{
//			// 0.36
//			double anglesPerLaser = ((double)angleRange / laserCount);
//
//			// 120
//			int anglesMidPoint = angleRange / 2;
//
//			// index * 0.36 - 120
//			return index * anglesPerLaser - anglesMidPoint;
//		}
//
//		double belief()
//		{
//			return (this->mBelief);
//		}

		int x()
		{
			return (this->mX);
		}

		int y()
		{
			return (this->mY);
		}

		double yaw()
		{
			return (this->mYaw);
		}
};

#endif
