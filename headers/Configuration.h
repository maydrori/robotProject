/*
 * Configuration.h
 *
 *  Created on: Jul 22, 2017
 *      Author: user
 */

#ifndef HEADERS_CONFIGURATION_H_
#define HEADERS_CONFIGURATION_H_

#include <unordered_map>
#include <string>

using namespace std;

typedef struct
{
	int x;
	int y;
	int yaw;
} StartLocation;

typedef struct
{
	int x;
	int y;
} Goal;

typedef struct
{
	int height;
	int width;
} RobotSize;

class Configuration
{
	private:
		static Configuration* _instance;

		Configuration(const char* szFile);
		//unordered_map<string, string> nMap;
		std::unordered_map<std::string, std::string> mConfigMap;

		//std::unordered_map<std::string, std::string> nConfigMap;
		StartLocation mStartLocation;
		Goal mGoal;
		RobotSize mRobotSize;
		double mMapResolutionCM;

		std::string ReadString(const char* szKey);
		int	ReadIntToken(const char* szKey, int nTokenNum);
		float ReadFloat(const char* szKey);

	public:
		static Configuration* Instance()
		{
			return (_instance);
		}

		static void Init(const char* szFile)
		{
			_instance = new Configuration(szFile);
		}

		StartLocation start()
		{
			return (this->mStartLocation);
		}

		Goal goal()
		{
			return (this->mGoal);
		}

		RobotSize robotSize()
		{
			return (this->mRobotSize);
		}

		double mapResolution()
		{
			return (this->mMapResolutionCM);
		}
};


#endif /* HEADERS_CONFIGURATION_H_ */
