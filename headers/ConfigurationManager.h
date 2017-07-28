/*
 * Configuration.h
 *
 *  Created on: Jul 22, 2017
 *      Author: user
 */

#ifndef HEADERS_CONFIGURATIONMANAGER_H_
#define HEADERS_CONFIGURATIONMANAGER_H_

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

class ConfigurationManager
{
	private:
		static ConfigurationManager* _instance;

		ConfigurationManager(const char* szFile);

		std::unordered_map<std::string, std::string> mConfigMap;
		StartLocation mStartLocation;
		Goal mGoal;
		RobotSize mRobotSize;
		double mMapResolutionCM;

		std::string ReadString(const char* szKey);
		int	ReadIntToken(const char* szKey, int nTokenNum);
		float ReadFloat(const char* szKey);

	public:
		static ConfigurationManager* Instance()
		{
			return (_instance);
		}

		static void Init(const char* szFile)
		{
			_instance = new ConfigurationManager(szFile);
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


#endif /* HEADERS_CONFIGURATIONMANAGER_H_ */
