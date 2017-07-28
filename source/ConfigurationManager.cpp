#include <fstream>
#include <string>
#include <algorithm>
#include <functional>
#include <cctype>

#include "../headers/ConfigurationManager.h"

using namespace std;

ConfigurationManager* ConfigurationManager::_instance = NULL;

// trim from start
static inline string &ltrim(string &s) {
	s.erase(s.begin(), find_if(s.begin(), s.end(), not1(ptr_fun<int, int>(isspace))));
	return s;
}

// trim from end
static inline string &rtrim(string &s) {
	s.erase(find_if(s.rbegin(), s.rend(), not1(ptr_fun<int, int>(isspace))).base(), s.end());
	return s;
}

// trim from both ends
static inline string &trim(string &s) {
	return ltrim(rtrim(s));
}

ConfigurationManager::ConfigurationManager(const char* szFile) : mConfigMap()
{
	fstream fsConfig;
	fsConfig.open(szFile, ios::in);

	string strLine;
	while (getline(fsConfig, strLine))
	{
		int index = strLine.find(":");
		string key = strLine.substr(0, index);;
		string value = strLine.substr(index + 1);
		value = trim(value);
		this->mConfigMap[key] = value;
	}

	fsConfig.close();

	// Read all configuration values=
	this->mStartLocation.x = this->ReadIntToken("startLocation", 0);
	this->mStartLocation.y = this->ReadIntToken("startLocation", 1);
	this->mStartLocation.yaw = this->ReadIntToken("startLocation", 2);
	this->mGoal.x = this->ReadIntToken("goal", 0);
	this->mGoal.y = this->ReadIntToken("goal", 1);
	this->mRobotSize.height = this->ReadIntToken("robotSize", 0);
	this->mRobotSize.width = this->ReadIntToken("robotSize", 1);
	this->mMapResolutionCM = this->ReadFloat("MapResolutionCM");
}

string ConfigurationManager::ReadString(const char* szKey)
{
	return this->mConfigMap[szKey];
}

int ConfigurationManager::ReadIntToken(const char* szKey, int nTokenNum)
{
	string strValue = this->mConfigMap[szKey];

	for (int i = 0; i < nTokenNum; i++)
	{
		int nIndex = strValue.find(" ");
		strValue = strValue.substr(nIndex + 1);
	}

	unsigned nNextSpace = strValue.find(" ");

	if (nNextSpace != string::npos)
	{
		strValue = strValue.substr(0, nNextSpace);
	}

	int nResult = stoi(strValue);

	return (nResult);
}

float ConfigurationManager::ReadFloat(const char* szKey)
{
	string strValue = this->mConfigMap[szKey];

	return (stof(strValue));
}

