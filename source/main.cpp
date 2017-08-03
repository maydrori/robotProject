#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "../headers/Map.h"
#include "../headers/PathPlanner.h"
#include "../headers/RobotManager.h"

// Debug mode
#define DEBUG

// Delete on May's computer
#define May

#ifdef GUY
#define CONFIG_PATH "/home/user/Desktop/robotProject/params/parameters.txt"
#else
#define CONFIG_PATH "/home/user/workspace/ObstacleAvoid/params/parameters.txt"
#endif

using namespace std;
using namespace HamsterAPI;
HamsterAPI::Hamster * hamster;

typedef vector<pair<int,int> > Path;

int main() {

	bool bConnectionSuccedd = false;
	while (!bConnectionSuccedd) {
		try {

			// Init the hamster
			hamster = new Hamster(1);
			sleep(3);

			// Init the configuration
			ConfigurationManager::Init(CONFIG_PATH);

			// Create a map by the hamster's slam map
			OccupancyGrid grid = hamster->getSLAMMap();

			Map* map = new Map(grid);

			RobotManager* manager = new RobotManager(hamster, map);
			manager->Start();

			bConnectionSuccedd = true;

		}
		catch(const HamsterAPI::HamsterError & e) {
			cout << "hamster error," <<  e.what() << endl;
		}
		catch(const std::exception& e) {
			cout << "exception," <<  e.what() << endl;
		}
	}
	return 0;
}







