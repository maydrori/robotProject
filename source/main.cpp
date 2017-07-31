#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "../headers/Map.h"
#include "../headers/PathPlanner.h"
#include "../headers/RobotManager.h"

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

// Each pixel = 0.05meters
// means: 1meter = 20pixels
double OPEN_CV_RESOLUTION = 0.05;

void getScansBetween(double min, double max, std::vector<double> & distances) {
	HamsterAPI::LidarScan scan = hamster->getLidarScan();

	for (size_t i = 0; i < scan.getScanSize(); i++) {
		double degree = scan.getScanAngleIncrement() * i;
		if (degree >= min && degree <= max)
			distances.push_back(scan.getDistance(i));
	}
}

bool willCollide(std::vector<double> distances, int angle_from_center) {
	HamsterAPI::LidarScan scan = hamster->getLidarScan();

	int collisions = 0;

	for (size_t i = distances.size() / 2 - angle_from_center / 2;
			i < distances.size() / 2 + angle_from_center / 2; i++)
		if (distances[i] < scan.getMaxRange() / 4.0){
			collisions++;
		}

	return collisions >= angle_from_center / 4.0;
}

bool isFrontFree() {
	// Degrees : [90, 270]

	std::vector<double> distances;

	getScansBetween(90, 270, distances);

	return !willCollide(distances, 40);
}

bool isLeftFree() {
	// Degrees : [180,360]

	std::vector<double> distances;

	getScansBetween(180, 360, distances);

	return !willCollide(distances, 40);
}

bool isRightFree() {
	// Degrees : [0, 180]

	std::vector<double> distances;

	getScansBetween(0, 180, distances);

	return !willCollide(distances, 40);
}

bool isBackFree() {
	// Degrees : [270,360], [0, 90]

	std::vector<double> distances;

	getScansBetween(270, 360, distances);
	getScansBetween(0, 90, distances);

	return !willCollide(distances, 40);
}

void moveForward() {
//	HamsterAPI::Log::i("Client", "Moving Forward");
	hamster->sendSpeed(0.4, 0.0);
}

void turnLeft() {
//	HamsterAPI::Log::i("Client", "Turning Left");
	while (!isFrontFree())
		hamster->sendSpeed(0.04, 45.0);
}

void turnRight() {
//	HamsterAPI::Log::i("Client", "Turning Right");
	while (!isFrontFree())
		hamster->sendSpeed(0.04, -45.0);
}

void moveBackwards() {
//	HamsterAPI::Log::i("Client", "Moving Backwards");
	while (!isLeftFree() && !isRightFree() && isBackFree())
		hamster->sendSpeed(-0.4, 0.0);
	if (isLeftFree())
		turnLeft();
	else
		turnRight();
}

void stopMoving() {
	hamster->sendSpeed(0.0, 0.0);
}

void findObstacles(cv::Mat image) {

	HamsterAPI::LidarScan scan = hamster->getLidarScan();

	for (size_t i = 0; i < scan.getScanSize(); i++)
	{
		// If the distance smaller then the max range
		// so we find an obstacle
		if (scan.getDistance(i) < scan.getMaxRange()){

			cout << scan.getDistance(i) << "/" << scan.getMaxRange() << endl;

			Pose pose = hamster->getPose();
			double degrees = pose.getHeading()+180-scan.getScanAngleIncrement()*i;
			double radians = degrees * M_PI / 180;
			double xObs = pose.getX() + scan.getDistance(i)*cos(radians);
			double yObs = pose.getY() + scan.getDistance(i)*sin(radians);

			// If each pixel = 0.05m
			// then 1m = 20 pixels
			double pixelsPerMeter = 1 / OPEN_CV_RESOLUTION;

			// The + 200 is because the robot is in the middle of the
			// map so we need to shift the coordinates
			int xObsImage = yObs * pixelsPerMeter + 200;
			int yObsImage = xObs * pixelsPerMeter + 200;

			image.at<cv::Vec3b>(xObsImage, yObsImage)[0] = 255;
			image.at<cv::Vec3b>(xObsImage, yObsImage)[1] = 0;
			image.at<cv::Vec3b>(xObsImage, yObsImage)[2] = 0;

			cv::imshow("myWindow", image);
			cv::waitKey(1);

//			cout << "collision at: poseX= " << pose.getX() << ", poseY =" << pose.getY()
//					<< ", i= " << i << ", distances[i]=" << scan.getDistance(i) << ", poseAngle=" << pose.getHeading()
//					<< ", adding= " << scan.getDistance(i)*cos(radians)
//					<< ", x= " << xObs << ", y= " << yObs << endl;
		}
	}


}



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
	//	while (hamster->isConnected()) {
	//		map->show();
	//		sleep(0.2);
	//	}

	}
	catch(const HamsterAPI::HamsterError & message_error)
	{
		delete hamster;

		cout << "shit2" << endl;
	}
	}
	return 0;
}







