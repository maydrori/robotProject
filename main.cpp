/*
 * HamsterAPIClientSimpleBehaviourExample.cpp
 *
 *  Created on: Aug 10, 2016
 *      Author: ofir
 */

#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
using namespace std;
using namespace HamsterAPI;
HamsterAPI::Hamster * hamster;

// Each pixel = 0.05meters
// means: 1meter = 20pixels
double OPEN_CV_RESOLUTION = 20;

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

			// The + 200 is because the robot is in the middle of the
			// map so we need to shift the coordinates
			int xObsImage = yObs * OPEN_CV_RESOLUTION + 200;
			int yObsImage = xObs * OPEN_CV_RESOLUTION + 200;

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

int main(int argc, char ** argv) {
	try {
		hamster = new HamsterAPI::Hamster(1);
		cv::namedWindow("myWindow");
		cv::Mat image(400, 400, CV_8UC3, cv::Scalar(0,0,0));

		while (hamster->isConnected()) {
			try {
				findObstacles(image);

				if (isFrontFree())
					moveForward();
				else {
					stopMoving();
					if (isLeftFree())
						turnLeft();
					else if (isRightFree())
						turnRight();
					else if (isBackFree())
						moveBackwards();
					else
						HamsterAPI::Log::i("Client", "I am stuck!");
				}

				// Speed Getter
				// HamsterAPI::Speed speed = hamster.getSpeed();
			} catch (const HamsterAPI::HamsterError & message_error) {
				HamsterAPI::Log::i("Client", message_error.what());
			}

		}
	} catch (const HamsterAPI::HamsterError & connection_error) {
		HamsterAPI::Log::i("Client", connection_error.what());
	}
	return 0;
}

int oldMain2(int argc, char ** argv) {
	try {
		hamster = new HamsterAPI::Hamster(1);
		/*OccupancyGrid ogrid = hamster->getSLAMMap();
		 cout<< "resolution: "<< ogrid.getResolution()<<endl;
		 cout<< "Width: "<< ogrid.getWidth()<<endl;
		 cout<< "height: "<< ogrid.getHeight()<<endl;
		 */
		while (hamster->isConnected()) {
			try {
				HamsterAPI::LidarScan ld = hamster->getLidarScan();
				if (ld.getDistance(180) < 0.4) {
					hamster->sendSpeed(-0.5, 0);
					cout << "Front: " << ld.getDistance(180) << endl;
				} else if (ld.getDistance(180) < 0.8) {
					hamster->sendSpeed(0.5, 45);
					cout << "Front: " << ld.getDistance(180) << endl;
				}
				else
					hamster->sendSpeed(1.0, 0);
				//cout<<"Front: "<<ld.getDistance(180)<<endl;
				//cout<<"Left: "<<ld.getDistance(90)<<endl;
				//cout<<"Right: "<<ld.getDistance(270)<<endl;
			} catch (const HamsterAPI::HamsterError & message_error) {
				HamsterAPI::Log::i("Client", message_error.what());
			}

		}
	} catch (const HamsterAPI::HamsterError & connection_error) {
		HamsterAPI::Log::i("Client", connection_error.what());
	}
	return 0;
}
int main1() {
	try {
//		HamsterClientParameters params =  HamsterClientParameters();
//		params.base_address = "127.0.0";
//		params.port = 8102;
		hamster = new HamsterAPI::Hamster(1);

		cv::namedWindow("OccupancyGrid-view");
		while (hamster->isConnected()) {
			try {

				OccupancyGrid ogrid = hamster->getSLAMMap();
				int width = ogrid.getWidth();
				int height = ogrid.getHeight();
				unsigned char pixel;
				//CvMat* M = cvCreateMat(width, height, CV_8UC1);
				cv::Mat m = cv::Mat(width, height,CV_8UC1);

				for (int i = 0; i < height; i++)
					for (int j = 0; j < width; j++) {
						if (ogrid.getCell(i, j) == CELL_FREE)
							pixel = 255;
						else if (ogrid.getCell(i, j) == CELL_OCCUPIED)
							pixel = 0;
						else
							pixel = 128;
						//cvmSet(M, i, j, pixel);
						m.at<unsigned char>(i,j) = pixel;
					}

				cv::imshow("OccupancyGrid-view",m);
				cv::waitKey(1);

			} catch (const HamsterAPI::HamsterError & message_error) {
				HamsterAPI::Log::i("error1", message_error.what());
			}

		}
	} catch (const HamsterAPI::HamsterError & connection_error) {
		HamsterAPI::Log::i("error2", connection_error.what());
	}
	return 0;

}

