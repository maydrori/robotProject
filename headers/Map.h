/*
 * Map.h
 *
 *  Created on: Jun 3, 2017
 *      Author: user
 */

#ifndef MAP_H_
#define MAP_H_
#include <HamsterAPIClientCPP/Hamster.h>

#include "ConfigurationManager.h"
using namespace HamsterAPI;
using namespace std;
using namespace cv;

class Map {
private:
	ConfigurationManager* config;
	cv::Mat mat;

	void initMat(OccupancyGrid &grid);
	void rotateMat();
	void initCell(OccupancyGrid &grid, int i, int j);
	OccupancyGrid getBlownGrid(OccupancyGrid grid);
	int getNumOfPixelsToBlow(double mapResolution, int robotHeight, int robotWidth);
public:
	OccupancyGrid blownGrid;
	Map(OccupancyGrid &grid);
	void show();
	void paintCell(int i, int j, int pixelR, int pixelG, int pixelB);
	void paintCell(int i, int j, int pixel);
	Cell getCell(int row, int col);
	int getHeight();
	int getWidth();
	virtual ~Map();
};

#endif /* MAP_H_ */
