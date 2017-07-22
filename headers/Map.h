/*
 * Map.h
 *
 *  Created on: Jun 3, 2017
 *      Author: user
 */

#ifndef MAP_H_
#define MAP_H_
#include <HamsterAPIClientCPP/Hamster.h>
#include "Configuration.h"
using namespace HamsterAPI;
using namespace std;

class Map {
private:
	Configuration* config;
	OccupancyGrid& grid;
	OccupancyGrid* coarseGrid;
	OccupancyGrid* blownGrid;
	cv::Mat mat;
	//int robotHeightInPixels;
	//int robotWidthInPixels;
	int robotSizeInPixels;

	void initMat(OccupancyGrid &grid);
	void initCell(OccupancyGrid &grid, int i, int j);
	//OccupancyGrid getBlownGrid(OccupancyGrid grid, int robotHeight, int robotWidth);
	void convertToBlownGrid();
	void convertToCoarseGrid();
	int getNumOfPixelsToBlow(double mapResolution, int robotHeight, int robotWidth);
public:
	Map(OccupancyGrid &grid);
	void show();
	void paintCell(int i, int j, int pixelR, int pixelG, int pixelB);
	void paintCell(int i, int j, int pixel);
	virtual ~Map();
};

#endif /* MAP_H_ */
