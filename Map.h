/*
 * Map.h
 *
 *  Created on: Jun 3, 2017
 *      Author: user
 */

#ifndef MAP_H_
#define MAP_H_
#include <HamsterAPIClientCPP/Hamster.h>
using namespace HamsterAPI;
using namespace std;

class Map {
private:
	OccupancyGrid& grid;
	cv::Mat mat;
	void initMap();
	void initCell(int i, int j);
public:
	Map(OccupancyGrid &grid);
	void show() const;
	void paintCell(int i, int j, int pixelR, int pixelG, int pixelB);
	void paintCell(int i, int j, int pixel);
	virtual ~Map();
};

#endif /* MAP_H_ */
