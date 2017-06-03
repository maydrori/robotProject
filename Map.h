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

class Map {
private:
	OccupancyGrid& grid;
	cv::Mat mat;
	void initMap();
	void initCell(int i, int j);
public:
	Map(OccupancyGrid &grid);
	void show() const;
//	void blowGrid(int blowRadius);
	virtual ~Map();
};

#endif /* MAP_H_ */
