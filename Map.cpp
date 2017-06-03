/*
 * Map.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: user
 */

#include "Map.h"

Map::Map(OccupancyGrid &grid) :grid(grid) {
	cv::namedWindow("OccupancyGrid-view");
	initMap();
}

void Map::initMap() {
	mat = cv::Mat(grid.getHeight(), grid.getWidth(), CV_8UC3);

	for (int i=0; i<grid.getHeight(); i++) {
		for (int j=0; j<grid.getWidth(); j++) {
			initCell(i,j);
		}
	}
}

void Map::initCell (int i, int j) {
	Cell cell = grid.getCell(i, j);
	int pixel;
	if (cell == CELL_FREE) {
		pixel = 255;
	}
	else if (cell == CELL_OCCUPIED) pixel = 0;
	else pixel = 128;

	mat.at<cv::Vec3b>(i,j)[0] = pixel;
	mat.at<cv::Vec3b>(i,j)[1] = pixel;
	mat.at<cv::Vec3b>(i,j)[2] = pixel;
}

void Map::show () const{
	cv::imshow("OccupancyGrid-view", mat);
	cv::waitKey(1);
}

//void Map::blowGrid (int blowRadius) {
//
//	for (int i=0; i<grid.getHeight(); i++) {
//		for (int j=0; j<grid.getWidth(); j++) {
//
//			if (grid.getCell(i, j) == CELL_OCCUPIED) {
//
//				for (int i2 = i-blowRadius; i2<=i+blowRadius; i2++) {
//					for (int j2 = j-blowRadius; j2<=j+blowRadius; j2++) {
//
//						if (i2 >= 0 && i2 < grid.getHeight() && j2 >=0 && j2 < grid.getWidth()) {
//							grid.setCell(i2, j2, CELL_OCCUPIED);
//						}
//					}
//				}
//			}
//		}
//	}
//}

Map::~Map() {
	cv::destroyWindow("OccupancyGrid-view");
}

