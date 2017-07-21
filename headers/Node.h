/*
 * Node.h
 *
 *  Created on: Jun 17, 2017
 *      Author: user
 */

#ifndef NODE_H_
#define NODE_H_

struct Node {
	int col, row;
	float f, g, h;
	Node* parent;
};

#endif /* NODE_H_ */
