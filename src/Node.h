/*
 * Node.h
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#ifndef NODE_H_
#define NODE_H_
#include "Params.h"
#include <array>
#include <iostream>     // std::cout

class Node {
public:
	Node(float val,int depth);
	~Node();
	Node* 					addChild(int i);									//create a new children at position i with value val
	bool  					childExists(int i);									//checks if child i exists
	Node* 					getChild(int i);									//returns child i - creates it if it does not exists
	double					update(bool rec);									//update the current node according to its children, if rec=true, descendant are first updated (full update of the subtree)
	void 					setValue(float v){val_=v;}							//set the value val_ of the node to v
	double 					getValue(){return val_;}							//return the value val_ of the node
	bool 					isLeaf(){return isLeaf_;}							//return true if the node is a leaf
	friend std::ostream& 	operator<< (std::ostream& stream, const Node& n);	//print operator
	bool					isEpsilonObstacle();								//check if a node is an epsilon obstacle
	double					getVolume();										//returns the volume of the node
	double					getScale();											//returns the scaling factor to reach children

private:
	double 							val_;					//average value of the children
	std::array<Node*,TWOPOWDIM> 	children_;				//children array
	std::array<bool,TWOPOWDIM> 		childExists_;			//existing children indicator
	bool 							isLeaf_;				//whether that node is a leaf of the tree
	int								depth_;					//depth in the tree

	static const bool				pruningOn=true;			//whether to prune during updates
	static constexpr double 		epsilon=0.5;			//threshold for epsilon-obstacle
};

#endif /* NODE_H_ */
