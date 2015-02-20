/*
 * Tree.h
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#ifndef TREE_H_
#define TREE_H_

#include "State.h"
#include "Node.h"

class Tree {
public:
	Tree();
	~Tree();

	void addObstacle(State& obs);       										//create a node a finest resolution around s with value 1
	void updateRec(){root_->updateValRec();}									//recursively updates every value in the tree
	double getVal(State s);														//return the value of the node corresponding to state s
	inline Node* getRoot(){return root_;}										//returns the root of the tree
	void setStateBounds(State minState,State maxState);							//set the search space range
	inline void setMaxDepth(int depth){maxDepth_=depth;}						//set the maximum depth of the tree
	inline int getMaxDepth(){return maxDepth_;}									//returns the max depth
	inline State getRootState(){return rootState_;}								//returns the spacial position of the center of the search space
	inline std::array<State,TWOPOWDIM>* getDirections(){return &directions_;}	//returns the array of directions to find children
	Node* getNode(State s,int depth);											//find the node corresponding to state s and depth smaller than depth
	Node* getNode(State s){return getNode(s,maxDepth_);}						//find the node corresponding to state s at any depth

private:
	Node* root_;																//root of the tree
	int maxDepth_;																//max depth of the tree
	State rootState_;															//state corresponding to the center of the search space
	std::array<State,TWOPOWDIM> directions_;									//vectors towards children (defines the children order)
};

#endif /* TREE_H_ */
