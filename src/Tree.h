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
#include "Key.h"
#include <array>

template <unsigned int DIM> class Tree {
public:
	Tree();
	~Tree();

	void 				addObstacle(Key<DIM>& k);       											//create a node a finest resolution around s with value 1
	void 				updateRec(){root_->update(true);}												//recursively updates every value in the tree
	inline Node<DIM>* 	getRoot(){return root_;}														//returns the root of the tree
	void 				setStateBounds(const State<DIM>& minState,const State<DIM>& maxState);			//set the search space range
	inline void 		setMaxDepth(int depth)	;														//set the maximum depth of the tree
	inline int 			getMaxDepth(){return maxDepth_;}												//returns the max depth
	inline Key<DIM> 	getRootKey(){return rootKey_;}													//returns the spatial position of the center of the search space
	inline std::array<Key<DIM>,TwoPow<DIM>::value>* getDirections(){return &directions_;}				//returns the array of directions to find children
	Node<DIM>* 			getNode(const Key<DIM>& k);														//find the node corresponding to key k
	int 				getVolume(int depth);															//returns the volume of a node at a given depth (volume at maxDepth_ is 1)
	int 				getSize(int depth);																//returns the size at a given depth
	int 				getSize(Key<DIM> k);															//returns the size of a node given tis key k
	bool				getKey(const State<DIM>& s,Key<DIM>& k);										//return false for failure, or update k and return true
	State<DIM>			getState(const Key<DIM>& k);													//return the State corresponding to the key k

private:
	Node<DIM>* 			root_;																			//root of the tree
	int 				maxDepth_;																		//max depth of the tree
	Key<DIM> 			rootKey_;																		//key corresponding to the center of the search space
	State<DIM> 			stateMin_;																		//"bottom left corner" of the search space
	State<DIM> 			stateInc_;																		//state increment to reach the "top right corner" of the search space from stateMin
	std::array<Key<DIM>,TwoPow<DIM>::value> directions_;												//vectors towards children (defines the children order)
};

#include "Tree.hpp"

#endif /* TREE_H_ */
