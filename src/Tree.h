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
#include <array>

template <unsigned int DIM> class Tree {
public:
	Tree();
	~Tree();

	void addObstacle(State<DIM>& obs);       										//create a node a finest resolution around s with value 1
	void updateRec(){root_->update(true);}									//recursively updates every value in the tree
	double getVal(State<DIM> s);														//return the value of the node corresponding to state s
	inline Node<DIM>* getRoot(){return root_;}										//returns the root of the tree
	void setStateBounds(State<DIM> minState,State<DIM> maxState);							//set the search space range
	inline void setMaxDepth(int depth){maxDepth_=depth;}						//set the maximum depth of the tree
	inline int getMaxDepth(){return maxDepth_;}									//returns the max depth
	inline State<DIM> getRootState(){return rootState_;}								//returns the spacial position of the center of the search space
	inline std::array<State<DIM>,TwoPow<DIM>::value>* getDirections(){return &directions_;}	//returns the array of directions to find children
	Node<DIM>* getNode(State<DIM> obj,int depth,State<DIM>& s);								//find the node corresponding to state obj and depth smaller than depth, s becomes the corrected state
	Node<DIM>* getNode(State<DIM> s,State<DIM>& sc){return getNode(s,maxDepth_,sc);}			//find the node corresponding to state s at any depth, sc becomes the corrected state
	Node<DIM>* getNode(State<DIM> obj,int depth){State<DIM> s;return getNode(obj,depth,s);}	//find the node corresponding to state s and depth smaller than depth
	Node<DIM>* getNode(State<DIM> s){return getNode(s,maxDepth_);}						//find the node corresponding to state s at any depth

private:
	Node<DIM>* root_;																//root of the tree
	int maxDepth_;																//max depth of the tree
	State<DIM> rootState_;															//state corresponding to the center of the search space
	std::array<State<DIM>,TwoPow<DIM>::value> directions_;									//vectors towards children (defines the children order)
};

/*
 * Tree.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#include "Tree.h"
#include <iostream>     // std::cout

template <unsigned int DIM> Tree<DIM>::Tree() {
	root_=new Node<DIM>(0.0f,0);
	maxDepth_=0;
}

template <unsigned int DIM> Tree<DIM>::~Tree() {
	delete root_;
}


template <unsigned int DIM> void Tree<DIM>::addObstacle(State<DIM>& obs){
	Node<DIM>* node=root_;
	State<DIM> s=State<DIM>(rootState_);
	int depth=0;
	float scale=0.5;
	while(depth!=maxDepth_){
		int i=0;
		for(;i<TwoPow<DIM>::value;++i){
			if((s+directions_[i]*scale-obs).abs().isWithin(directions_[0]*scale)){
				break;
			}
		}
		if(i==TwoPow<DIM>::value){
			std::cout << "can only add obstacles at max depth for now" << std::endl;
			return;
		}
		s=s+directions_[i]*scale;
		node=node->getChild(i);
		depth=depth+1;
		scale*=0.5;
	}
	node->setValue(1.0f);
}

template <unsigned int DIM> Node<DIM>* Tree<DIM>::getNode(State<DIM> obj,int depth,State<DIM>& s){
	Node<DIM>* node=root_;
	s=rootState_;
	int ldepth=0;
	float scale=0.5;
	while(ldepth!=depth && !node->isLeaf() && (s-obj).normSq()!=0){ //TODO maybe add epsilon tolerance
		int i=0;
		for(;i<TwoPow<DIM>::value;++i){
			if((s+directions_[i]*scale-obj).abs().isWithin(directions_[0]*scale)){
				break;
			}
		}
		s=s+directions_[i]*scale;
		node=node->getChild(i);
		ldepth=ldepth+1;
		scale*=0.5;
	}
	return node;
}

template <unsigned int DIM> void Tree<DIM>::setStateBounds(State<DIM> minState,State<DIM> maxState){
	rootState_=(minState+maxState)*0.5;
	State<DIM> inc=(maxState-minState)*0.5;
	directions_[0]=inc.abs();
	int index=1;
	int pindex=1;
	for(int i=0;i<DIM;++i){
		for(int j=0;j<pindex;++j){
			State<DIM> s(directions_[j]);
			s[i]=-s[i];
			directions_[index]=s;
			++index;
		}
		pindex=index;
	}
}


#endif /* TREE_H_ */
