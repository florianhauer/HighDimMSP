/*
 * Tree.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#include "Tree.h"
#include <iostream>     // std::cout

Tree::Tree() {
	root_=new Node(0.0f,0);
}

Tree::~Tree() {
	delete root_;
}


void Tree::addObstacle(State& obs){
	Node* node=root_;
	State s=State(rootState_);
	int depth=0;
	float scale=0.5;
	while(depth!=maxDepth_){
		int i=0;
		for(;i<TWOPOWDIM;++i){
			if((s+directions_[i]*scale-obs).abs().isWithin(directions_[0]*scale)){
				break;
			}
		}
		s=s+directions_[i]*scale;
		node=node->getChild(i);
		depth=depth+1;
		scale*=0.5;
	}
	node->setValue(1.0f);
}

Node* Tree::getNode(State obj,int depth,State& s){
	Node* node=root_;
	s=rootState_;
	int ldepth=0;
	float scale=0.5;
	while(ldepth!=depth && !node->isLeaf() && (s-obj).normSq()!=0){ //TODO maybe add epsilon tolerance
		int i=0;
		for(;i<TWOPOWDIM;++i){
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

void Tree::setStateBounds(State minState,State maxState){
	rootState_=(minState+maxState)*0.5;
	State inc=(maxState-minState)*0.5;
	directions_[0]=inc.abs();
	int index=1;
	int pindex=1;
	for(int i=0;i<DIM;++i){
		for(int j=0;j<pindex;++j){
			State s=State(directions_[j]);
			s[i]=-s[i];
			directions_[index]=s;
			++index;
		}
		pindex=index;
	}
}
