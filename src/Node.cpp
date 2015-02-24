/*
 * Node.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#include "Node.h"
#include <algorithm>
#include <iomanip>

Node::Node(float val,int depth):val_(val),isLeaf_(true),depth_(depth) {
	childExists_.fill(false);
}

Node::~Node() {
	for(int i=0;i<TWOPOWDIM;++i)
		if(childExists_[i])
			delete children_[i];
}

Node* Node::addChild(int i){
	isLeaf_=false;
	childExists_[i]=true;
	children_[i]=new Node(0.0f,depth_+1);
	return children_[i];
}

bool  Node::childExists(int i){
	return childExists_[i];
}

double Node::update(bool rec){
	//pruning variables
	bool prunable=pruningOn;
	double prevVal=-1;
	//update
	double sum=0.0;
	for(int i=0;i<TWOPOWDIM;++i){
		if(childExists_[i]){
			if(rec){
				children_[i]->update(rec);
			}
			sum+=children_[i]->getValue();
			//check if pruning possible
			if(prunable){
				if(prevVal==-1){
					prevVal=children_[i]->getValue();
				}else{
					if(prevVal != children_[i]->getValue() || !children_[i]->isLeaf()){
						prunable=false;
					}
				}
			}
		}else{
			prunable=false;
		}
	}
	val_=sum/TWOPOWDIM;
	if(prunable){
		//prune
		for(int i=0;i<TWOPOWDIM;++i)
			if(childExists_[i])
				delete children_[i];
		isLeaf_=true;
		childExists_.fill(false);
	}
	return val_;
}

Node* Node::getChild(int i){
	if(childExists_[i]){
		return children_[i];
	}else{
		return addChild(i);
	}
}

bool Node::isEpsilonObstacle(){
	if(val_>1-epsilon/pow(2,DIM*(MAX_DEPTH-depth_)))
		return true;
	return false;
}

double Node::getVolume(){
	return pow(2,DIM*(MAX_DEPTH-depth_));
}

double Node::getScale(){
	return pow(0.5,depth_+1);
}

//print operator
std::ostream& operator<< (std::ostream& stream, const Node& n) {
	std::streamsize width=stream.width();
	stream << std::setw(width) << n.val_ << std::endl;
	for(int i=0;i<TWOPOWDIM;++i){
		if(n.childExists_[i]){
			stream <<std::setw(3)<< i << ", "<<std::setw(width+8)  <<*(n.children_[i]);
		}
	}
	stream.width(width);
	return stream;
}
