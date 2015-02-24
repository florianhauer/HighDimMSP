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

template <unsigned int DIM> class Node {
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
	template <unsigned int DIM2> friend std::ostream& 	operator<< (std::ostream& stream, const Node<DIM2>& n);	//print operator
	bool					isEpsilonObstacle();								//check if a node is an epsilon obstacle
	double					getVolume();										//returns the volume of the node
	double					getScale();											//returns the scaling factor to reach children

private:
	double 									val_;					//average value of the children
	std::array<Node*,TwoPow<DIM>::value> 	children_;				//children array
	std::array<bool,TwoPow<DIM>::value> 	childExists_;			//existing children indicator
	bool 									isLeaf_;				//whether that node is a leaf of the tree
	int										depth_;					//depth in the tree

	static const bool						pruningOn=true;			//whether to prune during updates
	static constexpr double 				epsilon=0.5;			//threshold for epsilon-obstacle
};



/*
 * Node.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: florian
 */

#include "Node.h"
#include <algorithm>
#include <iomanip>

template <unsigned int DIM> Node<DIM>::Node(float val,int depth):val_(val),isLeaf_(true),depth_(depth) {
	childExists_.fill(false);
}

template <unsigned int DIM> Node<DIM>::~Node() {
	for(int i=0;i<TwoPow<DIM>::value;++i)
		if(childExists_[i])
			delete children_[i];
}

template <unsigned int DIM> Node<DIM>* Node<DIM>::addChild(int i){
	isLeaf_=false;
	childExists_[i]=true;
	children_[i]=new Node<DIM>(0.0f,depth_+1);
	return children_[i];
}

template <unsigned int DIM> bool  Node<DIM>::childExists(int i){
	return childExists_[i];
}

template <unsigned int DIM> double Node<DIM>::update(bool rec){
	if(isLeaf()){
		return val_;
	}
	//pruning variables
	bool prunable=pruningOn;
	double prevVal=-1;
	//update
	double sum=0.0;
	for(int i=0;i<TwoPow<DIM>::value;++i){
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
	val_=sum/TwoPow<DIM>::value;
	if(prunable){
		//prune
		for(int i=0;i<TwoPow<DIM>::value;++i)
			if(childExists_[i])
				delete children_[i];
		isLeaf_=true;
		childExists_.fill(false);
	}
	return val_;
}

template <unsigned int DIM> Node<DIM>* Node<DIM>::getChild(int i){
	if(childExists_[i]){
		return children_[i];
	}else{
		return addChild(i);
	}
}

template <unsigned int DIM> bool Node<DIM>::isEpsilonObstacle(){
	if(val_>1-epsilon/pow(2,DIM*(MAX_DEPTH-depth_)))
		return true;
	return false;
}

template <unsigned int DIM> double Node<DIM>::getVolume(){
	return pow(2,DIM*(MAX_DEPTH-depth_));
}

template <unsigned int DIM> double Node<DIM>::getScale(){
	return pow(0.5,depth_+1);
}

//print operator
template <unsigned int DIM> std::ostream& operator<< (std::ostream& stream, const Node<DIM>& n) {
	std::streamsize width=stream.width();
	stream << std::setw(width) << n.val_ << std::endl;
	for(int i=0;i<TwoPow<DIM>::value;++i){
		if(n.childExists_[i]){
			stream <<std::setw(3)<< i << ", "<<std::setw(width+8)  <<*(n.children_[i]);
		}
	}
	stream.width(width);
	return stream;
}


#endif /* NODE_H_ */
