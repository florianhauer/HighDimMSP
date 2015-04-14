#include "Params.h"
#include "State.h"
#include "Node.h"
#include "Tree.h"
#include "MSP.h"
#include <iostream>     // std::cout

template <unsigned int DIM> bool isObstacle(State<DIM> s){
	if(s<State<DIM>())
		return true;
	else
		return false;
}

template <unsigned int DIM> bool addObstacles(Key<DIM> k, int depth, int size, Tree<DIM>* t){
	if(depth==t->getMaxDepth()){
		//finest resolution: update obstacle presence
		//if obstacles
		if(isObstacle(t->getState(k))){
			//add obstacle to the tree
			t->addObstacle(k);
			//indicate that the tree was updated
			return true;
		}else{
			//indicate that not changes were performed on the tree
			return false;
		}
	}else{
		bool update=false;
		//update children
		int size2=size>>1;
		 for(const Key<DIM>& dir: *(t->getDirections())){
			 update=addObstacles(k+dir*size2,depth+1,size2,t) || update;
		 }
		//if any children created, get node
		 if(update){
			 Node<DIM>* cur=t->getNode(k);
			 //prune and update val (single stage, no recurrence (children are up to date))
			 cur->update(false);
		 }
		 //indicate if updates were performed on the tree
		 return update;
	}
}

int main( int argc, const char* argv[] )
{
	//Create Tree
	Tree<4>* t=new Tree<4>();
	//Set Search Space Bounds
	State<4> minState={-1,-1,-1,-1};
	State<4> maxState={1,1,1,1};
	t->setStateBounds(minState,maxState);
	//Set Tree Max Depth
	int depth=4;
	t->setMaxDepth(depth);
	//Depth First Obstacle Creation
	addObstacles(t->getRootKey(),0,t->getRootKey()[0],t);
//	//print tree to check
//	std::streamsize prev=std::cout.width(0);
//	std::cout.flags(std::ios_base::right);
//	std::cout<<*(t->getRoot())<<std::endl;
//	std::cout.width(prev);

	//Create algo
	MSP<4> algo(t);
	//Set algo parameters
	State<4> start={1,-1,-1,-1};
	State<4> goal={1,1,1,1};
	algo.init(start*0.5,goal*0.5);
	//Run algo
	if(algo.run()){
		std::cout << "solution found" <<std::endl;
		std::deque<State<4>> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(typename std::deque<State<4>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
	}else{
		std::cout << "no solution found" <<std::endl;
	}
	//Visualize results

	delete t;

	std::cout << "no crash" << std::endl;
}
