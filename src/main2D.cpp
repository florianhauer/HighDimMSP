#include "Params.h"
#include "State.h"
#include "Node.h"
#include "Tree.h"
#include "MSP.h"
#include <iostream>     // std::cout

template <unsigned int DIM> bool isObstacle(State<DIM> s){
	if(s.norm()<0.2)
		return true;
	else
		return false;
}

template <unsigned int DIM> bool addObstacles(State<DIM> s, int depth, float scale, Tree<DIM>* t){
	if(depth==t->getMaxDepth()){
		//finest resolution: update obstacle presence
		//if obstacles
		if(isObstacle(s)){
			//add obstacle to the tree
			t->addObstacle(s);
			//indicate that the tree was updated
			return true;
		}else{
			//indicate that not changes were performed on the tree
			return false;
		}
	}else{
		bool update=false;
		//update children
		 for(const State<DIM>& dir: *(t->getDirections())){
			 update=addObstacles(s+dir*(0.5f*scale),depth+1,0.5f*scale,t) || update;
		 }
		//if any children created, get node
		 if(update){
			 Node<DIM>* cur=t->getNode(s,depth);
			 //prune and update val (single stage, no recurrence)
			 cur->update(false);
		 }
		 //indicate if updates were performed on the tree
		 return update;
	}
}

int main( int argc, const char* argv[] )
{
	//Create Tree
	Tree<2>* t=new Tree<2>();
	//Set Search Space Bounds
	State<2> minState={-1,-1};
	State<2> maxState={1,1};
	t->setStateBounds(minState,maxState);
	//Set Tree Max Depth
	int depth=4;
	t->setMaxDepth(depth);
	//Depth First Obstacle Creation
	//addObstacles(t->getRootState(),0,1.0f,t);
	State<2> sO={0.0625,0.0625};
	State<2> sI={0, 0.125};
	for(float i=-8;i<7;++i){
		if(i!=0){
			State<2> s=sO+sI*i;
			t->addObstacle(s);
		}
	}
	t->updateRec();
//	//print tree to check
//	std::streamsize prev=std::cout.width(0);
//	std::cout.flags(std::ios_base::right);
//	std::cout<<*(t->getRoot())<<std::endl;
//	std::cout.width(prev);

	//Create algo
	MSP<2> algo(t);
	//Set algo parameters
	State<2> start={-0.9,-0.9};
	State<2> goal={0.9,0.9};
	algo.init(start,goal);
	//Run algo
	if(algo.run()){
		std::cout << "solution found" <<std::endl;
		std::deque<State<2>> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<State<2>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
	}else{
		std::cout << "no solution found" <<std::endl;
	}
	//Visualize results

	delete t;

	std::cout << "no crash " << std::endl;
}
