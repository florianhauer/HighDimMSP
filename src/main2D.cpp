#include "Params.h"
#include "State.h"
#include "Node.h"
#include "Tree.h"
#include "MSP.h"
#include <iostream>     // std::cout

bool isObstacle(State s){
	if(s.norm()<0.2)
		return true;
	else
		return false;
}

bool addObstacles(State s, int depth, float scale, Tree* t){
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
		 for(const State& dir: *(t->getDirections())){
			 update=addObstacles(s+dir*(0.5f*scale),depth+1,0.5f*scale,t) || update;
		 }
		//if any children created, get node
		 if(update){
			 Node* cur=t->getNode(s,depth);
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
	Tree* t=new Tree();
	//Set Search Space Bounds
	VectorND minVec={-1,-1};
	State minState=State(minVec);
	VectorND maxVec={1,1};
	State maxState=State(maxVec);
	t->setStateBounds(minState,maxState);
	//Set Tree Max Depth
	int depth=MAX_DEPTH;
	t->setMaxDepth(depth);
	//Depth First Obstacle Creation
	addObstacles(t->getRootState(),0,1.0f,t);
	std::cout << "adding obstacles " << std::endl;
	VectorND obs={0.1,0.9};
	State sO(obs);
	VectorND inc={0,-0.1};
	State sI(inc);
	for(float i=0;i<19;++i){
		State s=sO+sI*i;
		t->addObstacle(s);
	}
	std::cout << "before tree update " << std::endl;
	t->updateRec();
	std::cout << "after tree update " << std::endl;
//	//print tree to check
	std::streamsize prev=std::cout.width(0);
	std::cout.flags(std::ios_base::right);
	std::cout<<*(t->getRoot())<<std::endl;
	std::cout.width(prev);

	//Create algo
	MSP algo(t);
	//Set algo parameters
	VectorND startVec={-0.9,-0.9};
	State start=State(startVec);
	VectorND goalVec={0.9,0.9};
	State goal=State(goalVec);
	algo.init(start,goal);
	//Run algo
	if(algo.run()){
		std::cout << "solution found" <<std::endl;
		std::deque<State> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<State>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
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
