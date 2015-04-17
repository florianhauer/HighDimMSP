#include "Params.h"
#include "State.h"
#include "Node.h"
#include "Tree.h"
#include "MSP.h"
#include <iostream>     // std::cout
#include <ctime>

template <unsigned int DIM> bool isObstacle(State<DIM> s){
	if(s.norm()<0.2)
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

Tree<2>* t;

bool obsCheck(State<2> s){
	Key<2> k;
	t->getKey(s,k,true);
	if(t->getNode(k)->getValue()>0.5)
		return true;
	return false;
}

int main( int argc, const char* argv[] )
{
	//Create Tree
	t=new Tree<2>();
	//Set Search Space Bounds
	State<2> minState={-1,-1};
	State<2> maxState={1,1};
	t->setStateBounds(minState,maxState);
	//Set Tree Max Depth
	int depth=4;
	t->setMaxDepth(depth);
	//Depth First Obstacle Creation
//	addObstacles(t->getRootKey(),0,t->getRootKey()[0],t);
	State<2> sO={0.0625,0.0625};
	State<2> sI={0, 0.125};
	for(float i=-8;i<8;++i){
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

	Tree<2>* t2=new Tree<2>();
	t2->copyParams(t);
	//Create algo
	MSP<2> algo(t);
	//Set algo parameters
	State<2> start={-0.9,-0.9};
	State<2> goal={0.9,0.9};
	algo.setMapLearning(true,100,obsCheck);
	bool init=algo.init(start,goal);
	std::cout << "init " << init << std::endl;
	//Run algoclock_t tc;
	clock_t tc = clock();
	if(init && algo.run()){
		tc = clock() - tc;
		printf ("It took me %d clicks (%f seconds).\n",(int)tc,((float)tc)/CLOCKS_PER_SEC);
		std::cout << "solution found" <<std::endl;
		std::deque<State<2>> sol=algo.getPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<State<2>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
		std::cout << "Smoothed solution " << std::endl;
		sol=algo.getSmoothedPath();
		std::cout << "Path length: " << sol.size() << std::endl;
		std::cout << "Path cost: " << algo.getPathCost() << std::endl;
		std::cout << "Path :" << std::endl;
		for(std::deque<State<2>>::iterator it=sol.begin(),end=sol.end();it!=end;++it){
			std::cout << (*it) << " -- ";
		}
		std::cout << std::endl;
	}else{
		tc = clock() - tc;
		printf ("It took me %d clicks (%f seconds).\n",(int)tc,((float)tc)/CLOCKS_PER_SEC);
		std::cout << "no solution found" <<std::endl;
	}
	//Visualize results


	delete t;
	delete t2;

	std::cout << "no crash " << std::endl;
	bool disppdf=false;
	if(disppdf){
		std::cout << "Compiling pdf" << std::endl;
		std::stringstream ss;
		ss << "cd " << RESDIR << ";pdflatex -interaction=nonstopmode beamer.tex >/dev/null;evince -i 0 -s beamer.pdf >/dev/null";
		system(ss.str().c_str());
	}
}
