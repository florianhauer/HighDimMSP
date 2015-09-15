#include "Params.h"
#include "State.h"
#include "Node.h"
#include "Tree.h"
#include "MSP.h"
#include <iostream>     // std::cout
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <chrono>   // std::chrono::seconds, milliseconds
#include <thread>   // std::this_thread::sleep_for


Tree<2>* t2=new Tree<2>();
Tree<3>* t3=new Tree<3>();
Tree<4>* t4=new Tree<4>();
Tree<5>* t5=new Tree<5>();
void delay(){
    std::this_thread::sleep_for(std::chrono::nanoseconds(10));
}

template <unsigned int DIM> bool isObstacle(State<DIM> s){
	delay();
	if(rand()%100<10)
		return true;
	else
		return false;
}

template <unsigned int DIM> bool isObstacle1(State<DIM> s){
	delay();
	Tree<DIM>* t;
	switch(DIM){
	case 2: t=(Tree<DIM>*)t2; break;
	case 3: t=(Tree<DIM>*)t3; break;
	case 4: t=(Tree<DIM>*)t4; break;
	case 5: t=(Tree<DIM>*)t5; break;
	}
	Key<DIM> k;
	t->getKey(s,k,true);
	return (t->getNode(k)->getValue()==1.0f);
}

template <unsigned int DIM> bool addObstacles(Key<DIM> k, int depth, int size, Tree<DIM>* t,std::vector<Key<DIM>>& vertices){
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
			vertices.push_back(k);
			return false;
		}
	}else{
		bool update=false;
		//update children
		int size2=size>>1;
		for(const Key<DIM>& dir: *(t->getDirections())){
			update=addObstacles(k+dir*size2,depth+1,size2,t,vertices) || update;
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

template <unsigned int DIM, unsigned int DEPTH> int hashKey(Key<DIM> k){
	int hash=0;
	for(int i=0;i<DIM;++i){
		hash=hash<<(DEPTH+1);
		hash+=k[i];
	}
	return hash;
}

template<unsigned int DIM, unsigned int DEPTH> State<6> createMapRunMSPRunAs(){
	bool success=false;
	while(!success){
		srand (time(NULL));
		State<6> result(0.0);
		t2->clear();t3->clear();t4->clear();t5->clear();
		//Create Tree
		Tree<DIM>* t;
		switch(DIM){
		case 2: t=(Tree<DIM>*)t2; break;
		case 3: t=(Tree<DIM>*)t3; break;
		case 4: t=(Tree<DIM>*)t4; break;
		case 5: t=(Tree<DIM>*)t5; break;
		}

		//Set Search Space Bounds
		State<DIM> minState(-1.0);
		State<DIM> maxState(1.0);
		t->setStateBounds(minState,maxState);
		//Set Tree Max Depth
		t->setMaxDepth(DEPTH);
		//Depth First Obstacle Creation
		std::vector<Key<DIM>> vertices;
		auto start_wall_clock = std::chrono::steady_clock::now();
		addObstacles(t->getRootKey(),0,t->getRootKey()[0],t,vertices);
		auto finish_wall_clock = std::chrono::steady_clock::now();
		result[0]=(finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1);

		bool init;
		State<DIM> start(-1+0.00001);
		State<DIM> goal ( 1-0.00001);

		//Create algo
		MSP<DIM> algo(t);
		algo.setSpeedUp(true);
		algo.setNewNeighboorCheck(false);
		algo.setMapLearning(false,0,0);
		algo.setMinRGcalc(false);
		//Set algo parameters
		init=algo.init(start,goal);
		//Run algo
		start_wall_clock = std::chrono::steady_clock::now();
		if(init && algo.run()){
			finish_wall_clock = std::chrono::steady_clock::now();
			result[2]=(finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1);
			algo.clear();
		}else{
			continue;
		}

		//Create algo
		MSP<DIM> algo2(t);
		algo2.setSpeedUp(true);
		algo2.setNewNeighboorCheck(true);
		algo2.setMapLearning(false,0,0);
		algo2.setMinRGcalc(false);
		//Set algo parameters
		init=algo2.init(start,goal);
		//Run algo
		start_wall_clock = std::chrono::steady_clock::now();
		if(init && algo2.run()){
			finish_wall_clock = std::chrono::steady_clock::now();
			result[3]=(finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1);
			algo2.clear();
		}else{
			continue;
		}

		//Create algo
		MSP<DIM> algo3(t);
		algo3.setSpeedUp(true);
		algo3.setNewNeighboorCheck(true);
		algo3.setMapLearning(true,10,&isObstacle1);
		algo3.setMinRGcalc(false);
		//Set algo parameters
		init=algo3.init(start,goal);
		//Run algo
		start_wall_clock = std::chrono::steady_clock::now();
		if(init && algo3.run()){
			finish_wall_clock = std::chrono::steady_clock::now();
			result[4]=(finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1);
			algo3.clear();
		}else{
			continue;
		}

		//Create algo
		MSP<DIM> algo4(t);
		algo4.setSpeedUp(true);
		algo4.setNewNeighboorCheck(true);
		algo4.setMapLearning(true,10,&isObstacle1);
		algo4.setMinRGcalc(true);
		//Set algo parameters
		init=algo4.init(start,goal);
		//Run algo
		start_wall_clock = std::chrono::steady_clock::now();
		if(init && algo4.run()){
			finish_wall_clock = std::chrono::steady_clock::now();
			result[5]=(finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1);
			algo4.clear();
		}else{
			continue;
		}

		//*
		//run A start on the same problem
		Key<DIM> ks(1);
		Key<DIM> kg=t->getRootKey()*2-ks;
		t->clear();
		int is=-1,ig=-1;
		astar::Graph graph;
		for(int i=0;i<vertices.size();++i){
			graph.add_vertex(hashKey<DIM,DEPTH>(vertices[i]),(vertices[i]-kg).norm());
			if((vertices[i]-ks).norm()==0){
				is=hashKey<DIM,DEPTH>(vertices[i]);
			}
			if((vertices[i]-kg).norm()==0){
				ig=hashKey<DIM,DEPTH>(vertices[i]);
			}
		}
		int maxValKey=1<<(DEPTH+1);
		for(int i=0;i<vertices.size();++i){
			for(int j=0;j<DIM;++j){
				Key<DIM> k(vertices[i]);
				k[j]+=2;
				if(k[j]<maxValKey){
					if(graph.get_vertex(hashKey<DIM,DEPTH>(k))!=NULL){
						graph.add_edge(hashKey<DIM,DEPTH>(vertices[i]),hashKey<DIM,DEPTH>(k),2);
						graph.add_edge(hashKey<DIM,DEPTH>(k),hashKey<DIM,DEPTH>(vertices[i]),2);
					}
				}
			}
		}
		vertices.clear();
		if(is==-1 || ig==-1){
			std::cout << "PROBLEM" << std::endl;
			exit(1);
		}
		start_wall_clock = std::chrono::steady_clock::now();
		//astar::YenTopKShortestPathsAlg yenAlg(graph, graph.get_vertex(is),graph.get_vertex(ig));
		astar::Astar sp;
		sp=astar::Astar(&graph);
		astar::BasePath* r=sp.get_shortest_path(graph.get_vertex(is),graph.get_vertex(ig));
		if(r->Weight()!=astar::Graph::DISCONNECT){
			finish_wall_clock = std::chrono::steady_clock::now();
			result[1]=(finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1);
			graph.clear();
			t->clear();
			return result;
		}else{
			continue;
		}//*/
	}
}

int main( int argc, const char* argv[] )
{
	/*auto start_wall_clock = std::chrono::steady_clock::now();
	clock_t t=clock();
	for(int i=0;i<1000;++i){
		int milisec = 10; // length of time to sleep, in miliseconds
		struct timespec req = {0};
		req.tv_sec = 0;
		req.tv_nsec = milisec * 1000000L;
		if(nanosleep(&req, (struct timespec *)NULL)==-1){
			std::cout << "error" <<std::endl;
		}
	}
	t=clock()-t;
	auto finish_wall_clock = std::chrono::steady_clock::now();
	std::cout << "1s = " << ((double)t)/CLOCKS_PER_SEC << std::endl;
    std::cout << "Wall clock: " << (finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1) << '\n';*/

	int nb_sim=20;

	std::cout << "map creation , a star , mspp , mspp fast neighbor , mspp sampling , mspp sampling min rg" << std::endl;

	State<6> results(0);
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<2,4>();
	}
	std::cout << std::endl << "results 2,4 : ";
	for(int i=0;i<6;++i){
		std::cout << results[i]/nb_sim/1000000000.0 << " , ";
	}
	std::cout << std::endl;

	results=State<6>(0);
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<3,4>();
	}
	std::cout << std::endl << "results 3,4 : ";
	for(int i=0;i<6;++i){
		std::cout << results[i]/nb_sim/1000000000.0 << " , ";
	}
	std::cout << std::endl;

	results=State<6>(0);
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<4,4>();
	}
	std::cout << std::endl << "results 4,4 : ";
	for(int i=0;i<6;++i){
		std::cout << results[i]/nb_sim/1000000000.0 << " , ";
	}
	std::cout << std::endl;

	//*
	nb_sim=5;
	results=State<6>(0);
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<5,4>();
	}
	std::cout << std::endl << "results 5,4 : " ;
	for(int i=0;i<6;++i){
		std::cout << results[i]/nb_sim/1000000000.0 << " , ";
	}
	std::cout << std::endl;
	//*/


	std::cout << "no crash" << std::endl;
}
