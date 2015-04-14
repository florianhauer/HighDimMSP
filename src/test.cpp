#include "Params.h"
#include "State.h"
#include "Node.h"
#include "Tree.h"
#include "MSP.h"
#include <iostream>     // std::cout
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

template <unsigned int DIM> bool isObstacle(State<DIM> s){
	if(rand()%100<10)
		return true;
	else
		return false;
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

template<unsigned int DIM, unsigned int DEPTH> State<2> createMapRunMSPRunAs(){
	bool success=false;
	srand (time(NULL));
	State<2> result;
	while(!success){
		//Create Tree
		Tree<DIM>* t=new Tree<DIM>();
		//Set Search Space Bounds
		State<DIM> minState(-1.0);
		State<DIM> maxState(1.0);
		t->setStateBounds(minState,maxState);
		//Set Tree Max Depth
		t->setMaxDepth(DEPTH);
		//Depth First Obstacle Creation
		std::vector<Key<DIM>> vertices;
		addObstacles(t->getRootKey(),0,t->getRootKey()[0],t,vertices);

		//Create algo
		MSP<DIM> algo(t);
		//Set algo parameters
		State<DIM> start(-1+0.00001);
		State<DIM> goal ( 1-0.00001);
		bool init=algo.init(start,goal);
		//Run algo
		clock_t tc = clock();
		if(init && algo.run()){
			tc = clock() - tc;
			success=true;
			result[0]=((double)tc)/CLOCKS_PER_SEC;
			//printf ("It took me %d clicks (%f seconds).\n",tc,((double)tc)/CLOCKS_PER_SEC);
			//std::cout << "solution found" <<std::endl;
			//run A start on the same problem
			Key<DIM> ks(1);
			Key<DIM> kg=t->getRootKey()*2-ks;
			int is=-1,ig=-1;
			kshortestpaths::Graph graph;
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
				/*
				for(int j=i+1;j<vertices.size();++j){
					if((vertices[i]-vertices[j]).norm()==2){
						graph.add_edge(i,j,2);
						graph.add_edge(j,i,2);
					}
				}*/
			}
			if(is==-1 || ig==-1){
				std::cout << "PROBLEM" << std::endl;
				exit(1);
			}
			tc = clock();
			kshortestpaths::YenTopKShortestPathsAlg yenAlg(graph, graph.get_vertex(is),graph.get_vertex(ig));
			if(yenAlg.has_next()){
				kshortestpaths::BasePath* r =yenAlg.next();
				tc = clock() - tc;
				success=true;
				result[1]=((double)tc)/CLOCKS_PER_SEC;
				return result;
			}
		}else{
			//std::cout << "no solution found" <<std::endl;
			srand (time(NULL)+rand());
		}
		//Visualize results

		delete t;
	}
}

int main( int argc, const char* argv[] )
{
	int nb_sim=3;

	State<2> results={0,0};
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<2,4>();
	}
	std::cout << std::endl << "results 2,4 : " << results[0]/nb_sim << " , " << results[1]/nb_sim << std::endl;
	std::cout << "Time reduced by " << (results[1]-results[0])/results[1]*100.0 << "%" << std::endl;

	results={0,0};
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<3,4>();
	}
	std::cout << std::endl << "results 3,4 : " << results[0]/nb_sim << " , " << results[1]/nb_sim << std::endl;
	std::cout << "Time reduced by "  << (results[1]-results[0])/results[1]*100.0 << "%" << std::endl;

	results={0,0};
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<4,4>();
	}
	std::cout << std::endl << "results 4,4 : " << results[0]/nb_sim << " , " << results[1]/nb_sim << std::endl;
	std::cout << "Time reduced by "  << (results[1]-results[0])/results[1]*100.0 << "%" << std::endl;

	nb_sim=1;
	results={0,0};
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<5,4>();
	}
	std::cout << std::endl << "results 5,4 : " << results[0]/nb_sim << " , " << results[1]/nb_sim << std::endl;
	std::cout << "Time reduced by "  << (results[1]-results[0])/results[1]*100.0 << "%" << std::endl;

	results={0,0};
	for(int i=0;i<nb_sim;++i){
		std::cout << i << " , " << std::flush;
		results=results+createMapRunMSPRunAs<6,4>();
	}
	std::cout << std::endl << "results 6,4 : " << results[0]/nb_sim << " , " << results[1]/nb_sim << std::endl;
	std::cout << "Time reduced by "  << (results[1]-results[0])/results[1]*100.0 << "%" << std::endl;


	std::cout << "no crash" << std::endl;
}
