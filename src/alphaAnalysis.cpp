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

int oleft=0;

template <unsigned int DIM> bool isObstacle(State<DIM> s){
	if(oleft>0){
		oleft--;
		return true;
	}
	if(rand()%100<4){
		oleft=rand()%40;
		return true;
	}else
		return false;
}

template <unsigned int DIM> bool isObstacle1(State<DIM> s){
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
			//t->addObstacle(k);
			//t->getNode(k)->setValue(0.0);
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

template <unsigned int DIM, unsigned int DEPTH> long hashKey(Key<DIM> k){
	long hash=0;
	for(int i=0;i<DIM;++i){
		hash=hash<<(DEPTH+1);
		hash+=k[i];
	}
	return hash;
}

template <unsigned int DIM, unsigned int DEPTH> Key<DIM> keyHash(long h){
	Key<DIM> k;
	for(int i=DIM-1;i>-1;--i){
		k[i]=h%(1<<(DEPTH+1));
		h=h>>(DEPTH+1);
	}
	return k;
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

		bool fail=false;
		double alpha=sqrt(DIM)/2;
		for(int d=1;d<DEPTH+3;++d,alpha*=2){
			//Create algo
			MSP<DIM> algo(t);
			algo.setSpeedUp(true);
			algo.setNewNeighboorCheck(true);
			algo.setMapLearning(false,0,0);
			algo.setMinRGcalc(true);
			algo.setAlpha(alpha);
			//Set algo parameters
			init=algo.init(start,goal);
			//Run algo
			start_wall_clock = std::chrono::steady_clock::now();
			if(init && algo.run()){
				finish_wall_clock = std::chrono::steady_clock::now();
				result[2]=(finish_wall_clock - start_wall_clock) / std::chrono::nanoseconds(1);
				std::cout << d << " , " << alpha << " , " << result[2] << " , " << algo.getPathCost() << " , " << algo.getPath().size() << std::endl;
				auto path=algo.getPathKeys();
				//*
				int count=0;
				for(auto it=path.begin();it!=path.end();++it){
					count+=t->getVolume(t->getNode(*it)->getDepth());
				}
				double length=0;
				double man=0;
				auto prev=path.begin();
				for(auto it=path.begin()+1;it!=path.end();++it){
					auto diff=(*it-*prev).abs();
					length+=diff.norm();
					man+=std::accumulate(diff.begin(),diff.end(),0.0);
					++prev;
				}
				std::cout << count << " , " << length << " , " << man << std::endl;//*/
				algo.clear();
			}else{
				fail=true;
				break;
			}
		}
		if(fail)
			continue;

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
			std::cout << "A*" << " , " << result[1] << " , " << r->Weight() << " , " << r->length() << std::endl;
			/*
			for(int i=0;i<r->length();++i){
				std::cout << keyHash<DIM,DEPTH>(r->GetVertex(i)->getID()) << " , ";
			}
			std::cout << std::endl;//*/
			double length=0;
			double man=0;
			auto prev=keyHash<DIM,DEPTH>(r->GetVertex(0)->getID());
			for(int i=1;i<r->length();++i){
				auto diff=(prev-keyHash<DIM,DEPTH>(r->GetVertex(i)->getID())).abs();
				length+=diff.norm();
				man+=std::accumulate(diff.begin(),diff.end(),0.0);
				 prev=keyHash<DIM,DEPTH>(r->GetVertex(i)->getID());
			}
			std::cout << r->length() << " , " << length << " , " << man << std::endl;//*/
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
	createMapRunMSPRunAs<4,5>();
	std::cout << "no crash" << std::endl;
}
