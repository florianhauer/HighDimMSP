#include <iostream>     // std::cout

template <unsigned int DIM> Tree<DIM>::Tree() {
	root_=new Node<DIM>(0.0f,0);
	maxDepth_=0;
	directions_[0]=Key<DIM>(1);
	int index=1;
	int pindex=1;
	for(int i=0;i<DIM;++i){
		for(int j=0;j<pindex;++j){
			Key<DIM> k(directions_[j]);
			k[i]=-k[i];
			directions_[index]=k;
			++index;
		}
		pindex=index;
	}
}

template <unsigned int DIM> Tree<DIM>::~Tree() {
	delete root_;
}

template <unsigned int DIM> void Tree<DIM>::setMaxDepth(int depth){
	maxDepth_=depth;
	int c=1<<depth;
	rootKey_=Key<DIM>(c);
}


template <unsigned int DIM> void Tree<DIM>::addObstacle(Key<DIM>& obs){
//	std::cout << "Adding obstacle at " << obs << std::endl;
	Node<DIM>* node=root_;
	Key<DIM> k(rootKey_);
	int height=maxDepth_-1;
	int size=k[0]>>1;
	Key<DIM> k2;
	while(height>=0){
//		std::cout << "Key " << k << " , height " << height << " , size " << size << std::endl;
		int i=0;
		for(;i<TwoPow<DIM>::value;++i){
			k2=k+(directions_[i]<<height)-obs;
//			std::cout << "k2 " << k2 << std::endl;
			if(std::all_of(k2.begin(),k2.end(),[size](int ki){return ki>-size && ki<size;})){
				break;
			}
		}
		if(i==TwoPow<DIM>::value){
			std::cout << obs << " , " << k2 << " , " << size << " , " << height << std::endl;
			std::cout << "can only add obstacles at max depth for now" << std::endl;
			return;
		}
		k=k2+obs;
		node=node->getChild(i);
		height--;
		size=size>>1;
	}
	node->setValue(1.0f);
}

template <unsigned int DIM> Node<DIM>* Tree<DIM>::getNode(const Key<DIM>& kt){
	Node<DIM>* node=root_;
	Key<DIM> k(rootKey_);
	int height=maxDepth_-1;
	int size=k[0]>>1;
	Key<DIM> k2;
	while(height>=0 && !node->isLeaf() && k!=kt){
		int i=0;
		for(;i<TwoPow<DIM>::value;++i){
			k2=k+(directions_[i]<<height)-kt;
			if(std::all_of(k2.begin(),k2.end(),[size](int ki){return ki>-size && ki<size;})){
				break;
			}
		}
		k=k2+kt;
		node=node->getChild(i);
		height--;
		size=size>>1;
	}
	return node;
}

template <unsigned int DIM> void Tree<DIM>::setStateBounds(const State<DIM>& minState,const State<DIM>& maxState){
	stateMin_=minState;
	stateInc_=maxState-minState;
}

template <unsigned int DIM> int Tree<DIM>::getVolume(int depth){
	return 1<<(DIM*(maxDepth_-depth));
}

template <unsigned int DIM> int Tree<DIM>::getSize(int depth){
	return 1<<(maxDepth_-depth);
}

template <unsigned int DIM> int Tree<DIM>::getSize(Key<DIM> k){
	return k[0]&-k[0];
}

template <unsigned int DIM> bool Tree<DIM>::getKey(const State<DIM>& s,Key<DIM>& k, bool inTree){
	if(s<stateMin_){
		return false;
	}
	State<DIM> ts=s-stateMin_;
	if(ts>stateInc_){
		return false;
	}
//	std::transform(ts.begin(),ts.end(),stateInc_.begin(),k.begin(),
//			[this](float si,float sInci){return (int)(si/sInci*(1<<(maxDepth_+1)));});
	k=rootKey_;
	Node<DIM>* node=root_;
	int height=maxDepth_-1;
	int size=k[0]>>1;
	while(height>=0 && getState(k)!=s && (!inTree || !node->isLeaf())){
		int i=0;
		std::array<double,TwoPow<DIM>::value> dist;
		for(;i<TwoPow<DIM>::value;++i){
			dist[i]=(getState(k+(directions_[i]<<height))-s).normSq();
		}
		i=std::distance(dist.begin(),std::min_element(dist.begin(),dist.end()));
		k=k+(directions_[i]<<height);
		if(inTree){
			node=node->getChild(i);
		}
		height--;
		size=size>>1;
	}
	return true;
}

template <unsigned int DIM> State<DIM> Tree<DIM>::getState(const Key<DIM>& k){
	State<DIM> s;
	std::transform(k.begin(),k.end(),stateInc_.begin(),s.begin(),[this](int ki,float sInci){return (ki*sInci/(1<<(maxDepth_+1)));});
	return stateMin_+s;
}
