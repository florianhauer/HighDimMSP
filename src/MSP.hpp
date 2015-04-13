#include <kshortestpaths/DijkstraShortestPathAlg.h>
#include <kshortestpaths/YenTopKShortestPathsAlg.h>
#include <set>
#include <cmath>
#include <numeric>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <algorithm>


template <unsigned int DIM> MSP<DIM>::MSP(Tree<DIM>* tree):m_tree(tree) {
	m_M=100*pow(2,DIM*tree->getMaxDepth());
	m_epsilon=0.5;
	m_current_size=0;
	m_end_index=0;
	m_nb_backtrack=0;
	m_nb_step=0;
	m_start_index=0;
	m_speed_up=true;
	m_path_found=false;
	m_alpha=0.55*sqrt(DIM);
	m_lambda1=0.999;
	m_lambda2=0.001;
}

template <unsigned int DIM> bool MSP<DIM>::isEpsilonObstacle(Node<DIM>* n){
	if(n->getValue()>1-m_epsilon/m_tree->getVolume(n->getDepth()))
		return true;
	return false;
}


template <unsigned int DIM> bool MSP<DIM>::init(State<DIM> start,State<DIM> end){
	Key<DIM> startKey;
	Key<DIM> goalKey;
	if(!(m_tree->getKey(start,startKey,true) && m_tree->getKey(end,goalKey,true))){
		std::cout << "Error converting state to key" << std::endl;
		return false;
	}
	Node<DIM>* nstart=m_tree->getNode(startKey);
	Node<DIM>* ngoal=m_tree->getNode(goalKey);
	if(nstart->isLeaf() && ngoal->isLeaf() && !isEpsilonObstacle(nstart) && !isEpsilonObstacle(ngoal)){
		m_current_coord=startKey;
		m_current_size=m_tree->getSize(nstart->getDepth());
		m_start_coord=startKey;
		m_end_coord=goalKey;
		m_current_path.clear();
		m_path_cost.clear();
		m_current_path.push_back(m_start_coord);
		m_misleading.clear();
		m_misleading[m_current_coord]=std::set<Key<DIM>>();
		m_nb_step=0;
		m_nb_backtrack=0;
		return true;
	}else{
		/*
		std::cout << "start or goal not leaf on free space" << std::endl;
		std::cout << "desired start " << start <<std::endl;
		std::cout << "start " << startKey << " , "
				<< m_tree->getState(startKey) << " , leaf : " <<nstart->isLeaf()
				<< " , epsilon obstacle : " << isEpsilonObstacle(nstart) <<std::endl;
		std::cout << "desired goal " << end <<std::endl;
		std::cout << "goal " << goalKey << " , "
				<< m_tree->getState(goalKey) << " , leaf : " <<ngoal->isLeaf()
				<< " , epsilon obstacle : " << isEpsilonObstacle(ngoal) <<std::endl;*/
		//exit(1);
	}
	return false;
}

template <unsigned int DIM> bool MSP<DIM>::step(){
	reducedGraph();
	kshortestpaths::YenTopKShortestPathsAlg yenAlg(m_graph, m_graph.get_vertex(m_start_index),m_graph.get_vertex(m_end_index));
	//if solution
	if(yenAlg.has_next()){
		//go forward // if goal return false;
		kshortestpaths::BasePath* result =yenAlg.next();
		iterationDetails(result);
		int next_point_id=result->GetVertex(1)->getID();
		//do stuff to prepare next iteration
		m_misleading[m_current_coord].insert(m_nodes[next_point_id].first);
		m_current_path.push_back(m_nodes[next_point_id].first);
		m_path_cost.push_back(m_cost[next_point_id]);

		if(m_speed_up){
			int mv_fwd=2;
			while(result->length()>mv_fwd){
				int next_point_id2=result->GetVertex(mv_fwd)->getID();
				if(m_tree->getNode(m_nodes[next_point_id2].first)->isLeaf()){
					m_misleading[m_nodes[next_point_id].first].insert(m_nodes[next_point_id2].first);
					m_current_path.push_back(m_nodes[next_point_id2].first);
					m_path_cost.push_back(m_cost[next_point_id2]);
					next_point_id=next_point_id2;
					++mv_fwd;
				}else{
					break;
				}
			}
		}

		m_current_coord=m_nodes[next_point_id].first;
		m_current_size=m_nodes[next_point_id].second;

		if(next_point_id==m_end_index){
			//std::cout << "goal reached in " << m_nb_step << " iterations" << std::endl;
			m_path_found=true;
			return false;
		}else{
			return true;
		}

	}else{
		iterationDetails();
		m_misleading[m_current_coord].clear();
		m_nb_backtrack++;
		m_current_path.pop_back();
		if(m_current_path.size()==0){
			//no possible path
			return false;
		}else{
			m_current_coord=m_current_path.back();
			m_current_size=m_tree->getSize(m_current_coord);
			m_path_cost.pop_back();
			return true;
		}
	}
}

template <unsigned int DIM> bool MSP<DIM>::run(){
	while(step()){/*std::cout<<*/++m_nb_step;}
	//std::cout<< "NB backtrack : " << m_nb_backtrack << std::endl;
	if(m_path_found){
		return true;
	}else{
		return false;
	}
}

template <unsigned int DIM> std::deque<State<DIM>> MSP<DIM>::getPath(){
	std::deque<State<DIM>> path(m_current_path.size());
	std::transform(m_current_path.begin(),m_current_path.end(),path.begin(),[this](Key<DIM> k){return m_tree->getState(k);});
	return path;
}

template <unsigned int DIM> std::deque<State<DIM>> MSP<DIM>::getSmoothedPath(){
//	m_tree->updateRec();
	std::deque<State<DIM>> sPath;
	sPath.push_back(m_tree->getState(m_current_path[0]));
	Key<DIM> cur=m_current_path[0];
	int i=2;
	while(i<m_current_path.size()){
		Key<DIM> cur2=m_current_path[i];
		auto keys=m_tree->getRayKeys(cur,cur2);
		if(std::any_of(keys.begin(),keys.end(),[this](Key<DIM> k){return isEpsilonObstacle(m_tree->getNode(k));})){
			cur=m_current_path[i-1];
			sPath.push_back(m_tree->getState(cur));
		}
		i++;
	}
	sPath.push_back(m_tree->getState(m_current_path[m_current_path.size()-1]));
	return sPath;
}

template <unsigned int DIM> bool MSP<DIM>::inPath(Key<DIM> pt,int size){
	return std::any_of(m_current_path.begin(),m_current_path.end(),
			[pt,size,this](Key<DIM> it){return this->is_in(it,std::pair<Key<DIM>,int>(pt,size)) && it!=m_current_path.back();});
}

template <unsigned int DIM> void MSP<DIM>::add_node_to_reduced_vertices(Node<DIM>* node,Key<DIM> coord, int size){
	//	std::cout << coord << " , " << scale << " , " << node->getValue() << " , " << node->isEpsilonObstacle() << " , " << inPath(coord,scale) << " , " << (m_current_forbidden.find(coord)==m_current_forbidden.end()) << std::endl;
	if( ( ((coord-m_current_coord).normSq()>(m_alpha*(size<<1)+sqrt(DIM)*m_current_size)*(m_alpha*(size<<1)+sqrt(DIM)*m_current_size)) || node->isLeaf())
			&& !inPath(coord,size)
			&& !isEpsilonObstacle(node)
			&& m_current_forbidden.find(coord)==m_current_forbidden.end()
	){
		m_nodes.push_back(std::pair<Key<DIM>,int>(coord,size));
		m_cost.push_back(cost(node));
	}else{
		int s=size>>1;
		if(!node->isLeaf()){
			for(int i=0;i<TwoPow<DIM>::value;++i){
				add_node_to_reduced_vertices(node->getChild(i),coord+(*(m_tree->getDirections()))[i]*s,size*0.5);
			}
		}
	}
}

template <unsigned int DIM> void MSP<DIM>::reducedGraph(){
	m_graph.clear();
	m_nodes.clear();
	m_cost.clear();
	m_start_index=-1;
	m_end_index=-1;
	try {
		m_current_forbidden=m_misleading.at(m_current_coord);
	}catch (const std::out_of_range& oor) {
		m_current_forbidden=std::set<Key<DIM>>();
	}
	add_node_to_reduced_vertices(m_tree->getRoot(),m_tree->getRootKey(),m_tree->getRootKey()[0]);

	int l=m_nodes.size();

	for(int i=0;i<l;++i){
		m_graph.add_vertex(i,m_lambda2*(m_nodes[i].first-m_end_coord).norm());  //TODO: use square of the cost to remove square roots
		if(is_start(m_nodes[i])){
			if(m_start_index!=-1){
				std::cout << "2 start nodes, fail" << std::endl;
				return;
				//exit(1);
			}
			m_start_index=i;
		}
		if(is_goal(m_nodes[i])){
			if(m_end_index!=-1){
				std::cout << "2 end nodes, fail" << std::endl;
				return;
				//exit(1);
			}
			m_end_index=i;
		}
	}
	for(int i=0;i<l;++i){
		for(int j=i+1;j<l;++j){
			if(neighboor(m_nodes[i],m_nodes[j])){
				m_graph.add_edge(i,j,m_cost[j]);
				m_graph.add_edge(j,i,m_cost[i]);
			}
		}
	}
	if(m_start_index==-1){
		std::cout << "0 start node, fail" << std::endl;
		return;
	}
	if(m_end_index==-1){
		std::cout << "0 end node, fail" << std::endl;
		return;
	}
}

template <unsigned int DIM> bool MSP<DIM>::is_in(Key<DIM>& pt,std::pair<Key<DIM>,int> node){
	Key<DIM> k=pt-node.first;
	if(std::all_of(k.begin(),k.end(),[&node](int ki){return ki>-node.second && ki<node.second;}))
		return true;
	return false;
}

template <unsigned int DIM> double MSP<DIM>::cost(Node<DIM>* n){
	if (!isEpsilonObstacle(n)){
		return (m_lambda1*n->getValue()+m_lambda2)*m_tree->getVolume(n->getDepth());
	}else{
		return m_M;
	}
}

template <unsigned int DIM> bool MSP<DIM>::neighboor(std::pair<Key<DIM>,int> &na,std::pair<Key<DIM>,int> &nb){
	int l=na.second+nb.second;
	Key<DIM> diff=(na.first-nb.first).abs();
	std::partial_sort(diff.begin(),diff.begin()+2,diff.end(),std::greater<int>());
	if(diff[0]==l && diff[1]!=l)
		return true;
	return false;
}

template <unsigned int DIM> void MSP<DIM>::iterationDetails(kshortestpaths::BasePath* result){
	bool console=false;
	if(console){
		std::cout << std::endl << std::endl << "Iteration " << m_nb_step << std::endl
				<< "nkipi: " << m_current_coord << " with scale factor " << m_current_size << std::endl;
		std::cout<< "rejects : ";
		for(auto& c : m_current_forbidden)
			std::cout << c << " , ";
		std::cout << std::endl;
		std::cout << "Gi:" <<std::endl;
		for(int i=0;i<m_nodes.size();++i){
			std::cout << "Vertex " << i << " at " << m_nodes[i].first << " with size " << m_nodes[i].second << " and cost " << m_cost[i] << ", neighbor with ";
			for(int j=i+1;j<m_nodes.size();++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
					std::cout << j << " , ";
				}
			}
			std::cout << std::endl;
		}
	}
	bool latex=false;
	if(latex && DIM==2){
		if(m_nb_step==0){
			//remove previous results
			std::stringstream ss2;
			ss2 << "rm -r " << RESDIR << "/iterationFiles";
			system(ss2.str().c_str());
			std::stringstream ss3;
			ss3 << "mkdir -p " << RESDIR << "/iterationFiles/";
			system(ss3.str().c_str());
			std::stringstream ss;
			ss << RESDIR << "/iterationFiles/environment.tex";
			std::fstream file(ss.str(),std::fstream::out);
			file << "\\begin{tikzpicture}[scale=\\picScale*32/" << (1<<m_tree->getMaxDepth()+1) << "]" << std::endl
					<< "\\tikzstyle{treenodes}=[black,thick,fill=white]" <<std::endl
					<< "\\tikzstyle{every node}=[circle,draw,minimum size=2pt,inner sep=1pt];" <<std::endl
					<< "\\node[rectangle,draw] at (" << (1<<m_tree->getMaxDepth()) << "," << (1<<m_tree->getMaxDepth()+1)+2 << ") {Environment};" <<std::endl
					<< "\\draw[black,thick,fill=white] (0,0) rectangle (" << (1<<m_tree->getMaxDepth()+1) << "," << (1<<m_tree->getMaxDepth()+1) << ");" <<std::endl;
			drawTree(file);
			file << "\\end{tikzpicture}" << std::endl;
			file.close();
		}
		std::stringstream ss;
		ss << RESDIR << "/iterationFiles/iteration" << m_nb_step << ".tex";
		std::fstream file(ss.str(),std::fstream::out);
		file << "\\begin{tikzpicture}[scale=\\picScale*32/" << (1<<m_tree->getMaxDepth()+1) << "]" << std::endl
				<< "\\tikzstyle{treenodes}=[black,thick,fill=white]" <<std::endl
				<< "\\tikzstyle{every node}=[circle,draw,minimum size=2pt,inner sep=1pt];" <<std::endl
				<< "\\node[rectangle,draw] at (" << (1<<m_tree->getMaxDepth()) << "," << (1<<m_tree->getMaxDepth()+1)+2 << ") {Iteration " << m_nb_step << "};" <<std::endl
				<< "\\draw[black,thick,fill=blue] (0,0) rectangle (" << (1<<m_tree->getMaxDepth()+1) << "," << (1<<m_tree->getMaxDepth()+1) << ");" <<std::endl;
		for(int i=0;i<m_nodes.size();++i){
			file << "\\draw[treenodes, fill=red!" << m_tree->getNode(m_nodes[i].first)->getValue()*100.0 << "] "
					<< m_nodes[i].first-(*(m_tree->getDirections()))[0]*m_nodes[i].second
					<< " rectangle "
					<< m_nodes[i].first+(*(m_tree->getDirections()))[0]*m_nodes[i].second
					<< ";" << std::endl;
		}
		for(int i=0;i<m_nodes.size();++i){
			file << "\\node";
			if(i==m_start_index)
				file << "[green,thick]";
			if(i==m_end_index)
				file << "[red,thick]";
			file << " at " << m_nodes[i].first << " (" << i << ") {" << i << "};" << std::endl;
		}
		for(int i=0;i<m_nodes.size();++i){
			for(int j=i+1;j<m_nodes.size();++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
					file << "\\path[draw] (" << i << ") -- (" << j << ");" << std::endl;
				}
			}
		}
		if(m_current_path.size()>1){
			file << "\\path[draw,thick] " << m_current_path.front();
			if(m_current_path.size()>2){
				for(typename std::deque<Key<DIM>>::iterator it=m_current_path.begin()+1,end=m_current_path.end()-1;it!=end;++it){
					file << " -- " << *it ;
				}
			}
			file << "-- (" << m_start_index << ");" << std::endl;
		}
		if(result!=NULL){
			file << "\\path[draw,thick,orange] (" << m_start_index <<")";
			for(int i=1;i<result->length()-1;++i){
				file << " -- (" << result->GetVertex(i)->getID() << ")";
			}
			file << "-- (" << m_end_index << ");" << std::endl;
		}
		file << "\\end{tikzpicture}" << std::endl;
		file.close();
	}
}

template <unsigned int DIM> void MSP<DIM>::drawTreeRec(std::ostream& stream, Key<DIM> k, Node<DIM>* n, int size){
	if(n->isLeaf()){
		if(isEpsilonObstacle(n)){
			stream << "\\draw[treenodes, fill=red] "
					<< k-(*(m_tree->getDirections()))[0]*size
					<< " rectangle "
					<< k+(*(m_tree->getDirections()))[0]*size
					<< ";" << std::endl;
		}
	}else{
		int s=size>>1;
		for(int i=0; i<TwoPow<DIM>::value;++i){
			if(n->childExists(i)){
				drawTreeRec(stream,k+(*(m_tree->getDirections()))[i]*s,n->getChild(i),s);
			}
		}
	}
}

template <unsigned int DIM> void MSP<DIM>::drawTree(std::ostream& stream){
	drawTreeRec(stream, m_tree->getRootKey(), m_tree->getRoot(),m_tree->getRootKey()[0]);
}
