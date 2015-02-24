#include "MSP.h"
#include "DijkstraShortestPathAlg.h"
#include "YenTopKShortestPathsAlg.h"
#include <set>
#include <cmath>
#include <numeric>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <algorithm>


MSP::MSP(Tree* tree):m_tree(tree) {
	m_M=100*pow(2,DIM*tree->getMaxDepth());
	m_epsilon=0.5;
	m_current_scale=0;
	m_end_index=0;
	m_nb_backtrack=0;
	m_nb_step=0;
	m_start_index=0;
	m_speed_up=false;
	m_path_found=false;
	m_alpha=0.55*sqrt(DIM);
	m_lambda1=0.999;
	m_lambda2=0.001;
}

bool MSP::init(State start,State end){
	State startc;
	State goalc;
	Node* nstart=m_tree->getNode(start,startc);
	Node* ngoal=m_tree->getNode(end,goalc);
	if(nstart->isLeaf() && ngoal->isLeaf() && !nstart->isEpsilonObstacle() && !ngoal->isEpsilonObstacle()){
		m_current_coord=startc;
		m_current_scale=nstart->getScale();
		m_start_coord=startc;
		m_end_coord=goalc;
		m_current_path.clear();
		m_path_cost.clear();
		m_current_path.push_back(m_start_coord);
		m_misleading.clear();
		m_misleading[m_current_coord]=std::set<State>();
		m_nb_step=0;
		m_nb_backtrack=0;
		return true;
	}else{
		std::cout << "start or goal not leaf on free space" << std::endl;
		exit(1);
	}
	return false;
}

bool MSP::step(){
	reducedGraph();
	iterationDetails();
	kshortestpaths::YenTopKShortestPathsAlg yenAlg(m_graph, m_graph.get_vertex(m_start_index),m_graph.get_vertex(m_end_index));
	//if solution
	if(yenAlg.has_next()){
		//go forward // if goal return false;
		kshortestpaths::BasePath* result =yenAlg.next();
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
		m_current_scale=m_nodes[next_point_id].second;

		if(next_point_id==m_end_index){
			std::cout << "goal reached" << std::endl;
			m_path_found=true;
			return false;
		}else{
			return true;
		}

	}else{
		m_misleading[m_current_coord].clear();
		m_nb_backtrack++;
		m_current_path.pop_back();
		if(m_current_path.size()==0){
			//no possible path
			return false;
		}else{
			m_current_coord=m_current_path.back();
			m_current_scale=m_tree->getNode(m_current_coord)->getScale();
			m_path_cost.pop_back();
			return true;
		}
	}
}

bool MSP::run(){
	while(step()){/*std::cout<<*/++m_nb_step;}
	std::cout<< "NB backtrack : " << m_nb_backtrack << std::endl;
	if(m_path_found){
		return true;
	}else{
		return false;
	}
}

bool MSP::inPath(State pt,double scale){
	return std::any_of(m_current_path.begin(),m_current_path.end(),
			[pt,scale,this](State it){return is_in(it,std::pair<State,double>(pt,scale)) && (it-m_current_path.back()).normSq()!=0;});
}

void MSP::add_node_to_reduced_vertices(Node* node,State coord, double scale){
//	std::cout << coord << " , " << scale << " , " << node->getValue() << " , " << node->isEpsilonObstacle() << " , " << inPath(coord,scale) << " , " << (m_current_forbidden.find(coord)==m_current_forbidden.end()) << std::endl;
	if(((coord-m_current_coord).norm()-0.5*sqrt(DIM)*m_current_scale*4*((*(m_tree->getDirections()))[0].max())>m_alpha*scale*4*((*(m_tree->getDirections()))[0].max()) || node->isLeaf())
			&& !inPath(coord,scale)
			&& !node->isEpsilonObstacle()
			&& m_current_forbidden.find(coord)==m_current_forbidden.end()
	){
		m_nodes.push_back(std::pair<State,double>(coord,scale));
		m_cost.push_back(cost(node));
	}else{
		if(!node->isLeaf()){
			for(int i=0;i<TWOPOWDIM;++i){
				add_node_to_reduced_vertices(node->getChild(i),coord+(*(m_tree->getDirections()))[i]*scale,scale*0.5);
			}
		}
	}
}

void MSP::reducedGraph(){
	m_graph.clear();
	m_nodes.clear();
	m_cost.clear();
	m_start_index=-1;
	m_end_index=-1;
	try {
		m_current_forbidden=m_misleading.at(m_current_coord);
	}catch (const std::out_of_range& oor) {
		m_current_forbidden=std::set<State>();
	}
	add_node_to_reduced_vertices(m_tree->getRoot(),m_tree->getRootState(),0.5);

	int l=m_nodes.size();

	for(int i=0;i<l;++i){
		m_graph.add_vertex(i,m_lambda2*(m_nodes[i].first-m_end_coord).norm());
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

bool MSP::is_in(State pt,std::pair<State,double> node){
	if((pt-node.first).abs().isWithin((*(m_tree->getDirections()))[0]*(2*node.second)))
		return true;
	return false;
}

double MSP::cost(Node* n){
	if (!n->isEpsilonObstacle()){
		return (m_lambda1*n->getValue()+m_lambda2)*n->getVolume();
	}else{
		return m_M;
	}
}

bool MSP::neighboor(std::pair<State,double> &na,std::pair<State,double> &nb){
	double l=2*(na.second+nb.second);
	State diff=(na.first-nb.first).abs()-(*(m_tree->getDirections()))[0]*l;
	diff.sort();
	if(fabs(diff[DIM-1])==0 && fabs(diff[DIM-1]-diff[DIM-2])!=0)
		return true;
	return false;
}

void MSP::iterationDetails(){
	std::cout << std::endl << std::endl << "Iteration " << m_nb_step << std::endl
			<< "nkipi: " << m_current_coord << " with scale factor " << m_current_scale << std::endl;
	std::cout<< "rejects : ";
	for(auto& c : m_current_forbidden)
		std::cout << c << " , ";
	std::cout << std::endl;
	std::cout << "Gi:" <<std::endl;
	for(int i=0;i<m_nodes.size();++i){
		std::cout << "Vertex " << i << " at " << m_nodes[i].first << " with scale " << ((int)16*m_nodes[i].second) << " and cost " << m_cost[i] << ", neighbor with ";
		for(int j=i+1;j<m_nodes.size();++j){
			if(neighboor(m_nodes[i],m_nodes[j])){
				std::cout << j << " , ";
			}
		}
		std::cout << std::endl;
	}
	bool latex=true;
	if(latex){
		std::stringstream ss;
		ss << "results/iterationFiles/iteration" << m_nb_step << ".tex";
		std::fstream file(ss.str(),std::fstream::out);
		file << "\\begin{tikzpicture}[scale=0.2]" << std::endl
				<< "\\tikzstyle{treenodes}=[black,thick,fill=white]" <<std::endl
				<< "\\tikzstyle{every node}=[circle,draw,minimum size=2pt,inner sep=2pt];" <<std::endl
				<< "\\draw[black,thick,fill=red] (-16,-16) rectangle (16,16);" <<std::endl;
		for(int i=0;i<m_nodes.size();++i){
			file << "\\draw[treenodes] "
					<< m_nodes[i].first-(*(m_tree->getDirections()))[0]*m_nodes[i].second*2
					<< " rectangle "
					<< m_nodes[i].first+(*(m_tree->getDirections()))[0]*m_nodes[i].second*2
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
		file << "\\path[draw,thick] " << m_current_path.front();
		for(std::deque<State>::iterator it=m_current_path.begin()+1,end=m_current_path.end();it!=end;++it){
			file << " -- " << *it ;
		}
		file << ";" << std::endl;
		file << "\\end{tikzpicture}" << std::endl;
		file.close();
	}
}

