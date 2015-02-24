
#include <deque>
#include <map>
#include <vector>
#include <utility>
#include <string>
#include "Graph.h"
#include "Tree.h"
#include "State.h"

class MSP{
public:
	MSP(Tree* tree);
	bool init(State start,State end);
	bool step();
	bool run();
	std::deque<State> getPath(){return m_current_path;}
	double getPathCost(){return std::accumulate(m_path_cost.begin(),m_path_cost.end(),0.0);}
	void setAlpha(double a){m_alpha=a;}
	void setSpeedUp(bool a){m_speed_up=a;}

protected:
	void iterationDetails(kshortestpaths::BasePath* result=NULL);
	bool inPath(State pt,double size);
	void reducedGraph();
	bool neighboor(std::pair<State,double> &na,std::pair<State,double> &nb);
	bool is_start(std::pair<State,double> &node){return is_in(m_current_coord,node);}
	bool is_goal(std::pair<State,double> &node){return is_in(m_end_coord,node);}
	bool is_in(State pt,std::pair<State,double> node);
	double cost(Node* n);
	void add_node_to_reduced_vertices(Node* node,State coord, double size);
	State m_start_coord;
	State m_end_coord;
	State m_current_coord;
	double m_current_scale;
	Tree* m_tree;
	int m_start_index;
	int m_nb_backtrack;
	int m_end_index;
	bool m_speed_up;
	bool m_path_found;
	std::deque<State> m_current_path;
	kshortestpaths::Graph m_graph;
	std::map<State,std::set<State>> m_misleading;
	std::set<State> m_current_forbidden;
	double m_alpha;//used in reduced graph as parameter for decomposition
	std::vector<std::pair<State,double> > m_nodes; //coord,size
	std::vector<double> m_cost;
	std::vector<double> m_path_cost;
	double m_epsilon;
	double m_M;
	double m_lambda1;
	double m_lambda2;
	int m_nb_step;
};
