///////////////////////////////////////////////////////////////////////////////
///  DijkstraShortestPathAlg.h
///  The implementation of Dijkstra algorithm to get the shortest path of 
///  a pair of vertices in a graph. 
///
///  @remarks <TODO: insert remarks here>
///
///  @author Yan Qi @date 5/30/2010
/// 
///  $Id: DijkstraShortestPathAlg.h 65 2010-09-08 06:48:36Z yan.qi.asu $
///
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>

#include "GraphElements.h"
#include "Graph.h"

using namespace std;

namespace astar{

class Astar
{
private: // members

	Graph* m_pDirectGraph;

	std::unordered_map<BaseVertex*, double> m_mpStartDistanceIndex;
	std::unordered_map<BaseVertex*, BaseVertex*> m_mpPredecessorVertex;

	std::unordered_set<long> m_stDeterminedVertices;
	
	std::multiset<BaseVertex*, WeightLess<BaseVertex> > m_quCandidateVertices;
	
	bool m_exploration;
	void (*m_explore)(long vertex_id);

public:
	Astar(Graph* pGraph,bool expl=false, void (*explore)(long vertex_id)=NULL)
		:m_pDirectGraph(pGraph), m_exploration(expl), m_explore(explore){}
	~Astar(void){clear();}

	void clear();

	BasePath* get_shortest_path(BaseVertex* source, BaseVertex* sink);

	void set_predecessor_vertex(BaseVertex* vt1, BaseVertex* vt2)
	{
		m_mpPredecessorVertex[vt1] = vt2;
	}

	double get_start_distance_at(BaseVertex* vertex)
	{
		return m_mpStartDistanceIndex.find(vertex)->second;
	}

	void set_start_distance_at(BaseVertex* vertex, double weight)
	{
		m_mpStartDistanceIndex[vertex] = weight;
	}

	void get_shortest_path_flower(BaseVertex* root)
	{
		determine_shortest_paths(NULL, root);
	}


protected:

	void determine_shortest_paths(BaseVertex* source, BaseVertex* sink);

	void improve2vertex(BaseVertex* cur_vertex_pt);

};

}

#include "Astar.hpp"
