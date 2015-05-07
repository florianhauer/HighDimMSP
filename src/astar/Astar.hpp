///////////////////////////////////////////////////////////////////////////////
///  DijkstraShortestPathAlg.cpp
///  The implementation of Dijkstra algorithm to get the shortest path of 
///  a pair of vertices in a graph. 
///
///  @remarks <TODO: insert remarks here>
///
///  @author Yan Qi @date 5/30/2010
/// 
/// $Id: DijkstraShortestPathAlg.cpp 65 2010-09-08 06:48:36Z yan.qi.asu $
///
///////////////////////////////////////////////////////////////////////////////

#include "Astar.h"

#include <set>
#include <map>
#include <vector>

#include "GraphElements.h"
#include "Graph.h"

namespace astar{

BasePath* Astar::get_shortest_path( BaseVertex* source, BaseVertex* sink )
{
	determine_shortest_paths(source, sink);

	std::vector<BaseVertex*> vertex_list;
	std::map<BaseVertex*, double>::const_iterator pos = 
			m_mpStartDistanceIndex.find(sink);
	double weight = pos != m_mpStartDistanceIndex.end() ? pos->second : Graph::DISCONNECT;

	if (weight < Graph::DISCONNECT)
	{
		BaseVertex* cur_vertex_pt = sink;
		do 
		{
			vertex_list.insert(vertex_list.begin(), cur_vertex_pt);

			std::map<BaseVertex*, BaseVertex*>::const_iterator pre_pos = 
					m_mpPredecessorVertex.find(cur_vertex_pt);

			if (pre_pos == m_mpPredecessorVertex.end()) break;

			cur_vertex_pt = pre_pos->second;

		} while (cur_vertex_pt != source);

		vertex_list.insert(vertex_list.begin(), source);
	}
	return new BasePath(vertex_list, weight);
}

void Astar::determine_shortest_paths( BaseVertex* source, BaseVertex* sink )
{
	//1. clear the intermediate variables
	clear();

	//2. initiate the local variables
	BaseVertex* end_vertex =  sink ;
	BaseVertex* start_vertex =  source ;
	m_mpStartDistanceIndex[start_vertex] = 0;
	start_vertex->Weight(0);
	m_quCandidateVertices.insert(start_vertex);

	//3. start searching for the shortest path
	while (!m_quCandidateVertices.empty())
	{
		multiset<BaseVertex*, WeightLess<BaseVertex> >::const_iterator pos = m_quCandidateVertices.begin();

		BaseVertex* cur_vertex_pt = *pos; //m_quCandidateVertices.top();
		m_quCandidateVertices.erase(pos);

		if (cur_vertex_pt == end_vertex) break;

		m_stDeterminedVertices.insert(cur_vertex_pt->getID());

		if(m_exploration){
			m_explore(cur_vertex_pt->getID());
		}

		improve2vertex(cur_vertex_pt);
	}
}

void Astar::improve2vertex( BaseVertex* cur_vertex_pt )
{
	// 1. get the neighboring vertices 
	set<BaseVertex*>* neighbor_vertex_list_pt = new set<BaseVertex*>();

	m_pDirectGraph->get_adjacent_vertices(cur_vertex_pt, *neighbor_vertex_list_pt);


	// 2. update the distance passing on the current vertex
	for(set<BaseVertex*>::iterator cur_neighbor_pos=neighbor_vertex_list_pt->begin(); 
			cur_neighbor_pos!=neighbor_vertex_list_pt->end(); ++cur_neighbor_pos)
	{
		//2.1 skip if it has been visited before
		if (m_stDeterminedVertices.find((*cur_neighbor_pos)->getID())!=m_stDeterminedVertices.end())
		{
			continue;
		}

		//2.2 calculate the distance
		map<BaseVertex*, double>::const_iterator cur_pos = m_mpStartDistanceIndex.find(cur_vertex_pt);
		double distance =  cur_pos != m_mpStartDistanceIndex.end() ? cur_pos->second : Graph::DISCONNECT;

		distance += m_pDirectGraph->get_edge_weight(cur_vertex_pt, *cur_neighbor_pos) ;

		//2.3 update the distance if necessary
		cur_pos = m_mpStartDistanceIndex.find(*cur_neighbor_pos);
		if (cur_pos == m_mpStartDistanceIndex.end() || cur_pos->second > distance)
		{
			m_mpStartDistanceIndex[*cur_neighbor_pos] = distance;
			m_mpPredecessorVertex[*cur_neighbor_pos] = cur_vertex_pt;

			(*cur_neighbor_pos)->Weight(distance);

			multiset<BaseVertex*, WeightLess<BaseVertex> >::const_iterator pos = m_quCandidateVertices.begin();
			for(; pos != m_quCandidateVertices.end(); ++pos)
			{
				if ((*pos)->getID() == (*cur_neighbor_pos)->getID())
				{
					break;
				}
			}
			if(pos != m_quCandidateVertices.end())
			{
				m_quCandidateVertices.erase(pos);
			}
			m_quCandidateVertices.insert(*cur_neighbor_pos);
		}
	}
}

void Astar::clear()
{
	m_stDeterminedVertices.clear();
	m_mpPredecessorVertex.clear();
	m_mpStartDistanceIndex.clear();
	m_quCandidateVertices.clear();
}

}
