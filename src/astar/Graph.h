///////////////////////////////////////////////////////////////////////////////
///  Graph.h
///  <TODO: insert file description here>
///
///  @remarks <TODO: insert remarks here>
///
///  @author Yan Qi @date 8/18/2010
/// 
///  $Id: Graph.h 65 2010-09-08 06:48:36Z yan.qi.asu $
///////////////////////////////////////////////////////////////////////////////


#pragma once
#include "GraphElements.h"
#include <unordered_set>
#include <unordered_map>
#include <functional>

using namespace std;
namespace astar{

typedef unordered_set<BaseVertex*> VertexPtSet;
typedef unordered_map<BaseVertex*, VertexPtSet*> BaseVertexPt2SetMap;

typedef VertexPtSet::iterator VertexPtSetIterator;
typedef BaseVertexPt2SetMap::iterator BaseVertexPt2SetMapIterator;

class Path : public BasePath
{
public: 

	Path(const std::vector<BaseVertex*>& vertex_list, double weight):BasePath(vertex_list,weight){}

	// display the content
	void PrintOut(std::ostream& out_stream) const
	{
		out_stream << "Cost: " << m_dWeight << " Length: " << m_vtVertexList.size() << std::endl;
		for(std::vector<BaseVertex*>::const_iterator pos=m_vtVertexList.begin(); pos!=m_vtVertexList.end();++pos)
		{
			out_stream << (*pos)->getID() << " ";
		}
		out_stream << std::endl <<  "*********************************************" << std::endl;	
	}
};

class Graph
{
public: // members

	const static double DISCONNECT; 

protected: // members

	// Basic information
	BaseVertexPt2SetMap m_mpFanoutVertices;
//	BaseVertexPt2SetMap m_mpFaninVertices;
	unordered_map<long, double> m_mpEdgeCodeWeight;
	vector<BaseVertex*> m_vtVertices;
	long m_nEdgeNum;
	long m_nVertexNum;

	unordered_map<long, BaseVertex*> m_mpVertexIndex;

	// Members for graph modification
//	set<long> m_stRemovedVertexIds;
//	set<pair<long,long> > m_stRemovedEdge;

public:

	// Constructors and Destructor
	Graph(const string& file_name);
	Graph();
	Graph(const Graph& rGraph);
	~Graph(void);

	void clear();

	void add_vertex(long node_id);
	void add_vertex(long node_id,double h);
	void add_edge(long start_vertex, long end_vertex, double edge_weight);

	BaseVertex* get_vertex(long node_id);
	
	long get_edge_code(const BaseVertex* start_vertex_pt, const BaseVertex* end_vertex_pt) const;
	VertexPtSet* get_vertex_set_pt(BaseVertex* vertex_, BaseVertexPt2SetMap& vertex_container_index);

	double get_original_edge_weight(const BaseVertex* source, const BaseVertex* sink);

	double get_edge_weight(const BaseVertex* source, const BaseVertex* sink);
	void get_adjacent_vertices(BaseVertex* vertex, VertexPtSet& vertex_set);
//	void get_precedent_vertices(BaseVertex* vertex, VertexPtSet& vertex_set);

	/// Methods for changing graph
//	void remove_edge(const pair<long,long> edge)
//	{
//		m_stRemovedEdge.insert(edge);
//	}
//
//	void remove_vertex(const long vertex_id)
//	{
//		m_stRemovedVertexIds.insert(vertex_id);
//	}
//
//	void recover_removed_edges()
//	{
//		m_stRemovedEdge.clear();
//	}
//
//	void recover_removed_vertices()
//	{
//		m_stRemovedVertexIds.clear();
//	}
//
//	void recover_removed_edge(const pair<long,long> edge)
//	{
//		m_stRemovedEdge.erase(m_stRemovedEdge.find(edge));
//	}
//
//	void recover_removed_vertex(long vertex_id)
//	{
//		m_stRemovedVertexIds.erase(m_stRemovedVertexIds.find(vertex_id));
//	}
	
private:
	void _import_from_file(const std::string& file_name);

};
}

#include "Graph.hpp"
