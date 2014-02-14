/**
 * @file   NavigationGraph.cpp
 * @author David Haensel (d.haensel@fz-juelich.de)
 * @date   January, 2014
 * @brief  The Navigation Graph is the metric abstraction layer in the cognitive map.
 *
 */

#include "NavigationGraph.h"

#include <iostream>
#include <fstream>

#include "navigation_graph/GraphEdge.h"
#include "../../geometry/Building.h"
#include "../../geometry/SubRoom.h"


/**
 * Constructors & Destructors
 */

NavigationGraph::NavigationGraph(const Building * building)
    : building(building)
{
}

NavigationGraph::NavigationGraph(const NavigationGraph & ng)
    : building(ng.building)
{
}

NavigationGraph::~NavigationGraph()
{
    //remove all vertices
    for(VerticesContainer::iterator it = vertices.begin(); it != vertices.end(); ++it)
    {
        delete it->second;
    }
}

void NavigationGraph::AddVertex(const SubRoom * const sub_room)
{
    vertices.emplace(sub_room, new GraphVertex(sub_room));
}

void NavigationGraph::AddEdge(const Crossing * crossing)
{
    VerticesContainer::iterator src_it = vertices.find(crossing->GetSubRoom1());
    VerticesContainer::iterator dest_it = vertices.find(crossing->GetSubRoom2());

    if(src_it != vertices.end() && dest_it != vertices.end())
    {
        src_it->second->AddOutEdge(dest_it->second, crossing);
        dest_it->second->AddOutEdge(src_it->second, crossing);
    }
}

void NavigationGraph::WriteToDotFile(const std :: string filepath) const
{
    std::ofstream dot_file;
    dot_file.open (filepath + "navigation_graph.dot");
    dot_file << " digraph graphname \n {\n";
    for(VerticesContainer::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
        dot_file << it->second->GetCaption() + "\n" ;
        const GraphVertex::EdgesContainer * edges = it->second->GetAllOutEdges();
        for(GraphVertex::EdgesContainer::const_iterator it2 = edges->begin(); it2 != edges->end(); ++it2)
        {
            dot_file << it->second->GetCaption() + " -> " + it2->first->GetCaption() + "\n";
        }

    }
    dot_file << "} \n";

    dot_file.close();
    return;
}
