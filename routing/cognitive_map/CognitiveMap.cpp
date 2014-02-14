/**
 * @file   CognitiveMap.cpp
 * @author David Haensel (d.haensel@fz-juelich.de)
 * @date   January, 2014
 * @brief  Cognitive Map models the pedestrian knowledge of building space in simulation.
 *
 */

#include "CognitiveMap.h"
#include "NavigationGraph.h"
#include "../../geometry/Crossing.h"
#include "../../geometry/Building.h"


using namespace std;

/**
 * Constructors & Destructors
 */

CognitiveMap::CognitiveMap(const Building * building)
    : building(building)
{
    navigation_graph = new NavigationGraph(building);
}

CognitiveMap::~CognitiveMap()
{
    delete navigation_graph;
}

void CognitiveMap::Add(const SubRoom * sub_room)
{
    navigation_graph->AddVertex(sub_room);
}

void CognitiveMap::Add(const Crossing * crossing)
{
    navigation_graph->AddEdge(crossing);
}

void CognitiveMap::AddExit(const Transition * exit)
{

}

const NavigationGraph * CognitiveMap::GetNavigationGraph()
{
    return navigation_graph;
}
