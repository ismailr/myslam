#include <math.h>
#include <fstream>
#include <iostream>
#include <cmath>

#include "layout_prediction/graph.h"
#include "layout_prediction/wall.h"

Graph::Graph (){}

void Graph::addVertex (Wall::Ptr wall){
    double rho = wall->rho();
    double theta = wall->theta();

    long cell_rho = static_cast<long> (rho > 0 ? floor (rho/10) : ceil (rho/10)); 
    long cell_theta = static_cast<long> (theta > 0 ? floor (theta/45) : ceil (theta/45));

    std::tuple <double, double> line;
    line = std::make_tuple (cell_rho, cell_theta);

    _local_maps[line].push_back (wall); 

//    std::ofstream myfile ("/home/ism/localmap", std::ios::out | std::ios::app);
//    if (myfile.is_open())
//    {
//        if (!std::isnan (rho))
//            myfile << rho << "->" << cell_rho << " | " << theta << "->" << cell_theta << std::endl;
//    }
//    myfile.close();


//    _walls.push_back (wall);
}

std::vector<Wall::Ptr> Graph::getAllVertices ()
{
    return _walls;
}

void Graph::updateGraph (std::vector<Wall::Ptr> newGraph)
{
    std::unique_lock<std::mutex> lock (_graphUpdateMutex);
    for (int i = 0; i < newGraph.size(); ++i)
        // _walls[newGrah[i].id] = newGraph[i];
    lock.unlock();
}

void Graph::addToMap (Wall::Ptr wall)
{

}
