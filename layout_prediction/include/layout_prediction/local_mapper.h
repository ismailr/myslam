#ifndef _LOCAL_MAPPER_H_
#define _LOCAL_MAPPER_H_

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/graph.h"


class LocalMapper
{
    public:
        typedef std::shared_ptr<LocalMapper> Ptr;
        typedef std::shared_ptr<const LocalMapper> ConstPtr;

        static int frame_counter;

        LocalMapper(Graph&);
        void optimize(std::vector<Wall::Ptr>);

    private:
        Graph* _graph;
        g2o::SparseOptimizer* _optimizer;
        std::vector<Wall::Ptr> _walls;
        std::map<std::tuple<double,double>,std::vector<Wall::Ptr> > _local_map;

        void occupancyGrid();

};

class LocalMapper2
{
    public:
        LocalMapper2(Graph2& graph);

        void addVertex (Pose2::Ptr& pose);
        void addVertex (Wall2::Ptr& wall);
        void addEdge (PoseMeasurement2::Ptr& poseMeasurement);
        void addEdge (WallMeasurement2::Ptr& wallMeasurement);
        void localOptimize ();

        Wall2::Ptr dataAssociation (Wall2::Ptr& wall);

    private:
        Graph2* _graph;
        g2o::SparseOptimizer* _optimizer;
        std::map<std::tuple<double,double>, Wall2::Ptr> _wallDatabase;

        void pushToGraph ();
        void clear();
};

#endif
