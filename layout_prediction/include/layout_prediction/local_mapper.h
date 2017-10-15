#ifndef _LOCAL_MAPPER_H_
#define _LOCAL_MAPPER_H_

#include <g2o/core/sparse_optimizer.h>

#include "layout_prediction/pose.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/system.h"

class System2;
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
        LocalMapper2(System2& system, Graph2& graph);

        void add_vertex (Pose2::Ptr& pose);
        void add_vertex (Wall2::Ptr& wall);
        void add_edge (PoseMeasurement2::Ptr& poseMeasurement);
        void add_edge (WallMeasurement2::Ptr& wallMeasurement);
        void local_optimize ();

        Wall2::Ptr data_association (Wall2::Ptr& wall);

    private:
        System2 *_system;
        Graph2 *_graph;
        g2o::SparseOptimizer *_optimizer;
        std::map<std::tuple<int,int>, Wall2::Ptr> _indexedWallDB; // for data association
        std::vector<Pose2::Ptr> _poseDB; 
        std::vector<Wall2::Ptr> _wallDB;
        std::vector<PoseMeasurement2::Ptr> _poseMeasurementDB;
        std::vector<WallMeasurement2::Ptr> _wallMeasurementDB;

        const float GRID_STEP = 5.0;
        const float ANGLE_STEP = 30.0;

        void set_fixed_vertices ();
        void push_to_graph ();
        void clear();
};

#endif
