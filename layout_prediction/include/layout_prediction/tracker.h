#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "layout_prediction/system.h"
#include "layout_prediction/frame.h"

class System;
class Graph;
class Frame;
class Tracker
{
    public:
        static bool init;
        unsigned long _previousFrameProcessed;

        Tracker (System&, Graph&);
        void run ();
        void track (Frame&);

    private:
        System *_system;
        Graph *_graph;

        pcl::PointCloud<pcl::PointXYZ>::Ptr _prevCloud; 
};

#endif
