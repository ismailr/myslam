#ifndef _DATA_ASSOCIATION_H_
#define _DATA_ASSOCIATION_H_

#include "layout_prediction/graph.h"

namespace MYSLAM {
    class DataAssociation {
        public:
            DataAssociation (Graph&);

        private:
            Graph* _graph;
    };
}

#endif
