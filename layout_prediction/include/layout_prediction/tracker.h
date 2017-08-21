#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "layout_prediction/system.h"

class System;
class Tracker
{
    public:
        bool _busy;  

        Tracker ();
        void detect ();
        void attachTo (System*);

    private:
        System *_system;
};

#endif
