#include <iostream>
#include "layout_prediction/tracker.h"

Tracker::Tracker () : _busy (false) {
};

void Tracker::attachTo (System *system)
{
    _system = system;
}

void Tracker::detect ()
{
    std::cout << "tracking..." << std::endl;

}
