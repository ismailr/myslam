#include <iostream>
#include "layout_prediction/tracker.h"
#include "layout_prediction/frame.h"

Tracker::Tracker (System& system, Graph& graph) 
    :_system (&system), _graph (&graph), _previousFrameProcessed (0) 
{
    _system->setTracker (*this);
}

void Tracker::run ()
{
    while (1)
    {
        if (_system->_framesQueue.empty())
            continue;


        std::unique_lock <std::mutex> lock_frames_queue (_system->_framesQueueMutex);
        for (int i = 0; i < _system->_framesQueue.size(); ++i)
        {
            Frame *frame = _system->_framesQueue.front();

            if (frame->_id == _previousFrameProcessed) // already process this frame
                continue;

            _previousFrameProcessed = frame->_id;

            int useCount = ++frame->_useCount; // use the frame and increment count
            track (*frame);
            if (useCount == 2) // WallDetector already used it, so pop and delete
            {
                _system->_framesQueue.pop ();
                delete frame; // it's a must, otherwise memory leak!
            }
        }
        lock_frames_queue.unlock ();

    }
}

void Tracker::track (Frame& frame)
{
    std::cout << "tracking .. ";

}
