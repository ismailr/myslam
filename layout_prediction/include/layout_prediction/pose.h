// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef _POSE_H_
#define _POSE_H_

#include <memory>
#include <string>

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "vertex_se2.h"

#include "layout_prediction/wall.h"

using namespace g2o;

class Pose : public BaseVertex<3, SE2>
{
    public:
    typedef std::shared_ptr<Pose> Ptr;
    typedef std::shared_ptr<const Pose> ConstPtr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Pose();

    virtual void setToOriginImpl() {
        _estimate = SE2();
    }

    virtual void oplusImpl(const double* update)
    {
        SE2 up(update[0], update[1], update[2]);
        _estimate *= up;
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    bool operator == (const Pose::Ptr& posePtr) const
    {
        if (this->id() == posePtr->id())
            return true;
        else
            return false;
    }
};

class Pose2: public VertexSE2
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    std::string nodetype;

    typedef std::shared_ptr<Pose2> Ptr;
    typedef std::shared_ptr<const Pose2> ConstPtr;
    Pose2 ();

    void setModel (SE2& t) { _model = &t; };
    SE2* getModel () const { return _model; };

    void insert_detected_wall (Wall2::Ptr& w) { _detectedWalls.push_back (w); };
    std::vector<Wall2::Ptr> get_detected_walls () { return _detectedWalls; };

    void insert_detected_wall3 (int);
    std::vector<int> get_detected_walls3 () const { return _detectedWalls3; };
    bool is_detected_wall (int);

    private:
    SE2* _model;
    std::vector<Wall2::Ptr> _detectedWalls;
    std::vector<int> _detectedWalls3;
};
#endif
