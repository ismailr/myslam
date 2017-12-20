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

#include "layout_prediction/pose.h"

Pose::Pose() : BaseVertex<3, SE2>()
{
}

bool Pose::read(std::istream& is)
{
    Eigen::Vector3d p;
    is >> p[0] >> p[1] >> p[2];
    _estimate.fromVector(p);
    return true;
}

bool Pose::write(std::ostream& os) const
{
    Eigen::Vector3d p = estimate().toVector();
    os << p[0] << " " << p[1] << " " << p[2];
    return os.good();
}

Pose2::Pose2() : VertexSE2()
{
    nodetype = "POSE2";
}

void Pose2::insert_detected_wall3(int id)
{
    if (!is_detected_wall(id))
        _detectedWalls3.push_back (id);
}

bool Pose2::is_detected_wall (int id)
{
    for (int i = 0; i < _detectedWalls3.size(); i++)
    {
        if (id == _detectedWalls3[i])
            return true;
    }

    return false;
}

namespace MYSLAM {
    unsigned int long Pose::_idGenerator = 0;

    Pose::Pose(){ 
        _id = Pose::_idGenerator++;
        _pose.setZero();
    };
}
