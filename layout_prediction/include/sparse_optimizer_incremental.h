// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Copyright (C) 2018 Ismail
//
#ifndef _SPARSE_OPTIMIZER_INCREMENTAL_H_
#define _SPARSE_OPTIMIZER_INCREMENTAL_H_

#include "g2o/core/sparse_block_matrix.h"
#include "g2o/examples/interactive_slam/g2o_incremental/linear_solver_cholmod_online.h"
#include "sparse_optimizer_online.h"

using namespace g2o;

namespace MYSLAM {
    class SparseOptimizerIncremental : public SparseOptimizerOnline
    {
        public:
        SparseOptimizerIncremental();
        ~SparseOptimizerIncremental();

        int optimize(int iterations, bool online = false);
        virtual bool updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset);
        virtual bool initSolver(int dimension, int batchEveryN);

        protected:
        SparseBlockMatrix<Eigen::MatrixXd> _updateMat;
        cholmod_common _cholmodCommon;
        CholmodExt* _cholmodSparse;
        cholmod_factor* _cholmodFactor;
        cholmod_triplet* _permutedUpdate;
        cholmod_factor* _L;
        LinearSolverCholmodOnlineInterface* _solverInterface;

        HyperGraph::VertexSet _touchedVertices;
        Eigen::VectorXi _perm;
        Eigen::VectorXi _cmember;

        Eigen::VectorXi _tripletWorkspace;
        CholmodExt* _permutedUpdateAsSparse;

        bool computeCholeskyUpdate();
        void convertTripletUpdateToSparse();
    };
};
#endif
