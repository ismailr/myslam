#include "sparse_optimizer_online.h"
#include "types_slam2d_online.h"

#include "g2o/stuff/macros.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;
using namespace Eigen;

namespace MYSLAM {

    namespace
    {
        template<int p, int l>
        std::unique_ptr<g2o::Solver> AllocatePCGSolver()
        {
            std::cerr << "# Using PCG online poseDim " << p << " landMarkDim " << l << " blockordering 1" << std::endl;

            auto linearSolver = g2o::make_unique<LinearSolverPCG<typename BlockSolverPL<p, l>::PoseMatrixType>>();
            linearSolver->setMaxIterations(6);
            return g2o::make_unique<BlockSolverPL<p, l>>(std::move(linearSolver));
        }
    }

    // force linking to the cholmod solver
    G2O_USE_OPTIMIZATION_LIBRARY(cholmod);

    SparseOptimizerOnline::SparseOptimizerOnline() : g2o::SparseOptimizerOnline ()
    {

    };

    int SparseOptimizerOnline::optimize(int iterations, bool online)
    {
        //return SparseOptimizer::optimize(iterations, online);

        (void) iterations; // we only do one iteration anyhow
        OptimizationAlgorithm* solver = _algorithm;

        int cjIterations=0;
        bool ok=true;

        solver->init(online);
        if (!online) {
            ok = _underlyingSolver->buildStructure();
            if (! ok) {
                cerr << __PRETTY_FUNCTION__ << ": Failure while building CCS structure" << endl;
                return 0;
            }
        }

        if (_usePcg)
            batchStep = true;

        if (! online || batchStep) {
            //cerr << "BATCH" << endl;
            //_underlyingSolver->buildStructure();
            // copy over the updated estimate as new linearization point
            for (size_t i = 0; i < indexMapping().size(); ++i) {
                if (dynamic_cast<OnlineVertexSE2*>(indexMapping()[i])) {
                    OnlineVertexSE2* v = static_cast<OnlineVertexSE2*>(indexMapping()[i]);
                    v->setEstimate (v->updatedEstimate);
                }
                else if (dynamic_cast<OnlineVertexPointXY*>(indexMapping()[i])) {
                    OnlineVertexPointXY* v = static_cast<OnlineVertexPointXY*>(indexMapping()[i]);
                    v->setEstimate (v->updatedEstimate);
                }
            }

            SparseOptimizer::computeActiveErrors();
            //SparseOptimizer::linearizeSystem();
            _underlyingSolver->buildSystem();
        } else {
            //cerr << "UPDATE" << endl;
            // compute the active errors for the required edges
            for (HyperGraph::EdgeSet::iterator it = newEdges->begin(); it != newEdges->end(); ++it) {
                OptimizableGraph::Edge * e = static_cast<OptimizableGraph::Edge*>(*it);
                e->computeError();
            }
            // linearize the constraints and update the Hessian
            for (HyperGraph::EdgeSet::iterator it = newEdges->begin(); it != newEdges->end(); ++it) {
                OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
                e->linearizeOplus(jacobianWorkspace());
                e->constructQuadraticForm();
            }
            // update the b vector
            for (int i = 0; i < static_cast<int>(indexMapping().size()); ++i) {
                OptimizableGraph::Vertex* v = indexMapping()[i];
                int iBase = v->colInHessian();
                v->copyB(_underlyingSolver->b() + iBase);
            }
        }

        ok = _underlyingSolver->solve();
        update(_underlyingSolver->x());

        ++cjIterations; 

        if (verbose()) {
            computeActiveErrors();
            cerr
              << "nodes = " << vertices().size()
              << "\t edges= " << _activeEdges.size()
              << "\t chi2= " << FIXED(activeChi2())
              << endl;
        }

        if (vizWithGnuplot)
            gnuplotVisualization();

        if (! ok)
            return 0;

        return 1;
    }

    void SparseOptimizerOnline::update(double* update)
    {
        for (size_t i=0; i < _ivMap.size(); ++i) {
            if (dynamic_cast<OnlineVertexSE2*>(_ivMap[i])) {
                OnlineVertexSE2* v= static_cast<OnlineVertexSE2*>(_ivMap[i]);
                v->oplusUpdatedEstimate(update);
                update += 3;
            } else if (dynamic_cast<OnlineVertexPointXY*>(_ivMap[i])) {
                OnlineVertexPointXY* v = static_cast<OnlineVertexPointXY*> (_ivMap[i]);
                v->oplusUpdatedEstimate(update);
                update += 3;
            }
        }
    }

    bool SparseOptimizerOnline::initSolver(int dimension, int /*batchEveryN*/)
    {
        slamDimension = dimension;
        OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
        OptimizationAlgorithmProperty solverProperty;
        if (_usePcg) {
            std::unique_ptr<Solver> s;
            s = AllocatePCGSolver<3, 2>();
            OptimizationAlgorithmGaussNewton* gaussNewton = new OptimizationAlgorithmGaussNewton(std::move(s));
            setAlgorithm(gaussNewton);
        }
        else {
            setAlgorithm(solverFactory->construct("gn_fix3_2_cholmod", solverProperty));
        }

        OptimizationAlgorithmGaussNewton* gaussNewton = dynamic_cast<OptimizationAlgorithmGaussNewton*>(solver());
        _underlyingSolver = &gaussNewton->solver();

        if (! solver()) {
            cerr << "Error allocating solver. Allocating CHOLMOD solver failed!" << endl;
            return false;
        }
        return true;
    }
};
