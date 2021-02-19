// 2D linear batch optimization on a mass-spring-damper sysetm.
//  Note that most of the source code is inherited from g2o's examples.
//
//  Amro Al Baali
//  18-Feb-2021

#include <iostream>
#include <cmath>

#include "vertex_R2.h"
#include "edge_R2.h"
#include "types_tutorial_slam_R2.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

int main()
{
  int numPoses = 10;
  int numLandmarks = 1;
  int numNodes = numPoses + numLandmarks;
  // Create system matrix
  //  mass
  double sys_mass  = 1;
  double sys_damp  = 1;
  double sys_sprng = 1;
  
  Eigen::Matrix2d sys_A;
  sys_A(0, 0) = 0;
  sys_A(0, 1) = 1;
  sys_A(1, 0) = - sys_sprng / sys_mass;
  sys_A(1, 1) = - sys_damp / sys_mass;

  // Input matrix
  Eigen::Matrix2d sys_B;
  sys_B(0, 0) = 0;
  sys_B(0, 1) = 0;
  sys_B(1, 0) = 1;
  sys_B(1, 1) = 0;

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
    g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  // adding the odometry to the optimizer
  // first adding all the vertices
  cerr << "Optimization: Adding robot poses ... ";
  for (size_t i = 0; i < numPoses; ++i) {
    VertexR2* robot =  new VertexR2;
    robot->setId(i);
    robot->setEstimate( Vector2(0, 0));
    optimizer.addVertex(robot);
  }
  cerr << "done." << endl;

  // second add the odometry constraints
  cerr << "Optimization: Adding odometry measurements ... ";
  for (size_t i = 0; i < numPoses - 1; ++i) {
    // EdgeSE2* odometry = new EdgeSE2;
    EdgeR2* odometry = new EdgeR2;
    odometry->setMatrix_A( sys_A);
    odometry->setMatrix_B( sys_B);

    std::cout << "\nTest: _sys_A:\n" << odometry->getMatrix_A();
    std::cout << "\t_sys_B:\n" << odometry->getMatrix_B() << std::endl;
    odometry->vertices()[0] = optimizer.vertex( i);
    odometry->vertices()[1] = optimizer.vertex( i + 1);
    // odometry->setMeasurement( SE2(1.0, 0., 0.));
    odometry->setMeasurement( Vector2(1.0, 0.));
    // odometry->setInformation( Eigen::Matrix3d::Identity());
    odometry->setInformation( Eigen::Matrix2d::Identity());
    optimizer.addEdge(odometry);
  }
  cerr << "done." << endl;

  cerr << "Adding loop closures....";
  // Add loop closure edge
  // EdgeSE2* lc = new EdgeSE2;
  EdgeR2* lc = new EdgeR2;
  lc->vertices()[0] = optimizer.vertex( 0);
  lc->vertices()[1] = optimizer.vertex( numPoses - 1);
  // lc->setMeasurement( SE2( numPoses - 1 + 0.1, 0., 0.));
  lc->setMeasurement( Vector2( numPoses - 1 + 0.1, 0.));
  lc->setInformation( 0.1 * Eigen::Matrix2d::Identity());
  optimizer.addEdge( lc);

  cerr << "done." << endl;
  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  // VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
  VertexR2* firstRobotPose = dynamic_cast<VertexR2*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  optimizer.save("tutorial_after.g2o");

  // freeing the graph memory
  optimizer.clear();

  return 0;
}
