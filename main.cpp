#include <iostream>
#include <cmath>

#include "vertex_R2.h"
#include "edge_R2_R2.h"
#include "types_slam_R2_example.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;
using namespace g2o::example;

#include "eigen3/Eigen/Dense"

// Debugging mode: Print to console
#if 0
#define debug(str) std::cout << "Debug: " << str << std::endl;
#else
#define debug(str) 
#endif

int main()
{
  // Set up mass-spring-damper system
  //  Mass
  double sys_m = 1.;
  // Damping 
  double sys_b = 1.;
  // Spring constant
  double sys_k = 1.;
  // Set up system matrix A (state space form)
  Eigen::Matrix2d sys_A;
  sys_A(0, 0) = 0.;
  sys_A(0, 1) = 1.;
  sys_A(1, 0) = - sys_k / sys_m;
  sys_A(1, 1) = - sys_b / sys_m;
  // Set up control matrix B
  Eigen::Vector2d sys_B;
  sys_B(0) = 0.;
  sys_B(1) = 1 / sys_m;

  // Number of poses
  int num_poses = 3;
  // Initial conditions
  Eigen::Vector2d x_0(0, 0);
  debug("Initial conditions: x_0\n" << x_0);
  
  // Generate odometry data
  double odom_arr[2] = {1., 1.};
  
  // Print the matrices for debugging
  debug( "Matrix A:\n" << sys_A);
  debug( "Matrix B:\n" << sys_B);

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
  for (int i = 0; i < num_poses; ++i) {
    VertexR2* pose =  new VertexR2;
    pose->setId(i);
    pose->setEstimate(Eigen::Vector2d::Random());
    optimizer.addVertex(pose);
  }
  cerr << "done." << endl;

  // second add the odometry constraints
  cerr << "Optimization: Adding odometry measurements ... ";
  for (int i = 0; i < num_poses - 1; ++i) {    

    EdgeR2R2* odometry = new EdgeR2R2;
    // Set system matrices
    odometry->set_matrices_AB( sys_A, sys_B);
    odometry->vertices()[0] = optimizer.vertex( i);
    odometry->vertices()[1] = optimizer.vertex( i + 1);
    odometry->setMeasurement( odom_arr[i]);
    odometry->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(odometry);
  }
  cerr << "done." << endl;

  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("slam_R2_example_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexR2* firstRobotPose = dynamic_cast<VertexR2*>(optimizer.vertex(0));
  firstRobotPose->setEstimate( x_0);
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  optimizer.save("slam_R2_example_after.g2o");

  // Output estiamtes
  for( int i = 0; i < num_poses; i++){
    std::cout << optimizer.vertex( i) << std::endl;
  }
  // freeing the graph memory
  optimizer.clear();

  return 0;
}
