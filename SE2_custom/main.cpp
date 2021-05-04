#include <iostream>

#include "inekf_se2.h"

#include "vertex_se2.h"
#include "b_edge_se2se2.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

int main(int argc, const char* argv[]){
    // Read config.yml. Specify the arguments in the settings.json file. For example,
    //  {
    //     "cmake.debugConfig": {
    //         "args": [
    //             "/home/aa/Documents/Data/Data_generator/SE2/config.yml"
    //         ]
    //     }
    // }
    YAML::Node config;
    std::string filename_config;
    filename_config = argv[1];
    std::cout << argc << " arguments passed: " << filename_config << std::endl;
    config = YAML::LoadFile( filename_config);
    // Get output file name
    const std::string filename_kf   = config["filename_kf"].as<std::string>();

    // ********************************************************
    // Load data
#ifndef NDEBUG
    std::cout << "Loading data" << std::endl;
#endif
    // Read sensor files
    // Prior
    const std::string filename_prior = config["filename_prior"].as<std::string>();
    //  Gyro
    const std::string filename_gyro  = config["filename_gyro"].as<std::string>();
    //  Velocity
    const std::string filename_vel   = config["filename_vel"].as<std::string>();
    //  GPS
    const std::string filename_gps   = config["filename_gps"].as<std::string>();
    // Estimated states
    // const std::string filename_out   = config["filename_out"].as<std::string>();

    // Import data
    //  Prior
    PoseEstimate meas_prior = RV::IO::import< PoseEstimate>( filename_prior)[0];
    //  Gyro
    std::vector< MeasGyro> meas_gyro      = RV::IO::import< MeasGyro>( filename_gyro);
    //  Velocity
    std::vector< MeasVel> meas_vel        = RV::IO::import< MeasVel>( filename_vel);
    //  GPS
    std::vector< MeasGps> meas_gps        = RV::IO::import< MeasGps>( filename_gps);
    
    // ********************************************************
    // Run L-InEKF    
    std::cout << "Running L-InEKF" << std::endl;
    
    // Note that the estimates PoseEstimate is of type RandomVariable
    std::vector< PoseEstimate> X_kf_rv = GetSe2InekfEstimates( 
            meas_prior,
            meas_gyro,
            meas_vel,
            meas_gps
            );

    // Number of poses
    const int K = X_kf_rv.size();

    // Compute vector of Pose (SE2d) elements
    std::vector< Pose> X_kf( K);
    for( int i = 0; i < K; i++){
        double x, y, real, imag;
        x    = X_kf_rv[i].mean()(0, 2);
        y    = X_kf_rv[i].mean()(1, 2);  
        real = X_kf_rv[i].mean()(0, 0);  
        imag = X_kf_rv[i].mean()(1, 0);  
        X_kf[i] = Pose( x, y, real, imag);
    }

    // ********************************************************
    // creating the optimization problem
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    g2o::SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    // ********************************************************
    // Building graph
    //      Adding poses/vertices
    std::cout << "Building factor graph..." << std::endl;
    for(int i = 0; i < K; i++){
        g2o::SE2::VertexSE2* robot = new g2o::SE2::VertexSE2;
        robot->setId(i);
        // Set estimate from the L-InEKF
        robot->setEstimate( X_kf[i]);
        optimizer.addVertex(robot);
    }
    std::cout << "Done" << std::endl;

    // ********************************************************
    // Trying custom g2o types
    g2o::SE2::VertexSE2* X1 = new g2o::SE2::VertexSE2;
    const double dx[3] = {1.0, 2.0, 0.0};
    X1->setId(1);
    X1->setEstimate( Pose( 0, 0, 0));
    // X1->oplus( dx);
    std::cout << X1->estimate().log().coeffs() << std::endl;
    X1->setTime( 0.0);
    std::cout << X1->time() << std::endl;
    
    g2o::SE2::VertexSE2* X2 = new g2o::SE2::VertexSE2;    
    X2->setId(2);
    X2->setEstimate( Pose( 1 + 0.1, 2 - 0.1, M_PI/4 + 0.01));
    std::cout << X2->estimate().log().coeffs() << std::endl;
    X2->setTime( 0.1);
    std::cout << X2->time() << std::endl;
    // std::cout << "X: " << X << std::endl;

    // Trying the custom SE2 <-> SE2 edge
    g2o::SE2::BEdgeSE2SE2* odometry = new g2o::SE2::BEdgeSE2SE2;
    odometry->vertices()[0] = X1;
    odometry->vertices()[1] = X2;
    // Set sampling period
    odometry->setDt( 0.1);
    // Set measurement
    odometry->setMeasurement(g2o::Vector3(1, 2, M_PI/4));
    odometry->setInformation(CovPose::Identity());
    std::cout << odometry->measurement() << std::endl;
    
    std::cout << odometry->information() << std::endl;

    odometry->computeError();
    std::cout << odometry->error() << std::endl;


    // TODO:
    //  Implement dead-reckoning batch!

    // RV::IO::write( X_kf, filename_kf, "X");
}
