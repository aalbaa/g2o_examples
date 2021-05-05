#include <iostream>

#include "inekf_se2.h"

#include "vertex_se2.h"
#include "b_edge_se2se2.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

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

    // dt_func returns sampling period at index k: dt_k = t_k - t_{k-1}
    auto dt_func = [&meas_gyro](int k){
        return meas_gyro[k+1].time() - meas_gyro[k].time();
    };

    // Compute vector of Pose (SE2d) elements
    std::vector< Pose> X_kf( K);
    for( int k = 0; k < K; k++){
        double x, y, real, imag;
        x    = X_kf_rv[k].mean()(0, 2);
        y    = X_kf_rv[k].mean()(1, 2);  
        real = X_kf_rv[k].mean()(0, 0);  
        imag = X_kf_rv[k].mean()(1, 0);  
        X_kf[k] = Pose( x, y, real, imag);
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
    // Building graph: 
    //      - Adding poses/vertices
    //      - Adding odometry measurements
    std::cout << "Building factor graph..." << std::endl;
    for(int k = 0; k < K; k++){
        g2o::SE2::VertexSE2* robot = new g2o::SE2::VertexSE2;
        robot->setId(k);
        // Set estimate from the L-InEKF
        robot->setEstimate( X_kf[k]);
        optimizer.addVertex(robot);

        if(k > 0){
            int km1 = k - 1;
            // Add odometry edges
            g2o::SE2::BEdgeSE2SE2* odom = new g2o::SE2::BEdgeSE2SE2;
            odom->vertices()[0] = optimizer.vertex( k - 1);
            odom->vertices()[1] = optimizer.vertex( k);
            // Compute the measurement vector
            double dt_km1 = dt_func(km1);
            g2o::Vector3 u_km1( 
                    dt_km1 * meas_vel [km1].mean()(0),
                    dt_km1 * meas_vel [km1].mean()(1),
                    dt_km1 * meas_gyro[km1].mean()(0)
                );
            odom->setDt( dt_km1);
            odom->setMeasurement( u_km1);
            // Compute process noise covariance
            CovQ Q_km1  = CovQ::Zero();
            Q_km1.block< dof_vel, dof_vel>(0, 0)   = meas_vel [km1].cov();
            Q_km1.block< dof_gyro, dof_gyro>(2, 2) = meas_gyro[km1].cov();
            // Jacobian of process model w.r.t. process noise w_km1
            JacF_wkm1 jac_F_wkm1 = dt_km1 * JacF_wkm1::Identity();
            odom->setInformation( (jac_F_wkm1 * Q_km1 * jac_F_wkm1.transpose()).inverse());

            // Add edge to graph
            optimizer.addEdge( odom);
        }            
    }

    // ********************************************************
    // Solving the optimization problem
    std::cout << "Setting up optimization problem" << std::endl;

#ifndef NDEBUG
    std::cout << "\tFixing first pose" << std::endl;
#endif
    // fix the first robot pose to account for gauge freedom
    g2o::SE2::VertexSE2* firstRobotPose = dynamic_cast<g2o::SE2::VertexSE2*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);

    // Set verbosity
    optimizer.setVerbose(true);

    std::cout << "Optimizing" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize( 3);
    
    std::cout << "Done" << std::endl;


    // ********************************************************
    // Export to random variable vector
    std::vector< PoseEstimate> X_batch_rv( K);
    for( int k = 0; k < K; k++){
        // Set time
        X_batch_rv[k].setTime( X_kf_rv[k].time());

        // Set mean
        g2o::SE2::VertexSE2* p_X_k = dynamic_cast<g2o::SE2::VertexSE2*>(optimizer.vertex(k));
        X_batch_rv[k].setMean( p_X_k->estimate().transform());       

        // // Set covariance
        // g2o::SparseBlockMatrixX sp_cov_X_k;
        // optimizer.computeMarginals( sp_cov_X_k, optimizer.vertex(k));
        // Eigen::MatrixXd dd =  sp_cov_X_k.block(0, 0)->eval();
        // X_batch_rv[k].setCov( 
                
        //     );
    }
        
    // Free graph memory
    optimizer.clear();

    // TODO:
    //  Implement dead-reckoning batch!

    // RV::IO::write( X_kf, filename_kf, "X");
}
