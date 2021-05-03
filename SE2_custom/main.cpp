#include <iostream>
#include "inekf_se2.h"
#include "types_slam_se2.h"

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
#ifndef NDEBUG
    std::cout << "Running L-InEKF" << std::endl;
#endif
    std::vector< PoseEstimate> X_hat = GetSe2InekfEstimates( 
            meas_prior,
            meas_gyro,
            meas_vel,
            meas_gps
            );

    // ********************************************************
    // Trying custom g2o types
    g2o::SE2::VertexSE2* X = new g2o::SE2::VertexSE2;
    const double dx[3] = {1.0, 2.0, 0.0};
    X->setId(1);
    X->setEstimate( Pose( 1, 2, M_PI/4));
    X->oplus( dx);
    std::cout << X->estimate().log().coeffs() << std::endl;
    X->setTime( 2.3);
    std::cout << X->time() << std::endl;
    delete X;
    // std::cout << "X: " << X << std::endl;

    // RV::IO::write( X_hat, filename_kf, "X");
}
