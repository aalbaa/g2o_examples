#ifndef G2O_TUTORIAL_EDGE_SE2_POINT_XY_H
#define G2O_TUTORIAL_EDGE_SE2_POINT_XY_H

#include "vertex_R2.h"
#include "slam_R2_example_api.h"

#include "g2o/core/base_binary_edge.h"

namespace g2o {

  namespace example {

    class G2O_SLAM_R2_API EdgeR2R2 : public BaseBinaryEdge<2, Eigen::Vector2d, VertexR2, VertexR2>
    {
      private:
        // Add system matrices
        Eigen::Matrix2d _sys_A;
        Eigen::Vector2d _sys_B;
        // Boolean variable to identify whether the variables were set or not
        bool _sys_A_initialized = false;
        bool _sys_B_initialized = false;

        // Odometry measurement
        double _odom_meas;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeR2R2(){};

        void computeError();
  
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        // Functions to set the system matrices
        void set_matrix_A( const Eigen::Matrix2d& sys_A_in){
          _sys_A = sys_A_in;
          _sys_A_initialized = true;
        }
        void set_matrix_B( const Eigen::Vector2d& sys_B_in){
          _sys_B = sys_B_in;
          _sys_B_initialized = true;
        }
        
        // Add a function that can add both matrices at once
        void set_matrices_AB( const Eigen::Matrix2d& sys_A_in, const Eigen::Vector2d& sys_B_in){
          set_matrix_A( sys_A_in);
          set_matrix_B( sys_B_in);
        }
      
        // Set odometry measurement
        void setMeasurement(const double& m){
          _odom_meas = m;
        }

      // Set measurements
      protected:
        virtual bool resolveCaches();
    };

  } // end namespace
} // end namespace

#endif
