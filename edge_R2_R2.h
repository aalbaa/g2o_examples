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
        EdgeR2R2();

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
        void setMeasurement(const Eigen::Vector2d& m){
          // _odom_meas = m;
          // _measurement[0] = m;
          // _measurement[1] = 0.;
          _measurement = m;
        }

        virtual bool setMeasurementData(const number_t* d){

          _measurement=Vector2(d[0], d[1]);
          return true;
        }

        virtual bool getMeasurementData(number_t* d) const {
          Eigen::Map<Eigen::Vector2d> m(d);
          m = _measurement;
          return true;
        }

        virtual int measurementDimension() const {return 2;}

        virtual bool setMeasurementFromState() {
          const VertexR2* v1 = static_cast<const VertexR2*>(_vertices[0]);
          const VertexR2* v2 = static_cast<const VertexR2*>(_vertices[1]);
          _measurement = v2->estimate()- _sys_A * (v1->estimate());
          return true;
        }
        
        virtual number_t initialEstimatePossible(const  OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 0;}

      // #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      //       virtual void linearizeOplus();
      // #endif


      // Set measurements
      protected:
        virtual bool resolveCaches();
    };

  } // end namespace
} // end namespace

#endif
