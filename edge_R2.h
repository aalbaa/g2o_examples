// Error function is linear and is the form
//    x_k = A * x_km1 + B * u_km1

#ifndef G2O_TUTORIAL_EDGE_R2_H
#define G2O_TUTORIAL_EDGE_R2_H

#include "vertex_R2.h"
#include "g2o_tutorial_slam_R2_api.h"
#include "g2o/core/base_binary_edge.h"

namespace g2o {

  namespace tutorial {

    /**
     * \brief 2D edge between two Vertex2, i.e., the odometry
     */
    class G2O_TUTORIAL_SLAM_R2_API EdgeR2 : public BaseBinaryEdge<2, Vector2, VertexR2, VertexR2>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeR2();
        // Constructor that takes the system matrices
        EdgeR2(const Eigen::Matrix2d, const Eigen::MatrixXd);
        // Destructor
        ~EdgeR2();

        void computeError();
        
        void setMeasurement(const Vector2& m){
          _measurement = m;
          _inverseMeasurement = -m;
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        // Function to set the A matrix
        void setMatrix_A(const Eigen::Matrix2d& A_in);

        Eigen::Matrix2d getMatrix_A(){ return _sys_A;};
        Eigen::MatrixXd getMatrix_B(){ return _sys_B;};

        // Function to set the B matrix
        void setMatrix_B(const Eigen::MatrixXd& B_in);        

      protected:
        Vector2 _inverseMeasurement;
        
        // System matrices
        Eigen::Matrix2d _sys_A;
        Eigen::MatrixXd _sys_B;

        // Boolean variables to specify whether a matrix is defined or not
        bool _sys_A_defined;
        bool _sys_B_defined;

        // Boolean to specify whether the system matrices were defined internally (in order to cealn them)
        bool _sys_A_isInternal = false;
        bool _sys_B_isInternal = false;
    };
  }
} // end namespace
#endif
