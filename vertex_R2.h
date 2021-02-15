// Implement R^2 vertex (Euclidean vector space)

#ifndef G2O_VERTEX_R2_H
#define G2O_VERTEX_R2_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "slam_R2_example_api.h"

#include <Eigen/Core>

namespace g2o {
  namespace example {

    class G2O_SLAM_R2_API VertexR2 : public BaseVertex<2, Eigen::Vector2d>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexR2();

        virtual void setToOriginImpl() {
          _estimate.setZero();
        }

        virtual void oplusImpl(const double* update)
        {
          // Implementation that takes a 2-element double array
          _estimate[0] += update[0];
          _estimate[1] += update[1];
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    };

  }
}

#endif
