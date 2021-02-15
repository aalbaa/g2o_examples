#ifndef G2O_TUTORIAL_EDGE_SE2_POINT_XY_H
#define G2O_TUTORIAL_EDGE_SE2_POINT_XY_H

#include "vertex_R2.h"
#include "slam_R2_example_api.h"

#include "g2o/core/base_binary_edge.h"

namespace g2o {

  namespace example {

    class G2O_SLAM_R2_API EdgeR2R2 : public BaseBinaryEdge<2, Eigen::Vector2d, VertexR2, VertexR2>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeR2R2();

        void computeError();
  
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
      
      protected:
        virtual bool resolveCaches();
    };

  } // end namespace
} // end namespace

#endif
