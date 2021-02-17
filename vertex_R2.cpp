#include "vertex_R2.h"

// #ifdef G2O_HAVE_OPENGL
// #include "g2o/stuff/opengl_wrapper.h"
// #include "g2o/stuff/opengl_primitives.h"
// #endif

#include <typeinfo>

#include "g2o/stuff/macros.h"

namespace g2o {
  namespace example {

    VertexR2::VertexR2() :
      BaseVertex<2, Eigen::Vector2d>()
    {
      _estimate.setZero();
    }

    bool VertexR2::read(std::istream& is)
    {
      Eigen::Vector2d p;
      is >> p[0] >> p[1];
      _estimate[0] = p[0];
      _estimate[1] = p[1];
      return true;
          // return internal::readVector(is, _estimate);
      // is >> _estimate[0] >> _estimate[1];
      // return true;
    }

    bool VertexR2::write(std::ostream& os) const
    {
      Eigen::Vector2d p = estimate();
      os << p[0] << " " << p[1];
      return os.good();
      // return internal::writeVector(os, estimate());
      // os << estimate()(0) << " " << estimate()(1);
      // return os.good();
    }
  } // end namespace
} // end namespace
