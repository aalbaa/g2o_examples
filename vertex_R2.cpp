#include "vertex_R2.h"

using namespace Eigen;

namespace g2o {
  namespace example {

    VertexR2::VertexR2() :
      BaseVertex<2, Vector2d>()
    {
      _estimate.setZero();
    }

    bool VertexR2::read(std::istream& is)
    {
      is >> _estimate[0] >> _estimate[1];
      return true;
    }

    bool VertexR2::write(std::ostream& os) const
    {
      os << estimate()(0) << " " << estimate()(1);
      return os.good();
    }

  } // end namespace
} // end namespace
