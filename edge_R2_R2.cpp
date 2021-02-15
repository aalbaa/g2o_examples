#include "edge_R2_R2.h"

using namespace Eigen;

namespace g2o {
  namespace example {

    EdgeR2R2::EdgeR2R2() :
      BaseBinaryEdge<2, Vector2d, VertexR2, VertexR2>()      
    {
      // resizeParameters(1);
      // installParameter(_sensorOffset, 0);
    }

    bool EdgeR2R2::read(std::istream& is)
    {
      int paramId;
      is >> paramId;
      if (! setParameterId(0,paramId))
        return false;
      // Output measurement values
      is >> _measurement[0] >> _measurement[1];
      // Output information matrix
      is >> information()(0,0) >> information()(0,1) >> information()(1,1);
      information()(1,0) = information()(0,1);
      return true;
    }

    bool EdgeR2R2::write(std::ostream& os) const
    {
      // os << _sensorOffset->id() << " ";
      os << measurement()[0] << " " << measurement()[1] << " ";
      os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
      return os.good();
    }

    void EdgeR2R2::computeError()
    {
      const VertexR2* l1 = static_cast<const VertexR2*>(_vertices[0]);
      const VertexR2* l2 = static_cast<const VertexR2*>(_vertices[1]);
      // _error = (_sensorCache->w2n() * l2->estimate()) - _measurement;
      _error = l2 -> estimate() - l1 -> estimate();
    }

    // bool VertexR2::resolveCaches()
    // {
    //   ParameterVector pv(1);
    //   pv[0] = _sensorOffset;
    //   resolveCache(_sensorCache, static_cast<OptimizableGraph::Vertex*>(_vertices[0]), "TUTORIAL_CACHE_SE2_OFFSET", pv);
    //   return _sensorCache != 0;
    // }

  } // end namespace
} // end namespace
