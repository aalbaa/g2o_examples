#include "edge_R2_R2.h"

// using namespace Eigen;

namespace g2o {
  namespace example {

    EdgeR2R2::EdgeR2R2() :
      BaseBinaryEdge<2, Eigen::Vector2d, VertexR2, VertexR2>()
    {
      _information.setIdentity();
      _error.setZero();
    }

    bool EdgeR2R2::read(std::istream& is)
    {
      Eigen::Vector2d p;
      internal::readVector(is, p);
      setMeasurement(p);
      readInformationMatrix(is);
      return true;
      // int paramId;
      // is >> paramId;
      // if (! setParameterId(0,paramId))
      //   return false;
      // // Output measurement values
      // is >> _measurement[0] >> _measurement[1];
      // // Output information matrix
      // is >> information()(0,0) >> information()(0,1) >> information()(1,1);
      // information()(1,0) = information()(0,1);
      // return true;
    }

    bool EdgeR2R2::write(std::ostream& os) const
    {
      internal::writeVector(os, measurement());
      return writeInformationMatrix(os);
      // // os << _sensorOffset->id() << " ";
      // os << measurement()[0] << " " << measurement()[1] << " ";
      // os << information()(0,0) << " " << information()(0,1) << " " << information()(1,1);
      // return os.good();
    }

    void EdgeR2R2::computeError()
    {
      if(!(_sys_A_initialized && _sys_B_initialized)){
        std::cout << "Missing A and B matrices" << std::endl;
        return;
      }
      
      const VertexR2* l1 = static_cast<const VertexR2*>(_vertices[0]);
      const VertexR2* l2 = static_cast<const VertexR2*>(_vertices[1]);
      // _error = (_sensorCache->w2n() * l2->estimate()) - _measurement;
      _error = l2 -> estimate() - _sys_A * ( l1 -> estimate()) - _sys_B * _measurement(0);
    }

    // I do not understand what this function is supposed to do. But without it, I cannot create EdgeR2R2 instances on the heap.
    bool EdgeR2R2::resolveCaches()
    {
      // ParameterVector pv(1);
      // pv[0] = _sensorOffset;
      // resolveCache(_sensorCache, static_cast<OptimizableGraph::Vertex*>(_vertices[0]), "TUTORIAL_CACHE_SE2_OFFSET", pv);
      // return _sensorCache != 0;
      return true;
    }
  } // end namespace
} // end namespace
