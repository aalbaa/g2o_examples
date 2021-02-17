// Implement R^2 vertex (Euclidean vector space)
// I looked at https://github.com/RainerKuemmerle/g2o/blob/master/g2o/types/slam2d/vertex_point_xy.h for guidance.

#ifndef G2O_VERTEX_R2_H
#define G2O_VERTEX_R2_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "slam_R2_example_api.h"

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

        // virtual bool setEstimateDataImpl(const number_t* est){
        //   _estimate[0] = est[0];
        //   _estimate[1] = est[1];
        //   return true;
        // }

        // virtual bool getEstimateData(number_t* est) const{
        //   est[0] = _estimate[0];
        //   est[1] = _estimate[1];
        //   return true;
        // }

        // virtual int estimateDimension() const { 
        //   return 2;
        // }

        // virtual bool setMinimalEstimateDataImpl(const number_t* est){
        //   return setEstimateData(est);
        // }

        //  virtual bool getMinimalEstimateData(number_t* est) const{
        //   return getEstimateData(est);
        // }

        // virtual int minimalEstimateDimension() const { 
        //   return 2;
        // }

        virtual void oplusImpl(const double* update)
        {
          // Implementation that takes a 2-element double array
          _estimate[0] += update[0];
          _estimate[1] += update[1];
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    };

    class G2O_SLAM_R2_API VertexR2WriteGnuplotAction: public WriteGnuplotAction {
    public:
      VertexR2WriteGnuplotAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
              HyperGraphElementAction::Parameters* params_);
    };

  // #ifdef G2O_HAVE_OPENGL
  //   class G2O_SLAM_R2_API VertexR2DrawAction: public DrawAction{
  //   public:
  //     VertexR2DrawAction();
  //     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
  //             HyperGraphElementAction::Parameters* params_);
  //   protected:
  //     FloatProperty *_pointSize;
  //     virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
  //   };
  // #endif

  } // end namespace example
} // end namespace g2o

#endif
