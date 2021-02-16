#include "vertex_R2.h"

// #ifdef G2O_HAVE_OPENGL
// #include "g2o/stuff/opengl_wrapper.h"
// #include "g2o/stuff/opengl_primitives.h"
// #endif

#include <typeinfo>

#include "g2o/stuff/macros.h"

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
          return internal::readVector(is, _estimate);
      // is >> _estimate[0] >> _estimate[1];
      // return true;
    }

    bool VertexR2::write(std::ostream& os) const
    {
      return internal::writeVector(os, estimate());
      // os << estimate()(0) << " " << estimate()(1);
      // return os.good();
    }

    VertexR2WriteGnuplotAction::VertexR2WriteGnuplotAction(): WriteGnuplotAction(typeid(VertexR2).name()){}

    HyperGraphElementAction* VertexR2WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
      if (typeid(*element).name()!=_typeName)
        return nullptr;

      WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
      if (!params->os){
        std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
        return nullptr;
      }

      VertexR2* v =  static_cast<VertexR2*>(element);
      *(params->os) << v->estimate().x() << " " << v->estimate().y() << std::endl;
      return this;
  }

  // #ifdef G2O_HAVE_OPENGL
  //   VertexR2DrawAction::VertexR2DrawAction() : DrawAction(typeid(VertexR2).name()), _pointSize(nullptr) {}

  //   bool VertexR2DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
  //     if (! DrawAction::refreshPropertyPtrs(params_))
  //       return false;
  //     if (_previousParams){
  //       _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
  //     } else {
  //       _pointSize = 0;
  //     }
  //     return true;
  //   }

  //   HyperGraphElementAction* VertexR2DrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params) {
  //     if (typeid(*element).name()!=_typeName)
  //       return nullptr;
  //     initializeDrawActionsCache();
  //     refreshPropertyPtrs(params);
  //     if (! _previousParams)
  //       return this;

  //     if (_show && !_show->value())
  //       return this;
  //     VertexR2* that = static_cast<VertexR2*>(element);

  //     glPushMatrix();
  //     glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
  //     glDisable(GL_LIGHTING);
  //     glColor3f(LANDMARK_VERTEX_COLOR);
  //     float ps = _pointSize ? _pointSize->value() :  1.0f;
  //     glTranslatef((float)that->estimate()(0),(float)that->estimate()(1),0.0f);
  //     opengl::drawPoint(ps);
  //     glPopAttrib();
  //     drawCache(that->cacheContainer(), params);
  //     drawUserData(that->userData(), params);
  //     glPopMatrix();
  //     return this;
  //   }
  // #endif
  } // end namespace
} // end namespace
