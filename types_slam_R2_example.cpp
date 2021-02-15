#include "types_slam_R2_example.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {
  namespace example {

  G2O_REGISTER_TYPE_GROUP(main);
  
  G2O_REGISTER_TYPE(EXAMPLE_VERTEX_R2, VertexR2);

//   G2O_REGISTER_TYPE(TUTORIAL_EDGE_SE2, EdgeSE2);
  G2O_REGISTER_TYPE(EXAMPLE_EDGE_R2_R2, EdgeR2R2);
  }
} // end namespace
