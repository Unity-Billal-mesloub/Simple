#include "simple/fwd.hpp"
#include <pinocchio/algorithm/crba.hpp>

template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const ::simple::context::MatrixXs & ::pinocchio::
  crba<::simple::context::Scalar, ::simple::context::Options, ::pinocchio::JointCollectionDefaultTpl, ::simple::context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Convention convention);
