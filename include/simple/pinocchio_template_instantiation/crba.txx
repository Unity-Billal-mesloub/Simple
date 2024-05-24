//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_pinocchio_template_instantiation_crba_txx__
#define __simple_pinocchio_template_instantiation_crba_txx__

extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const ::simple::context::MatrixXs & ::pinocchio::
  crba<::simple::context::Scalar, ::simple::context::Options, ::pinocchio::JointCollectionDefaultTpl, ::simple::context::VectorXs>(
    const context::Model &, context::Data &, const Eigen::MatrixBase<context::VectorXs> &, const Convention convention);

#endif // __simple_pinocchio_template_instantiation_crba_txx__
