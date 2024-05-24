//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_pinocchio_template_instantiation_aba_txx__
#define __simple_pinocchio_template_instantiation_aba_txx__

extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const ::simple::context::VectorXs & ::pinocchio::aba<
  ::simple::context::Scalar,
  ::simple::context::Options,
  ::pinocchio::JointCollectionDefaultTpl,
  ::simple::context::VectorXs,
  ::simple::context::VectorXs,
  ::simple::context::VectorXs>(
  const ::simple::context::Model &,
  ::simple::context::Data &,
  const Eigen::MatrixBase<context::VectorXs> &,
  const Eigen::MatrixBase<context::VectorXs> &,
  const Eigen::MatrixBase<context::VectorXs> &,
  const Convention);

extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI const ::simple::context::VectorXs & ::pinocchio::aba<
  ::simple::context::Scalar,
  ::simple::context::Options,
  ::pinocchio::JointCollectionDefaultTpl,
  ::simple::context::VectorXs,
  ::simple::context::VectorXs,
  ::simple::context::VectorXs,
  ::simple::context::Force,
  Eigen::aligned_allocator<::simple::context::Force>>(
  const ::simple::context::Model &,
  ::simple::context::Data &,
  const Eigen::MatrixBase<context::VectorXs> &,
  const Eigen::MatrixBase<context::VectorXs> &,
  const Eigen::MatrixBase<context::VectorXs> &,
  const std::vector<::simple::context::Force, Eigen::aligned_allocator<::simple::context::Force>> &,
  const Convention);

#endif
