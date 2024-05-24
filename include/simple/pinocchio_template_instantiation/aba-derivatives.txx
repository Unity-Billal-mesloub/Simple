//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_pinocchio_template_instantiation_aba_derivatives_txx__
#define __simple_pinocchio_template_instantiation_aba_derivatives_txx__

extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI void ::pinocchio::computeABADerivatives<
  ::simple::context::Scalar,
  ::simple::context::Options,
  ::pinocchio::JointCollectionDefaultTpl,
  ::simple::context::VectorXs,
  ::simple::context::VectorXs,
  ::simple::context::VectorXs,
  ::simple::context::Force,
  Eigen::aligned_allocator<::simple::context::Force>,
  Eigen::Ref<::simple::context::RowMatrixXs>,
  Eigen::Ref<::simple::context::RowMatrixXs>,
  Eigen::Ref<::simple::context::RowMatrixXs>>(
  const ::simple::context::Model &,
  ::simple::context::Data &,
  const Eigen::MatrixBase<::simple::context::VectorXs> &,
  const Eigen::MatrixBase<::simple::context::VectorXs> &,
  const Eigen::MatrixBase<::simple::context::VectorXs> &,
  const std::vector<::simple::context::Force, Eigen::aligned_allocator<::simple::context::Force>> &,
  const Eigen::MatrixBase<Eigen::Ref<::simple::context::RowMatrixXs>> &,
  const Eigen::MatrixBase<Eigen::Ref<::simple::context::RowMatrixXs>> &,
  const Eigen::MatrixBase<Eigen::Ref<::simple::context::RowMatrixXs>> &);

#endif // __simple_pinocchio_template_instantiation_aba_derivatives_txx__
