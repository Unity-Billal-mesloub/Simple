//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_pinocchio_template_instantiation_joint_model_txx__
#define __simple_pinocchio_template_instantiation_joint_model_txx__

extern template struct ::pinocchio::
  JointModelTpl<::simple::context::Scalar, ::simple::context::Options, ::pinocchio::JointCollectionDefaultTpl>;

extern template struct ::pinocchio::
  JointDataTpl<::simple::context::Scalar, ::simple::context::Options, ::pinocchio::JointCollectionDefaultTpl>;

#endif
