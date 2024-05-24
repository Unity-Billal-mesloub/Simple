#include "simple/fwd.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"

template struct ::pinocchio::JointModelTpl<::simple::context::Scalar, ::simple::context::Options, ::pinocchio::JointCollectionDefaultTpl>;

template struct ::pinocchio::JointDataTpl<::simple::context::Scalar, ::simple::context::Options, ::pinocchio::JointCollectionDefaultTpl>;
