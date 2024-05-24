//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __simple_core_constraints_problem_txx__
#define __simple_core_constraints_problem_txx__

#include "simple/core/constraints-problem.hpp"

namespace simple
{

  extern template struct ConstraintsProblemTpl<context::Scalar, context::Options, ::pinocchio::JointCollectionDefaultTpl>;

} // namespace simple

#endif // __simple_core_constraints_problem_txx__
