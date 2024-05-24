//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_core_fwd_hpp__
#define __simple_core_fwd_hpp__

#include "simple/fwd.hpp"
#include <pinocchio/utils/reference.hpp>

namespace simple
{
  namespace helper
  {
    using pinocchio::helper::get_pointer;
    using pinocchio::helper::get_ref;
  } // namespace helper

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl = ::pinocchio::JointCollectionDefaultTpl>
  struct ConstraintsProblemTpl;
  typedef ConstraintsProblemTpl<context::Scalar, context::Options> ConstraintsProblem;

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl = ::pinocchio::JointCollectionDefaultTpl>
  struct SimulatorTpl;
  typedef SimulatorTpl<context::Scalar, context::Options> Simulator;

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl = ::pinocchio::JointCollectionDefaultTpl>
  struct ContactSolverDerivativesTpl;
  typedef ContactSolverDerivativesTpl<context::Scalar, context::Options> ContactSolverDerivatives;

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl = ::pinocchio::JointCollectionDefaultTpl>
  struct SimulatorDerivativesTpl;
  typedef SimulatorDerivativesTpl<context::Scalar, context::Options> SimulatorDerivatives;

  template<typename Scalar, int Options>
  struct PlacementFromNormalAndPositionTpl;
  typedef PlacementFromNormalAndPositionTpl<context::Scalar, context::Options> PlacementFromNormalAndPosition;

} // namespace simple

#endif // ifndef __simple_core_fwd_hpp__
