//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_fwd_hpp__
#define __simple_fwd_hpp__

#ifdef _WIN32
  #include <windows.h>
  #undef far
  #undef near
#endif

#include <cassert>

#ifdef SIMPLE_EIGEN_CHECK_MALLOC
  #ifndef EIGEN_RUNTIME_NO_MALLOC
    #define EIGEN_RUNTIME_NO_MALLOC_WAS_NOT_DEFINED
    #define EIGEN_RUNTIME_NO_MALLOC
  #endif
#endif

// #include "simple/macros.hpp"
#include "simple/deprecated.hpp"
#include "simple/warning.hpp"
#include "simple/config.hpp"

// Include Eigen components
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#ifdef SIMPLE_WITH_ACCELERATE_SUPPORT
  #include <Eigen/AccelerateSupport>
#endif

#include <pinocchio/multibody/model.hpp>

namespace simple
{
  ///
  /// \brief Common traits structure to fully define base classes for CRTP.
  ///
  template<class C>
  struct traits
  {
  };

  namespace context
  {
    typedef double Scalar;
    enum
    {
      Options = 0
    };

    // Common eigen types
    using Vector2s = Eigen::Matrix<Scalar, 2, 1, Options>;
    using Vector3s = Eigen::Matrix<Scalar, 3, 1, Options>;
    using Vector6s = Eigen::Matrix<Scalar, 6, 1, Options>;
    using Matrix6s = Eigen::Matrix<Scalar, 6, 6, Options>;
    using Matrix63s = Eigen::Matrix<Scalar, 6, 3, Options>;
    using Matrix6Xs = Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options>;
    using VectorXs = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options>;
    using MatrixXs = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options>;
    using RowMatrixXs = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options | Eigen::RowMajor>;

    // Commong pinocchio types
    using Force = ::pinocchio::ForceTpl<Scalar, Options>;
    using Motion = ::pinocchio::MotionTpl<Scalar, Options>;
    using SE3 = ::pinocchio::SE3Tpl<Scalar, Options>;
    using Model = ::pinocchio::ModelTpl<Scalar, Options, ::pinocchio::JointCollectionDefaultTpl>;
    using Data = ::pinocchio::DataTpl<Scalar, Options, ::pinocchio::JointCollectionDefaultTpl>;
  } // namespace context

} // namespace simple

#endif // ifndef __simple_fwd_hpp__
