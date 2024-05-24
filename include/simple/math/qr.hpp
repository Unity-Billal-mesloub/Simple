//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_math_qr_hpp__
#define __simple_math_qr_hpp__

#include "simple/math/fwd.hpp"

#include <Eigen/QR>

namespace simple
{
  namespace math
  {
    template<typename _SolverType>
    struct SolveInPlaceWrapper : _SolverType
    {
      typedef _SolverType SolverType;

      template<typename MatrixType>
      void solveInPlace(const Eigen::MatrixBase<MatrixType> & mat) const
      {
        typename PINOCCHIO_EIGEN_PLAIN_TYPE(MatrixType) res(mat);
        res = this->solve(mat);
        mat.const_cast_derived() = res;
      }

    }; // struct SolveInPlaceWrapper

    template<typename _MatrixType>
    struct SolveInPlaceWrapper<Eigen::HouseholderQR<_MatrixType>> : Eigen::HouseholderQR<_MatrixType>
    {
      typedef Eigen::HouseholderQR<_MatrixType> SolverType;

      template<typename MatrixType>
      void solveInPlace(const Eigen::MatrixBase<MatrixType> & mat_) const
      {
        const Eigen::Index rank = (std::min)(this->rows(), this->cols());
        MatrixType & mat = mat_.const_cast_derived();

        mat.applyOnTheLeft(this->householderQ().setLength(rank).adjoint());

        this->m_qr.topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solveInPlace(mat.topRows(rank));
        mat.bottomRows(this->cols() - rank).setZero();
      }
    };

  } // namespace math
} // namespace simple

#endif // ifndef __simple_math_qr_hpp__
