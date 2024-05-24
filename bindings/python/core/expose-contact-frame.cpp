//
// Copyright (c) 2024 INRIA
//

#include "simple/core/contact-frame.hpp"
#include "simple/bindings/python/fwd.hpp"

#include <boost/python/tuple.hpp>

namespace bp = boost::python;

namespace simple
{
  namespace python
  {
    using SE3 = ::simple::PlacementFromNormalAndPosition::SE3;
    using Vector3s = ::simple::PlacementFromNormalAndPosition::Vector3s;
    using Matrix6x3s = ::simple::PlacementFromNormalAndPosition::Matrix6x3s;

    SE3 placementFromNormalAndPosition(const Vector3s & normal, const Vector3s & position)
    {
      SE3 M;
      ::simple::PlacementFromNormalAndPosition::calc(normal, position, M);
      return M;
    }

    bp::tuple placementFromNormalAndPositionDerivative(const SE3 & M)
    {
      Matrix6x3s dM_dnormal;
      Matrix6x3s dM_dposition;
      ::simple::PlacementFromNormalAndPosition::calcDiff(M, dM_dnormal, dM_dposition);
      return bp::make_tuple(dM_dnormal, dM_dposition);
    }

    void exposeContactFrame()
    {
      bp::def(
        "placementFromNormalAndPosition", placementFromNormalAndPosition, bp::args("normal", "position"),
        "Returns a placement such that `normal` is the z-axis of the placement's rotation and "
        "`position` is the translation of the placement.\n"
        "Parameters:\n"
        "\tnormal: z-axis of the placement's rotation.\n"
        "\tposition: translation part of the placement.\n\n"
        "Returns: M, the placement.");

      bp::def(
        "placementFromNormalAndPositionDerivative", placementFromNormalAndPositionDerivative, bp::args("M"),
        "Returns the derivatives of a placement w.r.t both the normal and position than generated "
        "it. The normal is the z-axis of the placement's rotation and the position is the "
        "translation of the part of the placement."
        "Parameters:\n"
        "\tM: a placement.\n\n"
        "Returns: a tuple (dM_dnormal, dM_dposition), both are 6x3 matrices.");
    }

  } // namespace python
} // namespace simple
