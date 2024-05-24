//
// Copyright (c) 2024 INRIA
//

#include "simple/bindings/python/fwd.hpp"
#include "simple/config.hpp"
#include "simple/fwd.hpp"

#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;
using namespace simple::python;

BOOST_PYTHON_MODULE(SIMPLE_PYTHON_MODULE_NAME)
{
  bp::docstring_options module_docstring_options(true, true, false);

  bp::scope().attr("__version__") = bp::str(SIMPLE_VERSION);
  bp::scope().attr("__raw_version__") = bp::str(SIMPLE_VERSION);

  eigenpy::enableEigenPy();
  using Matrix6x3s = Eigen::Matrix<simple::context::Scalar, 6, 3, simple::context::Options>;
  eigenpy::enableEigenPySpecific<Matrix6x3s>();

  // Enable warnings
  bp::import("warnings");

  // Dependencies
  bp::import("hppfcl");
  bp::import("pinocchio");

  exposeContactFrame();
  exposeConstraintsProblem();
  exposeSimulator();
}
