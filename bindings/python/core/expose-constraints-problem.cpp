//
// Copyright (c) 2024 INRIA
//

#include "simple/bindings/python/core/constraints-problem.hpp"

namespace bp = boost::python;

namespace simple
{
  namespace python
  {

    void exposeConstraintsProblem()
    {
      bp::class_<ConstraintsProblem::ContactMapper>(
        "ContactMapper",
        "Maps a single collision pair to contact points information (contact position, contact "
        "normal, contact placements, constraint models/datas, friction cones, elasticities, "
        "penetration depths).",
        bp::no_init)
        .def_readwrite(
          "begin", &ConstraintsProblem::ContactMapper::begin, "The id of the first contact point for the considered collision pair.")
        .def_readwrite("count", &ConstraintsProblem::ContactMapper::count, "Number of contact points for the considered collision pair");

      using ContactMode = ConstraintsProblem::ContactMode;
      bp::enum_<ContactMode>("ContactMode")
        .value("BREAKING", ContactMode::BREAKING)
        .value("STICKING", ContactMode::STICKING)
        .value("SLIDING", ContactMode::SLIDING)
        .export_values();

      // Register a handle to ConstraintsProblem
      using ConstraintsProblemHandle = std::shared_ptr<ConstraintsProblem>;
      bp::register_ptr_to_python<ConstraintsProblemHandle>();

      ConstraintsProblemPythonVisitor<ConstraintsProblem>::expose();
    }

  } // namespace python
} // namespace simple
