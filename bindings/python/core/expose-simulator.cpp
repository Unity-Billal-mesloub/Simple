//
// Copyright (c) 2024 INRIA
//

#include "simple/bindings/python/core/simulator.hpp"

namespace simple
{
  namespace python
  {

    void exposeSimulator()
    {
      SimulatorPythonVisitor<Simulator>::expose();
    }

  } // namespace python
} // namespace simple
