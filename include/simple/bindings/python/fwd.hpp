//
// Copyright (c) 2024 INRIA
//

#ifndef __simple_python_fwd_hpp__
#define __simple_python_fwd_hpp__

#include <eigenpy/eigenpy.hpp>

namespace simple
{
  namespace python
  {
    void exposeContactFrame();
    void exposeConstraintsProblem();
    void exposeSimulator();
  } // namespace python
} // namespace simple

#endif // #ifndef __simple_python_fwd_hpp__
