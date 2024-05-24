//
// Copyright (c) 2025 INRIA
//

#ifndef __simple_utils_visitors_hpp__
#define __simple_utils_visitors_hpp__

#include <boost/variant.hpp>
#include <pinocchio/macros.hpp>
#include <pinocchio/algorithm/constraints/visitors/constraint-model-visitor.hpp>

namespace simple
{
  namespace visitors
  {
    // Declaration of a helper to accumulate operator() overloads from lambdas.
    template<typename ReturnType, typename... Lambdas>
    struct lambda_visitor_helper;

    // Specialization for initial case without any lambda
    template<typename ReturnType>
    struct lambda_visitor_helper<ReturnType>
    {
      // Default constructor
      lambda_visitor_helper()
      {
      }

      // Operator that handles boost::blank case
      ReturnType operator()(const boost::blank &) const
      {
        PINOCCHIO_THROW_PRETTY(std::invalid_argument, "Cannot call operator() on boost::blank.");
        return ::pinocchio::visitors::internal::NoRun<ReturnType>::run();
      }
    };

    // Specialization for recursion
    template<typename ReturnType, typename Lambda, typename... Lambdas>
    struct lambda_visitor_helper<ReturnType, Lambda, Lambdas...>
    : Lambda
    , lambda_visitor_helper<ReturnType, Lambdas...>
    {
      typedef lambda_visitor_helper<ReturnType, Lambdas...> RecursiveHelper;

      // Initialize lambda and the recursion
      lambda_visitor_helper(Lambda lambda, Lambdas... lambdas)
      : Lambda(lambda)
      , RecursiveHelper(lambdas...)
      {
      }

      // operator() is overloaded by lambda and all overloa of the recursion
      using Lambda::operator();
      using RecursiveHelper::operator();
    };

    // Wrapper struct around helper that inherit static_visitor
    // It is an additional class to avoid diamond inheritance
    template<typename ReturnType, typename... Lambdas>
    struct lambda_visitor
    : boost::static_visitor<ReturnType>
    , lambda_visitor_helper<ReturnType, Lambdas...>
    {
      typedef lambda_visitor_helper<ReturnType, Lambdas...> Helper;

      lambda_visitor(Lambdas... lambdas)
      : Helper(lambdas...)
      {
      }

      using Helper::operator();
    };

    // Wrapper function to create a lambda_visitor instance.
    // This deduces the template arguments automatically, so you don't need to specify them.
    template<typename ReturnType = void, typename... Lambdas>
    lambda_visitor<ReturnType, Lambdas...> make_lambda_visitor(Lambdas... lambdas)
    {
      return lambda_visitor<ReturnType, Lambdas...>(lambdas...);
    }

  } // namespace visitors
} // namespace simple

#endif // __simple_utils_visitors_hpp__
