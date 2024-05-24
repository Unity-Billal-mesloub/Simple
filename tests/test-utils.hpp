#ifndef __simple__test_utils_hpp__
#define __simple__test_utils_hpp__

#define EIGEN_VECTOR_IS_APPROX(Va, Vb, precision)                                                                                          \
  BOOST_CHECK_MESSAGE(                                                                                                                     \
    ((Va) - (Vb)).isZero(precision), "check " #Va ".isApprox(" #Vb ") failed at precision "                                                \
                                       << precision << ". (" #Va " - " #Vb ").norm() = " << ((Va) - (Vb)).norm() << " [\n"                 \
                                       << (Va).transpose() << "\n!=\n"                                                                     \
                                       << (Vb).transpose() << "\n]")

#define INDEX_EQUALITY_CHECK(i1, i2) BOOST_CHECK_MESSAGE(i1 == i2, "check " #i1 "==" #i2 " failed. [" << i1 << " != " << i2 << "]")
#define INDEX_INEQUALITY_CHECK(i1, i2) BOOST_CHECK_MESSAGE(i1 <= i2, "check " #i1 "==" #i2 " failed. [" << i1 << " != " << i2 << "]")

#define REAL_IS_APPROX(a, b, precision)                                                                                                    \
  BOOST_CHECK_MESSAGE(                                                                                                                     \
    std::abs((a) - (b)) < precision,                                                                                                       \
    "check std::abs(" #a " - " #b ") = " << std::abs((a) - (b)) << " < " << precision << " failed. [" << a << " != " << b << "]")

#endif
