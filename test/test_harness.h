// Tiny assertion harness. Deliberately dependency-free: the point of the host
// tests is that anyone can run them with nothing but a C++ compiler, and that
// CI needs no package installation step.

#pragma once

#include <cstdio>
#include <cstdlib>

inline int testsRun = 0;
inline int testsFailed = 0;
inline const char *currentTest = "";

#define TEST(name)                     \
  currentTest = name;                  \
  printf("  %s\n", name);

#define CHECK(cond)                                                     \
  do {                                                                  \
    testsRun++;                                                         \
    if (!(cond)) {                                                      \
      testsFailed++;                                                    \
      printf("    FAIL %s:%d in [%s]\n         expected: %s\n",         \
             __FILE__, __LINE__, currentTest, #cond);                   \
    }                                                                   \
  } while (0)

#define CHECK_EQ(actual, expected)                                      \
  do {                                                                  \
    testsRun++;                                                         \
    long long a_ = (long long)(actual);                                 \
    long long e_ = (long long)(expected);                               \
    if (a_ != e_) {                                                     \
      testsFailed++;                                                    \
      printf("    FAIL %s:%d in [%s]\n         %s\n"                    \
             "         expected: %lld\n         actual:   %lld\n",      \
             __FILE__, __LINE__, currentTest, #actual, e_, a_);         \
    }                                                                   \
  } while (0)

// Assert a floating point value is within tolerance.
#define CHECK_NEAR(actual, expected, tol)                               \
  do {                                                                  \
    testsRun++;                                                         \
    double a_ = (double)(actual);                                       \
    double e_ = (double)(expected);                                     \
    double d_ = a_ - e_;                                                \
    if (d_ < 0) d_ = -d_;                                               \
    if (!(d_ <= (double)(tol))) {                                       \
      testsFailed++;                                                    \
      printf("    FAIL %s:%d in [%s]\n         %s\n"                    \
             "         expected: %f (+/- %f)\n         actual:   %f\n", \
             __FILE__, __LINE__, currentTest, #actual, e_,              \
             (double)(tol), a_);                                        \
    }                                                                   \
  } while (0)

inline int testSummary(const char *suite) {
  if (testsFailed == 0) {
    printf("  %s: %d checks passed\n\n", suite, testsRun);
    return 0;
  }
  printf("  %s: %d of %d checks FAILED\n\n", suite, testsFailed, testsRun);
  return 1;
}
