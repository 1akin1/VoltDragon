/**
 * @file unit.h
 * @brief Minimal unit-test harness for host-built firmware modules.
 *
 * Each test is a void function. A failed CHECK reports the location and ends
 * that test; the program exits non-zero if any test failed, so ctest sees it.
 */
#ifndef UNIT_H
#define UNIT_H

#include <stdio.h>
#include <string.h>

extern int unit_failures;
extern int unit_current_failed;

#define CHECK(cond)                                                                 \
    do                                                                              \
    {                                                                               \
        if (!(cond))                                                                \
        {                                                                           \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                \
            unit_current_failed = 1;                                                \
            return;                                                                 \
        }                                                                           \
    } while (0)

#define CHECK_EQ(actual, expected)                                                  \
    do                                                                              \
    {                                                                               \
        const long long a_ = (long long)(actual);                                   \
        const long long e_ = (long long)(expected);                                 \
        if (a_ != e_)                                                               \
        {                                                                           \
            printf("  FAIL %s:%d: %s == %lld, expected %lld\n", __FILE__, __LINE__, \
                   #actual, a_, e_);                                                \
            unit_current_failed = 1;                                                \
            return;                                                                 \
        }                                                                           \
    } while (0)

#define CHECK_STR(actual, expected)                                                 \
    do                                                                              \
    {                                                                               \
        if (strcmp((actual), (expected)) != 0)                                      \
        {                                                                           \
            printf("  FAIL %s:%d: %s == \"%s\", expected \"%s\"\n", __FILE__,       \
                   __LINE__, #actual, (actual), (expected));                        \
            unit_current_failed = 1;                                                \
            return;                                                                 \
        }                                                                           \
    } while (0)

#define RUN(test)                                                                   \
    do                                                                              \
    {                                                                               \
        unit_current_failed = 0;                                                    \
        test();                                                                     \
        printf("%s %s\n", unit_current_failed ? "FAIL" : "ok  ", #test);            \
        unit_failures += unit_current_failed;                                       \
    } while (0)

/** Defines the harness globals; use once per test executable, before main(). */
#define UNIT_MAIN_DEFINITIONS   \
    int unit_failures = 0;      \
    int unit_current_failed = 0

#endif /* UNIT_H */
