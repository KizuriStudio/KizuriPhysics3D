// KizuriPhysics - tests/TestFramework.h
// Minimal single-header test framework.
#pragma once

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <type_traits>

namespace kztest {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& Registry() {
    static std::vector<TestCase> registry;
    return registry;
}

inline int& Failures() { static int f = 0; return f; }
inline int& Checks() { static int c = 0; return c; }

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) {
        Registry().push_back({ name, std::move(fn) });
    }
};

inline void ReportFailure(const char* expr, const char* file, int line) {
    ++Failures();
    std::printf("    \033[31mFAIL\033[0m %s:%d: %s\n", file, line, expr);
}

template <typename T>
inline void ReportCompare(const char* expr, const T& a, const T& b, const char* file, int line) {
    ++Failures();
    std::printf("    \033[31mFAIL\033[0m %s:%d: %s\n", file, line, expr);
    if constexpr (std::is_arithmetic_v<T>) {
        std::printf("         lhs = %g\n", double(a));
        std::printf("         rhs = %g\n", double(b));
    } else {
        (void)a; (void)b;
    }
}

inline void ReportNear(const char* expr, double a, double b, double tol, const char* file, int line) {
    ++Failures();
    std::printf("    \033[31mFAIL\033[0m %s:%d: %s\n", file, line, expr);
    std::printf("         |%g - %g| = %g > %g\n", a, b, std::fabs(a - b), tol);
}

inline int RunAll() {
    int passed = 0, failed = 0;
    auto start = std::chrono::high_resolution_clock::now();
    for (auto& tc : Registry()) {
        int before = Failures();
        std::printf("  [ RUN  ] %s\n", tc.name.c_str());
        try {
            tc.fn();
        } catch (const std::exception& e) {
            ReportFailure(e.what(), "<exception>", 0);
        } catch (...) {
            ReportFailure("unknown exception", "<exception>", 0);
        }
        if (Failures() == before) {
            std::printf("  [ \033[32mPASS\033[0m ] %s\n", tc.name.c_str());
            ++passed;
        } else {
            std::printf("  [ \033[31mFAIL\033[0m ] %s\n", tc.name.c_str());
            ++failed;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::printf("\n%d passed, %d failed (%d checks) in %.1f ms\n",
                passed, failed, Checks(), ms);
    return failed == 0 ? 0 : 1;
}

} // namespace kztest

#define KZ_TEST(name)                                                        \
    static void name();                                                      \
    static ::kztest::Registrar reg_##name(#name, name);                      \
    static void name()

#define CHECK(expr)                                                          \
    do {                                                                     \
        ++::kztest::Checks();                                                \
        if (!(expr)) ::kztest::ReportFailure(#expr, __FILE__, __LINE__);     \
    } while (0)

#define CHECK_EQ(a, b)                                                       \
    do {                                                                     \
        ++::kztest::Checks();                                                \
        auto _a = (a); auto _b = (b);                                        \
        if (!(_a == _b)) ::kztest::ReportCompare(#a " == " #b, _a, _b, __FILE__, __LINE__); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        ++::kztest::Checks();                                                \
        double _a = double(a); double _b = double(b);                        \
        if (std::fabs(_a - _b) > double(tol))                                \
            ::kztest::ReportNear(#a " ~= " #b, _a, _b, double(tol), __FILE__, __LINE__); \
    } while (0)

#define CHECK_TRUE(expr)  CHECK(expr)
#define CHECK_FALSE(expr) CHECK(!(expr))
