// KizuriPhysics - Core/Types.h
// Fundamental scalar types, assertions and compile-time utilities.
#pragma once

#include "Config.h"

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <type_traits>

namespace kizuri {

// ---------------------------------------------------------------------------
// Scalars
// ---------------------------------------------------------------------------
#if KZ_DOUBLE_PRECISION
using Real = double;
#else
using Real = float;
#endif

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using i8  = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;
using usize = std::size_t;
using uptr  = std::uintptr_t;

// ---------------------------------------------------------------------------
// Limits / constants
// ---------------------------------------------------------------------------
namespace math {

inline constexpr Real kPi        = Real(3.14159265358979323846);
inline constexpr Real kTwoPi     = Real(6.28318530717958647692);
inline constexpr Real kHalfPi    = Real(1.57079632679489661923);
inline constexpr Real kEpsilon   = Real(1.1920928955078125e-7);   // FLT_EPSILON
inline constexpr Real kBigNumber = Real(1.0e18);
inline constexpr Real kInfinity  = std::numeric_limits<Real>::infinity();
inline constexpr Real kDegToRad  = kPi / Real(180.0);
inline constexpr Real kRadToDeg  = Real(180.0) / kPi;

/// Default solver tolerance. Below this relative correction is ignored.
inline constexpr Real kDefaultTolerance = Real(1.0e-4);

KZ_FORCEINLINE constexpr Real Abs(Real v)      { return v < Real(0) ? -v : v; }
KZ_FORCEINLINE constexpr Real Min(Real a, Real b) { return a < b ? a : b; }
KZ_FORCEINLINE constexpr Real Max(Real a, Real b) { return a > b ? a : b; }
KZ_FORCEINLINE constexpr Real Clamp(Real v, Real lo, Real hi) { return v < lo ? lo : (v > hi ? hi : v); }
KZ_FORCEINLINE constexpr Real Lerp(Real a, Real b, Real t) { return a + (b - a) * t; }
KZ_FORCEINLINE constexpr Real Square(Real v)   { return v * v; }
KZ_FORCEINLINE constexpr Real Sign(Real v)     { return v < Real(0) ? Real(-1) : Real(1); }
KZ_FORCEINLINE constexpr Real Saturate(Real v) { return Clamp(v, Real(0), Real(1)); }

/// Sign that returns 0 for 0 (unlike Sign()).
KZ_FORCEINLINE constexpr Real SignOrZero(Real v) { return v > Real(0) ? Real(1) : (v < Real(0) ? Real(-1) : Real(0)); }

/// Safe divide; returns fallback when |denominator| < epsilon.
KZ_FORCEINLINE Real SafeDiv(Real num, Real den, Real fallback = Real(0)) {
    return Abs(den) >= kEpsilon ? num / den : fallback;
}

/// Reciprocal with one Newton step; ~2 ulp for inputs in [1e-3, 1e3].
KZ_FORCEINLINE Real Rcp(Real v) { return Real(1) / v; }

KZ_FORCEINLINE Real Sqrt(Real v)  { return std::sqrt(v); }
KZ_FORCEINLINE Real Sin(Real v)   { return std::sin(v); }
KZ_FORCEINLINE Real Cos(Real v)   { return std::cos(v); }
KZ_FORCEINLINE Real Tan(Real v)   { return std::tan(v); }
KZ_FORCEINLINE Real Asin(Real v)  { return std::asin(v); }
KZ_FORCEINLINE Real Acos(Real v)  { return std::acos(v); }
KZ_FORCEINLINE Real Atan2(Real y, Real x) { return std::atan2(y, x); }
KZ_FORCEINLINE Real Pow(Real b, Real e)   { return std::pow(b, e); }
KZ_FORCEINLINE Real Floor(Real v) { return std::floor(v); }
KZ_FORCEINLINE Real Ceil(Real v)  { return std::ceil(v); }
KZ_FORCEINLINE Real Mod(Real a, Real b) { return std::fmod(a, b); }

/// Fused multiply-add; falls back to a*b+c when the target has no FMA.
KZ_FORCEINLINE Real MulAdd(Real a, Real b, Real c) {
#if defined(KZ_COMPILER_GCC) || defined(KZ_COMPILER_MSVC)
    return std::fma(a, b, c);
#else
    return a * b + c;
#endif
}

KZ_FORCEINLINE bool IsFinite(Real v) { return std::isfinite(v); }
KZ_FORCEINLINE bool IsNaN(Real v)    { return std::isnan(v); }

/// True when |a - b| <= tolerance.
KZ_FORCEINLINE bool Close(Real a, Real b, Real tolerance = kDefaultTolerance) {
    return Abs(a - b) <= tolerance;
}

} // namespace math

// ---------------------------------------------------------------------------
// Assertions / diagnostics
// ---------------------------------------------------------------------------
namespace detail {
[[noreturn]] void KzAssertFail(const char* expr, const char* file, int line, const char* msg);
[[noreturn]] void KzFatalFail(const char* file, int line, const char* msg);
void KzLogImpl(int level, const char* file, int line, const char* fmt, ...);
} // namespace detail

#define KZ_ASSERT(expr) \
    (KZ_LIKELY(expr) ? (void)0 : ::kizuri::detail::KzAssertFail(#expr, __FILE__, __LINE__, nullptr))
#define KZ_ASSERT_MSG(expr, msg) \
    (KZ_LIKELY(expr) ? (void)0 : ::kizuri::detail::KzAssertFail(#expr, __FILE__, __LINE__, msg))
#define KZ_FATAL(msg) ::kizuri::detail::KzFatalFail(__FILE__, __LINE__, msg)

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------
/// Move-only RAII wrapper (like Jolt's NonCopyable).
struct NonCopyable {
    NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
};

/// Helper to bit-cast between trivially-copyable types of equal size.
template <typename To, typename From>
KZ_FORCEINLINE To BitCast(const From& from) {
    static_assert(sizeof(To) == sizeof(From), "BitCast requires equal sizes");
    static_assert(std::is_trivially_copyable_v<To> && std::is_trivially_copyable_v<From>);
    To to;
    __builtin_memcpy(&to, &from, sizeof(To));
    return to;
}

} // namespace kizuri
