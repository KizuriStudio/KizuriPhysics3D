// KizuriPhysics - Core/Config.h
// Compile-time configuration and platform detection.
#pragma once

// ---------------------------------------------------------------------------
// Platform / compiler detection
// ---------------------------------------------------------------------------
#if defined(_WIN32) || defined(_WIN64)
    #define KZ_PLATFORM_WINDOWS 1
#elif defined(__APPLE__)
    #define KZ_PLATFORM_MACOS 1
#elif defined(__linux__)
    #define KZ_PLATFORM_LINUX 1
#elif defined(__EMSCRIPTEN__)
    #define KZ_PLATFORM_WASM 1
#endif

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #define KZ_ARCH_X86 1
#elif defined(__aarch64__) || defined(_M_ARM64)
    #define KZ_ARCH_ARM64 1
#endif

#if defined(_MSC_VER)
    #define KZ_COMPILER_MSVC 1
    #define KZ_FORCEINLINE __forceinline
    #define KZ_RESTRICT __restrict
#elif defined(__GNUC__) || defined(__clang__)
    #define KZ_COMPILER_GCC 1
    #define KZ_FORCEINLINE inline __attribute__((always_inline))
    #define KZ_RESTRICT __restrict__
#else
    #define KZ_FORCEINLINE inline
    #define KZ_RESTRICT
#endif

#define KZ_NODISCARD [[nodiscard]]
#define KZ_MAYBE_UNUSED [[maybe_unused]]
#define KZ_LIKELY(x)   (__builtin_expect(!!(x), 1))
#define KZ_UNLIKELY(x) (__builtin_expect(!!(x), 0))

#define KZ_UNUSED(x) (void)(x)

// ---------------------------------------------------------------------------
// SIMD
// ---------------------------------------------------------------------------
// KZ_USE_SSE: 4-wide float SIMD via <immintrin.h>. Enabled by default on x86.
// KZ_USE_AVX2: 8-wide float SIMD. Requires -mavx2.
#if !defined(KZ_USE_SSE)
    #if defined(KZ_ARCH_X86)
        #define KZ_USE_SSE 1
    #else
        #define KZ_USE_SSE 0
    #endif
#endif

// ---------------------------------------------------------------------------
// Precision
// ---------------------------------------------------------------------------
// KZ_DOUBLE_PRECISION: use double for the simulation scalar.
// Float is ~2x faster and matches Jolt; double is for large-world / scientific.
#if !defined(KZ_DOUBLE_PRECISION)
    #define KZ_DOUBLE_PRECISION 0
#endif

// ---------------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------------
// KZ_DETERMINISTIC: forbids floating point reductions whose order depends on
// thread scheduling and disables SIMD reductions that are not bit-exact.
// When set, the engine guarantees identical results for identical inputs and
// identical thread counts. Use for lockstep networking.
#if !defined(KZ_DETERMINISTIC)
    #define KZ_DETERMINISTIC 0
#endif

// ---------------------------------------------------------------------------
// Bounds / limits
// ---------------------------------------------------------------------------
namespace kizuri::config {

/// Maximum number of bodies in a single PhysicsWorld. BodyID stores the index
/// in 26 bits (with a 6-bit sequence for slot-reuse validation), so this must
/// not exceed 2^26 = 67,108,864.
inline constexpr unsigned kMaxBodies = 1u << 22; // 4,194,304

/// Maximum number of body pairs the contact manager will track per step.
inline constexpr unsigned kMaxBodyPairs = 1u << 22;

/// Maximum number of contact constraints generated per step.
inline constexpr unsigned kMaxContactConstraints = 1u << 23;

/// Number of layers in the broadphase. Bodies are assigned to a layer via
/// ObjectLayer; layers do not interact across the ObjectLayerPairFilter.
inline constexpr unsigned kMaxObjectLayers = 32;

/// Maximum number of bodies in a single island before it is split.
inline constexpr unsigned kMaxBodiesPerIsland = 1024;

} // namespace kizuri::config
