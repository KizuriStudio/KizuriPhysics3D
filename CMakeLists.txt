cmake_minimum_required(VERSION 3.20)
project(KizuriPhysics
    VERSION 0.1.0
    DESCRIPTION "KizuriPhysics - high performance 3D rigid body physics engine"
    LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(KZ_BUILD_TESTS     "Build unit tests"           ON)
option(KZ_BUILD_EXAMPLES  "Build examples"             ON)
option(KZ_BUILD_SAMPLES   "Build interactive samples"  OFF)
option(KZ_ENABLE_SSE      "Enable SSE4.2 SIMD paths"   ON)
option(KZ_ENABLE_AVX2     "Enable AVX2 SIMD paths"     OFF)
option(KZ_DOUBLE_PRECISION "Use double precision"      OFF)
option(KZ_DETERMINISTIC   "Bit-exact determinism mode" OFF)
option(KZ_ENABLE_ASAN     "Enable AddressSanitizer"    OFF)
option(KZ_ENABLE_TSAN     "Enable ThreadSanitizer"     OFF)
option(KZ_ENABLE_PROFILING "Enable built-in profiler"  ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Warnings
# ---------------------------------------------------------------------------
if(MSVC)
    add_compile_options(/W4 /permissive- /Zc:__cplusplus /Zc:preprocessor)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
endif()

# ---------------------------------------------------------------------------
# SIMD flags
# ---------------------------------------------------------------------------
if(KZ_ENABLE_SSE AND (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i686"))
    if(MSVC)
        # SSE2 is baseline on x64
    else()
        add_compile_options(-msse4.2)
    endif()
    add_compile_definitions(KZ_USE_SSE=1)
endif()

if(KZ_ENABLE_AVX2 AND (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64"))
    if(NOT MSVC)
        add_compile_options(-mavx2 -mfma)
    else()
        add_compile_options(/arch:AVX2)
    endif()
    add_compile_definitions(KZ_USE_AVX2=1)
endif()

if(KZ_DOUBLE_PRECISION)
    add_compile_definitions(KZ_DOUBLE_PRECISION=1)
endif()
if(KZ_DETERMINISTIC)
    add_compile_definitions(KZ_DETERMINISTIC=1)
endif()

if(KZ_ENABLE_ASAN AND NOT MSVC)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()
if(KZ_ENABLE_TSAN AND NOT MSVC)
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
file(GLOB_RECURSE KZ_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(KizuriPhysics STATIC ${KZ_SOURCES})
add_library(Kizuri::Physics ALIAS KizuriPhysics)

target_include_directories(KizuriPhysics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

find_package(Threads REQUIRED)
target_link_libraries(KizuriPhysics PUBLIC Threads::Threads)

if(KZ_ENABLE_PROFILING)
    target_compile_definitions(KizuriPhysics PUBLIC KZ_ENABLE_PROFILER=1)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(KZ_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
if(KZ_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS KizuriPhysics
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
