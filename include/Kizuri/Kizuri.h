// KizuriPhysics - Kizuri.h
// Umbrella header: include everything you need.
#pragma once

// Core
#include "Kizuri/Core/Config.h"
#include "Kizuri/Core/Types.h"
#include "Kizuri/Core/Math.h"
#include "Kizuri/Core/Memory.h"
#include "Kizuri/Core/JobSystem.h"

// Geometry
#include "Kizuri/Geometry/Shape.h"

// Collision
#include "Kizuri/Collision/GJK.h"
#include "Kizuri/Collision/EPA.h"
#include "Kizuri/Collision/ContactManifold.h"

// Broad phase
#include "Kizuri/BroadPhase/BroadPhase.h"

// Bodies
#include "Kizuri/Body/Body.h"
#include "Kizuri/Body/BodyManager.h"

// Constraints
#include "Kizuri/Constraint/Constraints.h"

// Solver
#include "Kizuri/Solver/ContactSolver.h"

// World
#include "Kizuri/World/PhysicsWorld.h"

#define KIZURI_PHYSICS_VERSION_MAJOR 0
#define KIZURI_PHYSICS_VERSION_MINOR 1
#define KIZURI_PHYSICS_VERSION_PATCH 0
#define KIZURI_PHYSICS_VERSION_STRING "0.1.0"
