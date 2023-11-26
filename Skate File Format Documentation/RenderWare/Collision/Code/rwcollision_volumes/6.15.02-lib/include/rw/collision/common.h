// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_COMMON_H
#define RWCOLLISION_COMMON_H

// This header file should be included first by all public rw/collision headers to include
// the standard set of core features in a controlled manner as well as defining some
// standard macros for use through the package.

#include "eaphysics/base.h"             // for EABase, EAAssert, serialization, 
#include "eaphysics/rwpmath.h"          // rwpmath namespace
#include "eaphysics/sizeandalignment.h" // EA::Physics::SizeAndAlignment etc
#include "eaphysics/message.h"          // EAPHYSICS_MESSAGE & EAPHYSICS_WARNING etc

#if defined _MSC_VER
#pragma warning(push, 0)            // ignore all warnings from MS headers
#elif defined __SNC__
#pragma diag_suppress=229,341,1363
#elif defined __CWCC__
#pragma warning off (10107, 10411)
#endif
#include <new> // for placement new
#if defined _MSC_VER
#pragma warning(pop)
#elif defined __SNC__
#pragma diag_default=229,341,1363
#elif defined __CWCC__
#pragma warning reset (10107, 10411)
#endif

// Check for a common cause of build problems
#if defined(RWCROSS)
#error rwcollision_volumes does not support cross builds
#endif

/// Assert that an address is aligned
#define rwcASSERTALIGN(ADDR, ALIGN) EA_ASSERT(EA::Physics::IsMemAligned(ADDR, ALIGN))

// Deprecated simulation functions should be in deprecatedmethods.h and use rwcDEPRECATED
// rwcDEPRECATED is used to print out a single message for each deprecated function
// that is called in debug builds.
// Usage: rwcDEPRECATED("Please use ...");
// On debug builds rwcDEPRECATED will print out the message through rwMESSAGE.
// On release builds rwcDEPRECATED will do nothing.
#if defined EA_DEBUG

#define rwcDEPRECATED(MSG)                                                                  \
    static bool deprecatedMessageShown = false;                                             \
    if (!deprecatedMessageShown)                                                            \
    {                                                                                       \
        EAPHYSICS_MESSAGE("%s(%u) : deprecated: %s", __FILE__, __LINE__, EA_CURRENT_FUNCTION);   \
        EAPHYSICS_MESSAGE(MSG);                                                                  \
        deprecatedMessageShown = true;                                                      \
    }

#else

#define rwcDEPRECATED(MSG)

#endif

/// For legacy reasons, we use a 32-bit integer for boolean reasons.
typedef uint32_t RwpBool;

#ifndef FALSE
#define FALSE   0
#endif

#ifndef TRUE
#define TRUE    (!FALSE)
#endif

///////////////////////////////////////////////////////////////////////////
// Control over inlining
///////////////////////////////////////////////////////////////////////////

#define RW_COLLISION_INLINE inline

// First check to see if the user has specified that force inlining should be
// enabled or disabled explicitly.
#if defined(RW_COLLISION_ENABLE_FORCE_INLINING) || defined(RW_COLLISION_DISABLE_FORCE_INLINING)

#if defined(RW_COLLISION_ENABLE_FORCE_INLINING) && defined(RW_COLLISION_DISABLE_FORCE_INLINING)
#error The RW_COLLISION_ENABLE_FORCE_INLINING and RW_COLLISION_DISABLE_FORCE_INLINING macros are mutually exclusive.
#endif

#ifdef RW_COLLISION_ENABLE_FORCE_INLINING
#define RW_COLLISION_DETAIL_FORCE_INLINING 1
#else
#define RW_COLLISION_DETAIL_FORCE_INLINING 0
#endif

#else

// The user has not explicitly specified that force inlining should be
// enabled or disabled.  So, enable/disable force inlining based on
// the build configuration.
#if defined EA_DEBUG
#define RW_COLLISION_DETAIL_FORCE_INLINING 0
#else
#define RW_COLLISION_DETAIL_FORCE_INLINING 1
#endif

#endif

#if !RW_COLLISION_DETAIL_FORCE_INLINING
#define RW_COLLISION_FORCE_INLINE inline
#else
#if defined _MSC_VER
#define RW_COLLISION_FORCE_INLINE __forceinline
#elif (__GNUC__ >= 4)
#define RW_COLLISION_FORCE_INLINE inline __attribute__ ((always_inline))
#else
#define RW_COLLISION_FORCE_INLINE inline
#endif
#endif

// uncomment this to enable vectorization optimizations of the KdTreeLineQueryBase::ProcessBranchNode
// this has shown to provide little speed-up over the non-vectorized code at an increased cost to readability
// #define RWC_KDTREELINEQUERYBASE_OPT 1

//Include the custom math functions
#include "mathutils.h"

#endif // RWCOLLISION_COMMON_H
