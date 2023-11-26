// (c) Electronic Arts. All Rights Reserved.
#ifndef WRAPCOMPUTECONTACTS_H
#define WRAPCOMPUTECONTACTS_H

/*************************************************************************************************************

 File: genericcomputecontacts.h

 Purpose:  The functions here simply call the eacollision_primitives CCP fns.

The GenericContactHandler supports the Branching handler API (AddPoint, etc) as well as some extra things.

The "Generic" label can be mapped to
     1. the Branching API
     2. a wrapper around the Branching API
     3. a wrapper around the Branchless API
     4. a EA_FORCE_INLINE wrapper around the Branchless API

You might wonder why would we ever do (2 or 3) instead of (1 or 4), because a wrapper is just adding extra overhead.
The reason is that some compilers (like wii) get "out of memory" error when you try to do too much inlining.  And some
runtime (ps3) crash with "insufficient stack".  Therefore we have to tweak each platform.

*/

#include <rw/collision/common.h>
#include "genericcontacthandler.h"


//---------------------------------------------------------------------------------------------------------------
#if defined EA_PLATFORM_XENON

// Xenon prefers branchless, and inlining a lot.
#include "wrapcomputecontactsinline.h"

#include "eacollision/boxbox_branching.h"
#include "eacollision/boxcapsule_branching.h"

// Hack: Avoid ICE on xenonsdk-2.0.21256.1
#undef EA_FORCE_INLINE
#define EA_FORCE_INLINE inline

#include "eacollision/cylinderbox_branching.h"
#include "eacollision/cylindercapsule_branching.h"
#include "eacollision/cylindercylinder_branching.h"
#include "eacollision/cylindersphere_branching.h"
#include "eacollision/cylindertriangle_branching.h"
#include "eacollision/trianglecapsule_branching.h"
#include "eacollision/triangletriangle_branching.h"

#undef  EA_FORCE_INLINE
#define EA_FORCE_INLINE __forceinline

#define ComputeContactPointsBoxBox_Generic              ::EA::Collision::ComputeContactPointsBoxBox_Branching
#define ComputeContactPointsBoxCapsule_Generic          ::EA::Collision::ComputeContactPointsBoxCapsule_Branching
#define ComputeContactPointsBoxSphere_Generic           ::rw::collision::ComputeContactPointsBoxSphere_BranchlessInlineWrapper
#define ComputeContactPointsCapsuleCapsule_Generic      ::rw::collision::ComputeContactPointsCapsuleCapsule_BranchlessInlineWrapper
#define ComputeContactPointsCapsuleSphere_Generic       ::rw::collision::ComputeContactPointsCapsuleSphere_BranchlessInlineWrapper
#define ComputeContactPointsCylinderBox_Generic         ::EA::Collision::ComputeContactPointsCylinderBox_Branching
#define ComputeContactPointsCylinderCapsule_Generic     ::EA::Collision::ComputeContactPointsCylinderCapsule_Branching
#define ComputeContactPointsCylinderCylinder_Generic    ::EA::Collision::ComputeContactPointsCylinderCylinder_Branching
#define ComputeContactPointsCylinderSphere_Generic      ::EA::Collision::ComputeContactPointsCylinderSphere_Branching
#define ComputeContactPointsCylinderTriangle_Generic    ::EA::Collision::ComputeContactPointsCylinderTriangle_Branching
#define ComputeContactPointsSphereSphere_Generic        ::rw::collision::ComputeContactPointsSphereSphere_BranchlessInlineWrapper
#define ComputeContactPointsTriangleBox_Generic         ::rw::collision::ComputeContactPointsTriangleBox_BranchlessInlineWrapper
#define ComputeContactPointsTriangleCapsule_Generic     ::rw::collision::ComputeContactPointsTriangleCapsule_BranchlessInlineWrapper
#define ComputeContactPointsTriangleSphere_Generic      ::rw::collision::ComputeContactPointsTriangleSphere_BranchlessInlineWrapper
#define ComputeContactPointsTriangleTriangle_Generic    ::EA::Collision::ComputeContactPointsTriangleTriangle_Branching


//---------------------------------------------------------------------------------------------------------------
#elif defined EA_PLATFORM_PS3 || defined EA_PLATFORM_PS3_PPU || defined EA_PLATFORM_PS3_SPU

// PS3 prefers branchless, and no inlining.

#include "eacollision/boxbox_branching.h"
#include "eacollision/boxcapsule_branching.h"
#include "eacollision/boxsphere_branchless.h"
#include "eacollision/capsulecapsule_branchless.h"
#include "eacollision/capsulesphere_branchless.h"
#include "eacollision/cylinderbox_branching.h"
#include "eacollision/cylindercapsule_branching.h"
#include "eacollision/cylindercylinder_branching.h"
#include "eacollision/cylindersphere_branching.h"
#include "eacollision/cylindertriangle_branching.h"
#include "eacollision/spheresphere_branchless.h"
#include "eacollision/trianglebox_branchless.h"
#include "eacollision/trianglecapsule_branching.h"
#include "eacollision/trianglecapsule_branchless.h"
#include "eacollision/trianglesphere_branchless.h"
#include "eacollision/triangletriangle_branching.h"

#define ComputeContactPointsBoxBox_Generic              ::rw::collision::ComputeContactPointsBoxBox_BranchingWrapper
#define ComputeContactPointsBoxCapsule_Generic          ::rw::collision::ComputeContactPointsBoxCapsule_BranchingWrapper
#define ComputeContactPointsBoxSphere_Generic           ::rw::collision::ComputeContactPointsBoxSphere_BranchlessWrapper
#define ComputeContactPointsCapsuleCapsule_Generic      ::rw::collision::ComputeContactPointsCapsuleCapsule_BranchlessWrapper
#define ComputeContactPointsCapsuleSphere_Generic       ::rw::collision::ComputeContactPointsCapsuleSphere_BranchlessWrapper
#define ComputeContactPointsCylinderBox_Generic         ::rw::collision::ComputeContactPointsCylinderBox_BranchingWrapper
#define ComputeContactPointsCylinderCapsule_Generic     ::rw::collision::ComputeContactPointsCylinderCapsule_BranchingWrapper
#define ComputeContactPointsCylinderCylinder_Generic    ::rw::collision::ComputeContactPointsCylinderCylinder_BranchingWrapper
#define ComputeContactPointsCylinderSphere_Generic      ::rw::collision::ComputeContactPointsCylinderSphere_BranchingWrapper
#define ComputeContactPointsCylinderTriangle_Generic    ::rw::collision::ComputeContactPointsCylinderTriangle_BranchingWrapper
#define ComputeContactPointsSphereSphere_Generic        ::rw::collision::ComputeContactPointsSphereSphere_BranchlessWrapper
#define ComputeContactPointsTriangleBox_Generic         ::rw::collision::ComputeContactPointsTriangleBox_BranchlessWrapper
#define ComputeContactPointsTriangleCapsule_Generic     ::rw::collision::ComputeContactPointsTriangleCapsule_BranchingWrapper //Using Branching as Branchless test is unreliable -> requires branching as fallback.
#define ComputeContactPointsTriangleSphere_Generic      ::rw::collision::ComputeContactPointsTriangleSphere_BranchlessWrapper
#define ComputeContactPointsTriangleTriangle_Generic    ::rw::collision::ComputeContactPointsTriangleTriangle_BranchingWrapper


//---------------------------------------------------------------------------------------------------------------
#else   // EA_PLATFORM_WINDOWS or anything else

// PC prefers branching, and a lot of inlining

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/boxbox_branching.h"
#include "eacollision/boxcapsule_branching.h"
#include "eacollision/boxsphere_branching.h"
#include "eacollision/capsulecapsule_branching.h"
#include "eacollision/capsulesphere_branching.h"
#include "eacollision/cylinderbox_branching.h"
#include "eacollision/cylindercapsule_branching.h"
#include "eacollision/cylindercylinder_branching.h"
#include "eacollision/cylindersphere_branching.h"
#include "eacollision/cylindertriangle_branching.h"
#include "eacollision/spheresphere_branching.h"
#include "eacollision/trianglebox_branching.h"
#include "eacollision/trianglecapsule_branching.h"
#include "eacollision/trianglesphere_branching.h"
#include "eacollision/triangletriangle_branching.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif

#define ComputeContactPointsBoxBox_Generic              ::EA::Collision::ComputeContactPointsBoxBox_Branching
#define ComputeContactPointsBoxCapsule_Generic          ::EA::Collision::ComputeContactPointsBoxCapsule_Branching
#define ComputeContactPointsBoxSphere_Generic           ::EA::Collision::ComputeContactPointsBoxSphere_Branching
#define ComputeContactPointsCapsuleCapsule_Generic      ::EA::Collision::ComputeContactPointsCapsuleCapsule_Branching
#define ComputeContactPointsCapsuleSphere_Generic       ::EA::Collision::ComputeContactPointsCapsuleSphere_Branching
#define ComputeContactPointsCylinderBox_Generic         ::EA::Collision::ComputeContactPointsCylinderBox_Branching
#define ComputeContactPointsCylinderCapsule_Generic     ::EA::Collision::ComputeContactPointsCylinderCapsule_Branching
#define ComputeContactPointsCylinderCylinder_Generic    ::EA::Collision::ComputeContactPointsCylinderCylinder_Branching
#define ComputeContactPointsCylinderSphere_Generic      ::EA::Collision::ComputeContactPointsCylinderSphere_Branching
#define ComputeContactPointsCylinderTriangle_Generic    ::EA::Collision::ComputeContactPointsCylinderTriangle_Branching
#define ComputeContactPointsSphereSphere_Generic        ::EA::Collision::ComputeContactPointsSphereSphere_Branching
#define ComputeContactPointsTriangleBox_Generic         ::EA::Collision::ComputeContactPointsTriangleBox_Branching
#define ComputeContactPointsTriangleCapsule_Generic     ::EA::Collision::ComputeContactPointsTriangleCapsule_Branching
#define ComputeContactPointsTriangleSphere_Generic      ::EA::Collision::ComputeContactPointsTriangleSphere_Branching
#define ComputeContactPointsTriangleTriangle_Generic    ::EA::Collision::ComputeContactPointsTriangleTriangle_Branching


#endif



namespace rw
{
namespace collision
{



uint32_t ComputeContactPointsBoxBox_BranchingWrapper(GenericContactHandler& handler,
                                        rwpmath::Vector3::InParam  boxCenterA,
                                        rwpmath::Vector3::InParam  boxUnitAxisA0,
                                        rwpmath::Vector3::InParam  boxUnitAxisA1,
                                        rwpmath::Vector3::InParam  boxUnitAxisA2,
                                        rwpmath::VecFloatInParam   boxHalfLengthA0,
                                        rwpmath::VecFloatInParam   boxHalfLengthA1,
                                        rwpmath::VecFloatInParam   boxHalfLengthA2,
                                        rwpmath::VecFloatInParam   boxRadiusA,
                                        rwpmath::Vector3::InParam  boxCenterB,
                                        rwpmath::Vector3::InParam  boxUnitAxisB0,
                                        rwpmath::Vector3::InParam  boxUnitAxisB1,
                                        rwpmath::Vector3::InParam  boxUnitAxisB2,
                                        rwpmath::VecFloatInParam   boxHalfLengthB0,
                                        rwpmath::VecFloatInParam   boxHalfLengthB1,
                                        rwpmath::VecFloatInParam   boxHalfLengthB2,
                                        rwpmath::VecFloatInParam   boxRadiusB,
                                        rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                        rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                        rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared,
                                        rwpmath::VecFloatInParam   clippingLengthTolerance);

uint32_t ComputeContactPointsBoxCapsule_BranchingWrapper(GenericContactHandler& handler,
                                        rwpmath::Vector3::InParam  boxACenter,
                                        rwpmath::Vector3::InParam  boxAUnitAxis0,
                                        rwpmath::Vector3::InParam  boxAUnitAxis1,
                                        rwpmath::Vector3::InParam  boxAUnitAxis2,
                                        rwpmath::VecFloatInParam   boxAHalfLength0,
                                        rwpmath::VecFloatInParam   boxAHalfLength1,
                                        rwpmath::VecFloatInParam   boxAHalfLength2,
                                        rwpmath::VecFloatInParam   boxARadius,
                                        rwpmath::Vector3::InParam  capsuleBCenter,
                                        rwpmath::Vector3::InParam  capsuleBUnitAxis,
                                        rwpmath::VecFloatInParam   capsuleBHalfLength,
                                        rwpmath::VecFloatInParam   capsuleBRadius,
                                        rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                        rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                        rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsCylinderBox_BranchingWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  cylinderCenter,
                                            rwpmath::Vector3::InParam  cylinderUnitAxis,
                                            rwpmath::VecFloatInParam   cylinderHalfLength,
                                            rwpmath::VecFloatInParam   cylinderInnerRadius,
                                            rwpmath::VecFloatInParam   cylinderOuterRadius,
                                            rwpmath::Vector3::InParam  boxCenter,
                                            rwpmath::Vector3::InParam  boxUnitAxis0,
                                            rwpmath::Vector3::InParam  boxUnitAxis1,
                                            rwpmath::Vector3::InParam  boxUnitAxis2,
                                            rwpmath::VecFloatInParam   boxHalfLength0,
                                            rwpmath::VecFloatInParam   boxHalfLength1,
                                            rwpmath::VecFloatInParam   boxHalfLength2,
                                            rwpmath::VecFloatInParam   boxRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared,
                                            rwpmath::VecFloatInParam   clippingLengthTolerance);

uint32_t ComputeContactPointsCylinderCapsule_BranchingWrapper(GenericContactHandler& handler,
                                                 rwpmath::Vector3::InParam  cylinderCenter,
                                                 rwpmath::Vector3::InParam  cylinderUnitAxis,
                                                 rwpmath::VecFloatInParam   cylinderHalfLength,
                                                 rwpmath::VecFloatInParam   cylinderInnerRadius,
                                                 rwpmath::VecFloatInParam   cylinderOuterRadius,
                                                 rwpmath::Vector3::InParam  capsuleCenter,
                                                 rwpmath::Vector3::InParam  capsuleUnitAxis,
                                                 rwpmath::VecFloatInParam   capsuleHalfLength,
                                                 rwpmath::VecFloatInParam   capsuleRadius,
                                                 rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                                 rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                                 rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsCylinderCylinder_BranchingWrapper(GenericContactHandler& handler,
                                                  rwpmath::Vector3::InParam  cylinderACenter,
                                                  rwpmath::Vector3::InParam  cylinderAUnitAxis,
                                                  rwpmath::VecFloatInParam   cylinderAHalfLength,
                                                  rwpmath::VecFloatInParam   cylinderAInnerRadius,
                                                  rwpmath::VecFloatInParam   cylinderAOuterRadius,
                                                  rwpmath::Vector3::InParam  cylinderBCenter,
                                                  rwpmath::Vector3::InParam  cylinderBUnitAxis,
                                                  rwpmath::VecFloatInParam   cylinderBHalfLength,
                                                  rwpmath::VecFloatInParam   cylinderBInnerRadius,
                                                  rwpmath::VecFloatInParam   cylinderBOuterRadius,
                                                  rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                                  rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                                  rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared,
                                                  rwpmath::VecFloatInParam   clippingLengthTolerance);

uint32_t ComputeContactPointsCylinderSphere_BranchingWrapper(GenericContactHandler& handler,
                                                rwpmath::Vector3::InParam  cylinderCenter,
                                                rwpmath::Vector3::InParam  cylinderUnitAxis,
                                                rwpmath::VecFloatInParam   cylinderHalfLength,
                                                rwpmath::VecFloatInParam   cylinderInnerRadius,
                                                rwpmath::VecFloatInParam   cylinderOuterRadius,
                                                rwpmath::Vector3::InParam  sphereCenter,
                                                rwpmath::VecFloatInParam   sphereRadius,
                                                rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                                rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsCylinderTriangle_BranchingWrapper(GenericContactHandler& handler,
                                                  rwpmath::Vector3::InParam  cylinderCenter,
                                                  rwpmath::Vector3::InParam  cylinderUnitAxis,
                                                  rwpmath::VecFloatInParam   cylinderHalfLength,
                                                  rwpmath::VecFloatInParam   cylinderInnerRadius,
                                                  rwpmath::VecFloatInParam   cylinderOuterRadius,
                                                  rwpmath::Vector3::InParam  triangleVertex0,
                                                  rwpmath::Vector3::InParam  triangleVertex1,
                                                  rwpmath::Vector3::InParam  triangleVertex2,
                                                  rwpmath::VecFloatInParam   triangleRadius,
                                                  rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                                  rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                                  rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared,
                                                  rwpmath::VecFloatInParam   clippingLengthTolerance);

uint32_t ComputeContactPointsTriangleTriangle_BranchingWrapper(GenericContactHandler& handler,
                                                  rwpmath::Vector3::InParam  triangleAVertex0,
                                                  rwpmath::Vector3::InParam  triangleAVertex1,
                                                  rwpmath::Vector3::InParam  triangleAVertex2,
                                                  rwpmath::VecFloatInParam   triangleARadius,
                                                  rwpmath::Vector3::InParam  triangleBVertex0,
                                                  rwpmath::Vector3::InParam  triangleBVertex1,
                                                  rwpmath::Vector3::InParam  triangleBVertex2,
                                                  rwpmath::VecFloatInParam   triangleBRadius,
                                                  rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                                  rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                                  rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared,
                                                  rwpmath::VecFloatInParam   clippingLengthTolerance);

uint32_t ComputeContactPointsBoxSphere_BranchingWrapper(GenericContactHandler& handler,
                                      rwpmath::Vector3::InParam  boxCenter,
                                      rwpmath::Vector3::InParam  boxUnitAxis0,
                                      rwpmath::Vector3::InParam  boxUnitAxis1,
                                      rwpmath::Vector3::InParam  boxUnitAxis2,
                                      rwpmath::VecFloatInParam   boxHalfLength0,
                                      rwpmath::VecFloatInParam   boxHalfLength1,
                                      rwpmath::VecFloatInParam   boxHalfLength2,
                                      rwpmath::VecFloatInParam   boxRadius,
                                      rwpmath::Vector3::InParam  sphereCenter,
                                      rwpmath::VecFloatInParam   sphereRadius,
                                      rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                      rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsCapsuleCapsule_BranchingWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  capsuleACenter,
                                            rwpmath::Vector3::InParam  capsuleAUnitAxis,
                                            rwpmath::VecFloatInParam   capsuleAHalfLength,
                                            rwpmath::VecFloatInParam   capsuleARadius,
                                            rwpmath::Vector3::InParam  capsuleBCenter,
                                            rwpmath::Vector3::InParam  capsuleBUnitAxis,
                                            rwpmath::VecFloatInParam   capsuleBHalfLength,
                                            rwpmath::VecFloatInParam   capsuleBRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsCapsuleSphere_BranchingWrapper(GenericContactHandler& handler,
                                          rwpmath::Vector3::InParam  capsuleCenter,
                                          rwpmath::Vector3::InParam  capsuleUnitAxis,
                                          rwpmath::VecFloatInParam   capsuleHalfLength,
                                          rwpmath::VecFloatInParam   capsuleRadius,
                                          rwpmath::Vector3::InParam  sphereCenter,
                                          rwpmath::VecFloatInParam   sphereRadius,
                                          rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                          rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsSphereSphere_BranchingWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  sphereACenter,
                                            rwpmath::VecFloatInParam   sphereARadius,
                                            rwpmath::Vector3::InParam  sphereBCenter,
                                            rwpmath::VecFloatInParam   sphereBRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsTriangleBox_BranchingWrapper(GenericContactHandler& handler,
                                        rwpmath::Vector3::InParam  triangleVertex0,
                                        rwpmath::Vector3::InParam  triangleVertex1,
                                        rwpmath::Vector3::InParam  triangleVertex2,
                                        rwpmath::VecFloatInParam   triangleRadius,
                                        rwpmath::Vector3::InParam  boxCenter,
                                        rwpmath::Vector3::InParam  boxUnitAxis0,
                                        rwpmath::Vector3::InParam  boxUnitAxis1,
                                        rwpmath::Vector3::InParam  boxUnitAxis2,
                                        rwpmath::VecFloatInParam   boxHalfLength0,
                                        rwpmath::VecFloatInParam   boxHalfLength1,
                                        rwpmath::VecFloatInParam   boxHalfLength2,
                                        rwpmath::VecFloatInParam   boxRadius,
                                        rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                        rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                        rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared,
                                        rwpmath::VecFloatInParam   clippingLengthTolerance);

uint32_t ComputeContactPointsTriangleCapsule_BranchingWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  triangleVertex0,
                                            rwpmath::Vector3::InParam  triangleVertex1,
                                            rwpmath::Vector3::InParam  triangleVertex2,
                                            rwpmath::VecFloatInParam   triangleRadius,
                                            rwpmath::Vector3::InParam  capsuleCenter,
                                            rwpmath::Vector3::InParam  capsuleUnitAxis,
                                            rwpmath::VecFloatInParam   capsuleHalfLength,
                                            rwpmath::VecFloatInParam   capsuleRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsTriangleSphere_BranchingWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  triangleVertex0,
                                            rwpmath::Vector3::InParam  triangleVertex1,
                                            rwpmath::Vector3::InParam  triangleVertex2,
                                            rwpmath::VecFloatInParam   triangleRadius,
                                            rwpmath::Vector3::InParam  sphereCenter,
                                            rwpmath::VecFloatInParam   sphereRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);
//-----------------
//  Branchless

uint32_t ComputeContactPointsBoxSphere_BranchlessWrapper(GenericContactHandler& handler,
                                      rwpmath::Vector3::InParam  boxCenter,
                                      rwpmath::Vector3::InParam  boxUnitAxis0,
                                      rwpmath::Vector3::InParam  boxUnitAxis1,
                                      rwpmath::Vector3::InParam  boxUnitAxis2,
                                      rwpmath::VecFloatInParam   boxHalfLength0,
                                      rwpmath::VecFloatInParam   boxHalfLength1,
                                      rwpmath::VecFloatInParam   boxHalfLength2,
                                      rwpmath::VecFloatInParam   boxRadius,
                                      rwpmath::Vector3::InParam  sphereCenter,
                                      rwpmath::VecFloatInParam   sphereRadius,
                                      rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                      rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsCapsuleCapsule_BranchlessWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  capsuleACenter,
                                            rwpmath::Vector3::InParam  capsuleAUnitAxis,
                                            rwpmath::VecFloatInParam   capsuleAHalfLength,
                                            rwpmath::VecFloatInParam   capsuleARadius,
                                            rwpmath::Vector3::InParam  capsuleBCenter,
                                            rwpmath::Vector3::InParam  capsuleBUnitAxis,
                                            rwpmath::VecFloatInParam   capsuleBHalfLength,
                                            rwpmath::VecFloatInParam   capsuleBRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsCapsuleSphere_BranchlessWrapper(GenericContactHandler& handler,
                                          rwpmath::Vector3::InParam  capsuleCenter,
                                          rwpmath::Vector3::InParam  capsuleUnitAxis,
                                          rwpmath::VecFloatInParam   capsuleHalfLength,
                                          rwpmath::VecFloatInParam   capsuleRadius,
                                          rwpmath::Vector3::InParam  sphereCenter,
                                          rwpmath::VecFloatInParam   sphereRadius,
                                          rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                          rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsSphereSphere_BranchlessWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  sphereACenter,
                                            rwpmath::VecFloatInParam   sphereARadius,
                                            rwpmath::Vector3::InParam  sphereBCenter,
                                            rwpmath::VecFloatInParam   sphereBRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsTriangleBox_BranchlessWrapper(GenericContactHandler& handler,
                                        rwpmath::Vector3::InParam  triangleVertex0,
                                        rwpmath::Vector3::InParam  triangleVertex1,
                                        rwpmath::Vector3::InParam  triangleVertex2,
                                        rwpmath::VecFloatInParam   triangleRadius,
                                        rwpmath::Vector3::InParam  boxCenter,
                                        rwpmath::Vector3::InParam  boxUnitAxis0,
                                        rwpmath::Vector3::InParam  boxUnitAxis1,
                                        rwpmath::Vector3::InParam  boxUnitAxis2,
                                        rwpmath::VecFloatInParam   boxHalfLength0,
                                        rwpmath::VecFloatInParam   boxHalfLength1,
                                        rwpmath::VecFloatInParam   boxHalfLength2,
                                        rwpmath::VecFloatInParam   boxRadius,
                                        rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                        rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                        rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared,
                                        rwpmath::VecFloatInParam   clippingLengthTolerance);

uint32_t ComputeContactPointsTriangleCapsule_BranchlessWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  triangleVertex0,
                                            rwpmath::Vector3::InParam  triangleVertex1,
                                            rwpmath::Vector3::InParam  triangleVertex2,
                                            rwpmath::VecFloatInParam   triangleRadius,
                                            rwpmath::Vector3::InParam  capsuleCenter,
                                            rwpmath::Vector3::InParam  capsuleUnitAxis,
                                            rwpmath::VecFloatInParam   capsuleHalfLength,
                                            rwpmath::VecFloatInParam   capsuleRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   cosSquaredMaximumAngleConsideredParallel,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

uint32_t ComputeContactPointsTriangleSphere_BranchlessWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  triangleVertex0,
                                            rwpmath::Vector3::InParam  triangleVertex1,
                                            rwpmath::Vector3::InParam  triangleVertex2,
                                            rwpmath::VecFloatInParam   triangleRadius,
                                            rwpmath::Vector3::InParam  sphereCenter,
                                            rwpmath::VecFloatInParam   sphereRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared);

} // collision
} // rw


#endif // WRAPCOMPUTECONTACTS_H
