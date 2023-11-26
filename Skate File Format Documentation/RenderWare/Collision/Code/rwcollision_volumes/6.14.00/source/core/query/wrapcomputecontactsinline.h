// (c) Electronic Arts. All Rights Reserved.
#ifndef WRAPCOMPUTECONTACTSINLINE_H
#define WRAPCOMPUTECONTACTSINLINE_H

/*************************************************************************************************************

 File: wrapcomputecontactsinline.h

 Purpose: wrappers for the branchless CCP function, which are force-inlined

*/

#include <rw/collision/common.h>
#include "genericcontacthandler.h"


/*
*************************************************************************************************************
    Branchless inline wrappers
*************************************************************************************************************
*/

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/boxsphere_branchless.h"
#include "eacollision/capsulecapsule_branchless.h"
#include "eacollision/capsulesphere_branchless.h"
#include "eacollision/spheresphere_branchless.h"
#include "eacollision/trianglebox_branchless.h"
#include "eacollision/trianglecapsule_branchless.h"
#include "eacollision/trianglecapsule_branching.h"
#include "eacollision/trianglesphere_branchless.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif

/*
    To avoid silly warnings on PS3
*/
#if defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_PPU) || defined(EA_PLATFORM_PS3_SPU)
#define INIT_ZERO (0u)
#else
#define INIT_ZERO
#endif



namespace rw
{
namespace collision
{

/*************************************************************************************************************
    BOX SPHERE
*/

EA_FORCE_INLINE uint32_t
ComputeContactPointsBoxSphere_BranchlessInlineWrapper(GenericContactHandler& handler,
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
                                      rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection             INIT_ZERO;
    rwpmath::Vector3 contact0_Sphere                  INIT_ZERO;
    rwpmath::Vector3 contact0_Box                     INIT_ZERO;
    rwpmath::MaskScalar contact0_Returned             INIT_ZERO;

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsBoxSphere_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_Box, contact0_Sphere,
                                boxCenter, boxUnitAxis0,  boxUnitAxis1, boxUnitAxis2,
                                boxHalfLength0, boxHalfLength1, boxHalfLength2, boxRadius,
                                sphereCenter, sphereRadius,
                                minimumSeparatingDistance,
                                validDirectionMinimumLengthSquared);

    if (math::And(ok, contact0_Returned).GetBool())
    {
        handler.BeginContactQuick(contactUnitDirection);
        handler.AddPointQuick(contact0_Box, contact0_Sphere, contact0_Returned);
        return 1u;
    }
    return 0u;
}

/*************************************************************************************************************
    CAPSULE CAPSULE
*/
EA_FORCE_INLINE uint32_t
ComputeContactPointsCapsuleCapsule_BranchlessInlineWrapper(GenericContactHandler& handler,
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
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection             INIT_ZERO;
    rwpmath::Vector3 contact0_CapsuleB                INIT_ZERO;
    rwpmath::Vector3 contact1_CapsuleB                INIT_ZERO;
    rwpmath::Vector3 contact0_CapsuleA                INIT_ZERO;
    rwpmath::Vector3 contact1_CapsuleA                INIT_ZERO;
    rwpmath::MaskScalar contact0_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact1_Returned             INIT_ZERO;

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsCapsuleCapsule_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_CapsuleA, contact0_CapsuleB,
                                contact1_Returned, contact1_CapsuleA, contact1_CapsuleB,
                                capsuleACenter, capsuleAUnitAxis, capsuleAHalfLength, capsuleARadius,
                                capsuleBCenter, capsuleBUnitAxis, capsuleBHalfLength, capsuleBRadius,
                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                validDirectionMinimumLengthSquared);

    if (rwpmath::And(ok, rwpmath::Or(contact0_Returned,contact1_Returned)).GetBool())
    {
        handler.BeginContactQuick(contactUnitDirection);
        handler.AddPointConditional(contactUnitDirection, contact0_CapsuleA, contact0_CapsuleB, contact0_Returned);
        handler.AddPointConditional(contactUnitDirection, contact1_CapsuleA, contact1_CapsuleB, contact1_Returned);
        return 1u;
    }
    return 0u;
}

/*************************************************************************************************************
    CAPSULE SPHERE
*/
EA_FORCE_INLINE uint32_t
ComputeContactPointsCapsuleSphere_BranchlessInlineWrapper(GenericContactHandler& handler,
                                          rwpmath::Vector3::InParam  capsuleCenter,
                                          rwpmath::Vector3::InParam  capsuleUnitAxis,
                                          rwpmath::VecFloatInParam   capsuleHalfLength,
                                          rwpmath::VecFloatInParam   capsuleRadius,
                                          rwpmath::Vector3::InParam  sphereCenter,
                                          rwpmath::VecFloatInParam   sphereRadius,
                                          rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                          rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection             INIT_ZERO;
    rwpmath::Vector3 contact0_Sphere                  INIT_ZERO;
    rwpmath::Vector3 contact0_Capsule                 INIT_ZERO;
    rwpmath::MaskScalar contact0_Returned             INIT_ZERO;

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsCapsuleSphere_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_Capsule, contact0_Sphere,
                                capsuleCenter, capsuleUnitAxis, capsuleHalfLength, capsuleRadius,
                                sphereCenter, sphereRadius,
                                minimumSeparatingDistance,
                                validDirectionMinimumLengthSquared);

    if (rwpmath::And(ok, contact0_Returned).GetBool())
    {
        handler.BeginContactQuick(contactUnitDirection);
        handler.AddPointConditional(contactUnitDirection, contact0_Capsule, contact0_Sphere, contact0_Returned);
        return 1u;
    }
    return 0u;
}

/*************************************************************************************************************
    SPHERE SPHERE
*/
EA_FORCE_INLINE uint32_t
ComputeContactPointsSphereSphere_BranchlessInlineWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  sphereACenter,
                                            rwpmath::VecFloatInParam   sphereARadius,
                                            rwpmath::Vector3::InParam  sphereBCenter,
                                            rwpmath::VecFloatInParam   sphereBRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection             INIT_ZERO;
    rwpmath::Vector3 contact0_SphereA                 INIT_ZERO;
    rwpmath::Vector3 contact0_SphereB                 INIT_ZERO;
    rwpmath::MaskScalar contact0_Returned             INIT_ZERO;

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsSphereSphere_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_SphereA, contact0_SphereB,
                                sphereACenter, sphereARadius,
                                sphereBCenter, sphereBRadius,
                                minimumSeparatingDistance,
                                validDirectionMinimumLengthSquared);

    if (rwpmath::And(ok, contact0_Returned).GetBool())
    {
        handler.BeginContactQuick(contactUnitDirection);
        handler.AddPointQuick(contact0_SphereA, contact0_SphereB, contact0_Returned);
        return 1u;
    }
    return 0u;
}


/*************************************************************************************************************
    TRIANGLE BOX
*/
EA_FORCE_INLINE uint32_t
ComputeContactPointsTriangleBox_BranchlessInlineWrapper(GenericContactHandler& handler,
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
                                        rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    rwpmath::Vector3 contactUnitDirection             INIT_ZERO;
    rwpmath::Vector3 contact0_Box                     INIT_ZERO;
    rwpmath::Vector3 contact1_Box                     INIT_ZERO;
    rwpmath::Vector3 contact2_Box                     INIT_ZERO;
    rwpmath::Vector3 contact3_Box                     INIT_ZERO;
    rwpmath::Vector3 contact4_Box                     INIT_ZERO;
    rwpmath::Vector3 contact5_Box                     INIT_ZERO;
    rwpmath::Vector3 contact6_Box                     INIT_ZERO;
    rwpmath::Vector3 contact0_Triangle                INIT_ZERO;
    rwpmath::Vector3 contact1_Triangle                INIT_ZERO;
    rwpmath::Vector3 contact2_Triangle                INIT_ZERO;
    rwpmath::Vector3 contact3_Triangle                INIT_ZERO;
    rwpmath::Vector3 contact4_Triangle                INIT_ZERO;
    rwpmath::Vector3 contact5_Triangle                INIT_ZERO;
    rwpmath::Vector3 contact6_Triangle                INIT_ZERO;
    rwpmath::MaskScalar contact0_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact1_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact2_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact3_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact4_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact5_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact6_Returned             INIT_ZERO;

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsTriangleBox_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_Triangle, contact0_Box,
                                contact1_Returned, contact1_Triangle, contact1_Box,
                                contact2_Returned, contact2_Triangle, contact2_Box,
                                contact3_Returned, contact3_Triangle, contact3_Box,
                                contact4_Returned, contact4_Triangle, contact4_Box,
                                contact5_Returned, contact5_Triangle, contact5_Box,
                                contact6_Returned, contact6_Triangle, contact6_Box,
                                triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
                                boxCenter, boxUnitAxis0,  boxUnitAxis1, boxUnitAxis2,
                                boxHalfLength0, boxHalfLength1, boxHalfLength2, boxRadius,
                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                validDirectionMinimumLengthSquared, clippingLengthTolerance);

    rwpmath::MaskScalar anyHit = 
        rwpmath::Or(rwpmath::Or(rwpmath::Or(contact0_Returned,contact1_Returned),
                                rwpmath::Or(contact2_Returned,contact3_Returned)),
                    rwpmath::Or(rwpmath::Or(contact4_Returned,contact5_Returned), 
                                contact6_Returned));
    if (rwpmath::And(ok, anyHit).GetBool())
    {
        if (handler.BeginContact(contactUnitDirection))
        {
            handler.AddPointConditional(contactUnitDirection, contact0_Triangle, contact0_Box, contact0_Returned);
            handler.AddPointConditional(contactUnitDirection, contact1_Triangle, contact1_Box, contact1_Returned);
            handler.AddPointConditional(contactUnitDirection, contact2_Triangle, contact2_Box, contact2_Returned);
            handler.AddPointConditional(contactUnitDirection, contact3_Triangle, contact3_Box, contact3_Returned);
            handler.AddPointConditional(contactUnitDirection, contact4_Triangle, contact4_Box, contact4_Returned);
            handler.AddPointConditional(contactUnitDirection, contact5_Triangle, contact5_Box, contact5_Returned);
            handler.AddPointConditional(contactUnitDirection, contact6_Triangle, contact6_Box, contact6_Returned);
            handler.EndContact();
        }
        return 1u;
    }
    return 0u;
}


/*************************************************************************************************************
    TRIANGLE CAPSULE
*/
EA_FORCE_INLINE uint32_t
ComputeContactPointsTriangleCapsule_BranchlessInlineWrapper(GenericContactHandler& handler,
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
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection             INIT_ZERO;
    rwpmath::Vector3 contact0_Capsule                 INIT_ZERO;
    rwpmath::Vector3 contact1_Capsule                 INIT_ZERO;
    rwpmath::Vector3 contact0_Triangle                INIT_ZERO;
    rwpmath::Vector3 contact1_Triangle                INIT_ZERO;
    rwpmath::MaskScalar contact0_Returned             INIT_ZERO;
    rwpmath::MaskScalar contact1_Returned             INIT_ZERO;

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsTriangleCapsule_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_Triangle, contact0_Capsule,
                                contact1_Returned, contact1_Triangle, contact1_Capsule,
                                triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
                                capsuleCenter, capsuleUnitAxis, capsuleHalfLength, capsuleRadius,
                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                validDirectionMinimumLengthSquared);

    if (rwpmath::And(ok, rwpmath::Or(contact0_Returned,contact1_Returned)).GetBool())
    {
        if (handler.BeginContact(contactUnitDirection))
        {
            handler.AddPointConditional(contactUnitDirection, contact0_Triangle, contact0_Capsule, contact0_Returned);
            handler.AddPointConditional(contactUnitDirection, contact1_Triangle, contact1_Capsule, contact1_Returned);
            handler.EndContact();
        }
        return 1u;
    }

    //------------------------------------------------------
    //  Temporary hack!!
    //
    // If the Branchless function fails, then try the branching.  TODO - why does the branchless sometimes fail?

    uint32_t result = EA::Collision::ComputeContactPointsTriangleCapsule_Branching(handler,
                                triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
                                capsuleCenter, capsuleUnitAxis, capsuleHalfLength, capsuleRadius,
                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                validDirectionMinimumLengthSquared);
    return result;

}

/*************************************************************************************************************
    TRIANGLE SPHERE
*/
EA_FORCE_INLINE uint32_t
ComputeContactPointsTriangleSphere_BranchlessInlineWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  triangleVertex0,
                                            rwpmath::Vector3::InParam  triangleVertex1,
                                            rwpmath::Vector3::InParam  triangleVertex2,
                                            rwpmath::VecFloatInParam   triangleRadius,
                                            rwpmath::Vector3::InParam  sphereCenter,
                                            rwpmath::VecFloatInParam   sphereRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection             INIT_ZERO;
    rwpmath::Vector3 contact0_Sphere                  INIT_ZERO;
    rwpmath::Vector3 contact0_Triangle                INIT_ZERO;
    rwpmath::MaskScalar contact0_Returned             INIT_ZERO;

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsTriangleSphere_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_Triangle, contact0_Sphere,
                                triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
                                sphereCenter, sphereRadius,
                                minimumSeparatingDistance,
                                validDirectionMinimumLengthSquared);

    if (rwpmath::And(ok, contact0_Returned).GetBool())
    {
        if (handler.BeginContact(contactUnitDirection))
        {
            handler.AddPointConditional(contactUnitDirection, contact0_Triangle, contact0_Sphere, contact0_Returned);
            handler.EndContact();
        }
        return 1u;
    }
    return 0u;
}


#undef INIT_ZERO


} // collision
} // rw


#endif // WRAPCOMPUTECONTACTSINLINE_H
