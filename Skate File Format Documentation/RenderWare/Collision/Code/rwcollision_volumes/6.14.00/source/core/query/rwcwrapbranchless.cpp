// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

 File: rwcwrapbranchless.cpp

 Purpose:  The functions here simply call the eacollision_primitives CCP fns.

*/

#include <rw/collision/common.h>
#include "wrapcomputecontacts.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/boxsphere_branchless.h"
#include "eacollision/capsulecapsule_branchless.h"
#include "eacollision/capsulesphere_branchless.h"
#include "eacollision/spheresphere_branchless.h"
#include "eacollision/trianglebox_branchless.h"
#include "eacollision/trianglecapsule_branching.h"        // fallback
#include "eacollision/trianglecapsule_branchless.h"
#include "eacollision/trianglesphere_branchless.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif


namespace rw
{
namespace collision
{


//------------------------------------------------------------
// BoxSphere_Branchless Wrapper

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
                                      rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection            = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Sphere                 = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::MaskScalar contact0_Returned            = rwpmath::GetMaskScalar_False();

    rwpmath::MaskScalar ok = EA::Collision::ComputeContactPointsBoxSphere_Branchless(
                                contactUnitDirection,
                                contact0_Returned, contact0_Box, contact0_Sphere,
                                boxCenter, boxUnitAxis0,  boxUnitAxis1, boxUnitAxis2,
                                boxHalfLength0, boxHalfLength1, boxHalfLength2, boxRadius,
                                sphereCenter, sphereRadius,
                                minimumSeparatingDistance,
                                validDirectionMinimumLengthSquared);

    if (rwpmath::And(ok, contact0_Returned).GetBool())
    {
        handler.BeginContactQuick(contactUnitDirection);
        handler.AddPointQuick(contact0_Box, contact0_Sphere, contact0_Returned);
        return 1u;
    }
    return 0u;
}


//------------------------------------------------------------
// CapsuleCapsule_Branchless Wrapper

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
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection            = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_CapsuleB               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact1_CapsuleB               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_CapsuleA               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact1_CapsuleA               = rwpmath::GetVector3_Zero();
    rwpmath::MaskScalar contact0_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact1_Returned            = rwpmath::GetMaskScalar_False();

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

//------------------------------------------------------------
// CapsuleSphere_Branchless Wrapper

uint32_t ComputeContactPointsCapsuleSphere_BranchlessWrapper(GenericContactHandler& handler,
                                          rwpmath::Vector3::InParam  capsuleCenter,
                                          rwpmath::Vector3::InParam  capsuleUnitAxis,
                                          rwpmath::VecFloatInParam   capsuleHalfLength,
                                          rwpmath::VecFloatInParam   capsuleRadius,
                                          rwpmath::Vector3::InParam  sphereCenter,
                                          rwpmath::VecFloatInParam   sphereRadius,
                                          rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                          rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection            = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Sphere                 = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Capsule                = rwpmath::GetVector3_Zero();
    rwpmath::MaskScalar contact0_Returned            = rwpmath::GetMaskScalar_False();

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


//------------------------------------------------------------
// SphereSphere_Branchless Wrapper

uint32_t ComputeContactPointsSphereSphere_BranchlessWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  sphereACenter,
                                            rwpmath::VecFloatInParam   sphereARadius,
                                            rwpmath::Vector3::InParam  sphereBCenter,
                                            rwpmath::VecFloatInParam   sphereBRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection            = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_SphereA                = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_SphereB                = rwpmath::GetVector3_Zero();
    rwpmath::MaskScalar contact0_Returned            = rwpmath::GetMaskScalar_False();

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


//------------------------------------------------------------
// TriangleBox_Branchless Wrapper

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
                                        rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    rwpmath::Vector3 contactUnitDirection            = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact1_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact2_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact3_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact4_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact5_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact6_Box                    = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact1_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact2_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact3_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact4_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact5_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact6_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::MaskScalar contact0_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact1_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact2_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact3_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact4_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact5_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact6_Returned            = rwpmath::GetMaskScalar_False();

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


//------------------------------------------------------------
// TriangleCapsule_Branchless Wrapper

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
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection            = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Capsule                = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact1_Capsule                = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact1_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::MaskScalar contact0_Returned            = rwpmath::GetMaskScalar_False();
    rwpmath::MaskScalar contact1_Returned            = rwpmath::GetMaskScalar_False();

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



//------------------------------------------------------------
// TriangleSphere_Branchless Wrapper

uint32_t ComputeContactPointsTriangleSphere_BranchlessWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  triangleVertex0,
                                            rwpmath::Vector3::InParam  triangleVertex1,
                                            rwpmath::Vector3::InParam  triangleVertex2,
                                            rwpmath::VecFloatInParam   triangleRadius,
                                            rwpmath::Vector3::InParam  sphereCenter,
                                            rwpmath::VecFloatInParam   sphereRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    rwpmath::Vector3 contactUnitDirection            = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Sphere                 = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 contact0_Triangle               = rwpmath::GetVector3_Zero();
    rwpmath::MaskScalar contact0_Returned            = rwpmath::GetMaskScalar_False();

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


} // collision
} // rw

