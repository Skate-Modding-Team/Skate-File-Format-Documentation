// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

 File: rwcwrapboxcapsulebranching.cpp

 Purpose:  The functions here simply call the eacollision_primitives CCP Box* and Capsule* fns.

*/

#include <rw/collision/common.h>
#include "wrapcomputecontacts.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/boxbox_branching.h"
#include "eacollision/boxcapsule_branching.h"
#include "eacollision/boxsphere_branching.h"
#include "eacollision/capsulecapsule_branching.h"
#include "eacollision/capsulesphere_branching.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif


namespace rw
{
namespace collision
{

//------------------------------------------------------------
// BoxBox_Branching Wrapper

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
                                        rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    return EA::Collision::ComputeContactPointsBoxBox_Branching(handler,
                                                boxCenterA, boxUnitAxisA0, boxUnitAxisA1, boxUnitAxisA2, boxHalfLengthA0,  boxHalfLengthA1, boxHalfLengthA2, boxRadiusA,
                                                boxCenterB, boxUnitAxisB0, boxUnitAxisB1, boxUnitAxisB2, boxHalfLengthB0, boxHalfLengthB1, boxHalfLengthB2, boxRadiusB,
                                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
}



//------------------------------------------------------------
// BoxCapsule_Branching Wrapper

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
                                        rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsBoxCapsule_Branching(handler,
        boxACenter, boxAUnitAxis0, boxAUnitAxis1, boxAUnitAxis2, boxAHalfLength0,  boxAHalfLength1, boxAHalfLength2, boxARadius,
        capsuleBCenter, capsuleBUnitAxis, capsuleBHalfLength, capsuleBRadius,
        minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared);
}


//------------------------------------------------------------
// BoxSphere_Branching Wrapper

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
                                      rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
   return EA::Collision::ComputeContactPointsBoxSphere_Branching(handler,
                        boxCenter, boxUnitAxis0, boxUnitAxis1, boxUnitAxis2, boxHalfLength0, boxHalfLength1, boxHalfLength2, boxRadius,
                        sphereCenter, sphereRadius, minimumSeparatingDistance, validDirectionMinimumLengthSquared);
}



//------------------------------------------------------------
// CapsuleCapsule_Branching Wrapper

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
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsCapsuleCapsule_Branching(handler,
        capsuleACenter, capsuleAUnitAxis, capsuleAHalfLength, capsuleARadius,
        capsuleBCenter, capsuleBUnitAxis, capsuleBHalfLength, capsuleBRadius,
        minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared);
}



//------------------------------------------------------------
// CapsuleSphere_Branching Wrapper

uint32_t ComputeContactPointsCapsuleSphere_BranchingWrapper(GenericContactHandler& handler,
                                          rwpmath::Vector3::InParam  capsuleCenter,
                                          rwpmath::Vector3::InParam  capsuleUnitAxis,
                                          rwpmath::VecFloatInParam   capsuleHalfLength,
                                          rwpmath::VecFloatInParam   capsuleRadius,
                                          rwpmath::Vector3::InParam  sphereCenter,
                                          rwpmath::VecFloatInParam   sphereRadius,
                                          rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                          rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsCapsuleSphere_Branching(handler,
                            capsuleCenter,
                            capsuleUnitAxis,
                            capsuleHalfLength,
                            capsuleRadius,
                            sphereCenter,
                            sphereRadius,
                            minimumSeparatingDistance,
                            validDirectionMinimumLengthSquared);
}



} // collision
} // rw

