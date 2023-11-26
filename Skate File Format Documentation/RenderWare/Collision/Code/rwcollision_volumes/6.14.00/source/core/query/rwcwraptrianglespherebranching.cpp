// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

 File: rwcwraptrianglespherebranching.cpp

 Purpose:  The functions here simply call the eacollision_primitives CCP fns.


*/

#include <rw/collision/common.h>
#include "wrapcomputecontacts.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/spheresphere_branching.h"
#include "eacollision/trianglebox_branching.h"
#include "eacollision/trianglecapsule_branching.h"
#include "eacollision/trianglecapsule_branching.h"
#include "eacollision/trianglesphere_branching.h"
#include "eacollision/triangletriangle_branching.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif

namespace rw
{
namespace collision
{


//------------------------------------------------------------
// TriangleTriangle_Branching Wrapper

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
                                                  rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    return EA::Collision::ComputeContactPointsTriangleTriangle_Branching(handler,
        triangleAVertex0, triangleAVertex1, triangleAVertex2, triangleARadius,
        triangleBVertex0, triangleBVertex1, triangleBVertex2, triangleBRadius,
        minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
}


//------------------------------------------------------------
// SphereSphere_Branching Wrapper

uint32_t ComputeContactPointsSphereSphere_BranchingWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  sphereACenter,
                                            rwpmath::VecFloatInParam   sphereARadius,
                                            rwpmath::Vector3::InParam  sphereBCenter,
                                            rwpmath::VecFloatInParam   sphereBRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsSphereSphere_Branching(handler,
                        sphereACenter,
                        sphereARadius,
                        sphereBCenter,
                        sphereBRadius,
                        minimumSeparatingDistance,
                        validDirectionMinimumLengthSquared);
}



//------------------------------------------------------------
// TriangleBox_Branching Wrapper

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
                                        rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    return EA::Collision::ComputeContactPointsTriangleBox_Branching(handler,
        triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
        boxCenter, boxUnitAxis0, boxUnitAxis1, boxUnitAxis2, boxHalfLength0, boxHalfLength1, boxHalfLength2, boxRadius,
        minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
}



//------------------------------------------------------------
// TriangleCapsule_Branching Wrapper

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
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsTriangleCapsule_Branching(handler,
        triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
        capsuleCenter, capsuleUnitAxis, capsuleHalfLength, capsuleRadius,
        minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
        validDirectionMinimumLengthSquared);
}




//------------------------------------------------------------
// TriangleSphere_Branching Wrapper

uint32_t ComputeContactPointsTriangleSphere_BranchingWrapper(GenericContactHandler& handler,
                                            rwpmath::Vector3::InParam  triangleVertex0,
                                            rwpmath::Vector3::InParam  triangleVertex1,
                                            rwpmath::Vector3::InParam  triangleVertex2,
                                            rwpmath::VecFloatInParam   triangleRadius,
                                            rwpmath::Vector3::InParam  sphereCenter,
                                            rwpmath::VecFloatInParam   sphereRadius,
                                            rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                            rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsTriangleSphere_Branching(handler,
                        triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
                        sphereCenter, sphereRadius,
                        minimumSeparatingDistance, validDirectionMinimumLengthSquared);
}


} // collision
} // rw

