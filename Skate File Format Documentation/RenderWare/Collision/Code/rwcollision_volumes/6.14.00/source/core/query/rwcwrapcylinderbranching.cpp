// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

 File: rwcwrapcylinderbranching.cpp

 Purpose:  The functions here simply call the eacollision_primitives CCP fns.


*/

#include <rw/collision/common.h>
#include "wrapcomputecontacts.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/cylindercapsule_branching.h"
#include "eacollision/cylindercylinder_branching.h"
#include "eacollision/cylindersphere_branching.h"
#include "eacollision/cylindertriangle_branching.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif

namespace rw
{
namespace collision
{


//------------------------------------------------------------
// CylinderCapsule_Branching Wrapper

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
                                                 rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsCylinderCapsule_Branching(handler,
        cylinderCenter, cylinderUnitAxis, cylinderHalfLength, cylinderInnerRadius, cylinderOuterRadius,
        capsuleCenter, capsuleUnitAxis, capsuleHalfLength, capsuleRadius,
        minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared);
}



//------------------------------------------------------------
// CylinderCylinder_Branching Wrapper

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
                                                  rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    return EA::Collision::ComputeContactPointsCylinderCylinder_Branching(handler,
                                                          cylinderACenter, cylinderAUnitAxis, cylinderAHalfLength, cylinderAInnerRadius, cylinderAOuterRadius,
                                                          cylinderBCenter, cylinderBUnitAxis, cylinderBHalfLength, cylinderBInnerRadius, cylinderBOuterRadius,
                                                          minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
}



//------------------------------------------------------------
// CylinderSphere_Branching Wrapper

uint32_t ComputeContactPointsCylinderSphere_BranchingWrapper(GenericContactHandler& handler,
                                                rwpmath::Vector3::InParam  cylinderCenter,
                                                rwpmath::Vector3::InParam  cylinderUnitAxis,
                                                rwpmath::VecFloatInParam   cylinderHalfLength,
                                                rwpmath::VecFloatInParam   cylinderInnerRadius,
                                                rwpmath::VecFloatInParam   cylinderOuterRadius,
                                                rwpmath::Vector3::InParam  sphereCenter,
                                                rwpmath::VecFloatInParam   sphereRadius,
                                                rwpmath::VecFloatInParam   minimumSeparatingDistance,
                                                rwpmath::VecFloatInParam   validDirectionMinimumLengthSquared)
{
    return EA::Collision::ComputeContactPointsCylinderSphere_Branching(handler,
      cylinderCenter, cylinderUnitAxis, cylinderHalfLength, cylinderInnerRadius, cylinderOuterRadius,
      sphereCenter, sphereRadius,
      minimumSeparatingDistance, validDirectionMinimumLengthSquared);
}


//------------------------------------------------------------
// CylinderTriangle_Branching Wrapper

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
                                                  rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    return EA::Collision::ComputeContactPointsCylinderTriangle_Branching(handler,
      cylinderCenter, cylinderUnitAxis, cylinderHalfLength, cylinderInnerRadius, cylinderOuterRadius,
      triangleVertex0, triangleVertex1, triangleVertex2, triangleRadius,
      minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
}



} // collision
} // rw

