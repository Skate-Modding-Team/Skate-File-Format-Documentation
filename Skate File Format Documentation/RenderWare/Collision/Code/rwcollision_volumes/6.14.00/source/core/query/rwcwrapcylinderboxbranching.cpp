// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

 File: rwcwrapcylinderboxbranching.cpp

 Purpose:  The functions here simply call the eacollision_primitives CCP fns.


*/

#include <rw/collision/common.h>
#include "wrapcomputecontacts.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/cylinderbox_branching.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif


namespace rw
{
namespace collision
{

//------------------------------------------------------------
// CylinderBox_Branching Wrapper

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
                                            rwpmath::VecFloatInParam   clippingLengthTolerance)
{
    return EA::Collision::ComputeContactPointsCylinderBox_Branching(handler,
          cylinderCenter, cylinderUnitAxis, cylinderHalfLength, cylinderInnerRadius, cylinderOuterRadius,
          boxCenter, boxUnitAxis0, boxUnitAxis1, boxUnitAxis2, boxHalfLength0, boxHalfLength1, boxHalfLength2, boxRadius,
          minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
}

} // collision
} // rw

