// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_COMPUTECONTACTS_H
#define PUBLIC_RW_COLLISION_COMPUTECONTACTS_H

/*************************************************************************************************************

 File: computecontacts.h

 Purpose:
 */

#include <rw/collision/common.h>
#include <rw/collision/volume.h>
#include <rw/collision/primitivepairquery.h>

namespace rw
{
namespace collision
{


uint32_t
ComputeContacts(const GPInstance& gp1, const GPInstance& gp2,
                GPInstance::ContactPoints& result,
                rwpmath::VecFloatInParam minimumSeparatingDistance = COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance,
                rwpmath::VecFloatInParam edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                rwpmath::VecFloatInParam convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon,
                rwpmath::VecFloatInParam triangleFaceNormalTolerance = COMPUTECONTACTS_DEFAULT_TriangleFaceNormalTolerance,
                rwpmath::VecFloatInParam featureSimplificationThreshold = COMPUTECONTACTS_DEFAULT_FeatureSimplificationThreshold,
                rwpmath::VecFloatInParam cosSquaredMaximumAngleConsideredParallel = COMPUTECONTACTS_DEFAULT_CosSquaredMaximumAngleConsideredParallel,
                rwpmath::VecFloatInParam validDirectionMinimumLengthSquared = COMPUTECONTACTS_DEFAULT_ValidDirectionMinimumLengthSquared,
                rwpmath::VecFloatInParam clippingLengthTolerance = COMPUTECONTACTS_DEFAULT_ClippingLengthTolerance);

uint32_t
ComputeContacts(const GPInstance& gp1, const GPInstance& gp2,
                PrimitivePairIntersectResult& result,
                rwpmath::VecFloatInParam minimumSeparatingDistance = COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance,
                rwpmath::VecFloatInParam edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                rwpmath::VecFloatInParam convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon,
                rwpmath::VecFloatInParam triangleFaceNormalTolerance = COMPUTECONTACTS_DEFAULT_TriangleFaceNormalTolerance,
                rwpmath::VecFloatInParam featureSimplificationThreshold = COMPUTECONTACTS_DEFAULT_FeatureSimplificationThreshold,
                rwpmath::VecFloatInParam cosSquaredMaximumAngleConsideredParallel = COMPUTECONTACTS_DEFAULT_CosSquaredMaximumAngleConsideredParallel,
                rwpmath::VecFloatInParam validDirectionMinimumLengthSquared = COMPUTECONTACTS_DEFAULT_ValidDirectionMinimumLengthSquared,
                rwpmath::VecFloatInParam clippingLengthTolerance = COMPUTECONTACTS_DEFAULT_ClippingLengthTolerance);


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_COMPUTECONTACTS_H
