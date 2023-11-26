// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CAPSULECAPSULEQUERY_H
#define PUBLIC_RW_COLLISION_CAPSULECAPSULEQUERY_H

#include "rw/collision/common.h"


namespace rw
{
namespace collision
{


inline void CapsuleCapsuleQuery_Branching(rwpmath::Vector3::InParam centerA, rwpmath::Vector3::InParam axisA,
                                          rwpmath::VecFloatInParam halfLengthA, rwpmath::VecFloatInParam radiusA,
                                          rwpmath::Vector3::InParam centerB, rwpmath::Vector3::InParam axisB,
                                          rwpmath::VecFloatInParam halfLengthB, rwpmath::VecFloatInParam radiusB,
                                          rwpmath::VecFloatInParam padding, rwpmath::VecFloatInParam angleTolerance,
                                          rwpmath::Vector3 &normal, rwpmath::VecFloat &distance,
                                          rwpmath::Vector3 &pointA0, rwpmath::Vector3 &pointA1,
                                          rwpmath::Vector3 &pointB0, rwpmath::Vector3 &pointB1,
                                          rwpmath::MaskScalar &usePoint0, rwpmath::MaskScalar &usePoint1)
{
    using namespace rwpmath;
    Vector3 offset = centerB - centerA;

    // Dot products
    VecFloat axisA_dot_axisB  = Dot(axisA, axisB);
    VecFloat offset_dot_axisA = Dot(offset, axisA);
    VecFloat offset_dot_axisB = Dot(offset, axisB);
    VecFloat axesCrossSquared = GetVecFloat_One() - axisA_dot_axisB * axisA_dot_axisB;

    // Clamped parametric positions of the projections of the end points of the other capsule
    VecFloat projectionA = halfLengthB * physics::mathutils::ReplaceSign(axisA_dot_axisB, offset_dot_axisB);
    VecFloat projectionB = halfLengthA * physics::mathutils::ReplaceSign(axisA_dot_axisB, offset_dot_axisA);
    VecFloat tA0 = physics::mathutils::ClampMagnitude(offset_dot_axisA - projectionA, halfLengthA);
    VecFloat tA1 = physics::mathutils::ClampMagnitude(offset_dot_axisA + projectionA, halfLengthA);
    VecFloat tB0 = physics::mathutils::ClampMagnitude(-offset_dot_axisB - projectionB, halfLengthB);
    VecFloat tB1 = physics::mathutils::ClampMagnitude(-offset_dot_axisB + projectionB, halfLengthB);

    bool usePoint1Bool = axesCrossSquared < angleTolerance;
    bool useEndPoints = true;
    if (!usePoint1Bool)
    {
        // Parametric positions of closest points of the infinite lines
        // these will be invalid in the near parallel case
        VecFloat recip = ReciprocalFast(axesCrossSquared);
        VecFloat tAi = recip * (offset_dot_axisA - axisA_dot_axisB * offset_dot_axisB);
        VecFloat tBi = recip * (axisA_dot_axisB * offset_dot_axisA - offset_dot_axisB);

        if (physics::mathutils::TestRangeUnordered(tAi, tA0, tA1).GetBool() &&
            physics::mathutils::TestRangeUnordered(tBi, tB0, tB1).GetBool())
        {
            pointA0 = centerA + tAi * axisA;
            pointB0 = centerB + tBi * axisB;
            useEndPoints = false;
        }
    }
    if (useEndPoints)
    {
        VecFloat dist00 = MagnitudeSquared(offset - tA0 * axisA + tB0 * axisB);
        VecFloat dist01 = MagnitudeSquared(offset - tA0 * axisA + tB1 * axisB);
        VecFloat dist10 = MagnitudeSquared(offset - tA1 * axisA + tB0 * axisB);
        VecFloat dist11 = MagnitudeSquared(offset - tA1 * axisA + tB1 * axisB);
        if (Min(dist00, dist01) < Min(dist10, dist11))
        {
            pointA0 = centerA + tA0 * axisA;
            if (usePoint1Bool)
            {
                pointA1 = centerA + tA1 * axisA;
            }
        }
        else
        {
            pointA0 = centerA + tA1 * axisA;
            if (usePoint1Bool)
            {
                pointA1 = centerA + tA0 * axisA;
            }
        }
        if (Min(dist00, dist10) < Min(dist01, dist11))
        {
            pointB0 = centerB + tB0 * axisB;
            if (usePoint1Bool)
            {
                pointB1 = centerB + tB1 * axisB;
            }
        }
        else
        {
            pointB0 = centerB + tB1 * axisB;
            if (usePoint1Bool)
            {
                pointB1 = centerB + tB0 * axisB;
            }
        }
    }

    // Results
    distance = NormalizeReturnMagnitude(pointB0 - pointA0, normal) - radiusA - radiusB;
    pointA0 += normal * radiusA;
    pointB0 -= normal * radiusB;
    if (usePoint1Bool)
    {
        pointA1 += normal * radiusA;
        pointB1 -= normal * radiusB;
    }
    usePoint0 = CompLessThan(distance, padding);
    usePoint1 = MaskScalar(usePoint1Bool);
}


inline void CapsuleCapsuleQuery_Branchless(rwpmath::Vector3::InParam centerA, rwpmath::Vector3::InParam axisA,
                                           rwpmath::VecFloatInParam halfLengthA, rwpmath::VecFloatInParam radiusA,
                                           rwpmath::Vector3::InParam centerB, rwpmath::Vector3::InParam axisB,
                                           rwpmath::VecFloatInParam halfLengthB, rwpmath::VecFloatInParam radiusB,
                                           rwpmath::VecFloatInParam padding, rwpmath::VecFloatInParam angleTolerance,
                                           rwpmath::Vector3 &normal, rwpmath::VecFloat &distance,
                                           rwpmath::Vector3 &pointA0, rwpmath::Vector3 &pointA1,
                                           rwpmath::Vector3 &pointB0, rwpmath::Vector3 &pointB1,
                                           rwpmath::MaskScalar &usePoint0, rwpmath::MaskScalar &usePoint1)
{
    using namespace rwpmath;
    Vector3 offset = centerB - centerA;

    // Dot products
    VecFloat axisA_dot_axisB  = Dot(axisA, axisB);
    VecFloat offset_dot_axisA = Dot(offset, axisA);
    VecFloat offset_dot_axisB = Dot(offset, axisB);
    VecFloat axesCrossSquared = GetVecFloat_One() - axisA_dot_axisB * axisA_dot_axisB;
    MaskScalar nearlyParallel(CompLessThan(axesCrossSquared, angleTolerance));

    // Clamped parametric positions of the projections of the end points of the other capsule
    VecFloat projectionA = halfLengthB * physics::mathutils::ReplaceSign(axisA_dot_axisB, offset_dot_axisB);
    VecFloat projectionB = halfLengthA * physics::mathutils::ReplaceSign(axisA_dot_axisB, offset_dot_axisA);
    VecFloat tA0 = physics::mathutils::ClampMagnitude(offset_dot_axisA - projectionA, halfLengthA);
    VecFloat tA1 = physics::mathutils::ClampMagnitude(offset_dot_axisA + projectionA, halfLengthA);
    VecFloat tB0 = physics::mathutils::ClampMagnitude(-offset_dot_axisB - projectionB, halfLengthB);
    VecFloat tB1 = physics::mathutils::ClampMagnitude(-offset_dot_axisB + projectionB, halfLengthB);

    // Parametric positions of closest points of the infinite lines
    // these will be invalid in the near parallel case
    VecFloat recip = ReciprocalFast(axesCrossSquared);
    VecFloat tAi = recip * (offset_dot_axisA - axisA_dot_axisB * offset_dot_axisB);
    VecFloat tBi = recip * (axisA_dot_axisB * offset_dot_axisA - offset_dot_axisB);
    MaskScalar useInfinitePoints = And(Not(nearlyParallel),
        And(physics::mathutils::TestRangeUnordered(tAi, tA0, tA1), physics::mathutils::TestRangeUnordered(tBi, tB0, tB1)));
    VecFloat dist00 = MagnitudeSquared(offset - tA0 * axisA + tB0 * axisB);
    VecFloat dist01 = MagnitudeSquared(offset - tA0 * axisA + tB1 * axisB);
    VecFloat dist10 = MagnitudeSquared(offset - tA1 * axisA + tB0 * axisB);
    VecFloat dist11 = MagnitudeSquared(offset - tA1 * axisA + tB1 * axisB);
    MaskScalar useEndA0 = CompLessThan(Min(dist00, dist01), Min(dist10, dist11));
    MaskScalar useEndB0 = CompLessThan(Min(dist00, dist10), Min(dist01, dist11));

    // Results
    usePoint1 = nearlyParallel;
    VecFloat tA = Select(useInfinitePoints, tAi, Select(useEndA0, tA0, tA1));
    VecFloat tB = Select(useInfinitePoints, tBi, Select(useEndB0, tB0, tB1));
    VecFloat tA2 = Select(useEndA0, tA1, tA0);
    VecFloat tB2 = Select(useEndB0, tB1, tB0);
    pointA0 = centerA + tA * axisA;
    pointB0 = centerB + tB * axisB;
    pointA1 = centerA + tA2 * axisA;
    pointB1 = centerB + tB2 * axisB;
    distance = NormalizeReturnMagnitude(pointB0 - pointA0, normal) - radiusA - radiusB;
    pointA0 += normal * radiusA;
    pointB0 -= normal * radiusB;
    pointA1 += normal * radiusA;
    pointB1 -= normal * radiusB;
    usePoint0 = CompLessThan(distance, padding);
}


} // namespace collision
} // namespace rw


#endif //PUBLIC_RW_COLLISION_CAPSULECAPSULEQUERY_H
