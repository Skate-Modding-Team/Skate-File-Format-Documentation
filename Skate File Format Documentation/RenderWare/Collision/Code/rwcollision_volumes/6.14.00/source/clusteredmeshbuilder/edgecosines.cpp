// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/meshbuilder/edgecosines.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


void EdgeCosines::ComputeEdgeCosine(
    rwpmath::VecFloat &edgeCosine,
    rwpmath::MaskScalar &convex,
    rwpmath::Vector3::InParam triangleOneNormal,
    rwpmath::Vector3::InParam triangleTwoNormal,
    rwpmath::Vector3::InParam normalizedEdgeDirectionInTriangleOne)
{
    // TODO: Hardcoded epsilon - also duplicated in other methods
    const rwpmath::VecFloat epsilon(-1e-6f);
    const rwpmath::VecFloat one(rwpmath::GetVecFloat_One());

    // theta is the angle between the normals.  theta > 0 means the edge is convex
    const rwpmath::VecFloat cos_theta = Dot(triangleOneNormal, triangleTwoNormal);
    const rwpmath::VecFloat sin_theta = Dot(normalizedEdgeDirectionInTriangleOne, Cross(triangleOneNormal, triangleTwoNormal));

    // Note the clamp is required below to ensure numerical noise doesn't make the result be
    // outside the range -1 to 1.
    edgeCosine = rwpmath::Clamp(cos_theta, -one, one);
    convex = rwpmath::CompGreaterThan(sin_theta, epsilon);
}


rwpmath::VecFloat EdgeCosines::ComputeExtendedEdgeCosine(
    rwpmath::Vector3::InParam triangleOneNormal,
    rwpmath::Vector3::InParam triangleTwoNormal,
    rwpmath::Vector3::InParam normalizedEdgeDirectionInTriangleOne)
{
    const rwpmath::VecFloat one(rwpmath::GetVecFloat_One());
    const rwpmath::VecFloat two(rwpmath::GetVecFloat_Two());
    const rwpmath::VecFloat three(one + two);

    // theta is the angle between the normals.  theta > 0 means the edge is convex
    const rwpmath::VecFloat cos_theta = Dot(triangleOneNormal, triangleTwoNormal);
    const rwpmath::VecFloat sin_theta = Dot(normalizedEdgeDirectionInTriangleOne, Cross(triangleOneNormal, triangleTwoNormal));

    // Note the Max and Min are required below to ensure numerical noise doesn't make the result be
    // outside the range -1 to 3.

    if (sin_theta > rwpmath::VecFloat(-1e-6f))
    {
        // Convex angle - the edge cosine value lies between -1.0f and 1.0f
        // and is represented identically in the enhanced range
        return rwpmath::Max(cos_theta, -one);
    }

    // Reflex angle - represented by an enhanced edge cosine value between 1.0f and 3.0f
    return rwpmath::Min(two - cos_theta, three);
}


void EdgeCosines::DecodeExtendedEdgeCosine(
    rwpmath::VecFloat &edgeCosine,
    rwpmath::MaskScalar &convex,
    const rwpmath::VecFloat &extendedEdgeCosine)
{
    const rwpmath::VecFloat one(rwpmath::GetVecFloat_One());
    const rwpmath::VecFloat two(rwpmath::GetVecFloat_Two());

    // The edge is convex if its extended edge cosine is in the normal range [-1, 1]
    convex = rwpmath::CompLessThan(extendedEdgeCosine, one);

    // In the reflex case the normal range edge cosine is 2 minus the extended edge cosine
    const rwpmath::VecFloat twoMinusExtended(two - extendedEdgeCosine);
    edgeCosine = rwpmath::Select(convex, extendedEdgeCosine, twoMinusExtended);
}


} // namespace meshbuilder
} // collision
} // namespace rw
