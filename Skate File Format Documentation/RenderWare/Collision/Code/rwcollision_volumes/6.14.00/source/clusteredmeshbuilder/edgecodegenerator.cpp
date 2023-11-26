// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/edgecodegenerator.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilderutils.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


void EdgeCodeGenerator::InitializeTriangleEdgeCodes(TriangleEdgeCodesList & triangleEdgeCodes)
{
    const uint32_t numTriangles(triangleEdgeCodes.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        TriangleEdgeCodes & tEdgeCodes = triangleEdgeCodes[triangleIndex];

        tEdgeCodes.encodedEdgeCos[0] = 0;
        tEdgeCodes.encodedEdgeCos[1] = 0;
        tEdgeCodes.encodedEdgeCos[2] = 0;
    }
}


void EdgeCodeGenerator::GenerateTriangleEdgeCodes(
    TriangleEdgeCodesList & triangleEdgeCodes,
    const TriangleEdgeCosinesList & triangleEdgeCosines,
    const TriangleNeighborsList & triangleNeighbors,
    const rwpmath::VecFloat & minConcaveEdgeCosine)
{
    const uint32_t numTriangles(triangleEdgeCodes.size());
    for(uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        TriangleEdgeCodes &edgeCodes = triangleEdgeCodes[triangleIndex];
        const TriangleEdgeCosines & tEdgeCosines = triangleEdgeCosines[triangleIndex];
        const TriangleNeighbors & tNeighbors = triangleNeighbors[triangleIndex];

        const bool matched0 = (tNeighbors.neighbor[0] != CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH);
        const bool matched1 = (tNeighbors.neighbor[1] != CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH);
        const bool matched2 = (tNeighbors.neighbor[2] != CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH);

        const uint8_t edgeCode0 = GenerateEdgeCode(
            tEdgeCosines.edgeCos[0],
            minConcaveEdgeCosine,
            matched0);

        const uint8_t edgeCode1 = GenerateEdgeCode(
            tEdgeCosines.edgeCos[1],
            minConcaveEdgeCosine,
            matched1);

        const uint8_t edgeCode2 = GenerateEdgeCode(
            tEdgeCosines.edgeCos[2],
            minConcaveEdgeCosine,
            matched2);

        edgeCodes.encodedEdgeCos[0] = static_cast<uint8_t>(edgeCodes.encodedEdgeCos[0] | edgeCode0);
        edgeCodes.encodedEdgeCos[1] = static_cast<uint8_t>(edgeCodes.encodedEdgeCos[1] | edgeCode1);
        edgeCodes.encodedEdgeCos[2] = static_cast<uint8_t>(edgeCodes.encodedEdgeCos[2] | edgeCode2);
    }
}


uint8_t EdgeCodeGenerator::GenerateEdgeCode(
    const rwpmath::VecFloat & extendedEdgeCosine,
    const rwpmath::VecFloat & minConcaveEdgeCosine,
    const bool matched)
{
    uint8_t result = detail::ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(extendedEdgeCosine);

    // In the extended edge cosine range of [-1, +3], any value less than +1 represents convex.
    if (extendedEdgeCosine < rwpmath::GetVecFloat_One())
    {
        // Set the EDGEFLAG_EDGECONVEX flag
        result = static_cast<uint8_t>(result | rw::collision::EDGEFLAG_EDGECONVEX);
    }

    // Ensure the minConcaveEdgeCosine is within the correct range
    const rwpmath::VecFloat cappedMinConcaveEdgeCosine = rwpmath::Clamp(
        minConcaveEdgeCosine,
        rwpmath::GetVecFloat_NegativeOne(),
        rwpmath::GetVecFloat_One());

    // If the extended edge cosine is greater than one that means the edge is concave
    // The degree of concavity is indicated by the value, with 1 meaning planar
    // and 3 meaning vanishingly small interior region. We want to allow the user
    // control how severely concave an edge must be before it is disabled.
    // The user supplies a cosine value range [-1,+1], which is the cosine of the
    // limit angle: smaller angles (with more negative cosines) are considered
    // "too concave" and are disabled. So +1 means anything even slightly concave
    // gets disabled, while -1 means any region not completely closed is enabled.
    // Since the range [-1, +1] is mapped to [+3, +1] for concave edges, we produce
    // an "extended" threshold value by subtracting the provided cosine limit from
    // two, the midpoint of the concave extended range.
    if (extendedEdgeCosine > (rwpmath::GetVecFloat_Two() - cappedMinConcaveEdgeCosine))
    {
        // This effectively disables the edge while clearing all flags
        result = static_cast<uint8_t>(rw::collision::EDGEFLAG_ANGLEZERO);
    }

    if (!matched)
    {
        // Set the EDGEFLAG_EDGEUNMATCHED flag
        result = static_cast<uint8_t>(result | rw::collision::EDGEFLAG_EDGEUNMATCHED);
    }

    return result;
}



} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

