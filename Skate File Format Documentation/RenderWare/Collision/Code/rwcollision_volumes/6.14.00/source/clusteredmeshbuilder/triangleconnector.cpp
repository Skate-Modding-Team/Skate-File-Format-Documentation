// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/triangleconnector.h>

#include <rw/collision/meshbuilder/detail/vertextrianglemap.h>
#include <rw/collision/meshbuilder/detail/triangleneighborfinder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


bool TriangleConnector::GenerateTriangleConnectivity(
    TriangleEdgeCosinesList &triangleEdgeCosines,
    TriangleNeighborsList &triangleNeighbors,
    TriangleFlagsList &triangleFlags,
    EA::Allocator::ICoreAllocator &allocator,
    const VertexList &vertices,
    const TriangleList &triangles)
{
    // Initialize VertexTriangleMap
    detail::VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(triangles.size(), &allocator);

    if (vertexTriangleMap.IsValid())
    {
        detail::TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, triangles);
    }
    else
    {
        return false;
    }

    detail::TriangleNeighborFinder::InitializeTriangleEdgeCosines(triangleEdgeCosines);
    detail::TriangleNeighborFinder::InitializeTriangleNeighbors(triangleNeighbors);

    detail::TriangleNeighborFinder::FindTriangleNeighbors(
        triangles,
        triangleEdgeCosines,
        triangleNeighbors,
        triangleFlags,
        vertices,
        vertexTriangleMap);

    vertexTriangleMap.Release();

    return true;
}


} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

