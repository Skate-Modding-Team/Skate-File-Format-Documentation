// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/triangleneighborfinder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


void TriangleNeighborFinder::FindTriangleNeighbors(
    const TriangleList & triangles,
    TriangleEdgeCosinesList & triangleEdgeCosines,
    TriangleNeighborsList & triangleNeighbors,
    const TriangleFlagsList & triangleFlags,
    const VertexList & vertices,
    const detail::VertexTriangleMap & vertexTriangleMap)
{
    EA_ASSERT_MSG(triangles.size() != 0, "triangles count should not be zero");
    EA_ASSERT_MSG(triangleEdgeCosines.size() != 0, "triangleEdgeCosines count should not be zero");
    EA_ASSERT_MSG(triangleNeighbors.size() != 0, "triangleNeighbors count should not be zero");
    EA_ASSERT_MSG(triangleFlags.size() != 0, "triangleFlags count should not be zero");
    EA_ASSERT_MSG(vertices.size() != 0, "vert count should not be zero");
    EA_ASSERT_MSG(vertexTriangleMap.IsValid() == true, "vertexTriangleMap should be valid");

    // Cycle through all triangles
    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangle1Index = 0; triangle1Index < numTriangles; ++triangle1Index)
    {
        // Ignore disabled triangles
        if (triangleFlags[triangle1Index].enabled == false)
        {
            continue;
        }

        // Get the current triangle vertex indices
        const uint32_t * const vertexIndices = triangles[triangle1Index].vertices;

        // Cycle through each edge of the current triangle
        for (uint32_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            const uint32_t vertexIndex = vertexIndices[edgeIndex];
            detail::VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin( vertexIndex );
            detail::VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd( vertexIndex );

            while (atIt != atItEnd)
            {
                const uint32_t triangle2Index = *atIt;
                if (triangleFlags[triangle2Index].enabled && triangle1Index > triangle2Index)
                {
                    MateEdge(
                        triangles,
                        triangleEdgeCosines,
                        triangleNeighbors,
                        vertices,
                        edgeIndex,
                        triangle1Index,
                        triangle2Index);
                }

                ++atIt;
            }
        }
    }
}


bool TriangleNeighborFinder::MateEdge(
    const TriangleList & triangles,
    TriangleEdgeCosinesList & triangleEdgeCosines,
    TriangleNeighborsList & triangleNeighbors,
    const VertexList & vertices,
    const uint32_t edge1Index,
    const uint32_t triangle1Index,
    const uint32_t triangle2Index)
{
    uint32_t edge1NextIndex = (edge1Index < 2) ? (edge1Index + 1) : 0;

    const uint32_t *const triangle1VertexIndices = triangles[triangle1Index].vertices;
    const uint32_t *const triangle2VertexIndices = triangles[triangle2Index].vertices;

    float *const triangle1EdgeCosines = triangleEdgeCosines[triangle1Index].edgeCos;
    float *const triangle2EdgeCosines = triangleEdgeCosines[triangle2Index].edgeCos;
    uint32_t *const triangle1NeighborIndices = triangleNeighbors[triangle1Index].neighbor;
    uint32_t *const triangle2NeighborIndices = triangleNeighbors[triangle2Index].neighbor;

    const rwpmath::Vector3 t1Normal(TriangleNormal::ComputeTriangleNormalFast(
        rwpmath::Vector3(vertices[triangle1VertexIndices[0]]),
        rwpmath::Vector3(vertices[triangle1VertexIndices[1]]),
        rwpmath::Vector3(vertices[triangle1VertexIndices[2]])));

    const rwpmath::Vector3 t2Normal(TriangleNormal::ComputeTriangleNormalFast(
        rwpmath::Vector3(vertices[triangle2VertexIndices[0]]),
        rwpmath::Vector3(vertices[triangle2VertexIndices[1]]),
        rwpmath::Vector3(vertices[triangle2VertexIndices[2]])));

    const rwpmath::VecFloat edgeCosine = EdgeCosines::ComputeExtendedEdgeCosine(
        t1Normal,
        t2Normal,
        rwpmath::Vector3(vertices[triangle1VertexIndices[edge1NextIndex]] - vertices[triangle1VertexIndices[edge1Index]]));

    // Test e1 of triangle i1 against all edges of triangle i2.
    for (uint32_t edge2Index = 2u, edge2NextIndex = 0u; edge2NextIndex < 3u; edge2Index = edge2NextIndex++)
    {
        // Test e1==e2next and e2==e1next
        if (triangle1VertexIndices[edge1Index] == triangle2VertexIndices[edge2NextIndex] &&
             triangle2VertexIndices[edge2Index] == triangle1VertexIndices[edge1NextIndex])
        {
            if (triangle1NeighborIndices[edge1Index] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH)
            {
                if (triangle2NeighborIndices[edge2Index] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH)
                {
                    triangle1NeighborIndices[edge1Index] = triangle2Index;
                    triangle2NeighborIndices[edge2Index] = triangle1Index;
                    triangle1EdgeCosines[edge1Index] = triangle2EdgeCosines[edge2Index] = edgeCosine;
                }
                else
                {
                    // t2 already has a match, try to find the least convex match
                    if (edgeCosine > triangle2EdgeCosines[edge2Index])
                    {
                        // t1-t2 is a better match than t3-t2
                        uint32_t triangle3Index = triangle2NeighborIndices[edge2Index];
                        float *triangle3EdgeCosines = triangleEdgeCosines[triangle3Index].edgeCos;
                        uint32_t *triangle3NeighborIndices = triangleNeighbors[triangle3Index].neighbor;

                        triangle1NeighborIndices[edge1Index] = triangle2Index;
                        triangle2NeighborIndices[edge2Index] = triangle1Index;
                        triangle1EdgeCosines[edge1Index] = triangle2EdgeCosines[edge2Index] = edgeCosine;

                        // search edges of triangle3 for the edge with neighbor == triangle2
                        uint32_t edge3Index = FindEdgeByNeighbor(triangle3NeighborIndices, triangle2Index);

                        // mark t3 as unmatched
                        triangle3NeighborIndices[edge3Index] = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;
                        triangle3EdgeCosines[edge3Index] = 1.0f;
                    }
                }
            }
            else
            {
                if (triangle2NeighborIndices[edge2Index] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH)
                {
                    // t1 already a match, try to find the least convex match
                    if ( edgeCosine > triangle1EdgeCosines[edge1Index] )
                    {
                        // t1-t2 is a better match than t1-t3
                        uint32_t triangle3Index = triangle1NeighborIndices[edge1Index];
                        float *triangle3EdgeCosines = triangleEdgeCosines[triangle3Index].edgeCos;
                        uint32_t *triangle3NeighborIndices = triangleNeighbors[triangle3Index].neighbor;

                        triangle1NeighborIndices[edge1Index] = triangle2Index;
                        triangle2NeighborIndices[edge2Index] = triangle1Index;
                        triangle1EdgeCosines[edge1Index] = triangle2EdgeCosines[edge2Index] = edgeCosine;

                        // search edges of triangle3 for the edge with neighbor == triangle2
                        uint32_t edge3Index = FindEdgeByNeighbor(triangle3NeighborIndices, triangle1Index );

                        // mark t3 as unmatched
                        triangle3NeighborIndices[edge3Index] = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;
                        triangle3EdgeCosines[edge3Index] = 1.0f;
                    }
                }
                else
                {
                    // both t1 and t2 are already a matched, try to find the least convex match

                    uint32_t triangle3Index = triangle1NeighborIndices[edge1Index];
                    uint32_t triangle4Index = triangle2NeighborIndices[edge2Index];

                    if (triangle1Index != triangle4Index && triangle2Index != triangle3Index)
                    {
                        if (edgeCosine > triangle1EdgeCosines[edge1Index] && edgeCosine > triangle2EdgeCosines[edge2Index])
                        {
                            float *triangle3EdgeCosines = triangleEdgeCosines[triangle3Index].edgeCos;
                            uint32_t *triangle3NeighborIndices = triangleNeighbors[triangle3Index].neighbor;

                            // search edges of triangle3 for the edge with neighbor == triangle1
                            uint32_t edge3Index = FindEdgeByNeighbor(triangle3NeighborIndices, triangle1Index);

                            float *triangle4EdgeCosines = triangleEdgeCosines[triangle4Index].edgeCos;
                            uint32_t *triangle4NeighborIndices = triangleNeighbors[triangle4Index].neighbor;

                            // search edges of triangle4 for the edge with neighbor == triangle2
                            uint32_t edge4Index = FindEdgeByNeighbor(triangle4NeighborIndices, triangle2Index);

                            // t1-t2 is a better match than t1-t3 or t2-t4
                            triangle1NeighborIndices[edge1Index] = triangle2Index;
                            triangle2NeighborIndices[edge2Index] = triangle1Index;
                            triangle1EdgeCosines[edge1Index] = triangle2EdgeCosines[edge2Index] = edgeCosine;

                            // mark t3 as unmatched
                            triangle3NeighborIndices[edge3Index] = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;
                            triangle3EdgeCosines[edge3Index] = 1.0f;
                            // mark t4 as unmatched
                            triangle4NeighborIndices[edge4Index] = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;
                            triangle4EdgeCosines[edge4Index] = 1.0f;
                        }
                    }
                }
            }

            return true;                 //  RETURN -- found match
        }
    }

    return false;            // no match found
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

