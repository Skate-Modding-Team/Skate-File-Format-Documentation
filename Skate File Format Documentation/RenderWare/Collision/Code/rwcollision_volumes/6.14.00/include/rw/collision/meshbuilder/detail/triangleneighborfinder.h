// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLENEIGHBORFINDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLENEIGHBORFINDER_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/edgecosines.h>

#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/vertextrianglemap.h>
#include <rw/collision/meshbuilder/detail/trianglenormal.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
*/
class TriangleNeighborFinder
{

public:

    typedef detail::VertexList VertexList;
    typedef detail::TriangleList TriangleList;
    typedef detail::TriangleEdgeCosinesList TriangleEdgeCosinesList;
    typedef detail::TriangleFlagsList TriangleFlagsList;
    typedef detail::TriangleNeighborsList TriangleNeighborsList;

    /**
    \brief Initializes triangle edge cosine data.

    \param triangleEdgeCodes triangle edge cosine codes container
    */
    static void InitializeTriangleEdgeCosines(
        TriangleEdgeCosinesList & triangleEdgeCosines);

    /**
    \brief Initializes triangle neighbor data.

    \param triangleNeighbors triangle neighbor data container
    */
    static void InitializeTriangleNeighbors(
        TriangleNeighborsList & triangleNeighbors);

    /**
    \brief Create a map which maps vertex indices to triangle indices. Initializes map, inserting
    all entries and sorting the map.

    \param vertexTriangleMap map from vertex indices to triangle indices
    \param triangles collection of triangles
    */
    static void InitializeVertexTriangleMap(
        detail::VertexTriangleMap & vertexTriangleMap,
        const TriangleList & triangles);

    /**
    \brief Builds triangle neighboring connectivity information.

    \param triangles collection of triangles
    \param triangleEdgeCodes collection of triangle edge cosine codes
    \param triangleFlags collection of triangle flags
    \param vertices collection of vertices
    \param vertexTriangleMap map from vertex indices to triangle indices
    */
    static void FindTriangleNeighbors(
        const TriangleList & triangles,
        TriangleEdgeCosinesList & triangleEdgeCosines,
        TriangleNeighborsList & triangleNeighbors,
        const TriangleFlagsList & triangleFlags,
        const VertexList & vertices,
        const detail::VertexTriangleMap & vertexTriangleMap);

private:

    /**
    \brief Attempt to mate two triangles along a given edge.

    Mate edge (edgeAIndex) on triangle (triangleAIndex) with any edge on triangle (triangleBIndex)

    \param triangles collection of triangles
    \param triangleEdgeCodes collection of triangle edge cosine codes
    \param vertices collection of vertices
    \param edgeAIndex The edge number 0,1,2 on i1
    \param triangleAIndex A triangle index
    \param triangleBIndex A triangle index

    \return True if mate found, false otherwise.
    */
    static bool MateEdge(
        const TriangleList & triangles,
        TriangleEdgeCosinesList & triangleEdgeCosines,
        TriangleNeighborsList & triangleNeighbors,
        const VertexList & vertices,
        const uint32_t edge1Index,
        const uint32_t triangle1Index,
        const uint32_t triangle2Index);

    /**
    \brief Finds an edge index given a two triangle indices.

    \param t TriangleDataEx holding edge information.
    \param n Index of 2nd triangle.

    \return Index of shared edge.
    */
    static uint32_t FindEdgeByNeighbor(
        const uint32_t *const neighbors,
        const uint32_t n);
};


inline void TriangleNeighborFinder::InitializeTriangleEdgeCosines(TriangleEdgeCosinesList & triangleEdgeCosines)
{
    const uint32_t numTriangles(triangleEdgeCosines.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        TriangleEdgeCosines & tEdgeCosines = triangleEdgeCosines[triangleIndex];

        tEdgeCosines.edgeCos[0] = CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE;
        tEdgeCosines.edgeCos[1] = CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE;
        tEdgeCosines.edgeCos[2] = CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE;
    }
}


inline void TriangleNeighborFinder::InitializeTriangleNeighbors(TriangleNeighborsList & triangleNeighbors)
{
    const uint32_t numTriangles(triangleNeighbors.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        TriangleNeighbors & tNeighbors = triangleNeighbors[triangleIndex];

        tNeighbors.neighbor[0] = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;
        tNeighbors.neighbor[1] = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;
        tNeighbors.neighbor[2] = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;
    }
}


inline void TriangleNeighborFinder::InitializeVertexTriangleMap(
    detail::VertexTriangleMap & vertexTriangleMap,
    const TriangleList & triangles)
{
    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        // save typing
        const Triangle &t = triangles[triangleIndex];

        // Map the triangle index to each of the vertex indices
        vertexTriangleMap.Insert(t.vertices[0], triangleIndex);
        vertexTriangleMap.Insert(t.vertices[1], triangleIndex);
        vertexTriangleMap.Insert(t.vertices[2], triangleIndex);
    }

    vertexTriangleMap.SortAndIndex();
}


inline uint32_t TriangleNeighborFinder::FindEdgeByNeighbor(const uint32_t *const neighbors, const uint32_t n)
{
    uint32_t i;
    for (i=0; i < 3; ++i)
    {
        if (neighbors[i] == n) break;
    }

    EA_ASSERT(i < 3);
    return i;
}


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLENEIGHBORFINDER_H
