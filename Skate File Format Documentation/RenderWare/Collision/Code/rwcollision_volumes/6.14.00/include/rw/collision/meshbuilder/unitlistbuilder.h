// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_UNITLISTBUILDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_UNITLISTBUILDER_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/detail/containers.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
\brief Static helper class that builds a collection of cluster units from a collection of triangles.

The component pieces of clusters are units, which by convention are either single triangles or
pairs of triangles adjacent on a shared edge (misleadingly called "quads"). A cluster consists of
a limited number of such units.

This class provides a convenient way to generate a collection of units in preparation for
building a cluster. It provides methods for building a collection of units with quads or
triangles. Building of quad units is considerably more complex since it relies on triangle
connectivity/adjacency information and takes into account triangle group and surface IDs
(requiring that both triangles in a triangle pair share the same IDs). The building of quads
also requires the use of a separate container of unit IDs provided by the caller for internal
use.
*/
class UnitListBuilder
{

public:

    typedef detail::UnitList UnitList;
    typedef detail::IDList IDList;
    typedef detail::TriangleList TriangleList;
    typedef detail::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef detail::TriangleGroupIDList TriangleGroupIDList;
    typedef detail::TriangleNeighborsList TriangleNeighborsList;
    typedef detail::TriangleFlagsList TriangleFlagsList;
    typedef detail::VertexList VertexList;

    /**
    \brief Builds a unit collection, allowing the use of both triangle and quad (triangle pair) units.

    The units are stored in a pre-allocated container provided by the caller, which is expected
    to have enough capacity to hold the maximum possible number of units (which assumes no pairing
    of triangles in the worst case, so is equal to the number of input triangles).

    Specific pairs of triangles are converted into quad units. These pairs are
    triangles which share a longest edge. All triangles which are not paired are
    converted into triangle units.
   
    \note The provided unit container is expected to be empty on input, with a size of zero.
    The units are added via push_back and are not expected to already exist.

    \param unitList                 An empty collection of units to be filled by this method.
    \param compressedUnitIndex      Collection of unit IDs for internal use, expected to be the same size as the triangles container.
    \param triangles                Collection of triangles from which units are to be built.
    \param triangleEdgeCodes        Collection of per-triangle edge cosine code triples.
    \param triangleSurfaceIDs       Collection of per-triangle surface IDs.
    \param triangleGroupIDs         Collection of per-triangle group IDs.
    \param triangleFlags            Collection of per-triangle flags.
    \param vertices                 Collection of vertices referenced by the triangle collection.
    \param surfaceIDSize            The number of bytes to be used to store each surface ID.
    \param groupIDSize              The number of bytes to be used to store each group ID.

    \return The total number of units built.
    */
    static uint32_t BuildUnitListWithQuads(
        UnitList &unitList,
        IDList &compressedUnitIndex,
        const TriangleList &triangles,
        const TriangleSurfaceIDList &triangleSurfaceIDs,
        const TriangleGroupIDList &triangleGroupIDs,
        const TriangleNeighborsList &triangleNeighbors,
        const TriangleFlagsList &triangleFlags,
        const VertexList &vertices,
        const uint32_t surfaceIDSize,
        const uint32_t groupIDSize);

    /**
    \brief Builds a unit collection, allowing the use of only triangle units.

    The units are stored in a pre-allocated container provided by the caller, which is expected
    to have enough capacity to hold all units (equal to the number of input triangles).

    \note The provided unit container is expected to be empty on input, with a size of zero.
    The units are added via push_back and are not expected to already exist.

    \param unitList                 An empty collection of units to be filled by this method.
    \param triangles                Collection of triangles from which units are to be built.
    \param triangleEdgeCodes        Collection of per-triangle edge cosine code triples.
    \param triangleFlags            Collection of per-triangle flags.

    \return The number of triangle units built.
    */
    static uint32_t BuildUnitListWithTriangles(
        UnitList &unitList,
        const TriangleList &triangles,
        const TriangleFlagsList &triangleFlags);

private:

    typedef detail::Unit Unit;
    typedef detail::Triangle Triangle;
    typedef detail::TriangleSurfaceID TriangleSurfaceID;
    typedef detail::TriangleGroupID TriangleGroupID;
    typedef detail::TriangleNeighbors TriangleNeighbors;
    typedef detail::TriangleFlags TriangleFlags;
};


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_UNITLISTBUILDER_H
