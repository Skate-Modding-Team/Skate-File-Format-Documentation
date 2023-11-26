// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTERBUILDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTERBUILDER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU


#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>


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
class UnitClusterBuilder
{

public:

    typedef detail::TriangleList TriangleList;
    typedef detail::UnitList UnitList;

    /*
    \brief Adds a collection of units to a cluster.

    A collection of units are added, in order, to the cluster. The first unit to add is indicated by
    the start index parameter. The count of units to add is indicated by the numUnitToAdd parameter.
    Units are added to the cluster until all units are added or the vertex count limit is reached.

    \param clusterVertexIDs the collection of cluster vertex IDs.
    \param clusterVertexCount the count of cluster vertices.
    \param clusterUnitIDs the collection of cluster unit IDs.
    \param clusterUnitCount the count of cluster units.
    \param startUnitIndex the index, into the unitID collection, of the first unit to add to the cluster.
    \param numUnitsToAdd the number of units to add to the cluster.
    \param triangles collection of triangles.
    \param vertices collection of vertices.
    \param unitList collection of units.
    \param maxVerticesPerUnit the maximum number of vertices a single unit can have.
    **/
    static uint32_t AddUnitsToUnitCluster(
        UnitCluster::VertexSet & clusterVertexIDs,
        uint32_t & clusterVertexCount,
        UnitCluster::UnitID clusterUnitIDs [],
        uint32_t & clusterUnitCount,
        const uint32_t startUnitIndex,
        const uint32_t numUnitsToAdd,
        const TriangleList & triangles,
        const UnitList & unitList,
        const uint32_t maxVerticesPerUnit);

    /**
    \brief Add a unit to a cluster.

    Adds a unit's vertices to a collection while checking for duplication or a limiting count.
    Adds a unitID to a collection without checking for duplication.

    \param clusterVertexIDs collection of vertex IDs.
    \param clusterVertexCount count of vertexID collection.
    \param clusterUnitIDs collection of unit IDs.
    \param clusterUnitCount count of unit ID collection.
    \param unitID ID of unit to add.
    \param triangles collection of unit triangles.
    \param vertices collection of unit vertices.
    \param unitList collection of units.

    \return true if unit has been added to collection, false otherwize.
    */
    static bool AddUnitToCluster(
        UnitCluster::VertexSet & clusterVertexIDs,
        uint32_t & clusterVertexCount,
        UnitCluster::UnitID clusterUnitList [],
        uint32_t & clusterUnitCount,
        const uint32_t unitID,
        const TriangleList & triangles,
        const UnitList & unitList,
        const uint32_t maxVerticesPerUnit);
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTERBUILDER_H
