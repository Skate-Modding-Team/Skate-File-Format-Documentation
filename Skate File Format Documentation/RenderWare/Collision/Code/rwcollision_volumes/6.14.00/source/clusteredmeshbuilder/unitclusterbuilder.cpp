// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/unitclusterbuilder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


uint32_t UnitClusterBuilder::AddUnitsToUnitCluster(
    UnitCluster::VertexSet & clusterVertexIDs,
    uint32_t & clusterVertexCount,
    UnitCluster::UnitID clusterUnitIDs [],
    uint32_t & clusterUnitCount,
    const uint32_t startUnitIndex,
    const uint32_t numUnitsToAdd,
    const TriangleList & triangles,
    const UnitList & unitList,
    const uint32_t maxVerticesPerUnit)
{
    // For each unit to add to the UnitCluster
    uint32_t unitIndex = 0;
    for ( ; unitIndex < numUnitsToAdd ; ++unitIndex)
    {
        // Attempt to add the unit to the cluster
        const bool added = AddUnitToCluster(
            clusterVertexIDs,
            clusterVertexCount,
            clusterUnitIDs,
            clusterUnitCount,
            unitIndex + startUnitIndex,
            triangles,
            unitList,
            maxVerticesPerUnit);

        // If  the unit was not added to the cluster
        if (!added)
        {
            // return the number of units added to the cluster
            return unitIndex;
        }
    }

    // Sort and compress the cluster vertex set
    UnitCluster::SortAndCompressVertexSet(
        clusterVertexIDs,
        clusterVertexCount);

    // Return the number of units which have been added to the cluster
    return unitIndex;
}


bool UnitClusterBuilder::AddUnitToCluster(
    UnitCluster::VertexSet & clusterVertexIDs,
    uint32_t & clusterVertexCount,
    UnitCluster::UnitID clusterUnitIDs [],
    uint32_t & clusterUnitCount,
    const uint32_t unitID,
    const TriangleList & triangles,
    const UnitList & unitList,
    const uint32_t maxVerticesPerUnit)
{
    // If the cluster vertex count is near the count limit
    if (clusterVertexCount > ClusteredMeshCluster::MAX_VERTEX_COUNT - maxVerticesPerUnit)
    {
        // Sort and Compress the cluster vertex set
        UnitCluster::SortAndCompressVertexSet(clusterVertexIDs, clusterVertexCount);

        // If the cluster vertex count is near the limit after having sorted then
        // it can be considered to be full.
        if (clusterVertexCount > ClusteredMeshCluster::MAX_VERTEX_COUNT - maxVerticesPerUnit)
        {
            return false;
        }
    }

    // Add the unit to the cluster
    const Triangle &t1 = triangles[unitList[unitID].tri0];

    clusterVertexIDs[clusterVertexCount++] = t1.vertices[0];
    clusterVertexIDs[clusterVertexCount++] = t1.vertices[1];
    clusterVertexIDs[clusterVertexCount++] = t1.vertices[2];

    if ( unitList[unitID].type == Unit::TYPE_QUAD )
    {
        const Triangle &t2 = triangles[unitList[unitID].tri1];
        clusterVertexIDs[clusterVertexCount++] = t2.vertices[unitList[unitID].extraVertex];
    }

    clusterUnitIDs[clusterUnitCount++] = unitID;

    return true;
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

