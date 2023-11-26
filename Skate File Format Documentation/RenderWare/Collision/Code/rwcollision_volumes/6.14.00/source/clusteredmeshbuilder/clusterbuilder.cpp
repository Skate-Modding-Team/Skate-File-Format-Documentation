// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/clusterbuilder.h>

#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/unitclusterbuilder.h>
#include <rw/collision/meshbuilder/detail/unitclusterstack.h>
#include <rw/collision/meshbuilder/detail/clusterdatabuilder.h>
#include <rw/collision/meshbuilder/detail/clusterparametersbuilder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


void ClusterBuilder::InitializeClusterParameters(
    rw::collision::ClusterConstructionParameters & parameters,
    const uint32_t numVerticesInCluster,
    const uint32_t numUnitsInCluster,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitList & units,
    const UnitParameters & unitParameters,
    const uint8_t compressionMode)
{
    // The size is stored in a byte
    EA_ASSERT(numVerticesInCluster < ClusteredMeshCluster::MAX_VERTEX_COUNT);

    // Set the per-cluster parameters.
    parameters.mVertexCompressionMode = compressionMode;
    parameters.mVertexCount = static_cast<uint8_t>(numVerticesInCluster);
    parameters.mSurfaceIDSize = static_cast<uint16_t>(unitParameters.surfaceIDSize);
    parameters.mGroupIDSize = static_cast<uint16_t>(unitParameters.groupIDSize);

    // Set the per-unit parameters.
    for (uint32_t unitIndex = 0; unitIndex < numUnitsInCluster; ++unitIndex)
    {
        const Unit & unit = units[unitIndex];

        detail::ClusterParametersBuilder::SumUnitComponentCounts(
            parameters,
            unit.type,
            unitParameters.unitFlagsDefault,
            triangleGroupIDs[unit.tri0],
            triangleSurfaceIDs[unit.tri0]);
    }
}


bool ClusterBuilder::Build(
    rw::collision::ClusteredMeshCluster &cluster,
    EA::Allocator::ICoreAllocator &allocator,
    const BuildParameters &buildParameters,
    const VertexList &vertices,
    const TriangleList &triangles,
    const UnitList &units,
    const TriangleEdgeCodesList &triangleEdgeCodes,
    const TriangleSurfaceIDList &triangleSurfaceIDs,
    const TriangleGroupIDList &triangleGroupIDs,
    const uint8_t compressionMode,
    const rw::collision::ClusteredMeshCluster::Vertex32 &clusterOffset)
{
    // Allocate a single unit cluster
    detail::UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(
        &allocator,
        units.size());

    detail::UnitCluster *const unitCluster = unitClusterStack.GetUnitCluster();

    // Add the units to the unit cluster
    // Maximum number of vertices in a unit is 4, for quads (triangle pairs)
    const uint32_t startUnitIndex = 0;
    const uint32_t numUnitsToAdd = units.size();
    const uint32_t maxVerticesPerUnit = 4;

    detail::UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        numUnitsToAdd,
        triangles,
        units,
        maxVerticesPerUnit);

    // Set the vertex compression mode
    unitCluster->compressionMode = compressionMode;
    unitCluster->clusterOffset = clusterOffset;

    // Build the cluster data
    detail::ClusterDataBuilder::Build(
        cluster,
        *unitCluster,
        vertices,
        triangles,
        triangleEdgeCodes,
        triangleSurfaceIDs,
        triangleGroupIDs,
        units,
        buildParameters.unitParameters,
        buildParameters.vertexCompressionGranularity);

    unitClusterStack.Release();

    return true;
}


} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

