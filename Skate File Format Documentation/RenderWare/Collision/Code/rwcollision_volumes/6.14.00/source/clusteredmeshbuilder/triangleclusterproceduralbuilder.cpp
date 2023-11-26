// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/triangleclusterproceduralbuilder.h>

#include <rw/collision/meshbuilder/vertexcompression.h>
#include <rw/collision/meshbuilder/clusterbuilder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


TriangleClusterProcedural *
TriangleClusterProceduralBuilder::Build(
    EA::Allocator::ICoreAllocator &triangleClusterProceduralAllocator,
    EA::Allocator::ICoreAllocator &workspaceAllocator,
    const BuildParameters &buildParameters,
    const VertexList &vertices,
    const TriangleList &triangles,
    const UnitList &units,
    const TriangleEdgeCodesList &triangleEdgeCodes,
    const TriangleSurfaceIDList &triangleSurfaceIDs,
    const TriangleGroupIDList &triangleGroupIDs)
{
    uint8_t compressionMode;
    rw::collision::ClusteredMeshCluster::Vertex32 clusterOffset;

    DetermineVertexCompressionMode(
        buildParameters,
        compressionMode,
        clusterOffset,
        vertices);

    // Build a descriptor describing the cluster's storage requirements
    rw::collision::ClusterConstructionParameters clusterConstructionParameters;

    // This builder assumes a single cluster so all vertices are in the same cluster
    const uint32_t numVerticesInCluster(vertices.size());
    const uint32_t numUnitsInCluster(units.size());

    // Initialize the ClusteredMeshCluster construction parameters
    ClusterBuilder::InitializeClusterParameters(
        clusterConstructionParameters,
        numVerticesInCluster,
        numUnitsInCluster,
        triangleSurfaceIDs,
        triangleGroupIDs,
        units,
        buildParameters.unitParameters,
        compressionMode);

    // Initialize the TriangleClusterProcedural using the descriptor
    TriangleClusterProcedural *triangleClusterProcedural = InitializeTriangleClusterProcedural(
        triangleClusterProceduralAllocator,
        clusterConstructionParameters);

    triangleClusterProcedural->SetGroupIdSize(buildParameters.unitParameters.groupIDSize);
    triangleClusterProcedural->SetSurfaceIdSize(buildParameters.unitParameters.surfaceIDSize);
    triangleClusterProcedural->SetVertexCompressionGranularity(buildParameters.vertexCompressionGranularity);

    // Finalize the TriangleClusterProcedural
    FinalizeTriangleClusterProcedural(
        *triangleClusterProcedural,
        workspaceAllocator,
        buildParameters,
        vertices,
        triangles,
        units,
        triangleEdgeCodes,
        triangleSurfaceIDs,
        triangleGroupIDs,
        compressionMode,
        clusterOffset);

    // Return the TriangleClusterProcedural
    return triangleClusterProcedural;
}


void
TriangleClusterProceduralBuilder::DetermineVertexCompressionMode(
    const BuildParameters &buildParameters,
    uint8_t &compressionMode,
    rw::collision::ClusteredMeshCluster::Vertex32 &clusterOffset,
    const VertexList &vertices)
{
    // This code assumes there's only one cluster so we can just look at all the vertices
    const uint32_t numVertices(vertices.size());
    if (buildParameters.compressVertices && numVertices > 0)
    {
        int32_t x32 = (int32_t)(vertices[0].GetX() / buildParameters.vertexCompressionGranularity);
        int32_t y32 = (int32_t)(vertices[0].GetY() / buildParameters.vertexCompressionGranularity);
        int32_t z32 = (int32_t)(vertices[0].GetZ() / buildParameters.vertexCompressionGranularity);

        int32_t x32min = x32;
        int32_t x32max = x32;
        int32_t y32min = y32;
        int32_t y32max = y32;
        int32_t z32min = z32;
        int32_t z32max = z32;

        // Find the cluster's extents when converted into integer space
        for (uint32_t i = 1; i < numVertices; ++i)
        {
            const VectorType &vertex = vertices[i];

            x32 = (int32_t)(vertex.GetX() / buildParameters.vertexCompressionGranularity);
            y32 = (int32_t)(vertex.GetY() / buildParameters.vertexCompressionGranularity);
            z32 = (int32_t)(vertex.GetZ() / buildParameters.vertexCompressionGranularity);

            if (x32 < x32min) x32min = x32;
            if (x32 > x32max) x32max = x32;

            if (y32 < y32min) y32min = y32;
            if (y32 > y32max) y32max = y32;

            if (z32 < z32min) z32min = z32;
            if (z32 > z32max) z32max = z32;
        }

        VertexCompression::DetermineCompressionModeAndOffsetForRange(
            compressionMode,
            clusterOffset,
            x32min,
            x32max,
            y32min,
            y32max,
            z32min,
            z32max);
    }
    else
    {
        compressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;

        clusterOffset.x = 0;
        clusterOffset.y = 0;
        clusterOffset.z = 0;
    }
}


TriangleClusterProcedural * 
TriangleClusterProceduralBuilder::InitializeTriangleClusterProcedural(
    EA::Allocator::ICoreAllocator &triangleClusterProceduralAllocator,
    const rw::collision::ClusterConstructionParameters &constructionParameters)
{
    // Get the resources required by the TriangleClusterProcedural
    EA::Physics::SizeAndAlignment resDesc = rw::collision::TriangleClusterProcedural::GetResourceDescriptor(constructionParameters);
    EA::Physics::MemoryPtr res = triangleClusterProceduralAllocator.Alloc(
        static_cast<size_t>(resDesc.GetSize()),
        NULL,
        0u,
        static_cast<unsigned int>(resDesc.GetAlignment()));

    // Initialize the TriangleClusterProcedural
    return rw::collision::TriangleClusterProcedural::Initialize(
        res,
        constructionParameters);
}


void
TriangleClusterProceduralBuilder::FinalizeTriangleClusterProcedural(
    rw::collision::TriangleClusterProcedural &triangleClusterProcedural,
    EA::Allocator::ICoreAllocator &workspaceAllocator,
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
    // Get the ClusteredMeshCluster from the TriangleClusterProcedural
    rw::collision::ClusteredMeshCluster &cluster = triangleClusterProcedural.GetCluster();

    // Fill the cluster with unit data
    ClusterBuilder::BuildParameters params;
    params.unitParameters = buildParameters.unitParameters;
    params.vertexCompressionGranularity = buildParameters.vertexCompressionGranularity;

    ClusterBuilder::Build(
        cluster,
        workspaceAllocator,
        params,
        vertices,
        triangles,
        units,
        triangleEdgeCodes,
        triangleSurfaceIDs,
        triangleGroupIDs,
        compressionMode,
        clusterOffset);

    // Update the TriangleClusterProcedural, making it ready for runtime use.
    triangleClusterProcedural.UpdateThis();
}


} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

