// (c) Electronic Arts. All Rights Reserved.


#include "rw/collision/common.h"


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/vertexmerger.h>
#include <rw/collision/meshbuilder/triangleconnector.h>
#include <rw/collision/meshbuilder/edgecodegenerator.h>
#include <rw/collision/meshbuilder/unitlistbuilder.h>
#include <rw/collision/meshbuilder/vertexcompression.h>
#include <rw/collision/meshbuilder/clusterbuilder.h>

#include <common/common.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example demonstrates how to use the ClusteredMeshBuilder utilities to create
a single cluster from triangle input data consisting of a raw triangle soup with no
pre-existing connectivity information. The process is broken down into a number of steps.

Step A - Initializing Triangle Information

Step B - Merging Vertices

Step C - Generating Connectivity

Step D - Create a list of Units

Step E - Initializing the ClusteredMeshCluster

Step F - Copying the Triangle Data into the ClusteredMeshCluster
*/


typedef rw::collision::meshbuilder::VectorType VectorType;
typedef rw::collision::meshbuilder::AABBoxType AABBoxType;

typedef rw::collision::meshbuilder::ClusterBuilder::VertexList VertexList;
typedef rw::collision::meshbuilder::ClusterBuilder::TriangleList TriangleList;
typedef rw::collision::meshbuilder::ClusterBuilder::TriangleEdgeCodesList TriangleEdgeCodesList;
typedef rw::collision::meshbuilder::ClusterBuilder::TriangleSurfaceIDList TriangleSurfaceIDList;
typedef rw::collision::meshbuilder::ClusterBuilder::TriangleGroupIDList TriangleGroupIDList;
typedef rw::collision::meshbuilder::ClusterBuilder::UnitList UnitList;

typedef rw::collision::meshbuilder::TriangleConnector::TriangleEdgeCosinesList TriangleEdgeCosinesList;
typedef rw::collision::meshbuilder::TriangleConnector::TriangleNeighborsList TriangleNeighborsList;
typedef rw::collision::meshbuilder::TriangleConnector::TriangleFlagsList TriangleFlagsList;


static void CreateGridTriangleSoup(VertexList &vertices,
                                   const uint32_t triangleXCount,
                                   const uint32_t triangleZCount,
                                   TriangleList &triangles,
                                   TriangleSurfaceIDList & triangleSurfaceIDs,
                                   TriangleGroupIDList & triangleGroupIDs)
{
    uint32_t vertexIndex = 0;
    uint32_t triangleIndex = 0;

    for (uint32_t triangleXIndex = 0 ; triangleXIndex < triangleXCount ; ++triangleXIndex)
    {
        for (uint32_t triangleZIndex = 0 ; triangleZIndex < triangleZCount ; ++triangleZIndex)
        {
            VectorType v0(static_cast<float>(triangleXIndex) * 1.0f,
                          0.0f,
                          static_cast<float>(triangleZIndex) * 1.0f);
            VectorType v1(static_cast<float>(triangleXIndex) * 1.0f,
                          0.0f,
                          static_cast<float>(triangleZIndex + 1) * 1.0f);
            VectorType v2(static_cast<float>(triangleXIndex + 1) * 1.0f,
                          0.0f,
                          static_cast<float>(triangleZIndex) * 1.0f);
            VectorType v3(static_cast<float>(triangleXIndex + 1) * 1.0f,
                          0.0f,
                          static_cast<float>(triangleZIndex + 1) * 1.0f);

            vertices[vertexIndex + 0] = v0;
            vertices[vertexIndex + 1] = v1;
            vertices[vertexIndex + 2] = v2;

            triangles[triangleIndex].vertices[0] = vertexIndex + 0;
            triangles[triangleIndex].vertices[1] = vertexIndex + 1;
            triangles[triangleIndex].vertices[2] = vertexIndex + 2;

            vertexIndex += 3;
            ++triangleIndex;

            vertices[vertexIndex + 0] = v1;
            vertices[vertexIndex + 1] = v3;
            vertices[vertexIndex + 2] = v2;

            triangles[triangleIndex].vertices[0] = vertexIndex + 0;
            triangles[triangleIndex].vertices[1] = vertexIndex + 1;
            triangles[triangleIndex].vertices[2] = vertexIndex + 2;

            triangleSurfaceIDs[triangleIndex] = 0u;
            triangleGroupIDs[triangleIndex] = 0u;

            vertexIndex += 3;
            ++triangleIndex;
        }
    }
}


static void BuildAABBox(const VertexList &vertices,
                        const uint32_t vertexCount,
                        AABBoxType &aabbox)
{
    rwpmath::Vector3 boxMin(vertices[0]);
    rwpmath::Vector3 boxMax(vertices[0]);

    for (uint32_t vertexIndex = 1; vertexIndex < vertexCount; ++vertexIndex)
    {
        boxMin = rwpmath::Min(rwpmath::Vector3(vertices[vertexIndex]), boxMin);
        boxMax = rwpmath::Max(rwpmath::Vector3(vertices[vertexIndex]), boxMax);
    }

    aabbox.m_min = AABBoxType::Vector3Type(boxMin);
    aabbox.m_max = AABBoxType::Vector3Type(boxMax);
}


static void MergeVertices(const VertexList &vertices,
                          const uint32_t vertexCount,
                          TriangleList &triangles)
{
    typedef rw::collision::meshbuilder::VertexMerger::IDList IDList;

    const rwpmath::VecFloat vertexMergeDistanceTolerance = 0.01f;

    // Allocate vertex IDs, a map table used in vertex merging
    IDList *vertexIDs = IDList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), vertexCount, EA::Allocator::MEM_TEMP);
    vertexIDs->resize(vertexCount);

    // Initialize the vertex IDs to the initial trivial mapping
    for (uint32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
    {
        (*vertexIDs)[vertexIndex] = vertexIndex;
    }

    // Create an axis-aligned bounding box containing the vertices
    AABBoxType aabbox;
    BuildAABBox(vertices, vertexCount, aabbox);

    // Build up a vertex ID table describing the merge
    rw::collision::meshbuilder::VertexMerger::MergeVertexGroups(
        *vertexIDs,
        *EA::Allocator::ICoreAllocator::GetDefaultAllocator(),
        aabbox,
        vertexMergeDistanceTolerance,
        vertices);

    // Apply the vertex ID table to remap the triangle indices
    rw::collision::meshbuilder::VertexMerger::UpdateTriangleVertexIndices(
        triangles,
        *vertexIDs);

    IDList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), vertexIDs);
}


static void DetermineVertexCompressionMode(
    uint8_t &compressionMode,
    rw::collision::ClusteredMeshCluster::Vertex32 &clusterOffset,
    const VertexList &vertices,
    const float vertexCompressionGranularity)
{
    // This code assumes there's only one cluster so we can just look at all the vertices
    const uint32_t numVertices(vertices.size());
    if (numVertices > 0)
    {
        int32_t x32 = (int32_t)(vertices[0].GetX() / vertexCompressionGranularity);
        int32_t y32 = (int32_t)(vertices[0].GetY() / vertexCompressionGranularity);
        int32_t z32 = (int32_t)(vertices[0].GetZ() / vertexCompressionGranularity);

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

            x32 = (int32_t)(vertex.GetX() / vertexCompressionGranularity);
            y32 = (int32_t)(vertex.GetY() / vertexCompressionGranularity);
            z32 = (int32_t)(vertex.GetZ() / vertexCompressionGranularity);

            if (x32 < x32min) x32min = x32;
            if (x32 > x32max) x32max = x32;

            if (y32 < y32min) y32min = y32;
            if (y32 > y32max) y32max = y32;

            if (z32 < z32min) z32min = z32;
            if (z32 > z32max) z32max = z32;
        }

        rw::collision::meshbuilder::VertexCompression::DetermineCompressionModeAndOffsetForRange(
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


static void CreateUnits(TriangleList &triangles,
                        TriangleFlagsList &triangleFlags,
                        UnitList &units)
{
    rw::collision::meshbuilder::UnitListBuilder::BuildUnitListWithTriangles(
        units,
        triangles,
        triangleFlags);
}


static rw::collision::ClusteredMeshCluster *CreateClusteredMeshCluster(
    const VertexList & vertices,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitList & units,
    const rw::collision::UnitParameters & unitParameters,
    const uint8_t compressionMode)
{
    // There's only one cluster so all vertices and units belong to the cluster
    const uint32_t numVerticesInCluster(vertices.size());
    const uint32_t numUnitsInCluster(units.size());

    // Initialize the ClusteredMeshCluster contruction parameters
    rw::collision::ClusterConstructionParameters parameters;
    rw::collision::meshbuilder::ClusterBuilder::InitializeClusterParameters(
        parameters,
        numVerticesInCluster,
        numUnitsInCluster,
        triangleSurfaceIDs,
        triangleGroupIDs,
        units,
        unitParameters,
        compressionMode);

    // Allocate resources and initialize the ClusteredMeshCluster
    uint32_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);
    void * resource = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    return rw::collision::ClusteredMeshCluster::Initialize(resource, parameters);
}


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // ClusteredMeshCluster Unit parameters
    rw::collision::UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_EDGEANGLE;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 0;
    float vertexCompressionGranularity = 1.0f;

    // Create the triangle soup
    const uint32_t xCount = 2;
    const uint32_t zCount = 2;
    const uint32_t triangleCount = xCount * zCount * 2;
    const uint32_t vertexCount = triangleCount * 3;

    //
    // Create list of builder triangles
    //

    VertexList *vertices = VertexList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), vertexCount, EA::Allocator::MEM_PERM);
    TriangleList *triangles = TriangleList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    TriangleSurfaceIDList *triangleSurfaceIDs = TriangleSurfaceIDList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    TriangleGroupIDList *triangleGroupIDs = TriangleGroupIDList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);

    vertices->resize(vertexCount);
    triangles->resize(triangleCount);
    triangleSurfaceIDs->resize(triangleCount);
    triangleGroupIDs->resize(triangleCount);

    CreateGridTriangleSoup(
        *vertices,
        xCount,
        zCount,
        *triangles,
        *triangleSurfaceIDs,
        *triangleGroupIDs);

    //
    // Merge vertices
    //

    MergeVertices(
        *vertices,
        vertexCount,
        *triangles);

    //
    // Generate triangle connectivity info
    //

    TriangleEdgeCosinesList *triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    TriangleNeighborsList *triangleNeighbors = TriangleNeighborsList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    TriangleFlagsList *triangleFlags = TriangleFlagsList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);

    triangleEdgeCosines->resize(triangleCount);
    triangleNeighbors->resize(triangleCount);
    triangleFlags->resize(triangleCount);

    rw::collision::meshbuilder::TriangleConnector::GenerateTriangleConnectivity(
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *EA::Allocator::ICoreAllocator::GetDefaultAllocator(),
        *vertices,
        *triangles);

    TriangleEdgeCodesList *triangleEdgeCodes = TriangleEdgeCodesList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    triangleEdgeCodes->resize(triangleCount);

    rwpmath::VecFloat edgecosConcaveAngleTolerance = 0.0f;
    rw::collision::meshbuilder::EdgeCodeGenerator::GenerateTriangleEdgeCodes(
        *triangleEdgeCodes,
        *triangleEdgeCosines,
        *triangleNeighbors,
        edgecosConcaveAngleTolerance);

    //
    // Create a list of Units
    //

    UnitList *units = UnitList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);

    units->reserve(triangleCount);

    CreateUnits(
        *triangles,
        *triangleFlags,
        *units);

    //
    // Decide vertex compression mode
    //

    uint8_t compressionMode;
    rw::collision::ClusteredMeshCluster::Vertex32 clusterOffset;

    DetermineVertexCompressionMode(
        compressionMode,
        clusterOffset,
        *vertices,
        vertexCompressionGranularity);

    //
    // Initialize the ClusteredMeshCluster
    //

    rw::collision::ClusteredMeshCluster *const clusteredMeshCluster = CreateClusteredMeshCluster(
        *vertices,
        *triangleSurfaceIDs,
        *triangleGroupIDs,
        *units,
        unitParameters,
        compressionMode);

    //
    // Fill the cluster with data
    //

    rw::collision::meshbuilder::ClusterBuilder::BuildParameters fillerParams;
    fillerParams.unitParameters = unitParameters;
    fillerParams.vertexCompressionGranularity = vertexCompressionGranularity;

    rw::collision::meshbuilder::ClusterBuilder::Build(
        *clusteredMeshCluster,
        *EA::Allocator::ICoreAllocator::GetDefaultAllocator(),
        fillerParams,
        *vertices,
        *triangles,
        *units,
        *triangleEdgeCodes,
        *triangleSurfaceIDs,
        *triangleGroupIDs,
        compressionMode,
        clusterOffset);

    //
    // Output
    //

    common::DescribeCluster(
        *clusteredMeshCluster,
        unitParameters.unitFlagsDefault,
        unitParameters.groupIDSize,
        unitParameters.surfaceIDSize,
        vertexCompressionGranularity);

    //
    // Release resources
    // 

    if (NULL != clusteredMeshCluster)
    {
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(clusteredMeshCluster);
    }

    UnitList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), units);
    TriangleEdgeCodesList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleEdgeCodes);
    TriangleFlagsList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleFlags);
    TriangleNeighborsList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleNeighbors);
    TriangleEdgeCosinesList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleEdgeCosines);
    TriangleGroupIDList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleGroupIDs);
    TriangleSurfaceIDList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleSurfaceIDs);
    TriangleList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangles);
    VertexList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), vertices);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU
