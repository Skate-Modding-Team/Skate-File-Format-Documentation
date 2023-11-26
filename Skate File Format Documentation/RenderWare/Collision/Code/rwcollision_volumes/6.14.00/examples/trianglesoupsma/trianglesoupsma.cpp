// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/triangle.h>
#include <rw/collision/simplemappedarray.h>

#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/vertexmerger.h>
#include <rw/collision/meshbuilder/triangleconnector.h>
#include <rw/collision/meshbuilder/edgecosines.h>

#include <common/common.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example demonstrates how to use the ClusteredMeshBuilder Utilities to create
a SimpleMappedArray of TriangleVolumes with edge cosines. The input in this example takes
the form of a triangle soup. The process is broken down into a number of steps.

Step A - Initializing Triangle Information

Step B - Merging Vertices

Step C - Generating Connectivity

Step D - Initializing the SMA

Step E - Copying the Triangle Data into the SMA
*/


typedef rw::collision::meshbuilder::VectorType VectorType;
typedef rw::collision::meshbuilder::AABBoxType AABBoxType;

typedef rw::collision::meshbuilder::VertexMerger::VertexList VertexList;
typedef rw::collision::meshbuilder::VertexMerger::TriangleList TriangleList;

typedef rw::collision::meshbuilder::TriangleConnector::TriangleEdgeCosinesList TriangleEdgeCosinesList;
typedef rw::collision::meshbuilder::TriangleConnector::TriangleNeighborsList TriangleNeighborsList;
typedef rw::collision::meshbuilder::TriangleConnector::TriangleFlagsList TriangleFlagsList;


static void CreateGridTriangleSoup(
    VertexList &vertices,
    const uint32_t triangleXCount,
    const uint32_t triangleZCount,
    TriangleList &triangles)
{
    // Create a vertex list and an indexed triangle list. The builder expects indexed triangle
    // data, where each triangle consists of three indices into a unified vertex list.
    // Note however that we don't make use of that indexing here, preferring instead to
    // generate a separate copy of each vertex for each triangle in which it is referenced.
    // This choice is merely for convenience -- effectively we're relying on the vertex merging
    // process to remerge the redundant copies of the vertices.

    uint32_t vertexIndex = 0;
    uint32_t triangleIndex = 0;

    for (uint32_t triangleXIndex = 0; triangleXIndex < triangleXCount; ++triangleXIndex)
    {
        for (uint32_t triangleZIndex = 0; triangleZIndex < triangleZCount; ++triangleZIndex)
        {
            const VectorType v0(
                static_cast<float>(triangleXIndex),
                0.0f,
                static_cast<float>(triangleZIndex));

            const VectorType v1(
                static_cast<float>(triangleXIndex),
                0.0f,
                static_cast<float>(triangleZIndex + 1));

            const VectorType v2(
                static_cast<float>(triangleXIndex + 1),
                0.0f,
                static_cast<float>(triangleZIndex + 1));

            const VectorType v3(
                static_cast<float>(triangleXIndex + 1),
                0.0f,
                static_cast<float>(triangleZIndex));

            // First triangle in pair
            vertices[vertexIndex + 0] = v0;
            vertices[vertexIndex + 1] = v1;
            vertices[vertexIndex + 2] = v2;

            triangles[triangleIndex].vertices[0] = vertexIndex + 0;
            triangles[triangleIndex].vertices[1] = vertexIndex + 1;
            triangles[triangleIndex].vertices[2] = vertexIndex + 2;

            vertexIndex += 3;
            ++triangleIndex;

            // Second triangle
            vertices[vertexIndex + 0] = v2;
            vertices[vertexIndex + 1] = v3;
            vertices[vertexIndex + 2] = v0;

            triangles[triangleIndex].vertices[0] = vertexIndex + 0;
            triangles[triangleIndex].vertices[1] = vertexIndex + 1;
            triangles[triangleIndex].vertices[2] = vertexIndex + 2;

            vertexIndex += 3;
            ++triangleIndex;
        }
    }
}


static void BuildAABBox(
    const VertexList &vertices,
    const uint32_t vertexCount,
    AABBoxType &aabbox)
{
    rwpmath::Vector3 boxMin(vertices[0]);
    rwpmath::Vector3 boxMax(vertices[0]);

    // Build the axis-aligned bounding box of the vertices
    for (uint32_t vertexIndex = 1; vertexIndex < vertexCount; ++vertexIndex)
    {
        boxMin = rwpmath::Min(rwpmath::Vector3(vertices[vertexIndex]), boxMin);
        boxMax = rwpmath::Max(rwpmath::Vector3(vertices[vertexIndex]), boxMax);
    }

    aabbox.m_min = AABBoxType::Vector3Type(boxMin);
    aabbox.m_max = AABBoxType::Vector3Type(boxMax);
}


static void MergeVertices(
    const VertexList &vertices,
    const uint32_t vertexCount,
    TriangleList &triangles)
{
    typedef rw::collision::meshbuilder::VertexMerger::IDList IDList;

    // Pairs of vertices within this tolerance distance are merged
    const rwpmath::VecFloat vertexMergeDistanceTolerance = 0.01f;

    // Allocate the vertex IDs table, a map used in vertex merging
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


static void CopyTriangleDataIntoSMA(
    const VertexList &vertices,
    const TriangleList &triangles,
    const uint32_t triangleCount,
    TriangleEdgeCosinesList &triangleEdgeCosines,
    rw::collision::SimpleMappedArray *const sma)
{
    for (uint16_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex)
    {
        rw::collision::TriangleVolume *triangle = static_cast<rw::collision::TriangleVolume *>(sma->GetVolume(triangleIndex));

        // Set up triangle vertices
        triangle->SetPoints(
            rwpmath::Vector3(vertices[triangles[triangleIndex].vertices[0]]),
            rwpmath::Vector3(vertices[triangles[triangleIndex].vertices[1]]),
            rwpmath::Vector3(vertices[triangles[triangleIndex].vertices[2]]));

        // Calculate edge cosines and flags
        rwpmath::VecFloat edgeCosines[3] =
        {
            CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE,
            CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE,
            CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE
        };

        rwpmath::MaskScalar edgeConvexFlags[3] = { rwpmath::GetMaskScalar_False() };

        rw::collision::meshbuilder::EdgeCosines::DecodeExtendedEdgeCosine(
            edgeCosines[0],
            edgeConvexFlags[0],
            triangleEdgeCosines[triangleIndex].edgeCos[0]);

        rw::collision::meshbuilder::EdgeCosines::DecodeExtendedEdgeCosine(
            edgeCosines[1],
            edgeConvexFlags[1],
            triangleEdgeCosines[triangleIndex].edgeCos[1]);

        rw::collision::meshbuilder::EdgeCosines::DecodeExtendedEdgeCosine(
            edgeCosines[2],
            edgeConvexFlags[2],
            triangleEdgeCosines[triangleIndex].edgeCos[2]);

        // Set up edge cosines
        triangle->SetEdgeCos(edgeCosines[0], edgeCosines[1], edgeCosines[2]);

        // Set up triangle flags. Triangle edges are marked convex by default
        uint32_t triangleFlags = rw::collision::VOLUMEFLAG_TRIANGLEDEFAULT;
        if (!edgeConvexFlags[0].GetBool())
        {
            triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX);
        }

        if (!edgeConvexFlags[1].GetBool())
        {
            triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX);
        }

        if (!edgeConvexFlags[2].GetBool())
        {
            triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX);
        }

        triangle->SetFlags(triangleFlags);
    }
}


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // These parameters define the resolution of a triangulated rectangular grid
    const uint32_t xCount = 5;
    const uint32_t zCount = 5;
    const uint32_t triangleCount = xCount * zCount * 2;
    const uint32_t vertexCount = triangleCount * 3;

    //
    // Create indexed triangle list to feed to the builder
    //

    // Allocate the vertex and triangle lists. Three vertices per triangle
    VertexList *vertices = VertexList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), vertexCount, EA::Allocator::MEM_PERM);
    TriangleList *triangles = TriangleList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);

    vertices->resize(vertexCount);
    triangles->resize(triangleCount);

    CreateGridTriangleSoup(
        *vertices,
        xCount,
        zCount,
        *triangles);

    //
    // Merge vertices
    //

    MergeVertices(
        *vertices,
        vertexCount,
        *triangles);

    //
    // Generate triangle connectivity info: edge cosines and neighbor indices
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

    //
    // Initialize the SMA
    //

    EA::Physics::SizeAndAlignment sal = rw::collision::SimpleMappedArray::GetResourceDescriptor(triangleCount);
    void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
    rw::collision::SimpleMappedArray *sma = rw::collision::SimpleMappedArray::Initialize(mem, triangleCount);

    //
    // Copy the triangle data into the SMA
    //

    CopyTriangleDataIntoSMA(
        *vertices,
        *triangles,
        triangleCount,
        *triangleEdgeCosines,
        sma);

    common::DescribeSMA(sma);

    TriangleNeighborsList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleNeighbors);
    TriangleEdgeCosinesList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleEdgeCosines);
    TriangleList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangles);
    VertexList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), vertices);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU
