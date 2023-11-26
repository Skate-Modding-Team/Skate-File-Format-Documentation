// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/aggregatevolume.h>
#include <rw/collision/simplemappedarray.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/initialize.h>

#include <rw/collision/clusteredmeshofflinebuilder.h>

#include <common/common.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example shows how to use the ClusteredMeshOfflineBuilder to generate a
ClusteredMesh.

The ClusteredMeshOfflineBuilder is a specialized tool for building
clustered meshes from low-level triangle data in an offline environment. For its
internal workspace allocations it uses a general allocator provided by the caller.
This allocator is expected to implement EA::Allocator::ICoreAllocator. As well
as the internal temporary allocations, this allocator is also used to allocate the
final clustered mesh produced by the builder. It is the caller's responsibility
to de-allocate the clustered mesh after use.

The ClusteredMeshOfflineBuilder is given a simple input and then, using the default
build parameters, is used to generate a ClusteredMesh which takes the form of a cube.

The ClusteredMesh is then wrapped in an AggregateVolume, suitable for use with
line queries and bounding box tests (shown in other samples).
*/


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // Convenience typedefs and using statements
    using rw::collision::ClusteredMeshOfflineBuilder;

    using rw::collision::VolumeLineQuery;
    using rw::collision::VolumeBBoxQuery;

    typedef rw::collision::meshbuilder::VectorType VectorType;

    rw::collision::ClusteredMesh * clusteredMesh = NULL;
    rw::collision::AggregateVolume * meshVolume;

    // We have to initialize the vtables before using any volume features.
    rw::collision::InitializeVTables();

    //
    // Build the Clustered Mesh
    //

    {
        const uint32_t triangleCount = 12u;
        const uint32_t vertexCount = 8u;
        const uint32_t mergePlaneCount = 0u;

        // The input vertices
        VectorType vertices[vertexCount] = { VectorType(0.0f, 0.0f, 0.0f),
                                             VectorType(0.0f, 0.0f, 1.0f),
                                             VectorType(1.0f, 0.0f, 0.0f),
                                             VectorType(1.0f, 0.0f, 1.0f),
                                             VectorType(0.0f, 1.0f, 0.0f),
                                             VectorType(0.0f, 1.0f, 1.0f),
                                             VectorType(1.0f, 1.0f, 0.0f),
                                             VectorType(1.0f, 1.0f, 1.0f) };

        // The input triangle vertex indices
        uint32_t indices [] = { 0, 1, 2,    // triangle 1
                                1, 3, 2,    // triangle 2
                                5, 4, 7,    // triangle 3
                                4, 6, 7,    // triangle 4
                                6, 3, 7,    // triangle 5
                                6, 2, 3,    // triangle 6
                                5, 1, 0,    // triangle 7
                                5, 0, 4,    // triangle 8
                                4, 2, 6,    // triangle 9
                                4, 0, 2,    // triangle 10
                                7, 1, 5,    // triangle 11
                                7, 3, 1 };  // triangle 12

        // Use the builders default settings
        ClusteredMeshOfflineBuilder::Parameters params;

        // Initialize the builder with the triangle and vertex count, default parameters
        // and the default allocator.
        ClusteredMeshOfflineBuilder offlineBuilder(triangleCount,
                                                   vertexCount,
                                                   mergePlaneCount,
                                                   params,
                                                   EA::Allocator::ICoreAllocator::GetDefaultAllocator());

        // Set the triangle data
        for (uint32_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex)
        {
            uint32_t vertexIndices = triangleIndex * 3;

            // Set a single triangle in the builder with the triangle index,
            // and the indices of its vertices
            offlineBuilder.SetTriangle(triangleIndex,
                                       indices[vertexIndices],
                                       indices[vertexIndices + 1],
                                       indices[vertexIndices + 2]);
        }

        // Set the vertex data
        for (uint32_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
        {
            // Set a single vertex with its index and position
            offlineBuilder.SetVertex(vertexIndex, vertices[vertexIndex]);
        }

        // Build the ClusteredMesh
        clusteredMesh = offlineBuilder.BuildClusteredMesh();

        // Create an aggregate volume with the clusteredmesh as its aggregate.
        EA::Physics::SizeAndAlignment sal = rw::collision::AggregateVolume::GetResourceDescriptor(clusteredMesh);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
        meshVolume = rw::collision::AggregateVolume::Initialize(mem, clusteredMesh);
    }

    common::DescribeClusteredMesh(*clusteredMesh);

    // Release the AggregateVolume
    if (meshVolume != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(meshVolume);

    // Release the resources
    if (NULL != clusteredMesh)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(clusteredMesh);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU

