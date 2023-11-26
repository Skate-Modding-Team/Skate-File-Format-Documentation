// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/volumelinequery.h>
#include <rw/collision/aggregatevolume.h>
#include <rw/collision/simplemappedarray.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/initialize.h>

#include <rw/collision/clusteredmeshofflinebuilder.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example demonstrates using a VolumeLineQuery to run a line query against
a ClusteredMesh.

A VolumeLineQuery object is created and initialized with a line and a previously
generated ClusteredMesh AggregateVolume. The VolumeLineQuery is then repeatedly
queried for all intersections.
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
        uint32_t indices [] = { 0, 1, 2,     // triangle 1
                                1, 3, 2,     // triangle 2
                                5, 4, 7,     // triangle 3
                                4, 6, 7,     // triangle 4
                                6, 3, 7,     // triangle 5
                                6, 2, 3,     // triangle 6
                                5, 1, 0,     // triangle 7
                                5, 0, 4,     // triangle 8
                                4, 2, 6,     // triangle 9
                                4, 0, 2,     // triangle 10
                                7, 1, 5,     // triangle 11
                                7, 3, 1 };   // triangle 12

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


    //
    // Line intersection test
    //


    {
        // Initialize the size of the query stack and results buffer
        uint32_t stackSize = 1u;
        uint32_t resultBufferSize = 10u;

        // Initialize the line start and end point
        rwpmath::Vector3 lineStart(0.5f, 0.5f, -10.0f);
        rwpmath::Vector3 lineEnd(0.5f, 0.5f,  10.0f);

        // Create the volume line query
        EA::Physics::SizeAndAlignment sal = VolumeLineQuery::GetResourceDescriptor(stackSize, resultBufferSize);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
        VolumeLineQuery * volumeLineQuery = VolumeLineQuery::Initialize(mem, stackSize, resultBufferSize);

        const rw::collision::Volume * meshVolumePtr = meshVolume;

        // Initialize the specific query parameters with the mesh volume, no volume transforms
        // and the line start and end point
        volumeLineQuery->InitQuery(&meshVolumePtr,
                                   NULL,
                                   1,
                                   lineStart,
                                   lineEnd);

        // Continue while there are still volumes left to query
        while (!volumeLineQuery->Finished())
        {
            // Get as many results as possible
            uint32_t numRes = volumeLineQuery->GetAllIntersections();

            // Get the results buffer
            rw::collision::VolumeLineSegIntersectResult * results = volumeLineQuery->GetIntersectionResultsBuffer();

            // Process each of the results
            for (uint32_t i = 0; i < numRes; ++i)
            {
                // Get the intersected volume from the result
                const rw::collision::Volume * intersectedVolume = results[i].v;

                // At this point we have access to the intersected volume
                if (intersectedVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE)
                {
                    printf("Line intersected volume.\n");
                }
            }
        }

        // Release the volume line query
        if (volumeLineQuery != NULL)
            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volumeLineQuery);
    }


    // Release the AggregateVolume
    if (meshVolume != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(meshVolume);

    // Release the resources
    if (NULL != clusteredMesh)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(clusteredMesh);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU

