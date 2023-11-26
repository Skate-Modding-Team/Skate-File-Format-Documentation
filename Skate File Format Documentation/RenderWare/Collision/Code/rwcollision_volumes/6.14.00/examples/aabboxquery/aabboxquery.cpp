// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/volumebboxquery.h>
#include <rw/collision/aggregatevolume.h>
#include <rw/collision/simplemappedarray.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/initialize.h>

#include <rw/collision/clusteredmeshofflinebuilder.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain

/*
This code example demonstrates using a VolumeBBoxQuery to extract the overlapping
triangle between a ClusteredMesh and an Axis-Aligned Bounding Box.

A VolumeBBoxQuery is created using an Axis Aligned Bounding Box and the
ClusteredMesh. The Query is then used repeatedly to extract triangle volumes from the
ClusteredMesh. The triangle volumes extracted are instanced by the Query and are those
triangles which overlap with the AABBox.
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
    // Building the Clustered Mesh
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


    //
    // AABBox Query
    //


    {
        // Initialize the size of the query stack and results buffer
        uint32_t stackSize = 1u;
        uint32_t resultBufferSize = 10u;

        // Initialize the AABBox which will be used to query the mesh
        rw::collision::AABBox aabbox(rwpmath::Vector3(-1.0f, -1.0f, -1.0f),
                                     rwpmath::Vector3( 0.5f,  0.5f, 0.5f));

        // Create the volume line query
        EA::Physics::SizeAndAlignment sal = VolumeBBoxQuery::GetResourceDescriptor(stackSize, resultBufferSize);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
        VolumeBBoxQuery * volumeBBoxQuery = VolumeBBoxQuery::Initialize(mem, stackSize, resultBufferSize);

        const rw::collision::Volume * meshVolumePtr = meshVolume;

        //Initialize the specific query parameters with the mesh volume, no volume transforms
        // and the line start and end point
        volumeBBoxQuery->InitQuery(&meshVolumePtr,
                                   NULL,
                                   1,
                                   aabbox);

        // Repeat until we have processed all of the results
        while (!volumeBBoxQuery->Finished())
        {
            // Get the overlaps
            uint32_t numRes = volumeBBoxQuery->GetOverlaps();

            // Get the results buffer
            rw::collision::VolRef * results = volumeBBoxQuery->GetOverlapResultsBuffer();

            // Process each of the results
            for (uint32_t i = 0; i < numRes; ++i)
            {
                // Get the volume of the current result
                const rw::collision::Volume * overlappingVolume  = results[i].volume;

                // At this point we have access to the overlaping volume
                if (overlappingVolume->GetType() == rw::collision::VOLUMETYPETRIANGLE)
                    printf("AABBox Overlapped with a triangle.\n");
            }
        }

        // Release the volume line query
        if (volumeBBoxQuery != NULL)
            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volumeBBoxQuery);
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

