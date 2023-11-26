// (c) Electronic Arts. All Rights Reserved.

#include <stdio.h>

#include <rw/collision/initialize.h>
#include <rw/collision/volumebboxquery.h>
#include <rw/collision/aggregatevolume.h>
#include <rw/collision/procedural.h>
#include <rw/collision/clusteredmeshcluster.h>
#include <rw/collision/triangle.h>
#include <rw/collision/clusteredmeshofflinebuilder.h>
#include <rw/collision/scaledclusteredmesh.h>

#include <common/common.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // Convenience typedefs and using statements
    using rw::collision::ClusteredMeshOfflineBuilder;

    using rw::collision::VolumeLineQuery;
    using rw::collision::VolumeBBoxQuery;

    typedef rw::collision::meshbuilder::VectorType VectorType;

    rw::collision::ClusteredMesh * clusteredMesh = NULL;
    rw::collision::ScaledClusteredMesh * scaledClusteredMesh[2] = {NULL,NULL};
    rw::collision::AggregateVolume * meshVolume[2] = {NULL,NULL};

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
        VectorType vertices[vertexCount] = {
            VectorType(0.0f, 0.0f, 0.0f),
            VectorType(0.0f, 0.0f, 1.0f),
            VectorType(1.0f, 0.0f, 0.0f),
            VectorType(1.0f, 0.0f, 1.0f),
            VectorType(0.0f, 1.0f, 0.0f),
            VectorType(0.0f, 1.0f, 1.0f),
            VectorType(1.0f, 1.0f, 0.0f),
            VectorType(1.0f, 1.0f, 1.0f) };

        // The input triangle vertex indices
        uint32_t indices [] = {
            0, 1, 2,    // triangle 1
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

        // Wrap ScaledClustredMesh around the ClusteredMesh created above
        {
            EA::Physics::SizeAndAlignment sal = rw::collision::ScaledClusteredMesh::GetResourceDescriptor(clusteredMesh, 5.0f);
            void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
            scaledClusteredMesh[0] = rw::collision::ScaledClusteredMesh::Initialize(mem, clusteredMesh, 5.0f);
        }
        {
            EA::Physics::SizeAndAlignment sal = rw::collision::ScaledClusteredMesh::GetResourceDescriptor(clusteredMesh, 10.0f);
            void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
            scaledClusteredMesh[1] = rw::collision::ScaledClusteredMesh::Initialize(mem, clusteredMesh, 10.0f);
        }
        {
            // Create an aggregate volume with the scaledClusteredMesh as its aggregate.
            EA::Physics::SizeAndAlignment sal = rw::collision::AggregateVolume::GetResourceDescriptor(scaledClusteredMesh[0]);
            void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
            meshVolume[0] = rw::collision::AggregateVolume::Initialize(mem, scaledClusteredMesh[0]);
        }
        {
            // Create an aggregate volume with the scaledClusteredMesh as its aggregate.
            EA::Physics::SizeAndAlignment sal = rw::collision::AggregateVolume::GetResourceDescriptor(scaledClusteredMesh[1]);
            void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
            meshVolume[1] = rw::collision::AggregateVolume::Initialize(mem, scaledClusteredMesh[1]);
        }
    }


    //
    // AABBox Query
    //


    {
        // Initialize the size of the query stack and results buffer
        uint32_t stackSize = 1u;
        uint32_t resultBufferSize = 10u;

        // Initialize the AABBox which will be used to query the meshes
        rw::collision::AABBox aabbox(
            rwpmath::Vector3(-1.0f, 5.0f,-1.0f),
            rwpmath::Vector3(36.0f, 7.0f, 1.0f));

        // Create the volume bbox query
        EA::Physics::SizeAndAlignment sal = VolumeBBoxQuery::GetResourceDescriptor(stackSize, resultBufferSize);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
        VolumeBBoxQuery * volumeBBoxQuery = VolumeBBoxQuery::Initialize(mem, stackSize, resultBufferSize);

        rwpmath::Matrix44Affine meshTM0 = rwpmath::Matrix44AffineFromTranslation(rwpmath::Vector3( 0.f,-2.5f, 0.f));
        rwpmath::Matrix44Affine meshTM1 = rwpmath::Matrix44AffineFromTranslation(rwpmath::Vector3(10.f, 1.f, 0.f));
        rwpmath::Matrix44Affine meshTM2 = rwpmath::Matrix44AffineFromTranslation(rwpmath::Vector3(25.f,-10.f, 0.f));

        const rw::collision::Volume * meshVolumePtr[3] = {meshVolume[0], meshVolume[0], meshVolume[1]};
        const rwpmath::Matrix44Affine * meshTMPtr[3] = {&meshTM0, &meshTM1, &meshTM2};

        //Initialize the specific query parameters with the mesh volume, no volume transforms
        // and the line start and end point
        volumeBBoxQuery->InitQuery(
            meshVolumePtr,
            meshTMPtr,
            3,
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

    //
    // Line intersection test
    //


    {
        // Initialize the size of the query stack and results buffer
        uint32_t stackSize = 1u;
        uint32_t resultBufferSize = 10u;

        // Initialize the line start and end point to query the meshes
        rwpmath::Vector3 lineStart(0.0f, 6.0f, 0.0f);
        rwpmath::Vector3 lineEnd(35.0f, 6.0f, 0.0f);
        float fatness = 1.f;

        // Create the volume line query
        EA::Physics::SizeAndAlignment sal = VolumeLineQuery::GetResourceDescriptor(stackSize, resultBufferSize);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
        VolumeLineQuery * volumeLineQuery = VolumeLineQuery::Initialize(mem, stackSize, resultBufferSize);

        rwpmath::Matrix44Affine meshTM0 = rwpmath::Matrix44AffineFromTranslation(rwpmath::Vector3( 0.f,-2.5f, 0.f));
        rwpmath::Matrix44Affine meshTM1 = rwpmath::Matrix44AffineFromTranslation(rwpmath::Vector3(10.f, 1.f, 0.f));
        rwpmath::Matrix44Affine meshTM2 = rwpmath::Matrix44AffineFromTranslation(rwpmath::Vector3(25.f,-10.f, 0.f));

        const rw::collision::Volume * meshVolumePtr[3] = {meshVolume[0], meshVolume[0], meshVolume[1]};
        const rwpmath::Matrix44Affine * meshTMPtr[3] = {&meshTM0, &meshTM1, &meshTM2};

        // Initialize the specific query parameters with the mesh volume, no volume transforms
        // and the line start and end point
        volumeLineQuery->InitQuery(
            meshVolumePtr,
            meshTMPtr,
            3,
            lineStart,
            lineEnd,
            fatness);

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
    if (meshVolume[0] != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(meshVolume[0]);
    if (meshVolume[1] != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(meshVolume[1]);

    // Release the resources
    if (NULL != clusteredMesh)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(clusteredMesh);

    // Release the ScaledClusteredMesh
    if (scaledClusteredMesh[0] != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledClusteredMesh[0]);
    if (scaledClusteredMesh[1] != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledClusteredMesh[1]);

    return 0;
}
