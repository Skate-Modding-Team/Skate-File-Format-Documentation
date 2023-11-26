// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/aggregatevolume.h>
#include <rw/collision/simplemappedarray.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/initialize.h>

#include <rw/collision/clusteredmeshruntimebuilder.h>

#include <meshoperate/implementations/halfedgemesh/offlinemesh.h>
#include <meshoperate/shapes/geosphere.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example shows how to build a clustered mesh from mesh data stored in a
MeshOperate mesh.

MeshOperate is a powerful mesh processing toolkit built around a generic API.
Various mesh types are available, each of which supports a well-defined subset of the API.
The mesh used in this example is required to contain only triangular faces (as opposed
to arbitrary polygons) and is expected to be 2-manifold (with at most two faces
incident to each edge).

The ClusteredMeshRuntimeBuilder is used to generate the ClusteredMesh,
however the ClusteredMeshOfflineBuilder could be used just as easily.
*/
int EAMain(int /*argc*/, char ** /*argv*/)
{
    // Convenience typedefs and using statements
    using rw::collision::ClusteredMeshRuntimeBuilder;

    using rw::collision::VolumeLineQuery;
    using rw::collision::VolumeBBoxQuery;

    typedef rw::collision::meshbuilder::VectorType VectorType;

    rw::collision::ClusteredMesh * clusteredMesh = NULL;
    rw::collision::AggregateVolume * meshVolume;

    // We have to initialize the vtables before using any volume features.
    rw::collision::InitializeVTables();

    // Allocate a buffer for use by the runtime clustered mesh builder as its workspace.
    // The buffer needs to be "big enough" to process the data we're building.
    // If the buffer size is too small then the builder returns a null mesh pointer.
    // Note that the final clustered mesh produced by the builder is *not* allocated
    // within the working buffer. It's allocated using a separately provided allocator.
    const uint32_t builderBufferSize(64 * 1024);
    uint8_t *const builderBuffer(static_cast<uint8_t *>(EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(builderBufferSize, "workspace", 0)));
    if (!builderBuffer)
    {
        printf("Failed to allocate temporary buffer for builder");
        return 1;
    }

    //
    // Build a MeshOperate mesh with some test mesh data
    //

    typedef meshoperate::halfedge::offline::Mesh MeshType;
    MeshType mesh;

    {
        // We build a geosphere for convenience since its faces are triangular
        // and we can easily control its face count
        meshoperate::shapes::GeoSphere<MeshType>::Parameters geoSphereParams;
        geoSphereParams.mSubdivisionLevel = 2;
        meshoperate::shapes::GeoSphere<MeshType>::Generate(mesh, geoSphereParams);
    }

    //
    // Build the Clustered Mesh, feeding the MeshOperate mesh data to the builder
    //

    {
        // NOTE: We unshare the vertices here for simplicity
        // Optimally, we would create each vertex only once and reference it multiple times.
        // But that would require a map of some kind to store the target vertex index associated
        // with each source vertex handle. That's required because the indices of the vertices
        // of a MeshOperate mesh need not be contiguous (compact). If there are gaps then the
        // index of one or more vertex will be greater than or equal to the number of vertices,
        // and so out of range. Of course if the vertex indices are known a priori to be
        // contiguous (for example, because the mesh was built trivially immediately prior to
        // this) then things are much simpler.
        const uint32_t triangleCount = mesh.GetNumFaces();
        const uint32_t vertexCount = mesh.GetNumFaces() * 3;
        const uint32_t mergePlaneCount = 0u;

        // Use the builders default settings
        ClusteredMeshRuntimeBuilder::Parameters params;

        // Initialize the builder with the triangle and vertex count, default parameters,
        // workspace buffer, and the default allocator for allocation of the clustered mesh
        ClusteredMeshRuntimeBuilder runtimeBuilder(triangleCount,
                                                   vertexCount,
                                                   mergePlaneCount,
                                                   params,
                                                   builderBuffer,
                                                   builderBufferSize,
                                                   EA::Allocator::ICoreAllocator::GetDefaultAllocator());

        // Set the triangle data
        uint32_t triangleIndex(0);
        uint32_t vertexIndex(0);

        MeshType::FaceIterator triangles(mesh.FacesBegin());
        const MeshType::FaceIterator trianglesEnd(mesh.FacesEnd());

        while (triangles != trianglesEnd)
        {
            const MeshType::FaceHandle faceHandle(mesh.FaceIteratorToHandle(triangles));
            EA_ASSERT(mesh.GetNumFaceVertices(faceHandle) == 3);

            MeshType::FaceVertexCirculator triangleVertices(mesh.FaceVerticesBegin(faceHandle));

            uint32_t faceVertexIndex(0);
            while (faceVertexIndex < 3)
            {
                const MeshType::VertexHandle vertexHandle(mesh.FaceVertexCirculatorToHandle(triangleVertices));
                const MeshType::VectorType vertexPoint(mesh.GetVertexPosition(vertexHandle));

                // Set a single vertex with its index and position
                runtimeBuilder.SetVertex(vertexIndex + faceVertexIndex, vertexPoint);

                ++triangleVertices;
                ++faceVertexIndex;
            }

            // Set a single triangle in the builder with the triangle index,
            // and the indices of its vertices
            runtimeBuilder.SetTriangle(
                triangleIndex,
                vertexIndex + 0,
                vertexIndex + 1,
                vertexIndex + 2);

            ++triangles;
            ++triangleIndex;
            vertexIndex += 3;
        }

        // Build the ClusteredMesh
        clusteredMesh = runtimeBuilder.BuildClusteredMesh();

        // Create an aggregate volume with the clusteredmesh as its aggregate.
        EA::Physics::SizeAndAlignment sal = rw::collision::AggregateVolume::GetResourceDescriptor(clusteredMesh);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
        meshVolume = rw::collision::AggregateVolume::Initialize(mem, clusteredMesh);
    }

    // Free the temporary workspace buffer now that the builder is done with it
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(builderBuffer);

    // Release the AggregateVolume
    if (meshVolume != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(meshVolume);

    // Release the resources
    if (NULL != clusteredMesh)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(clusteredMesh);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU
