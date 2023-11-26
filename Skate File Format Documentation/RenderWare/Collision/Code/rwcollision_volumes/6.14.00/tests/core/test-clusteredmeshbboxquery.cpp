// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include "clusteredmeshtest_base.hpp"

using namespace rwpmath;
using namespace rw::collision;

#define COURTYARD  "courtyard.dat"
#define SKATEMESH_COMPRESSED_QUADS_IDS  "skatemesh_compressed_quads_ids.dat"
#define LEAVES_SPANNING_CLUSTERS  "mesh_leaves_spanning_clusters.dat"

namespace
{

const char *g_clusteredMeshFilenames[] =
{
    COURTYARD,
    SKATEMESH_COMPRESSED_QUADS_IDS,
    LEAVES_SPANNING_CLUSTERS
};

}

// Unit tests for bbox queries against ClusteredMeshes
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.

class TestClusteredMeshBBoxQuery: public ClusteredMeshTest_Base
{
public:
    virtual void Initialize()
    {
        SuiteName("TestClusteredMeshBBoxQuery");

#define CLUSTERED_MESH_TEST(F, D) EATEST_REGISTER(#F, D, TestClusteredMeshBBoxQuery, F)

        CLUSTERED_MESH_TEST(TestBBoxQuery,
            "Test a BBoxQuery against a clustered mesh");
        CLUSTERED_MESH_TEST(TestBBoxQueryInMappedArrayWithPrimitives,
            "Test a BBoxQuery against a clustered mesh where the mesh is in a mapped array with some primitives");

#if !defined(EA_PLATFORM_PS3_SPU)
        CLUSTERED_MESH_TEST(TestTriangleChildIndex,
            "Test the triangle child index returned from a BBoxQuery against a clustered mesh");
        CLUSTERED_MESH_TEST(TestQuadFirstTriangleChildIndex,
            "Test the quad-first-triangle child index returned from a BBoxQuery against a clustered mesh");
        CLUSTERED_MESH_TEST(TestQuadSecondTriangleChildIndex,
            "Test the quad-second-triangle child index returned from a BBoxQuery against a clustered mesh");
#endif // !defined(EA_PLATFORM_PS3_SPU)
    }

private:
    void TestBBoxQuery();
    void TestBBoxQueryInMappedArrayWithPrimitives();

#if !defined(EA_PLATFORM_PS3_SPU)
    inline void AssertTrianglesTheSameExcludingFlags(const TriangleVolume * volumeA, const TriangleVolume * volumeB)
    {
        Vector3 triangleAVertices[3];
        volumeA->GetPoints(triangleAVertices[0], triangleAVertices[1], triangleAVertices[2], NULL);
        Vector3 triangleANormal;
        volumeA->GetNormal(triangleANormal, NULL);

        Vector3 triangleBVertices[3];
        volumeB->GetPoints(triangleBVertices[0], triangleBVertices[1], triangleBVertices[2], NULL);
        Vector3 triangleBNormal;
        volumeB->GetNormal(triangleBNormal, NULL);

        EATESTAssert(IsSimilar(triangleAVertices[0], triangleBVertices[0]), "Triangle vertices[0] do not match");
        EATESTAssert(IsSimilar(triangleAVertices[1], triangleBVertices[1]), "Triangle vertices[1] do not match");
        EATESTAssert(IsSimilar(triangleAVertices[2], triangleBVertices[2]), "Triangle vertices[2] do not match");
        EATESTAssert(IsSimilar(triangleANormal, triangleBNormal), "Triangle normals do not match");
        EATESTAssert(IsSimilar(volumeA->GetEdgeCosVector(), volumeB->GetEdgeCosVector()), "Edge cosine data does not match");
        EATESTAssert(IsSimilar(volumeA->GetRadius(), volumeB->GetRadius()), "Volume radius does not match");
        EATESTAssert(volumeA->GetGroup()   == volumeB->GetGroup(), "Volume group does not match");
        EATESTAssert(volumeA->GetSurface() == volumeB->GetSurface(), "Volume surface id does not match");
    }

    void TestTriangleChildIndex();
    void TestQuadFirstTriangleChildIndex();
    void TestQuadSecondTriangleChildIndex();

#endif // !defined(EA_PLATFORM_PS3_SPU)

} TestClusteredMeshBBoxQuerySingleton;


// Define OUTPUT_NEW_DATA to create new data files in the current directory
// #define OUTPUT_NEW_DATA

void TestClusteredMeshBBoxQuery::TestBBoxQuery()
{
    // Test to make sure that no results are lost when the result buffer is not large enough to hold all
    // results.
    //
    // Two mesh are tested, one with only triangles the other with triangles and quads. This is to
    // make sure that restarting part way through a unit works.
    //
    // Various result buffer sizes are test starting with only enough space for a single triangle.

    const uint32_t RESBUFFERMAXSIZE = 5u;
    const uint32_t STACKSIZE = 1u;

    for (uint32_t cm = 0; cm < EAArrayCount(g_clusteredMeshFilenames); ++cm)
    {
        // Load ClusteredMesh
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(g_clusteredMeshFilenames[cm]);
        EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");

        AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
        ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());
        
        float cos45 = 0.707106781f;
        float sin45 = 0.707106781f;
        const rwpmath::Matrix44Affine transformMatrix(GetVector3_XAxis(), 
            rwpmath::Vector3(0.0f, cos45, -sin45), 
            rwpmath::Vector3(0.0f, sin45, cos45),            
            rwpmath::Vector3(0.0f, 0.123456f, 0.0f));

        RestartingBBoxQueryTester(clusteredMeshVolume, &transformMatrix, mesh->GetVolumeCount() * 2, STACKSIZE, RESBUFFERMAXSIZE);        

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
    }
}

void TestClusteredMeshBBoxQuery::TestBBoxQueryInMappedArrayWithPrimitives()
{
    //Load ClusteredMesh
    Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(COURTYARD);
    AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
    ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());
    
    EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");
    BBoxQueryInMappedArrayWithPrimitivesTester(clusteredMeshVolume);

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
}


#if !defined(EA_PLATFORM_PS3_SPU)
void TestClusteredMeshBBoxQuery::TestTriangleChildIndex()
{
    // TEST OVERVIEW:
    // This test tests the aabbox query overlap result tag.
    // The components of the tag are tested: Unit Triangle Index, Unit Offset, Cluster Index
    // The volume referred to by the tag is compared to the volume instanced by the query

    // WHAT THE TEST DOES:
    // An aabbox overlap query is executed, which encompasses a single triangle within the mesh.
    // The triangle volume referred to by the corresponding result tag is then obtained
    // and compared against the instanced volume. The two should be identical.

    const uint32_t RESBUFFERMAXSIZE = 1u;
    const uint32_t STACKSIZE = 1u;

    // Create aggregate volume in order to run the query
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New(triangleMesh);

    // Get the clusteredmesh aabbox
    rw::collision::AABBox volBBox(rwpmath::Vector3(0.0f, -0.1f, 0.0f),
                                  rwpmath::Vector3(0.1f, 0.1f, 0.1f));

    // Create a clusteredmesh volume pointer pointer
    const Volume *volArray[1] = { &*aggVol };

    // Create a VolumeBBoxQuery object with space for a single result
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(STACKSIZE, RESBUFFERMAXSIZE);
    // Initialize the query with the clusteredmesh volume and bounding box
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    // Attempt to pick up the first overlap
    EATESTAssert(p_VBBQ->GetOverlaps(), "Query should return at least 1 overlap");

    // Get the volume reference results buffer
    const VolRef *volRef = p_VBBQ->GetOverlapResultsBuffer();
    // Get the number of query results
    const uint32_t numResults = p_VBBQ->GetOverlapResultsBufferCount();

    // Check for the single result
    EATESTAssert(1 == numResults, "Num query results should be 1");

    // Get the child index
    uint32_t childIndex = triangleMesh->GetChildIndexFromTag(volRef[0].tag);

    // Check the components of the child index.
    EATESTAssert(0 == triangleMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex), "Unit triangle index should be 0");
    EATESTAssert(0 == triangleMesh->GetClusterIndexFromChildIndex(childIndex), "Cluster index should be 0");
    EATESTAssert(0 == triangleMesh->GetUnitOffsetFromChildIndex(childIndex), "Unit offset should be 0");

    // Get the Triangle Volume from the child index
    rw::collision::TriangleVolume * childIndexVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
    triangleMesh->GetVolumeFromChildIndex(*childIndexVolume,
                                          childIndex);

    // Get the instanced triangle volume from the query
    const rw::collision::TriangleVolume * instancedVolume = static_cast<const rw::collision::TriangleVolume*>(volRef->volume);

    // Compare the instanced volume to the volume retrieved through the child index
    AssertTrianglesTheSameExcludingFlags(childIndexVolume, instancedVolume);

    // Deallocate resources
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(childIndexVolume);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(p_VBBQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
}


void TestClusteredMeshBBoxQuery::TestQuadFirstTriangleChildIndex()
{
    // TEST OVERVIEW:
    // This test tests the aabbox query overlap result tag.
    // The components of the tag are tested: Unit Triangle Index, Unit Offset, Cluster Index
    // The volume referred to by the tag is compared to the volume instanced by the query

    // WHAT THE TEST DOES:
    // An aabbox overlap query is executed, which encompasses a single quad triangle within the mesh.
    // The triangle volume referred to by the corresponding result tag is then obtained
    // and compared against the instanced volume. The two should be identical.

    const uint32_t RESBUFFERMAXSIZE = 1u;
    const uint32_t STACKSIZE = 1u;

    // Create aggregate volume in order to run the query
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New(quadMesh);

    // Create an aabbox which overlaps the first triangle in a quad
    rw::collision::AABBox volBBox(rwpmath::Vector3(0.0f, -0.1f, 0.0f),
                                  rwpmath::Vector3(0.5f, 0.1f, 0.5f));

    // Create a clusteredmesh volume pointer pointer
    const Volume *volArray[1] = { &*aggVol };

    // Create a VolumeBBoxQuery object with space for a single result
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(STACKSIZE, RESBUFFERMAXSIZE);
    // Initialize the query with the clusteredmesh volume and bounding box
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    // Attempt to pick up the first overlap
    EATESTAssert(p_VBBQ->GetOverlaps(), "Query should return at least 1 overlap");
    EATESTAssert(p_VBBQ->GetOverlaps(), "Query should return at least 1 overlap");

    // Get the volume reference results buffer
    const VolRef *volRef = p_VBBQ->GetOverlapResultsBuffer();
    // Get the number of query results
    const uint32_t numResults = p_VBBQ->GetOverlapResultsBufferCount();

    // Check for the single result
    EATESTAssert(1 == numResults, "Num query results should be 1");

    // Get the child index
    uint32_t childIndex = quadMesh->GetChildIndexFromTag(volRef[0].tag);

    // Check the components of the child index.
    EATESTAssert(0 == quadMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex), "Unit triangle index should be 0");
    EATESTAssert(0 == quadMesh->GetClusterIndexFromChildIndex(childIndex), "Cluster index should be 0");
    EATESTAssert(0 == quadMesh->GetUnitOffsetFromChildIndex(childIndex), "Unit offset should be 0");

    // Get the Triangle Volume from the child index
    rw::collision::TriangleVolume * childIndexVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
    quadMesh->GetVolumeFromChildIndex(*childIndexVolume,
                                      childIndex);

    // Get the instanced triangle volume from the query
    const rw::collision::TriangleVolume * instancedVolume = static_cast<const rw::collision::TriangleVolume*>(volRef->volume);

    // Compare the instanced volume to the volume retrieved through the child index
    AssertTrianglesTheSameExcludingFlags(childIndexVolume, instancedVolume);

    // Deallocate resources
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(childIndexVolume);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(p_VBBQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
}


void TestClusteredMeshBBoxQuery::TestQuadSecondTriangleChildIndex()
{
    // TEST OVERVIEW:
    // This test tests the aabbox query overlap result tag.
    // The components of the tag are tested: Unit Triangle Index, Unit Offset, Cluster Index
    // The volume referred to by the tag is compared to the volume instanced by the query

    // WHAT THE TEST DOES:
    // An aabbox overlap query is executed, which encompasses a single quad triangle within the mesh.
    // The triangle volume referred to by the corresponding result tag is then obtained
    // and compared against the instanced volume. The two should be identical.

    const uint32_t RESBUFFERMAXSIZE = 1u;
    const uint32_t STACKSIZE = 1u;

    // Create aggregate volume in order to run the query
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New(quadMesh);

    // Create an aabbox which encompase
    rw::collision::AABBox volBBox(rwpmath::Vector3(0.0f, -0.1f, 0.0f),
                                  rwpmath::Vector3(0.5f, 0.1f, 0.5f));

    // Create a clusteredmesh volume pointer pointer
    const Volume *volArray[1] = { &*aggVol };

    // Create a VolumeBBoxQuery object with space for a single result
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(STACKSIZE, RESBUFFERMAXSIZE);
    // Initialize the query with the clusteredmesh volume and bounding box
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    // Attempt to pick up the second overlap
    EATESTAssert(p_VBBQ->GetOverlaps(), "Query should return at least 1 overlap");

    // Get the volume reference results buffer
    const VolRef *volRef = p_VBBQ->GetOverlapResultsBuffer();
    // Get the number of query results
    const uint32_t numResults = p_VBBQ->GetOverlapResultsBufferCount();

    // Check for the single result
    EATESTAssert(1 == numResults, "Num query results should be 1");

    // Get the child index
    uint32_t childIndex = quadMesh->GetChildIndexFromTag(volRef[0].tag);

    // Check the components of the child index.
    EATESTAssert(1 == quadMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex), "Unit triangle index should be 0");
    EATESTAssert(0 == quadMesh->GetClusterIndexFromChildIndex(childIndex), "Cluster index should be 0");
    EATESTAssert(0 == quadMesh->GetUnitOffsetFromChildIndex(childIndex), "Unit offset should be 0");

    // Get the Triangle Volume from the child index
    rw::collision::TriangleVolume * childIndexVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
    quadMesh->GetVolumeFromChildIndex(*childIndexVolume,
                                      childIndex);

    // Get the instanced triangle volume from the query
    const rw::collision::TriangleVolume * instancedVolume = static_cast<const rw::collision::TriangleVolume*>(volRef->volume);

    // Compare the instanced volume to the volume retrieved through the child index
    AssertTrianglesTheSameExcludingFlags(childIndexVolume, instancedVolume);

    // Deallocate resources
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(childIndexVolume);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(p_VBBQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
}

#endif // !defined(EA_PLATFORM_PS3_SPU)
