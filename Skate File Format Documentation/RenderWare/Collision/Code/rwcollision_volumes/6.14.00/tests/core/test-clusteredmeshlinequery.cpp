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

// Unit tests for clustered mesh line queries
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.

class TestClusteredMeshLineQuery: public ClusteredMeshTest_Base
{
public:
    virtual void Initialize()
    {
        SuiteName("TestClusteredMeshLineQuery");

#define CLUSTERED_MESH_TEST(F, D) EATEST_REGISTER(#F, D, TestClusteredMeshLineQuery, F)

        CLUSTERED_MESH_TEST(TestLineQuery, "Test a VolumeLineQuery against a clustered mesh");
        CLUSTERED_MESH_TEST(TestLineQueryRestart, "Test a VolumeLineQuery against a clustered mesh");

#if !defined(EA_PLATFORM_PS3_SPU)
        CLUSTERED_MESH_TEST(TestTriangleChildIndex,
            "Test the triangle child index returned from a VolumeLineQuery against a clustered mesh");
        CLUSTERED_MESH_TEST(TestQuadFirstTriangleChildIndex,
            "Test the quad-first-triangle child index returned from a VolumeLineQuery against a clustered mesh");
        CLUSTERED_MESH_TEST(TestQuadSecondTriangleChildIndex,
            "Test the quad-second-triangle child index returned from a VolumeLineQuery against a clustered mesh");
#endif // !defined(EA_PLATFORM_PS3_SPU)
    }

private:

    

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
#endif // !defined(EA_PLATFORM_PS3_SPU)

    void TestLineQuery();
    void TestLineQueryRestart();    
#if !defined(EA_PLATFORM_PS3_SPU)
    void TestTriangleChildIndex();
    void TestQuadFirstTriangleChildIndex();
    void TestQuadSecondTriangleChildIndex();
#endif // !defined(EA_PLATFORM_PS3_SPU)

} TestClusteredMeshLineQuerySingleton;

void TestClusteredMeshLineQuery::TestLineQuery()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 32;

    // Create line query to use against clustered mesh
    VolumeLineQuery* clusteredMeshLineQuery = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(STACKSIZE, RESBUFFERSIZE);
    EATESTAssert(clusteredMeshLineQuery, "Failed to create clustered mesh line query.");

    // Create line query to use against triangle
    VolumeLineQuery* triangleLineQuery = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(STACKSIZE, 1u);
    EATESTAssert(triangleLineQuery, "Failed to create triangle line query.");

    for (uint32_t cm = 0; cm < EAArrayCount(g_clusteredMeshFilenames); ++cm)
    {
        //Load ClusteredMesh
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(g_clusteredMeshFilenames[cm]);
        EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");

        AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
        ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());

        // Create bbox query to extract all triangle from the clustered mesh
        VolumeBBoxQuery* bboxQuery = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(STACKSIZE, mesh->GetVolumeCount() * 2);
        EATESTAssert(bboxQuery, "Failed to create BBox query.");        

        //mesh transform
        float cos45 = 0.707106781f;
        float sin45 = 0.707106781f;
        const rwpmath::Matrix44Affine transformMatrix(GetVector3_XAxis(), 
            rwpmath::Vector3(0.0f, cos45, -sin45), 
            rwpmath::Vector3(0.0f, sin45, cos45),  
            rwpmath::Vector3(0.0f, 0.123456f, 0.0f));

        LineQueryTester(clusteredMeshVolume, clusteredMeshVolume, &transformMatrix, bboxQuery, triangleLineQuery, clusteredMeshLineQuery, 1.0f, 1.0e-3f);

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(bboxQuery);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
    }
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(triangleLineQuery);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(clusteredMeshLineQuery);
}

void TestClusteredMeshLineQuery::TestLineQueryRestart()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE_MAX = 5;

    for (uint32_t cm = 0; cm < EAArrayCount(g_clusteredMeshFilenames); ++cm)
    {
        //Load ClusteredMesh
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(g_clusteredMeshFilenames[cm]);
        EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");

        AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
        ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());

        RestartingLineQueryTester(clusteredMeshVolume, mesh->GetVolumeCount() * 2, STACKSIZE, RESBUFFERSIZE_MAX);

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
    }      
}

#if !defined(EA_PLATFORM_PS3_SPU)
void TestClusteredMeshLineQuery::TestTriangleChildIndex()
{
    // TEST OVERVIEW:
    // This test tests the line query overlap result tag.
    // The components of the tag are tested: Unit Triangle Index, Unit Offset, Cluster Index
    // The volume referred to by the tag is compared to the volume instanced by the query

    // WHAT THE TEST DOES:
    // A line intersection query is executed, which intersects a single triangle within the mesh.
    // The triangle volume referred to by the corresponding result tag is then obtained
    // and compared against the instanced volume. The two should be identical.


    const uint32_t RESBUFFERMAXSIZE = 1u;
    const uint32_t STACKSIZE = 1u;

    // Create aggregate volume in order to run the query
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New(triangleMesh);

    // Create the line start and end points
    rwpmath::Vector3 lineStart = rwpmath::Vector3(0.1f, 1.0f, 0.1f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(0.1f, -1.0f, 0.1f);

    // Create a clusteredmesh volume pointer pointer
    const Volume *volArray[1] = { &*aggVol };

    // Create a VolumeLineQuery object with space for a single result
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(STACKSIZE, RESBUFFERMAXSIZE);
    // Initialize the query with the clusteredmesh volume and bounding box
    p_VLQ->InitQuery(&volArray[0], NULL, 1, lineStart, lineEnd);

    // Attempt to pick up the first overlap
    EATESTAssert(p_VLQ->GetAllIntersections(), "Query should return 1 intersection");

    // Get the volume reference results buffer
    const VolumeLineSegIntersectResult *queryResult = p_VLQ->GetIntersectionResultsBuffer();

    // Get the child index
    uint32_t childIndex = triangleMesh->GetChildIndexFromTag(queryResult->vRef.tag);

    // Check the components of the child index.
    EATESTAssert(0 == triangleMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex), "Unit triangle index should be 0");
    EATESTAssert(0 == triangleMesh->GetClusterIndexFromChildIndex(childIndex), "Cluster index should be 0");
    EATESTAssert(0 == triangleMesh->GetUnitOffsetFromChildIndex(childIndex), "Unit offset should be 0");

    // Get the Triangle Volume from the child index
    rw::collision::TriangleVolume * childIndexVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
    triangleMesh->GetVolumeFromChildIndex(*childIndexVolume,
                                          childIndex);

    // Get the instanced triangle volume from the query
    const rw::collision::TriangleVolume * instancedVolume = static_cast<const rw::collision::TriangleVolume*>(queryResult->vRef.volume);

    // Compare the instanced volume to the volume retrieved through the child index
    AssertTrianglesTheSameExcludingFlags(childIndexVolume, instancedVolume);

    // Deallocate resources
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(childIndexVolume);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(p_VLQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
}

void TestClusteredMeshLineQuery::TestQuadFirstTriangleChildIndex()
{
    // TEST OVERVIEW:
    // This test tests the line query overlap result tag.
    // The components of the tag are tested: Unit Triangle Index, Unit Offset, Cluster Index
    // The volume referred to by the tag is compared to the volume instanced by the query

    // WHAT THE TEST DOES:
    // A line intersection query is executed, which intersects a single quad triangle within the mesh.
    // The triangle volume referred to by the corresponding result tag is then obtained
    // and compared against the instanced volume. The two should be identical.

    const uint32_t RESBUFFERMAXSIZE = 1u;
    const uint32_t STACKSIZE = 1u;

    // Create aggregate volume in order to run the query
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New(quadMesh);

    // Create the line start and end points
    rwpmath::Vector3 lineStart = rwpmath::Vector3(0.1f, 1.0f, 0.1f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(0.1f, -1.0f, 0.1f);

    // Create a clusteredmesh volume pointer pointer
    const Volume *volArray[1] = { &*aggVol };

    // Create a VolumeLineQuery object with space for a single result
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(STACKSIZE, RESBUFFERMAXSIZE);
    // Initialize the query with the clusteredmesh volume and bounding box
    p_VLQ->InitQuery(&volArray[0], NULL, 1, lineStart, lineEnd);

    // Attempt to pick up the first overlap
    EATESTAssert(p_VLQ->GetAllIntersections(), "Query should return 1 intersection");

    // Get the volume reference results buffer
    const VolumeLineSegIntersectResult *queryResult = p_VLQ->GetIntersectionResultsBuffer();

    // Get the child index
    uint32_t childIndex = quadMesh->GetChildIndexFromTag(queryResult->vRef.tag);

    // Check the components of the child index.
    EATESTAssert(0 == quadMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex), "Unit triangle index should be 0");
    EATESTAssert(0 == quadMesh->GetClusterIndexFromChildIndex(childIndex), "Cluster index should be 0");
    EATESTAssert(0 == quadMesh->GetUnitOffsetFromChildIndex(childIndex), "Unit offset should be 0");

    // Get the Triangle Volume from the child index
    rw::collision::TriangleVolume * childIndexVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
    quadMesh->GetVolumeFromChildIndex(*childIndexVolume,
                                      childIndex);

    // Get the instanced triangle volume from the query
    const rw::collision::TriangleVolume * instancedVolume = static_cast<const rw::collision::TriangleVolume*>(queryResult->vRef.volume);

    // Compare the instanced volume to the volume retrieved through the child index
    AssertTrianglesTheSameExcludingFlags(childIndexVolume, instancedVolume);

    // Deallocate resources
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(childIndexVolume);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(p_VLQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
}


void TestClusteredMeshLineQuery::TestQuadSecondTriangleChildIndex()
{
    // TEST OVERVIEW:
    // This test tests the line query overlap result tag.
    // The components of the tag are tested: Unit Triangle Index, Unit Offset, Cluster Index
    // The volume referred to by the tag is compared to the volume instanced by the query

    // WHAT THE TEST DOES:
    // A line intersection query is executed, which intersects a single quad triangle within the mesh.
    // The triangle volume referred to by the corresponding result tag is then obtained
    // and compared against the instanced volume. The two should be identical.

    const uint32_t RESBUFFERMAXSIZE = 1u;
    const uint32_t STACKSIZE = 1u;

    // Create aggregate volume in order to run the query
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New(quadMesh);

    // Create the line start and end points
    rwpmath::Vector3 lineStart = rwpmath::Vector3(0.7f, 1.0f, 0.7f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(0.7f, -1.0f, 0.7f);

    // Create a clusteredmesh volume pointer pointer
    const Volume *volArray[1] = { &*aggVol };

    // Create a VolumeLineQuery object with space for a single result
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(STACKSIZE, RESBUFFERMAXSIZE);
    // Initialize the query with the clusteredmesh volume and bounding box
    p_VLQ->InitQuery(&volArray[0], NULL, 1, lineStart, lineEnd);

    // Attempt to pick up the first overlap
    EATESTAssert(p_VLQ->GetAllIntersections(), "Query should return 1 intersection");

    // Get the volume reference results buffer
    const VolumeLineSegIntersectResult *queryResult = p_VLQ->GetIntersectionResultsBuffer();

    // Get the child index
    uint32_t childIndex = quadMesh->GetChildIndexFromTag(queryResult->vRef.tag);

    // Check the components of the child index.
    EATESTAssert(1 == quadMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex), "Unit triangle index should be 0");
    EATESTAssert(0 == quadMesh->GetClusterIndexFromChildIndex(childIndex), "Cluster index should be 0");
    EATESTAssert(0 == quadMesh->GetUnitOffsetFromChildIndex(childIndex), "Unit offset should be 0");

    // Get the Triangle Volume from the child index
    rw::collision::TriangleVolume * childIndexVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
    quadMesh->GetVolumeFromChildIndex(*childIndexVolume,
                                      childIndex);

    // Get the instanced triangle volume from the query
    const rw::collision::TriangleVolume * instancedVolume = static_cast<const rw::collision::TriangleVolume*>(queryResult->vRef.volume);

    // Compare the instanced volume to the volume retrieved through the child index
    AssertTrianglesTheSameExcludingFlags(childIndexVolume, instancedVolume);

    // Deallocate resources
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(childIndexVolume);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(p_VLQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
}


#endif // !defined(EA_PLATFORM_PS3_SPU)
