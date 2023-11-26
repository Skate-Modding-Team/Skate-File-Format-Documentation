// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include "clusteredmeshtest_base.hpp"

using namespace rwpmath;
using namespace rw::collision;

// Unit tests for ScaledClusteredMeshes.

#define COURTYARD  "courtyard.dat"
#define SKATEMESH_COMPRESSED_QUADS_IDS  "skatemesh_compressed_quads_ids.dat"
#define LEAVES_SPANNING_CLUSTERS  "mesh_leaves_spanning_clusters.dat"

const char *g_clusteredMeshFilenames[] =
{
    COURTYARD,
    SKATEMESH_COMPRESSED_QUADS_IDS,
    LEAVES_SPANNING_CLUSTERS
};

class TestScaledClusteredMesh: public ClusteredMeshTest_Base
{
    virtual void Initialize()
    {
        SuiteName("TestScaledClusteredMesh");

#define SCALED_CLUSTERED_MESH_TEST(F, D) EATEST_REGISTER(#F, D, TestScaledClusteredMesh, F)
#if !defined(EA_PLATFORM_PS3_SPU)
        SCALED_CLUSTERED_MESH_TEST(TestConstruction, "Tests we can construct a ScaledClusteredMesh from a clustered mesh and a scale");
        SCALED_CLUSTERED_MESH_TEST(TestSetScale, "Tests we can set and get the scale on a ScaledClusteredMesh");
        SCALED_CLUSTERED_MESH_TEST(TestGetClusteredMesh, "Tests we can get the clustered mesh stored in a ScaledClusteredMesh");
        SCALED_CLUSTERED_MESH_TEST(TestGetSizeThis, "Tests we can get the size of a ScaledClusteredMesh object");
        SCALED_CLUSTERED_MESH_TEST(TestUpdateThis, "Tests we can update the bounding box of a ScaledClusteredMesh");
        SCALED_CLUSTERED_MESH_TEST(TestLineQuery, "Tests line queries against a ScaledClusteredMesh");        
        SCALED_CLUSTERED_MESH_TEST(TestLineQueryRestart, "Tests a line query can be correctly restarted");
        SCALED_CLUSTERED_MESH_TEST(TestBBoxQuery, "Tests BBox queries against a ScaledClusteredMesh");
        SCALED_CLUSTERED_MESH_TEST(TestBBoxQueryInMappedArrayWithPrimitives,
            "Test a BBoxQuery against a clustered mesh where the mesh is in a mapped array with some primitives");
#endif // !defined(EA_PLATFORM_PS3_SPU)
    }

#if !defined(EA_PLATFORM_PS3_SPU)
    void TestConstruction();
    void TestSetScale();
    void TestGetClusteredMesh();
    void TestGetSizeThis();
    void TestUpdateThis();
    void TestLineQuery();
    void TestLineQueryRestart();
    void TestBBoxQuery();
    void TestBBoxQueryInMappedArrayWithPrimitives();
#endif // !defined(EA_PLATFORM_PS3_SPU)

} TestScaledClusteredMeshSingleton;

#if !defined(EA_PLATFORM_PS3_SPU)
void TestScaledClusteredMesh::TestConstruction()
{    
    float triangleScale = 30.0f;
    
    //Allocate a ScaledClusteredMesh from a the triangle ClusteredMesh
    EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(triangleMesh, triangleScale);
    void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
    rw::collision::ScaledClusteredMesh* scaledTriangleMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, triangleMesh, triangleScale);

    EATESTAssert(scaledTriangleMesh != NULL, "Failed to allocate memory for scaledTriangleMesh");

    EATESTAssert(scaledTriangleMesh->GetType() == RWCOBJECTTYPE_SCALEDCLUSTEREDMESH, "Incorrect type for scaledClusteredMesh");

    //Free scaledTriangleMesh
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledTriangleMesh);
}

void TestScaledClusteredMesh::TestSetScale()
{    
    float triangleScale = 30.0f;
    float quadScale = 30.0f;

    rw::collision::ScaledClusteredMesh* scaledTriangleMesh;
    rw::collision::ScaledClusteredMesh* scaledQuadMesh;

    //Allocate a ScaledClusteredMesh using an allocator from the triangle mesh
    {
        EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(triangleMesh, triangleScale);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
        scaledTriangleMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, triangleMesh, triangleScale);
    }
    //Allocate a ScaledClusteredMesh using an allocator from the quad mesh
    {
        EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(quadMesh, quadScale);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
        scaledQuadMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, quadMesh, quadScale);
    }

    EATESTAssert(scaledTriangleMesh != NULL, "Failed to allocate memory for scaledTriangleMesh");
    EATESTAssert(scaledQuadMesh != NULL, "Failed to allocate memory for scaledQuadMesh");

    EATESTAssert(scaledTriangleMesh->GetScale() == triangleScale, "Scale was incorrectly set or retrived from ScaledClusteredMesh");
    EATESTAssert(scaledQuadMesh->GetScale()     == quadScale,     "Scale was incorrectly set or retrived from ScaledClusteredMesh");

    //Free scaledTriangleMesh
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledTriangleMesh);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledQuadMesh);
}

void TestScaledClusteredMesh::TestGetClusteredMesh()
{
    float triangleScale = 30.0f;
    float quadScale = 30.0f;

    rw::collision::ScaledClusteredMesh* scaledTriangleMesh;
    rw::collision::ScaledClusteredMesh* scaledQuadMesh;

    //Allocate a ScaledClusteredMesh using an allocator from the triangle mesh
    {
        EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(triangleMesh, triangleScale);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
        scaledTriangleMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, triangleMesh, triangleScale);
    }
    //Allocate a ScaledClusteredMesh using an allocator from the quad mesh
    {
        EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(quadMesh, quadScale);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
        scaledQuadMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, quadMesh, quadScale);
    }

    EATESTAssert(scaledTriangleMesh != NULL, "Failed to allocate memory for scaledTriangleMesh");
    EATESTAssert(scaledQuadMesh != NULL, "Failed to allocate memory for scaledQuadMesh");

    EATESTAssert(scaledTriangleMesh->GetClusteredMesh() == triangleMesh, "ClusteredMesh was incorrectly set or retrived from ScaledClusteredMesh");
    EATESTAssert(scaledQuadMesh->GetClusteredMesh()     == quadMesh,     "ClusteredMesh was incorrectly set or retrived from ScaledClusteredMesh");

    //Free scaledTriangleMesh
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledTriangleMesh);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledQuadMesh);   
}

void TestScaledClusteredMesh::TestGetSizeThis()
{       
    float triangleScale = 30.0f;
 
    rw::collision::ScaledClusteredMesh* scaledTriangleMesh;
  
    //Allocate a ScaledClusteredMesh using an allocator from the triangle mesh
    EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(triangleMesh, triangleScale);
    void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
    scaledTriangleMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, triangleMesh, triangleScale);
    
    uint32_t meshSize = scaledTriangleMesh->GetSizeThis();

    EATESTAssert(meshSize == sa.GetSize(), "Incorrect size returned for ScaledClusteredMesh");

    //Free scaledTriangleMesh
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledTriangleMesh);  
}

void TestScaledClusteredMesh::TestUpdateThis()
{
    float triangleScale = 1.0f;
    float quadScale = 1.0f;

    rw::collision::ScaledClusteredMesh* scaledTriangleMesh;
    rw::collision::ScaledClusteredMesh* scaledQuadMesh;

    //Allocate a ScaledClusteredMesh using an allocator from the triangle mesh
    {
        EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(triangleMesh, triangleScale);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
        scaledTriangleMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, triangleMesh, triangleScale);
    }
    //Allocate a ScaledClusteredMesh using an allocator from the quad mesh
    {
        EA::Physics::SizeAndAlignment sa = ScaledClusteredMesh::GetResourceDescriptor(quadMesh, quadScale);
        void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
        scaledQuadMesh = rw::collision::ScaledClusteredMesh::Initialize(mem, quadMesh, quadScale);
    }

    //Get the size of the bbox of the triangle mesh
    triangleMesh->UpdateThis();
    quadMesh->UpdateThis();
    rw::collision::AABBox triAABBox  = triangleMesh->GetBBox();
    rw::collision::AABBox quadAABBox = quadMesh->GetBBox();

    //get the size of the clustered mesh AABBox
    scaledTriangleMesh->UpdateThis();
    scaledQuadMesh->UpdateThis();
    EATESTAssert(scaledTriangleMesh->GetBBox().Max() == triAABBox.Max() && 
        scaledTriangleMesh->GetBBox().Min() == triAABBox.Min(),  
        "Incorrect bounding box for ScaledClusteredMesh");    
    EATESTAssert(scaledQuadMesh->GetBBox().Max() == quadAABBox.Max() && 
        scaledQuadMesh->GetBBox().Min() == quadAABBox.Min(), 
        "Incorrect bounding box for ScaledClusteredMesh");

    //change the scale and check bbox
    scaledTriangleMesh->SetScale(0.1f);
    scaledQuadMesh->SetScale(10.0f);  

    scaledTriangleMesh->UpdateThis();
    scaledQuadMesh->UpdateThis();
    EATESTAssert(scaledTriangleMesh->GetBBox().Max() == triAABBox.Max()*0.1f && 
        scaledTriangleMesh->GetBBox().Min() == triAABBox.Min()*0.1f,  
        "Incorrect bounding box for ScaledClusteredMesh");
    EATESTAssert(scaledQuadMesh->GetBBox().Max() == quadAABBox.Max()*10.0f && 
        scaledQuadMesh->GetBBox().Min() == quadAABBox.Min()*10.0f, 
        "Incorrect bounding box for ScaledClusteredMesh");

    //Free scaledTriangleMesh
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledTriangleMesh);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledQuadMesh);
}

//Idea is we perform a line query against the scaled clustered mesh to obtain all hits.
//We then perform a line query against each SCALED triangle in the scaled clustered mesh.
//We confirm that the hits are the same.
//Since this unit test is a consistency test we do not tests all clustered mesh assets in our library,
//we only test the first. The remaining mesh assets are line-query tested via the test-clusteredmeshlinequery
//test suite.
void TestScaledClusteredMesh::TestLineQuery()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 32;

    float scaleList[] = {1.0f, 100.0f, 0.1f};
    const uint16_t NUMSCALES = EAArrayCount(scaleList);
    
    // Create line query to use against clustered mesh
    VolumeLineQuery* scaledClusteredMeshLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(STACKSIZE, RESBUFFERSIZE);
    EATESTAssert(scaledClusteredMeshLQ, "Failed to create scaled clustered mesh line query.");

    // Create line query to use against triangle
    VolumeLineQuery* triangleLineQuery = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(STACKSIZE, 1u);
    EATESTAssert(triangleLineQuery, "Failed to create triangle line query.");

    //Load ClusteredMesh
    Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(g_clusteredMeshFilenames[0]);
    EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");

    AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
    ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());

    // Create bbox query to extract all triangle from the clustered mesh
    VolumeBBoxQuery* bboxQuery = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(STACKSIZE, mesh->GetVolumeCount() * 2);
    EATESTAssert(bboxQuery, "Failed to create BBox query.");        
        
    //Loop over a range of scales
    for(uint32_t scaleID = 0; scaleID != NUMSCALES; scaleID++)
    {
        //scaled mesh transform
        float cos45 = 0.707106781f;
        float sin45 = 0.707106781f;
        const rwpmath::Matrix44Affine transformMatrix(GetVector3_XAxis(), 
            rwpmath::Vector3(0.0f, cos45, -sin45), 
            rwpmath::Vector3(0.0f, sin45, cos45), 
            rwpmath::Vector3(0.0f, 0.123456f, 0.0f));
           
        float scale = scaleList[scaleID];

        //create a scaled clustered mesh
        ScaledClusteredMesh* scaledMesh = EA::Physics::UnitFramework::Creator<ScaledClusteredMesh>().New(mesh, scale);

        //set up the query volumes for the scaled clustered mesh            
        Volume * scaledMeshVolume = EA::Physics::UnitFramework::Creator<rw::collision::AggregateVolume>().New(scaledMesh);            
    
        //A smaller tolerance is used in scaled mesh tests. This allows for a loss in precision when scaling
        //triangles that are small and far from the origin.
        LineQueryTester(clusteredMeshVolume, scaledMeshVolume, &transformMatrix, bboxQuery, triangleLineQuery, scaledClusteredMeshLQ, scale, 1.0e-2f);
            
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMesh);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMeshVolume);

    } // loop over each scaled clustered mesh

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(bboxQuery);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledClusteredMeshLQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(triangleLineQuery);
}

void TestScaledClusteredMesh::TestLineQueryRestart()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE_MAX = 5;
    const float scale = 2.0f;

    for (uint32_t cm = 0; cm < EAArrayCount(g_clusteredMeshFilenames); ++cm)
    {
        //Load ClusteredMesh
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(g_clusteredMeshFilenames[cm]);
        EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");

        AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
        ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());

        //create a scaled clustered mesh
        ScaledClusteredMesh* scaledMesh = EA::Physics::UnitFramework::Creator<ScaledClusteredMesh>().New(mesh, scale);

        //set up the query volumes for the scaled clustered mesh            
        Volume * scaledMeshVolume = EA::Physics::UnitFramework::Creator<rw::collision::AggregateVolume>().New(scaledMesh);     

        RestartingLineQueryTester(scaledMeshVolume, mesh->GetVolumeCount() * 2, STACKSIZE, RESBUFFERSIZE_MAX);

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMeshVolume);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMesh);

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
    }
}

void TestScaledClusteredMesh::TestBBoxQuery()
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
    const uint32_t NUM_SCALES = 6;
    float scaleList[NUM_SCALES] = {1.0f, 0.5f, 2.0f, 10.0f, 100.0f, 0.1f};
    
    for (uint32_t cm = 0; cm < EAArrayCount(g_clusteredMeshFilenames); ++cm)
    {
        // Load ClusteredMesh
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(g_clusteredMeshFilenames[cm]);
        EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");

        AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
        ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());

        for(uint32_t scaleID = 0; scaleID != NUM_SCALES; scaleID++)
        {    
            float scale = scaleList[scaleID];

            //scaled mesh transform
            float cos45 = 0.707106781f;
            float sin45 = 0.707106781f;
            const rwpmath::Matrix44Affine transformMatrix(GetVector3_XAxis(), 
                rwpmath::Vector3(0.0f, cos45, -sin45), 
                rwpmath::Vector3(0.0f, sin45, cos45), 
                rwpmath::Vector3(0.0f, scale*0.123456f, 0.0f));

            //create a scaled clustered mesh
            ScaledClusteredMesh* scaledMesh = EA::Physics::UnitFramework::Creator<ScaledClusteredMesh>().New(mesh, scale);

            //set up the query volumes for the scaled clustered mesh            
            Volume * scaledMeshVolume = EA::Physics::UnitFramework::Creator<rw::collision::AggregateVolume>().New(scaledMesh);     

            RestartingBBoxQueryTester(scaledMeshVolume, &transformMatrix, scaledMesh->GetVolumeCount() * 2, STACKSIZE, RESBUFFERMAXSIZE);

            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMeshVolume);
            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMesh);
        }
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
    }
}

void TestScaledClusteredMesh::TestBBoxQueryInMappedArrayWithPrimitives()
{
    //Load ClusteredMesh
    Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(COURTYARD);
    EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");
    
    AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
    ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());

    const uint32_t NUM_SCALES = 6;
    float scaleList[NUM_SCALES] = {1.0f, 0.5f, 2.0f, 10.0f, 100.0f, 0.1f};
    
    for(uint32_t scaleID = 0; scaleID != NUM_SCALES; scaleID++)
    {        
        float scale = scaleList[scaleID];
        //create a scaled clustered mesh
        ScaledClusteredMesh* scaledMesh = EA::Physics::UnitFramework::Creator<ScaledClusteredMesh>().New(mesh, scale);

        //set up the query volumes for the scaled clustered mesh            
        Volume * scaledMeshVolume = EA::Physics::UnitFramework::Creator<rw::collision::AggregateVolume>().New(scaledMesh);     

        BBoxQueryInMappedArrayWithPrimitivesTester(scaledMeshVolume);

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMeshVolume);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(scaledMesh);       
    }
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
}

#endif // !defined(EA_PLATFORM_PS3_SPU)


