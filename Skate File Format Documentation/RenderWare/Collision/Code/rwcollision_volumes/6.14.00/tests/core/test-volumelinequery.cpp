// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <rw/collision/libcore.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;
using namespace rwpmath;

//*********************DEFINES*********************

#define CUBE_HALFLENGTH 0.5f
#define SPHERE_RADIUS 0.5f

// ***********************************************************************************************************
// Test suite

class TestVolumeLineQuery : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestVolumeLineQuery");
    
        // Test individual Methods
        EATEST_REGISTER("GetNearestIntersection", "GetNearestIntersection", TestVolumeLineQuery, GetNearestIntersection);
        EATEST_REGISTER("GetOverlapsAggInterTwoPrimInter", "GetOverlapsAggInterTwoPrimInter", TestVolumeLineQuery, GetOverlapsAggInterTwoPrimInter);
        EATEST_REGISTER("GetOverlapsAggInterNoPrimInter", "GetOverlapsAggInterNoPrimInter", TestVolumeLineQuery, GetOverlapsAggInterNoPrimInter);
        EATEST_REGISTER("GetOverlapsPrimitivesOverflowTwoAggInter", "GetOverlapsPrimitivesOverflowTwoAggInter", TestVolumeLineQuery, GetOverlapsPrimitivesOverflowTwoAggInter);
        EATEST_REGISTER("GetOverlapsStackOverflowTwoAggInter", "GetOverlapsStackOverflowTwoAggInter", TestVolumeLineQuery, GetOverlapsStackOverflowTwoAggInter);
        EATEST_REGISTER("GetOverlapsOverflowTwoPrimInter", "GetOverlapsOverflowTwoPrimInter", TestVolumeLineQuery, GetOverlapsOverflowTwoPrimInter);
        EATEST_REGISTER("GetOverlapsCorrectOnePrimInterResults", "GetOverlapsCorrectOnePrimInterResults", TestVolumeLineQuery, GetOverlapsCorrectOnePrimInterResults);
        EATEST_REGISTER("GetOverlapsCorrectTwoPrimInterResults", "GetOverlapsCorrectTwoPrimInterResults", TestVolumeLineQuery, GetOverlapsCorrectTwoPrimInterResults);
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        Volume::InitializeVTable();
    }

    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        Volume::ReleaseVTable();
        tests::TestSuiteBase::TeardownSuite();
    }

private:
    // Test individual Methods
    void GetNearestIntersection();
    void GetOverlapsAggInterTwoPrimInter();
    void GetOverlapsAggInterNoPrimInter();
    void GetOverlapsPrimitivesOverflowTwoAggInter();
    void GetOverlapsStackOverflowTwoAggInter();
    void GetOverlapsOverflowTwoPrimInter();
    void GetOverlapsCorrectOnePrimInterResults();
    void GetOverlapsCorrectTwoPrimInterResults();

} TestVolumeLineQuerySingleton;

/*
Test for BUG#31538: VolumeLineQuery::GetNearestIntersection can return incorrect results

For testing purposes, we create a very simple clustered mesh with two completely overlapping
leaf nodes having 6 triangles in each. The triangles have normal pointing down (-Y) and are arranged in a stack
with heights Y=0, 1, ... 5 in first leaf and Y=6, 7, ... 11 in second leaf.
*/
void TestVolumeLineQuery::GetNearestIntersection()
{
    const uint32_t numTris = 12;
    const uint32_t numVerts = 3*numTris;

    // Cluster unit data (no edge cos or IDs)
    struct UnitData 
    {
        uint8_t type;
        uint8_t vertIndices[3];
    };

    // Cluster data size
    uint32_t unitDataSize = numTris*sizeof(UnitData);
    uint32_t clusterSize = 16 * (1 + numVerts) + unitDataSize; // 1QW of header data

    // Create the ClusteredMesh
    rw::collision::ClusteredMesh::ObjectDescriptor descriptor;

    float ext = 100.0f;
    descriptor.m_bbox = rw::collision::AABBox(Vector3(-ext,-ext,-ext), Vector3(ext,ext,ext));
    descriptor.m_clusterDataSize = clusterSize;
    descriptor.m_includeKDSubTrees = FALSE;
    descriptor.m_maxClusters = 1;
    descriptor.m_maxUnits = numTris;
    descriptor.m_numBranchNodes = 1;

    ClusteredMesh * clusteredMesh = EA::Physics::UnitFramework::Creator<ClusteredMesh>().New(descriptor);

    // Set up cluster data
    clusteredMesh->SetGroupIdSize(0);
    clusteredMesh->SetSurfaceIdSize(0);
    clusteredMesh->SetOneSided(false);

    ClusteredMeshCluster *cluster = clusteredMesh->AllocateNextCluster(clusterSize, numTris);

    cluster->totalSize = uint16_t(clusterSize);
    cluster->vertexCount = uint8_t(numVerts);
    cluster->compressionMode = ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    cluster->normalCount = 0;
    cluster->normalStart = uint16_t(numVerts); // Quadword offset from start of vertex array (normal list actually empty)
    cluster->unitCount = uint16_t(numTris);    
    cluster->unitDataStart = uint16_t(numVerts); // Quadword offset from start of vertex array
    cluster->unitDataSize = uint16_t(unitDataSize);

    Vector3 *verts = cluster->vertexArray;
    UnitData *units = (UnitData *)cluster->UnitData();
    for (uint32_t i=0; i<numTris; ++i)
    {
        uint32_t iv0 = 3*i;
        
        // Stack of triangles all pointing downwards (-y)
        verts[iv0]   = Vector3(0.0f, float(i), 0.0f);
        verts[iv0+1] = Vector3(1.0f, float(i), 0.0f);
        verts[iv0+2] = Vector3(0.0f, float(i), 1.0f);
        
        units[i].type = UNITTYPE_TRIANGLE;
        units[i].vertIndices[0] = uint8_t(iv0);
        units[i].vertIndices[1] = uint8_t(iv0+1);
        units[i].vertIndices[2] = uint8_t(iv0+2);
    }

    // Initialize the KDTree
    KDTreeBase *kdtree = clusteredMesh->GetKDTreeBase();
    kdtree->m_branchNodes[0].m_axis = 2;
    kdtree->m_branchNodes[0].m_parent = 0;
    kdtree->m_branchNodes[0].m_extents[0] =  ext; // 100% overlapping child nodes
    kdtree->m_branchNodes[0].m_extents[1] = -ext;
    kdtree->m_branchNodes[0].m_childRefs[0].m_content = 6;
    kdtree->m_branchNodes[0].m_childRefs[0].m_index = 0;
    kdtree->m_branchNodes[0].m_childRefs[1].m_content = 6;
    kdtree->m_branchNodes[0].m_childRefs[1].m_index = 6*sizeof(UnitData);

    // Store overall bounding box
    clusteredMesh->Update();

    // Confirm clustered mesh is ok
    EA_ASSERT(clusteredMesh->IsValid());

    AggregateVolume * aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New(clusteredMesh);
    const Matrix44Affine *matrixArray[1] = {0};
    const Volume *volArray[1] = {aggVol};

    // Line heading upwards through full stack of triangles, nearest should be the first one
    // Restrict to 5 results to make sure the query has to do several batches (bug meant that later batches could
    // overwrite result from first batch)
    {
        VolumeLineQuery * vlq = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(1u, 5u);
        Vector3 lineStart(0.2f, -1.0f, 0.2f);
        Vector3 lineEnd(0.2f, 12.0f, 0.2f);
        float fatness = 0.0f;
        vlq->InitQuery(volArray, matrixArray, 1, lineStart, lineEnd, fatness);
        VolumeLineSegIntersectResult *result = vlq->GetNearestIntersection();
        EATESTAssert(result != 0, "Line should intersect a triangle");
        EATESTAssert(IsSimilar(result->position, Vector3(0.2f, 0.0f, 0.2f), 1e-5f), "Position should be on first triangle");
        EATESTAssert(IsSimilar(result->normal, Vector3(0.0f, -1.0f, 0.0f), 1e-6f), "Normal should be pointing downwards");
    }

    // Line heading upwards through tris 0-5. Make the first tri the last (6th) in leaf node to be tested
    // The bug meant that any results beyond 5 from a single leaf node would get thrown away.
    {
        VolumeLineQuery * vlq = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(1u, 128u);
        UnitData temp = units[5];
        units[5] = units[0];
        units[0] = temp;
        Vector3 lineStart(0.2f, -0.5f, 0.2f);
        Vector3 lineEnd(0.2f, 5.5f, 0.2f);
        float fatness = 0.0f;
        vlq->InitQuery(volArray, matrixArray, 1, lineStart, lineEnd, fatness);
        VolumeLineSegIntersectResult *result = vlq->GetNearestIntersection();

        EATESTAssert(result != 0, "Line should intersect a triangle");
        EATESTAssert(IsSimilar(result->position, Vector3(0.2f, 0.0f, 0.2f), 1e-5f), "Position should be on triangle[5]");
        EATESTAssert(IsSimilar(result->normal, Vector3(0.0f, -1.0f, 0.0f), 1e-6f), "Normal should be pointing downwards");
    }

    // BUG#32249 - Triangle fat line intersect returns wrong normal
    // http://eahq-dt4.rws.ad.ea.com/scripts/texcel/DevTrack/devtrack.dll?IssueInfo?&IssueAccessID=MNWIDUPUQCWJBWOTW
    {
        VolumeLineQuery * vlq = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(1u, 128u);
        Vector3 lineStart(5.f, .6f, 5.0f);
        Vector3 lineEnd(0.f, .6f, 0.f);
        float fatness = 1.0f;
        vlq->InitQuery(volArray, matrixArray, 1, lineStart, lineEnd, fatness);
        VolumeLineSegIntersectResult *result = vlq->GetNearestIntersection();

        EATESTAssert(result != 0, "Line should intersect a triangle");
        EATESTAssert(IsSimilar(result->position, Vector3(.5f, 1.f, .5f), 1e-5f), "position not correct");
        EATESTAssert(IsSimilar(result->normal, Vector3(0.648073f, -0.4f, 0.648073f), 1e-5f), "normal not correct");
        EATESTAssert(IsSimilar(result->volParam, Vector3(0.5f, 0.5f, 0), 1e-5f), "volParam not correct");
        EATESTAssert(IsSimilar(result->lineParam, VecFloat(0.77038544f), 1e-5f), "lineParam not corerct");
    }

    // BUG#32249 - Triangle fat line intersect returns wrong normal
    // http://eahq-dt4.rws.ad.ea.com/scripts/texcel/DevTrack/devtrack.dll?IssueInfo?&IssueAccessID=MNWIDUPUQCWJBWOTW
    // We need to check rotated meshes also!
    {
        rwpmath::Matrix44Affine mtx = rwpmath::Matrix44AffineFromXRotationAngle(1.0f);
        VolumeLineQuery * vlq = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(1u, 128u);
        Vector3 lineStart(5.f, .6f, 5.0f);
        Vector3 lineEnd(0.f, .6f, 0.f);
        lineStart = rwpmath::TransformPoint(lineStart, mtx);
        lineEnd = rwpmath::TransformPoint(lineEnd, mtx);
        float fatness = 1.0f;
        const rwpmath::Matrix44Affine *mtxPtr = &mtx;
        vlq->InitQuery(volArray, &mtxPtr, 1, lineStart, lineEnd, fatness);
        VolumeLineSegIntersectResult *result = vlq->GetNearestIntersection();

        EATESTAssert(result != 0, "Line should intersect a triangle");
        EATESTAssert(IsSimilar(result->position, rwpmath::TransformPoint(Vector3(.5f, 1.f, .5f), mtx), 1e-5f), "position not correct");
        EATESTAssert(IsSimilar(result->normal, rwpmath::TransformVector(Vector3(0.648073f, -0.4f, 0.648073f), mtx), 1e-5f), "normal not correct");
        EATESTAssert(IsSimilar(result->volParam, Vector3(0.5f, 0.5f, 0), 1e-5f), "volParam not correct");
        EATESTAssert(IsSimilar(result->lineParam, VecFloat(0.77038544f), 1e-5f), "lineParam not corerct");
    }
}

// This function runs the following tests on the VolumeLineQuery: GetAllIntersections, GetAnyIntersection, GetNearestIntersection
//
// TEST SINGLE AGGREGATE VOLUME INTERSECTION WITH TWO PRIMITIVE VOLUME INTERSECTIONS
//
void TestVolumeLineQuery::GetOverlapsAggInterTwoPrimInter()
{
    const uint32_t NUMINPUTS = 1;
    const uint32_t NUMPRIMS = 2;
    const uint32_t STACKSIZE = 2;
    const uint32_t RESBUFFERSIZE = 2; 

    // Aggregate holding two primitive volumes
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( NUMPRIMS );
    // BoxVolume set as unit cube.
    for( int i = 0 ; i < (int)NUMPRIMS ; i++ )
    {
        BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( (uint16_t) i ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // Displace the two primitives so they don't intersect with the AABB
    p_SMAggregate->GetVolume( 0 )->SetLocalTransform(Matrix44Affine(
                                                                    1.0f, 0.0f, 0.0f,
                                                                    0.0f, 1.0f, 0.0f,
                                                                    0.0f, 0.0f, 1.0f,
                                                                    10.0f, 0.0f, 0.0f 
                                                                    ));

    p_SMAggregate->GetVolume( 1 )->SetLocalTransform(Matrix44Affine(
                                                                    1.0f, 0.0f, 0.0f,
                                                                    0.0f, 1.0f, 0.0f,
                                                                    0.0f, 0.0f, 1.0f,
                                                                    -10.0f, 0.0f, 0.0f 
                                                                    ));

    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();

    // Create aggregate volume using aggregate.
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );

    Volume* volArray[ 1 ] = { aggVol };

    // VolumeLineQuery object
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , rwpmath::Vector3(-20.0f, 0.0f, 0.0f), rwpmath::Vector3(20.0f, 0.0f, 0.0f), CUBE_HALFLENGTH );

    // Run the queries
    uint32_t numHits = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits == 2, "GetAllIntersections failed");
    VolumeLineSegIntersectResult * resultAllBuffer = p_VLQ->GetIntersectionResultsBuffer();
    EATESTAssert( resultAllBuffer,  "GetAllIntersection failed");
    EATESTAssert( resultAllBuffer[0].inputIndex == 0,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer[1].inputIndex == 0,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( IsSimilar(resultAllBuffer[0].lineParam, (10.f - 2 * CUBE_HALFLENGTH) / 40.0f) || IsSimilar(resultAllBuffer[1].lineParam, (10.f - 2 * CUBE_HALFLENGTH) / 40.0f),  "GetAllIntersection: lineParam check failed");
    EATESTAssert( IsSimilar(resultAllBuffer[0].lineParam, (10.f - 2 * CUBE_HALFLENGTH) / 40.0f) || IsSimilar(resultAllBuffer[1].lineParam, (10.f - 2 * CUBE_HALFLENGTH) / 40.0f),  "GetAllIntersection: lineParam check failed");
    EATESTAssert( resultAllBuffer[0].vRef.volume != resultAllBuffer[1].vRef.volume,  "GetAllIntersection: Intersecting volumes check failed");
    EATESTAssert( resultAllBuffer[0].vRef.tag != resultAllBuffer[1].vRef.tag,  "GetAllIntersection: Intersecting volumes check failed");
    EATESTAssert( resultAllBuffer[0].vRef.tag == 1 || resultAllBuffer[0].vRef.tag == 2, "GetAllIntersection: Intersecting volume tag check failed");
    EATESTAssert( resultAllBuffer[1].vRef.tag == 1 || resultAllBuffer[1].vRef.tag == 2, "GetAllIntersection: Intersecting volume tag check failed");
    EATESTAssert( resultAllBuffer[0].vRef.volume == p_SMAggregate->GetVolume( 0 ) || resultAllBuffer[1].vRef.volume == p_SMAggregate->GetVolume( 0 ),  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( resultAllBuffer[0].vRef.volume == p_SMAggregate->GetVolume( 1 ) || resultAllBuffer[1].vRef.volume == p_SMAggregate->GetVolume( 1 ),  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->GetAllIntersections() == 0, "Second GetAllIntersections failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , rwpmath::Vector3(-20.0f, 0.0f, 0.0f), rwpmath::Vector3(20.0f, 0.0f, 0.0f), CUBE_HALFLENGTH );

    VolumeLineSegIntersectResult * resultAnyBuffer = p_VLQ->GetAnyIntersection();
    EATESTAssert( resultAnyBuffer,  "GetAnyIntersection failed");
    EATESTAssert( resultAnyBuffer->inputIndex == 0,  "GetAnyIntersection: input volume index check failed");
    EATESTAssert( IsSimilar(resultAllBuffer->lineParam, (10.f - 2 * CUBE_HALFLENGTH) / 40.0f) || IsSimilar(resultAllBuffer->lineParam, (10.f - 2 * CUBE_HALFLENGTH) / 40.0f),  "GetAnyIntersection: lineParam check failed");
    EATESTAssert( resultAllBuffer->vRef.tag == 1 || resultAllBuffer->vRef.tag == 2, "GetAnyIntersection: Intersecting volume tag check failed");
    EATESTAssert( resultAnyBuffer->vRef.volume == p_SMAggregate->GetVolume( 0 ) || resultAnyBuffer->vRef.volume == p_SMAggregate->GetVolume( 1 ),  "GetAnyIntersection: Intersecting volume check failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , rwpmath::Vector3(-20.0f, 0.0f, 0.0f), rwpmath::Vector3(20.0f, 0.0f, 0.0f), CUBE_HALFLENGTH );

    VolumeLineSegIntersectResult * resultNearestBuffer = p_VLQ->GetNearestIntersection();
    EATESTAssert( resultNearestBuffer,  "GetNearestIntersection failed");
    EATESTAssert( resultAnyBuffer->inputIndex == 0,  "GetNearestIntersection: input volume index check failed");
    EATESTAssert( IsSimilar(resultAllBuffer->lineParam, (10.f - 2 * CUBE_HALFLENGTH) / 40.0f),  "GetNearestIntersection: lineParam check failed");
    EATESTAssert( resultAllBuffer->vRef.tag == 2, "GetNearestIntersection: Intersecting volume tag check failed");
    EATESTAssert( resultAnyBuffer->vRef.volume == p_SMAggregate->GetVolume( 1 ),  "GetNearestIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");
}

// This function runs the following tests on the VolumeLineQuery: GetAllIntersections, GetAnyIntersection, GetNearestIntersection
//
// TEST SINGLE AGGREGATE VOLUME INTERSECTION WITH NO PRIMITIVE VOLUME INTERSECTIONS
//
void TestVolumeLineQuery::GetOverlapsAggInterNoPrimInter()
{
    const uint32_t NUMINPUTS = 1;
    const uint32_t NUMPRIMS = 2;
    const uint32_t STACKSIZE = 2;
    const uint32_t RESBUFFERSIZE = 2; 

    // Aggregate holding two primitive volumes
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( NUMPRIMS );
    // BoxVolume set as unit cube.
    for( int i = 0 ; i < (int)NUMPRIMS ; i++ )
    {
        BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( (uint16_t) i ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // Displace the two primitives so they don't intersect with the AABB
    p_SMAggregate->GetVolume( 0 )->SetLocalTransform(Matrix44Affine(1.0f, 0.0f, 0.0f,
                                                                    0.0f, 1.0f, 0.0f,
                                                                    0.0f, 0.0f, 1.0f,
                                                                    10.0f, 0.0f, 0.0f 
                                                                    ));

    p_SMAggregate->GetVolume( 1 )->SetLocalTransform(Matrix44Affine(1.0f, 0.0f, 0.0f,
                                                                    0.0f, 1.0f, 0.0f,
                                                                    0.0f, 0.0f, 1.0f,
                                                                    -10.0f, 0.0f, 0.0f 
                                                                    ));

    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();

    // Create aggregate volume using aggregate.
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );

    Volume* volArray[ 1 ] = { aggVol };

    // VolumeLineQuery object
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -15.0f), rwpmath::Vector3(0.0f, 0.0f, 15.0f), 5.0f );

    // Run the queries
    uint32_t numHits = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits == 0, "GetAllIntersections failed");
    EATESTAssert(p_VLQ->Finished(), "Finished failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -15.0f), rwpmath::Vector3(0.0f, 0.0f, 15.0f), 5.0f );

    VolumeLineSegIntersectResult * resultAnyBuffer = p_VLQ->GetAnyIntersection();
    EATESTAssert( resultAnyBuffer == 0,  "GetAnyIntersection failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -15.0f), rwpmath::Vector3(0.0f, 0.0f, 15.0f), 5.0f );

    VolumeLineSegIntersectResult * resultNearestBuffer = p_VLQ->GetNearestIntersection();
    EATESTAssert( resultNearestBuffer == 0,  "GetNearestIntersection failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");
}

// This function runs the following tests on the VolumeLineQuery: GetAllIntersections, GetAnyIntersection, GetNearestIntersection
//
// TEST PRIMITIVES BUFFER OVERFLOW FUNCTIONALITY USING 2 INTERSECTING AGGREGATE VOLUMES AND A STACK BUFFER OF SIZE 1
//  
void TestVolumeLineQuery::GetOverlapsPrimitivesOverflowTwoAggInter()
{
    const uint32_t NUMINPUTS = 2;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    SimpleMappedArray* smaArray[ NUMINPUTS ];
    AggregateVolume* aggVol[ NUMINPUTS ];
    Volume* volArray[ NUMINPUTS ];

    for( int i = 0 ; i < (int)NUMINPUTS ; i++ )
    {
        // Aggregate holding a single intersecting primitive volume
        smaArray[ i ] = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
        BoxVolume::Initialize( EA::Physics::MemoryPtr( smaArray[ i ]->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
        smaArray[ i ]->UpdateThis();
        aggVol[ i ] = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( smaArray[ i ] );
        volArray[ i ] = aggVol[ i ];
    }

    // VolumeLineQuery object
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    // Run the queries
    VolumeLineSegIntersectResult * resultAnyBuffer = p_VLQ->GetAnyIntersection();
    EATESTAssert( resultAnyBuffer != NULL,  "GetAnyIntersection failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    VolumeLineSegIntersectResult * resultNearestBuffer = p_VLQ->GetNearestIntersection();
    EATESTAssert( resultNearestBuffer != NULL,  "GetNearestIntersection failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    // Run query first time
    uint32_t numHits0 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits0 == 1, "GetAllIntersections failed");
    EATESTAssert( p_VLQ->GetIntersectionResultsBuffer(),  "GetAllIntersection failed");
    VolumeLineSegIntersectResult resultAllBuffer0 = p_VLQ->GetIntersectionResultsBuffer()[0];
    EATESTAssert( resultAllBuffer0.inputIndex == 0 || resultAllBuffer0.inputIndex == 1,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer0.vRef.volume == smaArray[ 0 ]->GetVolume( 0 ) || resultAllBuffer0.vRef.volume == smaArray[ 1 ]->GetVolume( 0 ),  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( !p_VLQ->Finished(), "Finished failed");

    // Run query a second time
    uint32_t numHits1 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits1 == 1, "Second GetAllIntersections failed");
    EATESTAssert( p_VLQ->GetIntersectionResultsBuffer(),  "Second GetAllIntersection failed");
    VolumeLineSegIntersectResult resultAllBuffer1 = p_VLQ->GetIntersectionResultsBuffer()[0];
    EATESTAssert( resultAllBuffer1.inputIndex != resultAllBuffer0.inputIndex,  "Second GetAllIntersection: results check failed");
    EATESTAssert( resultAllBuffer1.vRef.volume != resultAllBuffer0.vRef.volume,  "Second GetAllIntersection: Volume results check failed");
    EATESTAssert( resultAllBuffer1.inputIndex == 0 || resultAllBuffer1.inputIndex == 1,  "Second GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer1.vRef.volume == smaArray[ 0 ]->GetVolume( 0 ) || resultAllBuffer1.vRef.volume == smaArray[ 1 ]->GetVolume( 0 ),  "Second GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Run query again
    uint32_t numHits2 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits2 == 0, "Third GetAllIntersections failed");
    EATESTAssert(p_VLQ->Finished(), "Finished failed");
}

// This function runs the following tests on the VolumeLineQuery: GetAllIntersections, GetAnyIntersection, GetNearestIntersection
//
// TEST STACK BUFFER OVERFLOW FUNCTIONALITY USING 2 INTERSECTING AGGREGATE VOLUMES AND A STACK BUFFER OF SIZE 1
//  
void TestVolumeLineQuery::GetOverlapsStackOverflowTwoAggInter()
{
    const uint32_t NUMPRIMS = 2;
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    Volume* volArray[ NUMINPUTS ];
    Volume* resultsArray[ NUMPRIMS ];

    for( int i = 0 ; i < (int)NUMINPUTS ; i++ )
    {
        // Aggregate holding a single intersecting primitive volume
        SimpleMappedArray * smaArray0 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( NUMPRIMS );
        for (uint16_t j=0; j<NUMPRIMS; ++j)
        {
            SimpleMappedArray * smaArray1 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
            AggregateVolume::Initialize( EA::Physics::MemoryPtr( smaArray0->GetVolume( j ) ), smaArray1 );
            {
                BoxVolume::Initialize( EA::Physics::MemoryPtr( smaArray1->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
                smaArray1->UpdateThis();
                resultsArray[j] = smaArray1->GetVolume( 0 );
            }
        }
        smaArray0->UpdateThis();
        volArray[ i ] = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( smaArray0 );
    }

    // VolumeLineQuery object
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    // Run the queries
    VolumeLineSegIntersectResult * resultAnyBuffer = p_VLQ->GetAnyIntersection();
    EATESTAssert( resultAnyBuffer != NULL,  "GetAnyIntersection failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    VolumeLineSegIntersectResult * resultNearestBuffer = p_VLQ->GetNearestIntersection();
    EATESTAssert( resultNearestBuffer != NULL,  "GetNearestIntersection failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    // Run query first time
    uint32_t numHits0 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits0 == 1, "GetAllIntersections failed");
    EATESTAssert( p_VLQ->GetIntersectionResultsBuffer(),  "GetAllIntersection failed");
    VolumeLineSegIntersectResult resultAllBuffer0 = p_VLQ->GetIntersectionResultsBuffer()[0];
    EATESTAssert( resultAllBuffer0.inputIndex == 0,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer0.vRef.volume == resultsArray[ 0 ] || resultAllBuffer0.vRef.volume == resultsArray[ 1 ],  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Run query a second time
    uint32_t numHits1 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits1 == 0, "Second GetAllIntersections failed");
    EATESTAssert(p_VLQ->Finished(), "Finished failed");
}

// This function runs the following tests on the VolumeLineQuery: GetAllIntersections, GetAnyIntersection, GetNearestIntersection
//
// TEST RESULT BUFFER OVERFLOW FUNCTIONALITY USING 2 INTERSECTING PRIMITIVE VOLUMES AND A RESULTS ARRAY OF SIZE 1 
//
void TestVolumeLineQuery::GetOverlapsOverflowTwoPrimInter()
{
    const uint32_t NUMINPUTS = 2;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    Volume* volArray[ NUMINPUTS ];
    for( int i  = 0 ; i < (int)NUMINPUTS ;  i++ )
    {
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // VolumeLineQuery object
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    // Run the queries
    VolumeLineSegIntersectResult * resultAnyBuffer = p_VLQ->GetAnyIntersection();
    EATESTAssert( resultAnyBuffer != NULL,  "GetAnyIntersection failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    VolumeLineSegIntersectResult * resultNearestBuffer = p_VLQ->GetNearestIntersection();
    EATESTAssert( resultNearestBuffer != NULL,  "GetNearestIntersection failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    // Run query first time
    uint32_t numHits0 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits0 == 1, "GetAllIntersections failed");
    EATESTAssert( p_VLQ->GetIntersectionResultsBuffer(),  "GetAllIntersection failed");
    VolumeLineSegIntersectResult resultAllBuffer0 = p_VLQ->GetIntersectionResultsBuffer()[0];
    EATESTAssert( resultAllBuffer0.inputIndex == 0,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer0.vRef.volume == volArray[ 0 ],  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( !p_VLQ->Finished(), "Finished failed");

    // Run query a second time
    uint32_t numHits1 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits1 == 1, "Second GetAllIntersections failed");
    EATESTAssert( p_VLQ->GetIntersectionResultsBuffer(),  "Second GetAllIntersection failed");
    VolumeLineSegIntersectResult resultAllBuffer1 = p_VLQ->GetIntersectionResultsBuffer()[0];
    EATESTAssert( resultAllBuffer1.inputIndex != resultAllBuffer0.inputIndex,  "Second GetAllIntersection: results check failed");
    EATESTAssert( resultAllBuffer1.inputIndex == 1,  "Second GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer1.vRef.volume == volArray[ 1 ],  "Second GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Run query again
    uint32_t numHits2 = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits2 == 0, "Third GetAllIntersections failed");
    EATESTAssert(p_VLQ->Finished(), "Finished failed");
}

// This function runs the following tests on the VolumeLineQuery: GetAllIntersections, GetAnyIntersection, GetNearestIntersection
//
// TEST RESULT BUFFER FOR CORRECT INTERSECTION VALUES WHEN USING 2 PRIMITIVE VOLUMES, THE SECOND OF WHICH INTERSECTS
//
void TestVolumeLineQuery::GetOverlapsCorrectOnePrimInterResults()
{
    const uint32_t NUMINPUTS = 2;
    const uint32_t STACKSIZE = 2;
    const uint32_t RESBUFFERSIZE = 2;

    Volume* volArray[ NUMINPUTS ];
    for( int i = 0 ; i < (int)NUMINPUTS ; i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    Matrix44Affine* matArray[ NUMINPUTS ];

    // Setup first volume for non-intersection
    Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        10.0f, 0.0f, 0.0f 
        );

    matArray[ 0 ] = &mat;

    // Setup second volume for non-intersection
    matArray[ 1 ] = NULL;

    // VolumeLineQuery object
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    // Run the queries
    VolumeLineSegIntersectResult * resultAnyBuffer = p_VLQ->GetAnyIntersection();
    EATESTAssert( resultAnyBuffer != NULL,  "GetAnyIntersection failed");
    EATESTAssert( resultAnyBuffer->inputIndex == 1,  "GetAnyIntersection: input volume index check failed");
    EATESTAssert( resultAnyBuffer->vRef.volume == volArray[ 1 ],  "GetAnyIntersection: Intersecting volume check failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    VolumeLineSegIntersectResult * resultNearestBuffer = p_VLQ->GetNearestIntersection();
    EATESTAssert( resultNearestBuffer != NULL,  "GetNearestIntersection failed");
    EATESTAssert( resultNearestBuffer->inputIndex == 1,  "GetNearestIntersection: input volume index check failed");
    EATESTAssert( resultNearestBuffer->vRef.volume == volArray[ 1 ],  "GetNearestIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , rwpmath::Vector3(0.0f, 0.0f, -1.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), 0.0f );

    uint32_t numHits = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits == 1, "GetAllIntersections failed");
    VolumeLineSegIntersectResult * resultAllBuffer = p_VLQ->GetIntersectionResultsBuffer();
    EATESTAssert( resultAllBuffer,  "GetAllIntersection failed");
    EATESTAssert( resultAllBuffer[0].inputIndex == 1,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer[0].vRef.volume == volArray[ 1 ],  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert(p_VLQ->Finished(), "Finished failed");
}


// This function runs the following tests on the VolumeLineQuery: GetAllIntersections, GetAnyIntersection, GetNearestIntersection
//
// TEST RESULT BUFFER FOR CORRECT INTERSECTION VALUES WHEN USING 2 PRIMITIVE VOLUMES, BOTH OF WHICH INTERSECT
//
void TestVolumeLineQuery::GetOverlapsCorrectTwoPrimInterResults()
{
    const uint32_t NUMINPUTS = 2;
    const uint32_t STACKSIZE = 2;
    const uint32_t RESBUFFERSIZE = 2;

    Volume* volArray[ NUMINPUTS ];
    for( int i = 0 ; i < (int)NUMINPUTS ; i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    Matrix44Affine* matArray[ NUMINPUTS ];

    // Setup first volume for non-intersection
    Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        3.0f, 0.0f, 0.0f 
        );

    matArray[ 0 ] = &mat;

    // Setup second volume for non-intersection
    matArray[ 1 ] = NULL;

    // VolumeLineQuery object
    VolumeLineQuery* p_VLQ = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , rwpmath::Vector3(2.0f, 0.0f, -1.0f), rwpmath::Vector3(2.0f, 0.0f, 1.0f), 2.0f );

    // Run the queries
    VolumeLineSegIntersectResult * resultAnyBuffer = p_VLQ->GetAnyIntersection();
    EATESTAssert( resultAnyBuffer != NULL,  "GetAnyIntersection failed");
    EATESTAssert( resultAnyBuffer->inputIndex == 0 || resultAnyBuffer->inputIndex == 1,  "GetAnyIntersection: input volume index check failed");
    EATESTAssert( resultAnyBuffer->vRef.volume == volArray[ 0 ] || resultAnyBuffer->vRef.volume == volArray[ 1 ],  "GetAnyIntersection: Intersecting volume check failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , rwpmath::Vector3(2.0f, 0.0f, -1.0f), rwpmath::Vector3(2.0f, 0.0f, 1.0f), 2.0f );

    VolumeLineSegIntersectResult * resultNearestBuffer = p_VLQ->GetNearestIntersection();
    EATESTAssert( resultNearestBuffer != NULL,  "GetNearestIntersection failed");
    EATESTAssert( resultNearestBuffer->inputIndex == 0 || resultNearestBuffer->inputIndex == 1,  "GetNearestIntersection: input volume index check failed");
    EATESTAssert( resultNearestBuffer->vRef.volume == volArray[ 0 ] || resultNearestBuffer->vRef.volume == volArray[ 1 ],  "GetNearestIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->Finished(), "Finished failed");

    // Initialize query
    p_VLQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , rwpmath::Vector3(2.0f, 0.0f, -1.0f), rwpmath::Vector3(2.0f, 0.0f, 1.0f), 2.0f );

    uint32_t numHits = p_VLQ->GetAllIntersections();
    EATESTAssert(numHits == 2, "GetAllIntersections failed");
    VolumeLineSegIntersectResult * resultAllBuffer = p_VLQ->GetIntersectionResultsBuffer();
    EATESTAssert( resultAllBuffer,  "GetAllIntersection failed");
    EATESTAssert( resultAllBuffer[0].inputIndex == 0,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer[0].vRef.volume == volArray[ 0 ],  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( resultAllBuffer[1].inputIndex == 1,  "GetAllIntersection: input volume index check failed");
    EATESTAssert( resultAllBuffer[1].vRef.volume == volArray[ 1 ],  "GetAllIntersection: Intersecting volume check failed");
    EATESTAssert( p_VLQ->Finished(), "GetAllIntersections Finihed failed");
}
