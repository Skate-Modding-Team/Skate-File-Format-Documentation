// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/clusteredmeshruntimebuilder.h>
#include <rw/collision/clusteredmeshofflinebuilder.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilder.h>

#include "eaphysics/unitframework/serialization_test_helpers.hpp"
#include "benchmarkenvironment/allocator.h"

#include "testsuitebase.h" // For TestSuiteBase

#include "SimpleStream.hpp"

#include "stdio.h"     // for sprintf()

using namespace rw::collision;

#define USESIMPLETEST

// Unit tests for clustered mesh line queries
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.

class TestClusteredMeshBuilder : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestClusteredMeshBuilder");
        EATEST_REGISTER("TestRuntimeOfflineComparison", "Comparison of offline and runtime builds", TestClusteredMeshBuilder, TestRuntimeOfflineComparison);

        // Vertex Compression Tests
        EATEST_REGISTER("TestCompressibleVertexCompression", "Testing compressible vertex compression", TestClusteredMeshBuilder, TestCompressibleVertexCompression);
        EATEST_REGISTER("TestUncompressibleVertexCompression_X", "Testing uncompressible vertex compression along x axis", TestClusteredMeshBuilder, TestUncompressibleVertexCompression_X);
        EATEST_REGISTER("TestUncompressibleVertexCompression_Y", "Testing uncompressible vertex compression along y axis", TestClusteredMeshBuilder, TestUncompressibleVertexCompression_Y);
        EATEST_REGISTER("TestUncompressibleVertexCompression_Z", "Testing uncompressible vertex compression along z axis", TestClusteredMeshBuilder, TestUncompressibleVertexCompression_Z);
        EATEST_REGISTER("TestUncompressibleVertexCompression_X_Y", "Testing uncompressible vertex compression along x and y axis", TestClusteredMeshBuilder, TestUncompressibleVertexCompression_X_Y);
        EATEST_REGISTER("TestUncompressibleVertexCompression_X_Z", "Testing uncompressible vertex compression along x and z axis", TestClusteredMeshBuilder, TestUncompressibleVertexCompression_X_Z);
        EATEST_REGISTER("TestUncompressibleVertexCompression_Y_Z", "Testing uncompressible vertex compression along y and z axis", TestClusteredMeshBuilder, TestUncompressibleVertexCompression_Y_Z);
        EATEST_REGISTER("TestUncompressibleVertexCompression_X_Y_Z", "Testing uncompressible vertex compression along x,y and z axis", TestClusteredMeshBuilder, TestUncompressibleVertexCompression_X_Y_Z);

        // EdgeAngles Tests
        EATEST_REGISTER("TestEdgeAngles", "Testing EdgeAngle option", TestClusteredMeshBuilder, TestEdgeAngles);

        //// FindQuads Tests
        EATEST_REGISTER("TestQuads", "Testing the FindQuads option with quad input", TestClusteredMeshBuilder, TestQuads);
        EATEST_REGISTER("TestQuadsAndTriangles", "Testing the FindQuads option with triangle and quad input", TestClusteredMeshBuilder, TestQuadsAndTriangles);

        // SurfaceID Tests
        EATEST_REGISTER("Test1ByteSurfaceID", "Testing the SurfaceID option with an ID size of 1", TestClusteredMeshBuilder, Test1ByteSurfaceID);
        EATEST_REGISTER("Test2ByteSurfaceID", "Testing the SurfaceID option with an ID size of 2", TestClusteredMeshBuilder, Test2ByteSurfaceID);

        EATEST_REGISTER("Test1ByteGroupID", "Testing the GroupID option with an ID size of 1", TestClusteredMeshBuilder, Test1ByteGroupID);
        EATEST_REGISTER("Test2ByteGroupID", "Testing the GroupID option with an ID size of 2", TestClusteredMeshBuilder, Test2ByteGroupID);

        // Consistency
        // These tests are not included in the rev test suite as they require large amounts of memory.

        // These tests are designed to tests the vertex smoothing algorithm. They do no use the builder directly, but a local implementation of the
        // algorithm. They serve as a proof of concept.
        EATEST_REGISTER("TestVertexSmooth_VertexDisabled", "Testing the vertex smoothing algorithm", TestClusteredMeshBuilder, TestVertexSmooth_VertexDisabled);
        EATEST_REGISTER("TestVertexSmooth_Edge_B_Updated", "Testing the vertex smoothing algorithm", TestClusteredMeshBuilder, TestVertexSmooth_Edge_B_Updated);
        EATEST_REGISTER("TestVertexSmooth_Edge_A_Updated", "Testing the vertex smoothing algorithm", TestClusteredMeshBuilder, TestVertexSmooth_Edge_A_Updated);
        EATEST_REGISTER("TestVertexSmooth_Non_Contributor", "Testing the vertex smoothing algorithm", TestClusteredMeshBuilder, TestVertexSmooth_Non_Contributor);
        EATEST_REGISTER("TestVertexSmooth_Non_Contributor_OnLimit", "Testing the vertex smoothing algorithm", TestClusteredMeshBuilder, TestVertexSmooth_Non_Contributor_OnLimit);
        EATEST_REGISTER("TestVertexSmooth_Exanding_Group_Vertex_Disabled", "Testing the vertex smoothing algorithm", TestClusteredMeshBuilder, TestVertexSmooth_Exanding_Group_Vertex_Disabled);
        EATEST_REGISTER("TestVertexSmooth_VertexDisabled_OnLimit", "Testing the vertex smoothing algorithm", TestClusteredMeshBuilder, TestVertexSmooth_VertexDisabled_OnLimit);

        // Edge Case Tests
        EATEST_REGISTER("TestLeafNodeSpanningTwoClusters", "Testing a Leaf Node which spans two clusters", TestClusteredMeshBuilder, TestLeafNodeSpanningTwoClusters);
        EATEST_REGISTER("TestNoValidInputTriangles", "Testing No Valid Input Triangles", TestClusteredMeshBuilder, TestNoValidInputTriangles);
        EATEST_REGISTER("TestSingleMergedVertex", "Testing a collection of vertices which merge to 1 vertex", TestClusteredMeshBuilder, TestSingleMergedVertex);
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        rw::collision::InitializeVTables();
        EA_ASSERT(mAllocator == 0);
        mAllocator = new benchmarkenvironment::HeapAllocator();
    }

    virtual void TeardownSuite()
    {
        mAllocator->CheckForLeaks();
        mAllocator->CheckForTrampling();
        delete mAllocator;
        mAllocator = 0;
        tests::TestSuiteBase::TeardownSuite();
    }

private:

    void TestRuntimeOfflineComparison();
    void TestCompressibleVertexCompression();
    void TestUncompressibleVertexCompression_X();
    void TestUncompressibleVertexCompression_Y();
    void TestUncompressibleVertexCompression_Z();
    void TestUncompressibleVertexCompression_X_Y();
    void TestUncompressibleVertexCompression_Y_Z();
    void TestUncompressibleVertexCompression_X_Z();
    void TestUncompressibleVertexCompression_X_Y_Z();

    void TestOldTriangles();

    void TestEdgeAngles();

    void TestQuads();
    void TestQuadsAndTriangles();

    void Test1ByteSurfaceID();
    void Test2ByteSurfaceID();

    void Test1ByteGroupID();
    void Test2ByteGroupID();

    void TestConsistency();
    void TestConsistencyCustom();

    void TestLeafNodeSpanningTwoClusters();
    void TestNoValidInputTriangles();
    void TestSingleMergedVertex();

    enum
    {
        VERTEX_DISABLED,
        EDGE_A_UPDATED,
        EDGE_B_UPDATED,
        NON_CONTRIBUTOR
    };

    void TestVertexSmooth_VertexDisabled();
    void TestVertexSmooth_Edge_B_Updated();
    void TestVertexSmooth_Edge_A_Updated();
    void TestVertexSmooth_Non_Contributor();
    void TestVertexSmooth_Non_Contributor_OnLimit();
    void TestVertexSmooth_Exanding_Group_Vertex_Disabled();
    void TestVertexSmooth_VertexDisabled_OnLimit();

    uint32_t CheckNextEdge(rwpmath::Vector3& edgeA,
                       rwpmath::Vector3& edgeB,
                       rwpmath::Vector3& edgeC);

    uint32_t SizeofUnit(uint8_t * unitData,
                        uint32_t sizeGroupID,
                        uint32_t sizeSurfaceID);

    template<class BuilderType>
    void SetGridInput(BuilderType &builder,
                      uint32_t &triangleIndexOffset,
                      uint32_t &vertexIndexOffset,
                      uint32_t xCount,
                      uint32_t yCount,
                      uint32_t zCount,
                      float_t xLength,
                      float_t yLength,
                      float_t zLength);

    template<class BuilderType>
    void SetTriangleInput(BuilderType &builder,
                          uint32_t &triangleIndexOffset,
                          uint32_t &vertexIndexOffset,
                          uint32_t xCount,
                          uint32_t yCount,
                          uint32_t zCount,
                          rw::math::fpu::Vector3::InOutParam offset,
                          float_t xLength,
                          float_t yLength,
                          float_t zLength);

    benchmarkenvironment::HeapAllocator * mAllocator;

} TestClusteredMeshBuilderSingleton;

uint32_t
TestClusteredMeshBuilder::SizeofUnit(uint8_t * unitData,
                                     uint32_t sizeGroupID,
                                     uint32_t sizeSurfaceID)
{
    uint32_t size = 1;
    const uint8_t unitFlags = unitData[0];
    const uint32_t unitType = (uint32_t)(unitFlags & rw::collision::UNITTYPE_MASK);

    if (rw::collision::UNITTYPE_OLDTRIANGLE & unitType)
    {
        return size + 3;
    }
    else if (rw::collision::UNITTYPE_TRIANGLE & unitType)
    {
        size += 3;
    }
    else if (rw::collision::UNITTYPE_QUAD & unitType)
    {
        size += 4;
    }

    if (unitFlags & rw::collision::UNITFLAG_EDGEANGLE)
    {
        size += 3;
    }

    if (unitFlags & rw::collision::UNITFLAG_GROUPID)
    {
        size += sizeGroupID;
    }

    if (unitFlags & rw::collision::UNITFLAG_SURFACEID)
    {
        size += sizeSurfaceID;
    }

    return size;
}


template<class BuilderType>
void TestClusteredMeshBuilder::SetTriangleInput(BuilderType &builder,
                                                uint32_t &triangleIndexOffset,
                                                uint32_t &vertexIndexOffset,
                                                uint32_t xCount,
                                                uint32_t yCount,
                                                uint32_t zCount,
                                                rw::math::fpu::Vector3::InOutParam offset,
                                                float_t xLength,
                                                float_t yLength,
                                                float_t zLength)
{
    rw::math::fpu::Vector3 triangleVertices[3] =
    {
        rw::math::fpu::Vector3(          0.0f, 0.0f,           0.0f), // Vertex 1
        rw::math::fpu::Vector3(1.0f * xLength, 0.0f,           0.0f), // Vertex 2
        rw::math::fpu::Vector3(          0.0f, 0.0f, 1.0f * zLength), // Vertex 3
    };

    uint32_t triangleVertexIndices[3]=
    {
        0, 1, 2, // Triangle 1
    };

    uint32_t triangleIndex = triangleIndexOffset;
    uint32_t vertexIndex = vertexIndexOffset;

    rw::math::fpu::Vector3 localOffset(0.0f, 0.0f, 0.0f);

    for (uint32_t yIndex = 0 ; yIndex < yCount ; ++yIndex)
    {
        localOffset.SetY(1.0f * yIndex * yLength);
        for (uint32_t xIndex = 0 ; xIndex < xCount ; ++xIndex)
        {
            localOffset.SetX(1.0f * xIndex);
            for (uint32_t zIndex = 0 ; zIndex < zCount ; ++zIndex)
            {
                localOffset.SetZ(1.0f * zIndex);

                builder.SetTriangle(triangleIndex++, (vertexIndex), (vertexIndex + 1), (vertexIndex + 2));
                builder.SetVertex(vertexIndex++, triangleVertices[triangleVertexIndices[0]] + localOffset + offset);
                builder.SetVertex(vertexIndex++, triangleVertices[triangleVertexIndices[1]] + localOffset + offset);
                builder.SetVertex(vertexIndex++, triangleVertices[triangleVertexIndices[2]] + localOffset + offset);
            }
        }
    }

    triangleIndexOffset = triangleIndex;
    vertexIndexOffset = vertexIndex;
}

template<class BuilderType>
void TestClusteredMeshBuilder::SetGridInput(BuilderType &builder,
                                            uint32_t &triangleIndexOffset,
                                            uint32_t &vertexIndexOffset,
                                            uint32_t xCount,
                                            uint32_t yCount,
                                            uint32_t zCount,
                                            float_t xLength,
                                            float_t yLength,
                                            float_t zLength)
{
    rw::math::fpu::Vector3 quadVertices[4] =
    {
        rw::math::fpu::Vector3(          0.0f, 0.0f,           0.0f), // Vertex 1
        rw::math::fpu::Vector3(1.0f * xLength, 0.0f,           0.0f), // Vertex 2
        rw::math::fpu::Vector3(          0.0f, 0.0f, 1.0f * zLength), // Vertex 3
        rw::math::fpu::Vector3(1.0f * xLength, 0.0f, 1.0f * zLength), // Vertex 4
    };

    uint32_t quadVertexIndices[6]=
    {
        0, 1, 2, // Triangle 1
        1, 3, 2, // Triangle 2
    };

    uint32_t triangleIndex = triangleIndexOffset;
    uint32_t vertexIndex = vertexIndexOffset;
    rw::math::fpu::Vector3 offset(0.0f, 0.0f, 0.0f);

    for (uint32_t yIndex = 0 ; yIndex < yCount ; ++yIndex)
    {
        offset.SetY(1.0f * yIndex * yLength);
        for (uint32_t xIndex = 0 ; xIndex < xCount ; ++xIndex)
        {
            offset.SetX(1.0f * xIndex);
            for (uint32_t zIndex = 0 ; zIndex < zCount ; ++zIndex)
            {
                offset.SetZ(1.0f * zIndex);

                builder.SetTriangle(triangleIndex++, (vertexIndex), (vertexIndex + 1), (vertexIndex + 2), 0xAAAA, 0xBBBB);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[0]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[1]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[2]] + offset);

                builder.SetTriangle(triangleIndex++, (vertexIndex), (vertexIndex + 1), (vertexIndex + 2), 0xAAAA, 0xBBBB);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[3]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[4]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[5]] + offset);
            }
        }
    }

    triangleIndexOffset = triangleIndex;
    vertexIndexOffset = vertexIndex;
}


/**
Tests vertex compression using an small input set and compression granularity which should allow
16Bit compression to take place. The results are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestCompressibleVertexCompression()
{
    uint32_t xCount = 1;
    uint32_t yCount = 1;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // allows the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();


    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");
    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");
    // Expect 16bit compression
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_16BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests vertex compression using an small input set and compression granularity which should not allow
compression to take place due to large displacement, of input vertices, along the X axis. The results
are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestUncompressibleVertexCompression_X()
{
    uint32_t xCount = 1;
    uint32_t yCount = 1;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // shouldn't allow the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexMerge_Enable = false;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 100000.0f,
                 1.0f,
                 1.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    // Expect 32bit compressed
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_32BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests vertex compression using an small input set and compression granularity which should not allow
compression to take place due to large displacement, of input vertices, along the Y axis. The results
are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestUncompressibleVertexCompression_Y()
{
    uint32_t xCount = 1;
    uint32_t yCount = 2;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // shouldn't allow the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexMerge_Enable = false;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 100000.0f,
                 1.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    // Expect 32bit compression
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_32BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests vertex compression using an small input set and compression granularity which should not allow
compression to take place due to large displacement, of input vertices, along the Z axis. The results
are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestUncompressibleVertexCompression_Z()
{
    uint32_t xCount = 1;
    uint32_t yCount = 1;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // shouldn't allow the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexMerge_Enable = false;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 100000.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    // Expect 32bit compression
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_32BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests vertex compression using an small input set and compression granularity which should not allow
compression to take place due to large displacement, of input vertices, along the X & Y axis. The
results are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestUncompressibleVertexCompression_X_Y()
{
    uint32_t xCount = 1;
    uint32_t yCount = 2;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // shouldn't allow the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexMerge_Enable = false;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 100000.0f,
                 100000.0f,
                 1.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");
    // Expect 32bit compression
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_32BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests vertex compression using an small input set and compression granularity which should not allow
compression to take place due to large displacement, of input vertices, along the Y & Z axis. The
results are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestUncompressibleVertexCompression_Y_Z()
{
    uint32_t xCount = 1;
    uint32_t yCount = 2;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // shouldn't allow the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexMerge_Enable = false;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 100000.0f,
                 100000.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");
    // Expect 32bit compression
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_32BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests vertex compression using an small input set and compression granularity which should not allow
compression to take place due to large displacement, of input vertices, along the X & Z axis. The
results are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestUncompressibleVertexCompression_X_Z()
{
    uint32_t xCount = 1;
    uint32_t yCount = 1;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // shouldn't allow the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexMerge_Enable = false;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 100000.0f,
                 1.0f,
                 100000.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");
    // Expect 32bit compression
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_32BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests vertex compression using an small input set and compression granularity which should not allow
compression to take place due to large displacement, of input vertices, along the X & Y & Z axis. The
results are tested by checking the Cluster compression.
*/
void TestClusteredMeshBuilder::TestUncompressibleVertexCompression_X_Y_Z()
{
    uint32_t xCount = 1;
    uint32_t yCount = 1;
    uint32_t zCount = 1;

    // Enable vertex compression and set compression granularity to a value which
    // shouldn't allow the vertices to be compressed.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexCompression_Enable = true;
    builderParams.vertexCompression_Granularity = 1.0f;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the mesh builder input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 100000.0f,
                 100000.0f,
                 100000.0f);

    // Build the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect a single cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");
    // Expect 32bit compression
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED == clusteredMesh->GetCluster(0).compressionMode,
                 "Cluster compression mode should be VERTICES_32BIT_COMPRESSED");

    // Release the resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests consistency between the runtime and offline mesh builders. Using a fairly small input set
ClusteredMeshes are generated and then serialized. The serialized meshes are then compared for
equality. The two meshes are expected to be exactly the same.
*/
void TestClusteredMeshBuilder::TestRuntimeOfflineComparison()
{
    uint32_t xCount = 25;
    uint32_t yCount = 2;
    uint32_t zCount = 25;

    // Allocate memory for the runtime builder - too big for mAllocator to handle
    uint32_t sizeOfClusteredMeshBuilderAllocatorBuffer = static_cast<uint32_t>(1024*1024*5);
    EA::Allocator::ICoreAllocator * allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    uint8_t * clusteredMeshBuilderAllocatorBuffer = static_cast<uint8_t*>(allocator->Alloc(sizeOfClusteredMeshBuilderAllocatorBuffer, NULL, 0));

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the runtime builder
    rw::collision::ClusteredMeshRuntimeBuilder::Parameters runtimeBuilderParams;
    rw::collision::ClusteredMeshRuntimeBuilder runtimeBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              runtimeBuilderParams,
                                                              clusteredMeshBuilderAllocatorBuffer,
                                                              sizeOfClusteredMeshBuilderAllocatorBuffer,
                                                              mAllocator);

    // Initialize the offline builder
    rw::collision::ClusteredMeshOfflineBuilder::Parameters offlineBuilderParams;
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              offlineBuilderParams,
                                                              mAllocator);

    // Expect the runtime builder to be valid
    EATESTAssert(runtimeBuilder.IsBuilderValid(), "runtime clusteredmesh builder should be valid");

    // Set the input of both builders. The input has to be exactly the same.
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(runtimeBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    triangleIndex = 0;
    vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Create clusteredmeshes using both builders.
    rw::collision::ClusteredMesh * clusteredMeshRuntime = runtimeBuilder.BuildClusteredMesh();
    rw::collision::ClusteredMesh * clusteredMeshOffline = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMeshRuntime, "clusteredMesh should not be NULL");
    EATESTAssert(NULL != clusteredMeshOffline, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMeshRuntime->IsValid(), "ClusteredMesh should be valid");
    EATESTAssert(1 == clusteredMeshOffline->IsValid(), "ClusteredMesh should be valid");

    // Create buffers for the serialized clusteredmeshes
    const uint32_t bufferSize = 65536u;
    uint8_t *runtimeBuffer= (uint8_t*)mAllocator->Alloc(bufferSize, NULL,0,4);
    uint8_t *offlineBuffer= (uint8_t*)mAllocator->Alloc(bufferSize, NULL,0,4);

    // Define the archiveType
    typedef EA::Serialization::imaging_oarchive<EA::Serialization::Endian::LittleEndianConverter> archiveType;

    // Initialize the archives for each clusteredmesh
    archiveType clusteredMeshRuntimeArchive(runtimeBuffer, bufferSize);
    archiveType clusteredMeshOfflineArchive(offlineBuffer, bufferSize);

    ClusteredMesh *clusteredMeshRuntimePtr = clusteredMeshRuntime;
    ClusteredMesh *clusteredMeshOfflinePtr = clusteredMeshOffline;

    // Serialize each clusteredmesh
    clusteredMeshRuntimeArchive & EAPHYSICS_LL_SERIALIZABLE(ClusteredMesh, clusteredMeshRuntimePtr);
    clusteredMeshOfflineArchive & EAPHYSICS_LL_SERIALIZABLE(ClusteredMesh, clusteredMeshOfflinePtr);

    // Close each archived mesh
    clusteredMeshRuntimeArchive.Close();
    clusteredMeshOfflineArchive.Close();

    // Expect the size of each archived clusteredmesh to be the same
    EATESTAssert(clusteredMeshRuntimeArchive.GetFinalSize() == clusteredMeshOfflineArchive.GetFinalSize(), "Size of serialize meshes should be the same");

    // Expect the archived clusteredmeshes to be exactly the same
    EATESTAssert(memcmp(reinterpret_cast<void*>(clusteredMeshRuntimeArchive.GetOutputBuffer()),
                        reinterpret_cast<void*>(clusteredMeshOfflineArchive.GetOutputBuffer()),
                        clusteredMeshRuntimeArchive.GetFinalSize()) == 0,
                 "Serialized meshes should be exactly the same");

    // Deallocate archive resources
    mAllocator->Free(offlineBuffer);
    mAllocator->Free(runtimeBuffer);

    // Deallocate clusteredmesh resources
    if (NULL != clusteredMeshRuntime)
        mAllocator->Free(clusteredMeshRuntime);
    if (NULL != clusteredMeshOffline)
        mAllocator->Free(clusteredMeshOffline);

    // Deallocate runtime builder resources
    runtimeBuilder.Release();
    allocator->Free(clusteredMeshBuilderAllocatorBuffer);
}


/**
Tests generation of a clusteredmesh which contains edge angle data for each unit. Using a small input set
a clusteredmesh, composed of traingles, is generated which should include edge angle data for each unit.
*/
void TestClusteredMeshBuilder::TestEdgeAngles()
{
    uint32_t xCount = 1;
    uint32_t yCount = 1;
    uint32_t zCount = 1;

    // Enable edge angle generation
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.edgeAngles_Enable = true;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect 1 cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    rw::collision::ClusteredMeshCluster & cluster = clusteredMesh->GetCluster(0);

    // Expect the unit data size to be 14, since we expect 2 old triangles, each triangle using 7
    EATESTAssert(14 == cluster.unitDataSize, "unitDataSize for cluster should be 14 (7 bytes per unit/triangle)");

    // Expect the type of the unit to be UNITTYPE_TRIANGLE
    // Expect the unit to include EdgeAngle data
    uint32_t offset = 0;
    uint8_t * unitData = cluster.UnitData();
    EATESTAssert(rw::collision::UNITTYPE_TRIANGLE == (unitData[offset] & rw::collision::UNITTYPE_MASK), "Unit type should be UNITTYPE_TRIANGLE");
    EATESTAssert(rw::collision::UNITFLAG_EDGEANGLE == (unitData[offset] & ~rw::collision::UNITTYPE_MASK), "Unit flags should be UNITFLAG_EDGEANGLE");

    // Determine the offset to the second unit
    offset = SizeofUnit(unitData,
                        builderParams.groupId_NumBytes,
                        builderParams.surfaceId_NumBytes);

    // Expect the type of the unit to be UNITTYPE_TRIANGLE
    // Expect the unit to include EdgeAngle data
    EATESTAssert(rw::collision::UNITTYPE_TRIANGLE == (unitData[offset] & rw::collision::UNITTYPE_MASK), "Unit type should be UNITTYPE_TRIANGLE");
    EATESTAssert(rw::collision::UNITFLAG_EDGEANGLE == (unitData[offset] & ~rw::collision::UNITTYPE_MASK), "Unit flags should be UNITFLAG_EDGEANGLE");

    // Release resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests generation of a clusteredmesh which should consist entirely of quads. The type of each unit
of the generated clusteredmesh is tested.
*/
void TestClusteredMeshBuilder::TestQuads()
{
    uint32_t xCount = 4;
    uint32_t yCount = 6;
    uint32_t zCount = 4;

    // Enable edge angle generation
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.quads_Enable = true;
    builderParams.edgeAngles_Enable = false;

    uint32_t numTriangles = xCount * yCount * zCount * 2;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect 1 cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    rw::collision::ClusteredMeshCluster & cluster = clusteredMesh->GetCluster(0);

    // Expect the unit data size to be 5 * unit count
    char buffer[128];
    sprintf(buffer, "unitDataSize for cluster should be %d bytes (5 per quad)", 5 * xCount * yCount * zCount);
    EATESTAssert(5 * xCount * yCount * zCount == cluster.unitDataSize, buffer);

    // Iterate through each of the units and assert that its type is a quad
    uint8_t * unitData = cluster.UnitData();
    uint32_t offset = 0;
    for (uint32_t i = 0; i < xCount * yCount * zCount ; ++i)
    {
        EATESTAssert(rw::collision::UNITTYPE_QUAD == (unitData[offset] & rw::collision::UNITTYPE_MASK), "Unit type should be UNITTYPE_QUAD");
        offset += SizeofUnit(&unitData[offset],
                             builderParams.groupId_NumBytes,
                             builderParams.surfaceId_NumBytes);
    }

    // Release resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Tests the generation of a clusteredmesh which should contain both quads and triangles. Input set
contains a number of triangles which can be converted into quads and a number of triangles which
cannot be converted.
NOTE: The order of units is not guaranteed to be the same as the order of insertion into the builder
therefore we can only test that the correct number of quad units and triangle units are created, and
not their order.
*/
void TestClusteredMeshBuilder::TestQuadsAndTriangles()
{
    uint32_t xCount = 2;
    uint32_t yCount = 2;
    uint32_t zCount = 2;

    // Enable quad generation and disable edge angle generation
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.quads_Enable = true;
    builderParams.edgeAngles_Enable = false;

    uint32_t numTriangles = xCount * yCount * zCount * 3;
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize an offline mesh builder.
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the quad input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    rw::math::fpu::Vector3 offSet(0.0f, (yCount + 1) * 1.0f, 0.0f);

    // Set the triangle input
    SetTriangleInput(offlineBuilder,
                     triangleIndex,
                     vertexIndex,
                     xCount,
                     yCount,
                     zCount,
                     offSet,
                     1.0f,
                     1.0f,
                     1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect 1 cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    rw::collision::ClusteredMeshCluster & cluster = clusteredMesh->GetCluster(0);

    // Expect unit data size to be 5 * num quad units + 4 * num triangle units
    char buffer[128];
    sprintf(buffer, "unitDataSize for cluster should be %d bytes (5 per quad, 4 per triangle)", 9 * xCount * yCount * zCount);
    EATESTAssert(9 * xCount * yCount * zCount == cluster.unitDataSize, buffer);

    // Determine the number of quad and triangle units
    const uint32_t expectedQuadCount = xCount * yCount * zCount;
    uint32_t actualQuadCount = 0;
    const uint32_t expectedTriangleCount = xCount * yCount * zCount;
    uint32_t actualTriangleCount = 0;

    uint32_t unexpectedCount = 0;

    // Iterate through the units counting the unit types
    uint32_t offset = 0;
    uint8_t * unitData = cluster.UnitData();
    for (uint32_t i = 0; i < xCount * yCount * zCount * 2 ; ++i)
    {
        uint32_t unitSize = SizeofUnit(&unitData[offset],
                                       builderParams.groupId_NumBytes,
                                       builderParams.surfaceId_NumBytes);

        if (unitSize == 5)
            actualQuadCount += 1;
        else if (unitSize == 4)
            actualTriangleCount += 1;
        else
            unexpectedCount += 1;

        offset += unitSize;
    }

    // Expect xCount * yCount * zCount number of quads
    sprintf(buffer, "Quad count should be %d", xCount * yCount * zCount);
    EATESTAssert(expectedQuadCount == actualQuadCount, buffer);

    // Expect xCount * yCount * zCount number of triangle
    sprintf(buffer, "Triangle count should be %d", xCount * yCount * zCount);
    EATESTAssert(expectedTriangleCount == actualTriangleCount, buffer);

    // Expect no units with a non-triangle/quads
    EATESTAssert(0 == unexpectedCount, "Unexpected count should be 0");

    // Release resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}

/**
Test generation of a clusteredmesh which consists of units which include 8bit surfaceIDs. All units in the input
set are tagged with a non-default ID.
*/
void TestClusteredMeshBuilder::Test1ByteSurfaceID()
{
    uint32_t xCount = 4;
    uint32_t yCount = 4;
    uint32_t zCount = 4;

    // Disable edge angle generation, Enable 8bit surfaceIDs, Initialise the default surfaceID to non-zero
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.edgeAngles_Enable = false;
    builderParams.surfaceId_NumBytes = 1;

    uint32_t numTriangles = (xCount * yCount * zCount * 2);
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect 1 cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    rw::collision::ClusteredMeshCluster & cluster = clusteredMesh->GetCluster(0);

    // Expect unit data size to be 5 * xCount * yCount * zCount * 2 
    char buffer[128];
    sprintf(buffer, "unitDataSize for cluster should be %d bytes (5 per triangle)", 5 * xCount * yCount * zCount * 2);
    EATESTAssert(5 * xCount * yCount * zCount * 2 == cluster.unitDataSize, buffer);

    // Initialize the cluster params to indicate the units contain a 8bit surfaceID
    rw::collision::ClusterParams clusterParams;
    clusterParams.mGroupIdSize = 0;
    clusterParams.mSurfaceIdSize = 1;

    uint32_t unitGroupId = 0;
    uint32_t unitSurfaceId = 0;

    // Iterate through each unit and confirm that they include a 8bit surfaceID of 0 and no groupID
    uint8_t * unitData = cluster.UnitData();
    uint32_t offset = 0;
    for (uint32_t i = 0; i < xCount * yCount * zCount ; ++i)
    {
        // Expect the unit type to be UNITTYPE_TRIANGLE
        EATESTAssert(rw::collision::UNITTYPE_TRIANGLE == (unitData[offset] & rw::collision::UNITTYPE_MASK), "Unit type should be UNITTYPE_TRIANGLE");
        cluster.GetGroupAndSurfaceId(offset, clusterParams, unitGroupId, unitSurfaceId);
        // Expect unitSurfaceId to be zero
        EATESTAssert(0xBB == unitSurfaceId, "unitSurfaceId should be 0xBB");
        // Expect unitGroupId to be unchanged
        EATESTAssert(0 == unitGroupId, "unitGroupId should not have been set");
        offset += SizeofUnit(&unitData[offset],
                             builderParams.groupId_NumBytes,
                             builderParams.surfaceId_NumBytes);
    }

    // Release resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}

/**
Test generation of a clusteredmesh which consists of units which include 16bit surfaceIDs. All units in the input
set are tagged with a non-default ID.
*/
void TestClusteredMeshBuilder::Test2ByteSurfaceID()
{
    uint32_t xCount = 4;
    uint32_t yCount = 4;
    uint32_t zCount = 4;

    // Disable edge angle generation, Enable 16bit surfaceIDs, Initialise the default surfaceID to non-zero
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.edgeAngles_Enable = false;
    builderParams.surfaceId_NumBytes = 2;

    uint32_t numTriangles = (xCount * yCount * zCount * 2);
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect 1 cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    rw::collision::ClusteredMeshCluster & cluster = clusteredMesh->GetCluster(0);

    // Expect unit data size to be 5 * xCount * yCount * zCount * 2 
    char buffer[128];
    sprintf(buffer, "unitDataSize for cluster should be %d bytes (6 per triangle)", 6 * xCount * yCount * zCount * 2);
    EATESTAssert(6 * xCount * yCount * zCount * 2 == cluster.unitDataSize, buffer);

    // Initialize the cluster params to indicate the units contain a 16bit surfaceID
    rw::collision::ClusterParams clusterParams;
    clusterParams.mGroupIdSize = 0;
    clusterParams.mSurfaceIdSize = 2;

    uint32_t unitGroupId = 0;
    uint32_t unitSurfaceId = 0;

    // Iterate through each unit and confirm that they include a 16bit surfaceID of 0 and no groupID
    uint8_t * unitData = cluster.UnitData();
    uint32_t offset = 0;
    for (uint32_t i = 0; i < xCount * yCount * zCount ; ++i)
    {
        // Expect the unit type to be UNITTYPE_TRIANGLE
        EATESTAssert(rw::collision::UNITTYPE_TRIANGLE == (unitData[offset] & rw::collision::UNITTYPE_MASK), "Unit type should be UNITTYPE_TRIANGLE");
        cluster.GetGroupAndSurfaceId(offset, clusterParams, unitGroupId, unitSurfaceId);
        // Expect unitSurfaceId to be zero
        EATESTAssert(0xBBBB == unitSurfaceId, "unitSurfaceId should be 0xBBBB");
        // Expect unitGroupId to be unchanged
        EATESTAssert(0 == unitGroupId, "unitGroupId should not have been set");
        offset += SizeofUnit(&unitData[offset],
                             builderParams.groupId_NumBytes,
                             builderParams.surfaceId_NumBytes);
    }

    // Release resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}

/**
Test generation of a clusteredmesh which consists of units which include 8bit unitIDs. All units in the input
set are tagged with a non-default ID.
*/
void TestClusteredMeshBuilder::Test1ByteGroupID()
{
    uint32_t xCount = 4;
    uint32_t yCount = 4;
    uint32_t zCount = 4;

    // Disable edge angle generation, Enable 8bit unitIDs
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.edgeAngles_Enable = false;
    builderParams.groupId_NumBytes = 1;

    uint32_t numTriangles = (xCount * yCount * zCount * 2);
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect 1 cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    rw::collision::ClusteredMeshCluster & cluster = clusteredMesh->GetCluster(0);

    // Expect unit data size to be 5 * xCount * yCount * zCount * 2 
    char buffer[128];
    sprintf(buffer, "unitDataSize for cluster should be %d bytes (5 per triangle)", 5 * xCount * yCount * zCount * 2);
    EATESTAssert(5 * xCount * yCount * zCount * 2 == cluster.unitDataSize, buffer);

    // Initialize the cluster params to indicate the units contain a 8bit groupID
    rw::collision::ClusterParams clusterParams;
    clusterParams.mGroupIdSize = 1;
    clusterParams.mSurfaceIdSize = 0;

    uint32_t unitGroupId = 0;
    uint32_t unitSurfaceId = 0;

    // Iterate through each unit and confirm that they include a 8bit unitID of 0 and no surfaceID
    uint8_t * unitData = cluster.UnitData();
    uint32_t offset = 0;
    for (uint32_t i = 0; i < xCount * yCount * zCount ; ++i)
    {
        // Expect the unit type to be UNITTYPE_TRIANGLE
        EATESTAssert(rw::collision::UNITTYPE_TRIANGLE == (unitData[offset] & rw::collision::UNITTYPE_MASK), "Unit type should be UNITTYPE_TRIANGLE");
        cluster.GetGroupAndSurfaceId(offset, clusterParams, unitGroupId, unitSurfaceId);
        // Expect unitGroupId to be zero
        EATESTAssert(0xAA == unitGroupId, "unitGroupId should be 0xAA");
        // Expect unitSurfaceId to have not been set
        EATESTAssert(0 == unitSurfaceId, "unitSurfaceId should not have been set");
        offset += SizeofUnit(&unitData[offset],
                             builderParams.groupId_NumBytes,
                             builderParams.surfaceId_NumBytes);
    }

    // Release resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


/**
Test generation of a clusteredmesh which consists of units which include 8bit groupIDs. All units in the input
set are tagged with a non-default ID.
*/
void TestClusteredMeshBuilder::Test2ByteGroupID()
{
    uint32_t xCount = 4;
    uint32_t yCount = 4;
    uint32_t zCount = 4;

    // Disable edge angle generation, Enable 16bit unitIDs
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.edgeAngles_Enable = false;
    builderParams.groupId_NumBytes = 2;

    uint32_t numTriangles = (xCount * yCount * zCount * 2);
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(numTriangles,
                                                              numVertices,
                                                              numMergePlanes,
                                                              builderParams,
                                                              mAllocator);

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(offlineBuilder,
                 triangleIndex,
                 vertexIndex,
                 xCount,
                 yCount,
                 zCount,
                 1.0f,
                 1.0f,
                 1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh
    EATESTAssert(NULL != clusteredMesh, "clusteredMesh should not be NULL");

    // Expect a valid clustered mesh
    EATESTAssert(1 == clusteredMesh->IsValid(), "ClusteredMesh should be valid");

    // Expect 1 cluster
    EATESTAssert(1 == clusteredMesh->GetNumCluster(), "Cluster count should be 1");

    rw::collision::ClusteredMeshCluster & cluster = clusteredMesh->GetCluster(0);

    // Expect unit data size to be 5 * xCount * yCount * zCount * 2 
    char buffer[128];
    sprintf(buffer, "unitDataSize for cluster should be %d bytes (6 per triangle)", 6 * xCount * yCount * zCount * 2);
    EATESTAssert(6 * xCount * yCount * zCount * 2 == cluster.unitDataSize, buffer);

    // Initialize the cluster params to indicate the units contain a 16bit groupID
    rw::collision::ClusterParams clusterParams;
    clusterParams.mGroupIdSize = 2;
    clusterParams.mSurfaceIdSize = 0;

    uint32_t unitGroupId = 0;
    uint32_t unitSurfaceId = 0;

    // Iterate through each unit and confirm that they include a 16bit groupID of 0 and no surfaceID
    uint8_t * unitData = cluster.UnitData();
    uint32_t offset = 0;
    for (uint32_t i = 0; i < xCount * yCount * zCount ; ++i)
    {
        // Expect the unit type to be UNITTYPE_TRIANGLE
        EATESTAssert(rw::collision::UNITTYPE_TRIANGLE == (unitData[offset] & rw::collision::UNITTYPE_MASK), "Unit type should be UNITTYPE_TRIANGLE");
        cluster.GetGroupAndSurfaceId(offset, clusterParams, unitGroupId, unitSurfaceId);
        // Expect unitGroupId to be zero
        EATESTAssert(0xAAAA == unitGroupId, "unitGroupId should be 0xBB");
        // Expect unitSurfaceId to have not been set
        EATESTAssert(0 == unitSurfaceId, "unitSurfaceId should not have been set");
        offset += SizeofUnit(&unitData[offset],
                             builderParams.groupId_NumBytes,
                             builderParams.surfaceId_NumBytes);
    }

    // Release resources
    if (NULL != clusteredMesh)
        mAllocator->Free(clusteredMesh);
}


void TestClusteredMeshBuilder::TestLeafNodeSpanningTwoClusters()
{
    uint32_t xCount = 10;
    uint32_t yCount = 10;
    uint32_t zCount = 10;

    // Setting the kdTreeBuilder_SplitThreshold to 512 will result in a leaf node
    // which contains over 255 unique vertices, causing the leaf node to spread to
    // multiple clusters.
    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.kdTreeBuilder_SplitThreshold = 500;

    uint32_t numTriangles = (xCount * yCount * zCount * 2);
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(
        numTriangles,
        numVertices,
        numMergePlanes,
        builderParams,
        EA::Allocator::ICoreAllocator::GetDefaultAllocator());  // TODO: This test leaks.

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(
        offlineBuilder,
        triangleIndex,
        vertexIndex,
        xCount,
        yCount,
        zCount,
        1.0f,
        1.0f,
        1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh pointer
    EATESTAssert(NULL == clusteredMesh, "clusteredMesh should be NULL");
}


void TestClusteredMeshBuilder::TestNoValidInputTriangles()
{
    uint32_t xCount = 10;
    uint32_t yCount = 1;
    uint32_t zCount = 10;

    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;

    uint32_t numTriangles = (xCount * yCount * zCount * 2);
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(
        numTriangles,
        numVertices,
        numMergePlanes,
        builderParams,
        EA::Allocator::ICoreAllocator::GetDefaultAllocator());  // TODO: This test leaks.

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(
        offlineBuilder,
        triangleIndex,
        vertexIndex,
        xCount,
        yCount,
        zCount,
        rwpmath::MIN_FLOAT,
        rwpmath::MIN_FLOAT,
        rwpmath::MIN_FLOAT);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh pointer
    EATESTAssert(NULL == clusteredMesh, "clusteredMesh should be NULL");
}


void TestClusteredMeshBuilder::TestSingleMergedVertex()
{
    uint32_t xCount = 5;
    uint32_t yCount = 1;
    uint32_t zCount = 5;

    rw::collision::ClusteredMeshOfflineBuilder::Parameters builderParams;
    builderParams.vertexMerge_Enable = true;
    builderParams.vertexMerge_DistanceTolerance = rwpmath::MAX_FLOAT;
    builderParams.vertexMerge_ScaleTolerance = false;

    uint32_t numTriangles = (xCount * yCount * zCount * 2);
    uint32_t numVertices = numTriangles * 3;
    uint32_t numMergePlanes = 0;

    // Initialize the builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(
        numTriangles,
        numVertices,
        numMergePlanes,
        builderParams,
        EA::Allocator::ICoreAllocator::GetDefaultAllocator());  // TODO: This test leaks.

    // Set the input
    uint32_t triangleIndex = 0;
    uint32_t vertexIndex = 0;
    SetGridInput(
        offlineBuilder,
        triangleIndex,
        vertexIndex,
        xCount,
        yCount,
        zCount,
        1.0f,
        1.0f,
        1.0f);

    // Create the clusteredmesh
    rw::collision::ClusteredMesh * clusteredMesh = offlineBuilder.BuildClusteredMesh();

    // Expect a valid clustered mesh pointer
    EATESTAssert(NULL == clusteredMesh, "clusteredMesh should be NULL");
}


uint32_t TestClusteredMeshBuilder::CheckNextEdge(rwpmath::Vector3& edgeA,
                                                 rwpmath::Vector3& edgeB,
                                                 rwpmath::Vector3& edgeC)
{
    rwpmath::VecFloat AdotB(rwpmath::Dot(edgeA, edgeB));

    // Check that the next vector is on the negative size of the halfspace defined by A + B
    rwpmath::Vector3 halfSpace(edgeA + edgeB);
    if (rwpmath::Dot(halfSpace, -edgeC) >= rwpmath::GetVecFloat_Zero())
    {
        if ( (rwpmath::Dot(-edgeC, edgeA) >= AdotB) &&
             (rwpmath::Dot(-edgeC, edgeB) >= AdotB) )
        {
            // Point lies in no-tilt-zone, therefore vertex can be disabled.
            return VERTEX_DISABLED;
        }
    }

    const rwpmath::VecFloat AdotC(rwpmath::Dot(edgeA, edgeC));
    const rwpmath::VecFloat BdotC(rwpmath::Dot(edgeB, edgeC));
    if ( AdotC < BdotC && AdotC < AdotB)
    {
        edgeB = edgeC;
        return EDGE_B_UPDATED;
    }
    else if (BdotC < AdotB)
    {
        edgeA = edgeC;
        return EDGE_A_UPDATED;
    }

    return NON_CONTRIBUTOR;
}


void TestClusteredMeshBuilder::TestVertexSmooth_VertexDisabled()
{
    const uint32_t edgeCount = 3;
    rwpmath::Vector3 edgeVectors[edgeCount];
    uint32_t edgeAIndex = 0;
    uint32_t edgeBIndex = 1;
    uint32_t edgeCIndex = 2;

    edgeVectors[edgeAIndex] = rwpmath::Vector3(-1.0f, -1.0f, 0.0f);
    edgeVectors[edgeBIndex] = rwpmath::Vector3(1.0f, -1.0f, 0.0f);
    edgeVectors[edgeCIndex] =rwpmath::Vector3(0.0f, 1.0f, 0.0f);


    for (uint32_t i = 0 ; i < edgeCount ; ++i)
    {
        edgeVectors[i] = rwpmath::Normalize(edgeVectors[i]);
    }

    uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);

    EATESTAssert(ret == VERTEX_DISABLED, ("Vertex should have been disabled"));
}

void TestClusteredMeshBuilder::TestVertexSmooth_Edge_B_Updated()
{
    const uint32_t edgeCount = 3;
    rwpmath::Vector3 edgeVectors[edgeCount];
    uint32_t edgeAIndex = 0;
    uint32_t edgeBIndex = 1;
    uint32_t edgeCIndex = 2;

    edgeVectors[edgeAIndex] = rwpmath::Vector3(-1.0f, -1.0f, 0.0f);
    edgeVectors[edgeBIndex] = rwpmath::Vector3(1.0f, -1.0f, 0.0f);
    edgeVectors[edgeCIndex] =rwpmath::Vector3(5.0f, -1.0f, 0.0f);


    for (uint32_t i = 0 ; i < edgeCount ; ++i)
    {
        edgeVectors[i] = rwpmath::Normalize(edgeVectors[i]);
    }

    uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);

    EATESTAssert(ret == EDGE_B_UPDATED, ("Edge B Should have been updated"));
}

void TestClusteredMeshBuilder::TestVertexSmooth_Edge_A_Updated()
{
    const uint32_t edgeCount = 3;
    rwpmath::Vector3 edgeVectors[edgeCount];
    uint32_t edgeAIndex = 0;
    uint32_t edgeBIndex = 1;
    uint32_t edgeCIndex = 2;

    edgeVectors[edgeAIndex] = rwpmath::Vector3(-1.0f, -1.0f, 0.0f);
    edgeVectors[edgeBIndex] = rwpmath::Vector3(1.0f, -1.0f, 0.0f);
    edgeVectors[edgeCIndex] =rwpmath::Vector3(-5.0f, -1.0f, 0.0f);


    for (uint32_t i = 0 ; i < edgeCount ; ++i)
    {
        edgeVectors[i] = rwpmath::Normalize(edgeVectors[i]);
    }

    uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);

    EATESTAssert(ret == EDGE_A_UPDATED, ("Edge A Should have been updated"));
}

void TestClusteredMeshBuilder::TestVertexSmooth_Non_Contributor()
{
    const uint32_t edgeCount = 3;
    rwpmath::Vector3 edgeVectors[edgeCount];
    uint32_t edgeAIndex = 0;
    uint32_t edgeBIndex = 1;
    uint32_t edgeCIndex = 2;

    edgeVectors[edgeAIndex] = rwpmath::Vector3(-1.0f, -1.0f, 0.0f);
    edgeVectors[edgeBIndex] = rwpmath::Vector3(1.0f, -1.0f, 0.0f);
    edgeVectors[edgeCIndex] =rwpmath::Vector3(0.0f, -1.0f, 0.0f);

    for (uint32_t i = 0 ; i < edgeCount ; ++i)
    {
        edgeVectors[i] = rwpmath::Normalize(edgeVectors[i]);
    }

    uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);

    EATESTAssert(ret == NON_CONTRIBUTOR, ("Edge should have been a non-contributor"));
}

void TestClusteredMeshBuilder::TestVertexSmooth_Non_Contributor_OnLimit()
{
    const uint32_t edgeCount = 3;
    rwpmath::Vector3 edgeVectors[edgeCount];
    uint32_t edgeAIndex = 0;
    uint32_t edgeBIndex = 1;
    uint32_t edgeCIndex = 2;

    edgeVectors[edgeAIndex] = rwpmath::Vector3(-1.0f, -1.0f, 0.0f);
    edgeVectors[edgeBIndex] = rwpmath::Vector3(1.0f, -1.0f, 0.0f);
    edgeVectors[edgeCIndex] =rwpmath::Vector3(1.0f, -1.0f, 0.0f);

    for (uint32_t i = 0 ; i < edgeCount ; ++i)
    {
        edgeVectors[i] = rwpmath::Normalize(edgeVectors[i]);
    }

    uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);

    EATESTAssert(ret == NON_CONTRIBUTOR, ("Edge should have been a non-contributor"));
}


void TestClusteredMeshBuilder::TestVertexSmooth_Exanding_Group_Vertex_Disabled()
{
    const uint32_t edgeCount = 9;
    rwpmath::Vector3 edgeVectors[edgeCount];
    const uint32_t edgeAIndex = 0;
    const uint32_t edgeBIndex = 1;
    uint32_t edgeCIndex = 2;

    edgeVectors[0] = rwpmath::Vector3(-1.0f, -7.0f, 0.0f);
    edgeVectors[1] = rwpmath::Vector3(1.0f, -7.0f, 0.0f);
    edgeVectors[2] = rwpmath::Vector3(-2.0f, -6.0f, 0.0f);
    edgeVectors[3] = rwpmath::Vector3(2.0f, -6.0f, 0.0f);
    edgeVectors[4] = rwpmath::Vector3(-3.0f, -5.0f, 0.0f);
    edgeVectors[5] = rwpmath::Vector3(3.0f, -5.0f, 0.0f);
    edgeVectors[6] = rwpmath::Vector3(-4.0f, -4.0f, 0.0f);
    edgeVectors[7] = rwpmath::Vector3(4.0f, -4.0f, 0.0f);
    edgeVectors[8] = rwpmath::Vector3(0.0f, 1.0f, 0.0f);

    for (uint32_t i = 0 ; i < edgeCount ; ++i)
    {
        edgeVectors[i] = rwpmath::Normalize(edgeVectors[i]);
    }

    for (; edgeCIndex < edgeCount - 1; ++edgeCIndex)
    {
        uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);
        uint32_t expected = (edgeCIndex % 2 ? EDGE_B_UPDATED : EDGE_A_UPDATED);
        EATESTAssert(ret == expected, ("Edge should have been updated"));
    }

    uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);
    EATESTAssert(ret == VERTEX_DISABLED, ("Vertex should have been disabled"));
}


void TestClusteredMeshBuilder::TestVertexSmooth_VertexDisabled_OnLimit()
{
    const uint32_t edgeCount = 10;
    rwpmath::Vector3 edgeVectors[edgeCount];
    const uint32_t edgeAIndex = 0;
    const uint32_t edgeBIndex = 1;
    uint32_t edgeCIndex = 2;

    edgeVectors[0] = rwpmath::Vector3(-1.0f, -7.0f, 0.0f);
    edgeVectors[1] = rwpmath::Vector3(1.0f, -7.0f, 0.0f);
    edgeVectors[2] = rwpmath::Vector3(-2.0f, -6.0f, 0.0f);
    edgeVectors[3] = rwpmath::Vector3(2.0f, -6.0f, 0.0f);
    edgeVectors[4] = rwpmath::Vector3(-3.0f, -5.0f, 0.0f);
    edgeVectors[5] = rwpmath::Vector3(3.0f, -5.0f, 0.0f);
    edgeVectors[6] = rwpmath::Vector3(-4.0f, -4.0f, 0.0f);
    edgeVectors[7] = rwpmath::Vector3(4.0f, -4.0f, 0.0f);
    edgeVectors[8] = rwpmath::Vector3(-1.0f, 0.0f, 0.0f);
    edgeVectors[9] = rwpmath::Vector3(1.0f, 0.0f, 0.0f);

    for (uint32_t i = 0 ; i < edgeCount ; ++i)
    {
        edgeVectors[i] = rwpmath::Normalize(edgeVectors[i]);
    }

    for (; edgeCIndex < edgeCount - 1; ++edgeCIndex)
    {
        uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);
        uint32_t expected = (edgeCIndex % 2 ? EDGE_B_UPDATED : EDGE_A_UPDATED);
        EATESTAssert(ret == expected, ("Edge should have been updated"));
    }

    uint32_t ret = CheckNextEdge(edgeVectors[edgeAIndex], edgeVectors[edgeBIndex], edgeVectors[edgeCIndex]);
    EATESTAssert(ret == VERTEX_DISABLED, ("Vertex should have been disabled"));
}
