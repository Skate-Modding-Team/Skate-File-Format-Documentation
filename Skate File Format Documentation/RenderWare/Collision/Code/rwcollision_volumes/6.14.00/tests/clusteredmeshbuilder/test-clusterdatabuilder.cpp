// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/meshbuilder/detail/unitclusterstack.h>
#include <rw/collision/meshbuilder/detail/unitclusterbuilder.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/clusterdatabuilder.h>
#include <rw/collision/meshbuilder/edgecodegenerator.h>
#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/types.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision::meshbuilder::detail;
using namespace rw::collision::meshbuilder;
using namespace rw::collision;

class TestClusterDataBuilder : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestClusterDataBuilder");

        EATEST_REGISTER("TestBuild", "Testing Build", TestClusterDataBuilder, TestBuild);
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    }

private:

    void TestBuild();

    bool CompareVertex32(ClusteredMeshCluster::Vertex32 & expected,
                         ClusteredMeshCluster::Vertex32 & actual)
    {
        return (expected.x == actual.x) && (expected.y == actual.y) && (expected.z == actual.z);
    }

    bool CompareVertex(meshbuilder::VectorType & expected,
                       rwpmath::Vector3 & actual)
    {
        return (expected.x == actual.GetX()) && (expected.y == actual.GetY()) && (expected.z == actual.GetZ());
    }

    EA::Allocator::ICoreAllocator * m_allocator;

} TestClusterDataBuilderSingleton;

/**
Construct a single UnitCluster
*/
void
TestClusterDataBuilder::TestBuild()
{
    const uint32_t numTriangles = 3;
    const uint32_t numVertices = 6;

    // Initialize triangles
    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);
    (*triangles)[0].vertices[0] = 0;
    (*triangles)[0].vertices[1] = 1;
    (*triangles)[0].vertices[2] = 2;
    (*triangles)[1].vertices[0] = 2;
    (*triangles)[1].vertices[1] = 3;
    (*triangles)[1].vertices[2] = 4;
    (*triangles)[2].vertices[0] = 4;
    (*triangles)[2].vertices[1] = 3;
    (*triangles)[2].vertices[2] = 5;

    // Initialize vertices
    VertexList * vertices = VertexList::Allocate(m_allocator, numVertices, EA::Allocator::MEM_PERM);
    EATESTAssert(vertices, ("VertexList should have been allocated"));
    vertices->resize(numVertices);
    (*vertices)[0] = meshbuilder::VectorType(0.0f, 0.0f, 0.0f);
    (*vertices)[1] = meshbuilder::VectorType(1.0f, 0.0f, 0.0f);
    (*vertices)[2] = meshbuilder::VectorType(0.0f, 0.0f, 1.0f);
    (*vertices)[3] = meshbuilder::VectorType(1.0f, 0.0f, 1.0f);
    (*vertices)[4] = meshbuilder::VectorType(0.0f, 0.0f, 2.0f);
    (*vertices)[5] = meshbuilder::VectorType(1.0f, 0.0f, 2.0f);

    //Initialize TriangleEdgeCodes
    TriangleEdgeCodesList * triangleEdgeCodes = TriangleEdgeCodesList::Allocate(m_allocator, 3, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCodes, ("TriangleEdgeCodesList should have been allocated"));
    triangleEdgeCodes->resize(3);
    (*triangleEdgeCodes)[0].encodedEdgeCos[0] = EdgeCodeGenerator::GenerateEdgeCode(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, 0.0f, false);
    (*triangleEdgeCodes)[0].encodedEdgeCos[1] = EdgeCodeGenerator::GenerateEdgeCode(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, 0.0f, false);
    (*triangleEdgeCodes)[0].encodedEdgeCos[2] = EdgeCodeGenerator::GenerateEdgeCode(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, 0.0f, false);
    (*triangleEdgeCodes)[1].encodedEdgeCos[0] = EdgeCodeGenerator::GenerateEdgeCode(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, 0.0f, false);
    (*triangleEdgeCodes)[1].encodedEdgeCos[1] = EdgeCodeGenerator::GenerateEdgeCode(1.0f, 0.0f, false);
    (*triangleEdgeCodes)[1].encodedEdgeCos[2] = EdgeCodeGenerator::GenerateEdgeCode(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, 0.0f, false);
    (*triangleEdgeCodes)[2].encodedEdgeCos[0] = EdgeCodeGenerator::GenerateEdgeCode(1.0f, 0.0f, false);
    (*triangleEdgeCodes)[2].encodedEdgeCos[1] = EdgeCodeGenerator::GenerateEdgeCode(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, 0.0f, false);
    (*triangleEdgeCodes)[2].encodedEdgeCos[2] = EdgeCodeGenerator::GenerateEdgeCode(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, 0.0f, false);

    // Initialize TriangleSurfaceIDList
    TriangleSurfaceIDList * triangleSurfaceIDs = TriangleSurfaceIDList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleSurfaceIDs, ("TriangleSurfaceIDList should have been allocated"));
    triangleSurfaceIDs->resize(numTriangles);
    (*triangleSurfaceIDs)[0] = 0x0102;
    (*triangleSurfaceIDs)[1] = 0x0304;
    (*triangleSurfaceIDs)[2] = 0x0304;

    // Initialize TriangleGroupIDList
    TriangleGroupIDList * triangleGroupIDs = TriangleGroupIDList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleGroupIDs, ("TriangleGroupIDList should have been allocated"));
    triangleGroupIDs->resize(numTriangles);
    (*triangleGroupIDs)[0] = 0x0A0B;
    (*triangleGroupIDs)[1] = 0x0C0D;
    (*triangleGroupIDs)[2] = 0x0C0D;

    // Initialize UnitList
    UnitList * units = UnitList::Allocate(m_allocator, 2, EA::Allocator::MEM_PERM);
    EATESTAssert(units, ("UnitList should have been allocated"));
    units->resize(2);
    (*units)[0].tri0 = 0;
    (*units)[0].type = Unit::TYPE_TRIANGLE;
    (*units)[1].tri0 = 1;
    (*units)[1].tri1 = 2;
    (*units)[1].type = Unit::TYPE_QUAD;
    (*units)[1].extraVertex = 2;
    (*units)[1].edgeOpposingExtraVertex = 1;
    (*units)[1].longestEdgeOnTri1 = 0;

    // Initialize unitCluster
    uint32_t unitIDs[2] = {0, 1};
    UnitCluster unitCluster;
    unitCluster.clusterID = 0;
    unitCluster.unitIDs = unitIDs;
    unitCluster.numUnits = 2;
    unitCluster.vertexIDs[0] = 0;
    unitCluster.vertexIDs[1] = 1;
    unitCluster.vertexIDs[2] = 2;
    unitCluster.vertexIDs[3] = 3;
    unitCluster.vertexIDs[4] = 4;
    unitCluster.vertexIDs[5] = 5;
    unitCluster.numVertices = 6;
    unitCluster.compressionMode = ClusteredMeshCluster::VERTICES_UNCOMPRESSED;

    // Initialize UnitParameters
    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_GROUPID | rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_EDGEANGLE;
    unitParameters.groupIDSize = 2;
    unitParameters.surfaceIDSize = 2;

    // Initialize ClusterConstructionParameters
    rw::collision::ClusterConstructionParameters clusterConstructionParams;
    clusterConstructionParams.mVertexCount = numVertices;
    clusterConstructionParams.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    clusterConstructionParams.mTriangleUnitCount = 1;
    clusterConstructionParams.mQuadUnitCount = 1;
    clusterConstructionParams.mEdgeCosineCount = 7;
    clusterConstructionParams.mGroupIDCount = 2;
    clusterConstructionParams.mGroupIDSize = 2;
    clusterConstructionParams.mSurfaceIDCount = 2;
    clusterConstructionParams.mSurfaceIDSize = 2;

    // Initialize ClusteredMeshCluster
    uint16_t clusterSize =rw::collision::ClusteredMeshCluster::GetSize(clusterConstructionParams);
    void * clusterBuffer = m_allocator->Alloc(clusterSize, NULL, 0, 16);
    rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(clusterBuffer, clusterConstructionParams);

    ClusterDataBuilder::Build(
        *cluster,
        unitCluster,
        *vertices,
        *triangles,
        *triangleEdgeCodes,
        *triangleSurfaceIDs,
        *triangleGroupIDs,
        *units,
        unitParameters,
        0.0f);

    // Check the validity of the Cluster Counts, Starts and Compression Mode
    EATESTAssert(2 == cluster->unitCount, ("Cluster unit count should be 2"));
    EATESTAssert(24 == cluster->unitDataSize, ("Cluster unit data size should be 24"));
    EATESTAssert(6 == cluster->unitDataStart, ("Cluster unit data start should be 6"));
    EATESTAssert(6 == cluster->normalStart, ("Cluster normal start should be 6"));
    EATESTAssert(136 == cluster->totalSize, ("Cluster total size should be 136"));
    EATESTAssert(6 == cluster->vertexCount, ("Cluster vertex count should be 6"));
    EATESTAssert(0 == cluster->normalCount, ("Cluster normal count should be 0"));
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED == cluster->compressionMode, ("Cluster compression mode should be VERTICES_UNCOMPRESSED"));

    // Check the validity of the Cluster Vertices
    EATESTAssert(CompareVertex((*vertices)[0], cluster->vertexArray[0]), ("Cluster vertex 0 is incorrect"));
    EATESTAssert(CompareVertex((*vertices)[1], cluster->vertexArray[1]), ("Cluster vertex 1 is incorrect"));
    EATESTAssert(CompareVertex((*vertices)[2], cluster->vertexArray[2]), ("Cluster vertex 2 is incorrect"));
    EATESTAssert(CompareVertex((*vertices)[3], cluster->vertexArray[3]), ("Cluster vertex 3 is incorrect"));
    EATESTAssert(CompareVertex((*vertices)[4], cluster->vertexArray[4]), ("Cluster vertex 4 is incorrect"));
    EATESTAssert(CompareVertex((*vertices)[5], cluster->vertexArray[5]), ("Cluster vertex 5 is incorrect"));

    // Check the validity of the Cluster Units
    uint8_t * unitData = cluster->UnitData();
    // TriangleUnit
    // unitCode
    EATESTAssert((rw::collision::UNITFLAG_GROUPID |
                  rw::collision::UNITFLAG_SURFACEID |
                  rw::collision::UNITFLAG_EDGEANGLE |
                  rw::collision::UNITTYPE_TRIANGLE) == unitData[0],("Cluster unit data [0] is incorrect"));
    // vertex 0 code
    EATESTAssert(0 == unitData[1],("Cluster unit data [1] is incorrect"));
    // vertex 1 code
    EATESTAssert(1 == unitData[2],("Cluster unit data [2] is incorrect"));
    // vertex 2 code
    EATESTAssert(2 == unitData[3],("Cluster unit data [3] is incorrect"));
    // edgecos 0 code
    EATESTAssert(160 == unitData[4],("Cluster unit data [4] is incorrect"));
    // edgecos 1 code
    EATESTAssert(160 == unitData[5],("Cluster unit data [5] is incorrect"));
    // edgecos 2 code
    EATESTAssert(160 == unitData[6],("Cluster unit data [6] is incorrect"));
    // group id 0
    EATESTAssert(0x0B == unitData[7],("Cluster unit data [7] is incorrect"));
    // group id 1
    EATESTAssert(0x0A == unitData[8],("Cluster unit data [8] is incorrect"));
    // surface id 0
    EATESTAssert(0x02 == unitData[9],("Cluster unit data [9] is incorrect"));
    // surface id 1
    EATESTAssert(0x01 == unitData[10],("Cluster unit data [10] is incorrect"));

    // Quad Unit
    // unitCode
    EATESTAssert((rw::collision::UNITFLAG_GROUPID |
                  rw::collision::UNITFLAG_SURFACEID |
                  rw::collision::UNITFLAG_EDGEANGLE |
                  rw::collision::UNITTYPE_QUAD) == unitData[11],("Cluster unit data [11] is incorrect"));
    // vertex 0 code
    EATESTAssert(2 == unitData[12],("Cluster unit data [12] is incorrect"));
    // vertex 1 code
    EATESTAssert(3 == unitData[13],("Cluster unit data [13] is incorrect"));
    // vertex 2 code
    EATESTAssert(4 == unitData[14],("Cluster unit data [14] is incorrect"));
    // vertex 3 code
    EATESTAssert(5 == unitData[15],("Cluster unit data [15] is incorrect"));
    // edgecos 0 code
    EATESTAssert(160 == unitData[16],("Cluster unit data [16] is incorrect"));
    // edgecos 1 code
    EATESTAssert(160 == unitData[17],("Cluster unit data [17] is incorrect"));
    // edgecos 2 code
    EATESTAssert(160 == unitData[18],("Cluster unit data [18] is incorrect"));
    // edgecos 3 code
    EATESTAssert(160 == unitData[19],("Cluster unit data [19] is incorrect"));
    // group id 0
    EATESTAssert(0x0D == unitData[20],("Cluster unit data [20] is incorrect"));
    // group id 1
    EATESTAssert(0x0C == unitData[21],("Cluster unit data [21] is incorrect"));
    // surface id 0
    EATESTAssert(0x04 == unitData[22],("Cluster unit data [22] is incorrect"));
    // surface id 1
    EATESTAssert(0x03 == unitData[23],("Cluster unit data [23] is incorrect"));
}

