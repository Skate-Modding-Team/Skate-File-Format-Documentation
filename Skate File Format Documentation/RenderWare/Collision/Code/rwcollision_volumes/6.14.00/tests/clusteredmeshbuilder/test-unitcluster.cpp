// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/meshbuilder/detail/unitclusterbuilder.h>
#include <rw/collision/meshbuilder/detail/unitclusterstack.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/types.h>

#include "random.hpp"

#include "testsuitebase.h" // For TestSuiteBase

// Unit tests for the unit cluster builder

using namespace rw::collision::meshbuilder::detail;
using namespace rw::collision;

class TestUnitCluster : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestUnitCluster");

        EATEST_REGISTER("TestConstructor", "Testing the construction", TestUnitCluster, TestConstructor);
        EATEST_REGISTER("TestReset", "Testing the reset method", TestUnitCluster, TestReset);
        EATEST_REGISTER("TestSortAndCompressVertexSetNonRandom", "Testing SortAndCompressVertexSet", TestUnitCluster, TestSortAndCompressVertexSetNonRandom);
        EATEST_REGISTER("TestSortAndCompressVertexSetPseudoRandom", "Testing SortAndCompressVertexSet", TestUnitCluster, TestSortAndCompressVertexSetPseudoRandom);
        EATEST_REGISTER("TestGetVertexCode", "Testing GetVertexCode", TestUnitCluster, TestGetVertexCode);
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    }

private:

    void TestConstructor();
    void TestReset();
    void TestSortAndCompressVertexSetNonRandom();
    void TestSortAndCompressVertexSetPseudoRandom();
    void TestGetVertexCode();

    bool CompareVertex32(ClusteredMeshCluster::Vertex32 & expected,
                         ClusteredMeshCluster::Vertex32 & actual)
    {
        return (expected.x == actual.x) && (expected.y == actual.y) && (expected.z == actual.z);
    }

    EA::Allocator::ICoreAllocator * m_allocator;

} TestUnitClusterSingleton;

/**
Construct a single UnitCluster
*/
void
TestUnitCluster::TestConstructor()
{
    UnitCluster unitCluster;

    ClusteredMeshCluster::Vertex32 expectedClusterOffset;
    expectedClusterOffset.x = 0;
    expectedClusterOffset.y = 0;
    expectedClusterOffset.z = 0;

    EATESTAssert(0 == unitCluster.clusterID,("clusterID should be 0"));
    EATESTAssert(CompareVertex32(expectedClusterOffset, unitCluster.clusterOffset),("clusterOffset should be (0, 0, 0)"));
    EATESTAssert(NULL == unitCluster.unitIDs,("unitIDs should be NULL"));
    EATESTAssert(0 == unitCluster.numUnits,("numUnits should be 0"));
    EATESTAssert(0 == unitCluster.numVertices,("numVertices should be 0"));
    EATESTAssert(ClusteredMeshCluster::VERTICES_UNCOMPRESSED == unitCluster.compressionMode,("compressionMode should be VERTICES_UNCOMPRESSED"));
}

/**
Create a UnitCluster, set it to a non-initial state and reset it.
*/
void
TestUnitCluster::TestReset()
{
    UnitCluster unitCluster;

    uint32_t addressUint;

    // Set the unitCluster members
    unitCluster.clusterID = 128;
    unitCluster.numVertices = 128;
    unitCluster.numUnits = 128;
    unitCluster.unitIDs = &addressUint;

    const uint32_t expectedClusterID = 16;
    ClusteredMeshCluster::Vertex32 expectedClusterOffset;
    expectedClusterOffset.x = 0;
    expectedClusterOffset.y = 0;
    expectedClusterOffset.z = 0;

    // Reset the UnitCluster
    unitCluster.Reset(expectedClusterID, NULL);

    // Check the members which should have changed
    EATESTAssert(unitCluster.clusterID == expectedClusterID,("clusterID should be 16"));
    EATESTAssert(NULL == unitCluster.unitIDs,("unitIDs should be NULL"));
    EATESTAssert(0 == unitCluster.numUnits,("numUnits should be 0"));
    EATESTAssert(0 == unitCluster.numVertices,("numVertices should be 0"));

    // These members should not have been altered
    EATESTAssert(CompareVertex32(expectedClusterOffset, unitCluster.clusterOffset),("clusterOffset should be (0, 0, 0)"));
    EATESTAssert(ClusteredMeshCluster::VERTICES_UNCOMPRESSED == unitCluster.compressionMode,("compressionMode should be VERTICES_UNCOMPRESSED"));
}

/**
Construct a single UnitCluster with a number of known vertices.
Sort and compress that vertex set.
*/
void
TestUnitCluster::TestSortAndCompressVertexSetNonRandom()
{
    UnitCluster unitCluster;

    // Set the unitCluster vertex count and its vertices
    unitCluster.numVertices = 16;

    unitCluster.vertexIDs[0] = 34;
    unitCluster.vertexIDs[1] = 4567;
    unitCluster.vertexIDs[2] = 987;
    unitCluster.vertexIDs[3] = 986;
    unitCluster.vertexIDs[4] = 985;
    unitCluster.vertexIDs[5] = 989;
    unitCluster.vertexIDs[6] = 34;
    unitCluster.vertexIDs[7] = 4567;
    unitCluster.vertexIDs[8] = 1;
    unitCluster.vertexIDs[9] = 0;
    unitCluster.vertexIDs[10] = 9356;
    unitCluster.vertexIDs[11] = 26;
    unitCluster.vertexIDs[12] = 4652;
    unitCluster.vertexIDs[13] = 67823;
    unitCluster.vertexIDs[14] = 83;
    unitCluster.vertexIDs[15] = 34;

    UnitCluster::SortAndCompressVertexSet(unitCluster.vertexIDs, unitCluster.numVertices);

    EATESTAssert(13 == unitCluster.numVertices, ("Vertex count should be 13"));

    EATESTAssert(0 == unitCluster.vertexIDs[0], ("Vertex index should be 0"));
    EATESTAssert(1 == unitCluster.vertexIDs[1], ("Vertex index should be 1"));
    EATESTAssert(26 == unitCluster.vertexIDs[2], ("Vertex index should be 26"));
    EATESTAssert(34 == unitCluster.vertexIDs[3], ("Vertex index should be 34"));
    EATESTAssert(83 == unitCluster.vertexIDs[4], ("Vertex index should be 83"));
    EATESTAssert(985 == unitCluster.vertexIDs[5], ("Vertex index should be 985"));
    EATESTAssert(986 == unitCluster.vertexIDs[6], ("Vertex index should be 986"));
    EATESTAssert(987 == unitCluster.vertexIDs[7], ("Vertex index should be 987"));
    EATESTAssert(989 == unitCluster.vertexIDs[8], ("Vertex index should be 989"));
    EATESTAssert(4567 == unitCluster.vertexIDs[9], ("Vertex index should be 4567"));
    EATESTAssert(4652 == unitCluster.vertexIDs[10], ("Vertex index should be 4652"));
    EATESTAssert(9356 == unitCluster.vertexIDs[11], ("Vertex index should be 9356"));
    EATESTAssert(67823 == unitCluster.vertexIDs[12], ("Vertex index should be 67823"));
}

/**
Construct a single UnitCluster with a number of pseudo random vertices.
Sort and compress that vertex set.
*/
void
TestUnitCluster::TestSortAndCompressVertexSetPseudoRandom()
{
    UnitCluster unitCluster;

    // Set the unitCluster vertex count and its vertices
    unitCluster.numVertices = 255;

    SeedRandom(9u);

    const uint32_t low = 0u;
    const uint32_t high = 128u;

    for (uint32_t vertexIndex = 0 ; vertexIndex < unitCluster.numVertices ; ++vertexIndex)
    {
        unitCluster.vertexIDs[vertexIndex] = Random(low, high);
    }

    UnitCluster::SortAndCompressVertexSet(unitCluster.vertexIDs, unitCluster.numVertices);

    uint32_t vertexId = unitCluster.vertexIDs[0];
    uint32_t vertexCount = 1;

    for (uint32_t vertexIndex = 1 ; vertexIndex < unitCluster.numVertices ; ++vertexIndex)
    {
        EATESTAssert(vertexId <= unitCluster.vertexIDs[vertexIndex], ("Vertex index is out of order"));

        if (vertexId < unitCluster.vertexIDs[vertexIndex])
        {
            ++vertexCount;
        }

        vertexId = unitCluster.vertexIDs[vertexIndex];
    }

    EATESTAssert(vertexCount == unitCluster.numVertices, ("Vertex count is incorrect"));
}

/**
Construct a single UnitCluster with a number of known vertices.
Check the indices of those vertices.
*/
void
TestUnitCluster::TestGetVertexCode()
{
    UnitCluster unitCluster;

    // Set the unitCluster vertex count and its vertices
    unitCluster.numVertices = 16;

    unitCluster.vertexIDs[0] = 34;
    unitCluster.vertexIDs[1] = 4567;
    unitCluster.vertexIDs[2] = 987;
    unitCluster.vertexIDs[3] = 986;
    unitCluster.vertexIDs[4] = 985;
    unitCluster.vertexIDs[5] = 989;
    unitCluster.vertexIDs[6] = 34;
    unitCluster.vertexIDs[7] = 4567;
    unitCluster.vertexIDs[8] = 1;
    unitCluster.vertexIDs[9] = 0;
    unitCluster.vertexIDs[10] = 9356;
    unitCluster.vertexIDs[11] = 26;
    unitCluster.vertexIDs[12] = 4652;
    unitCluster.vertexIDs[13] = 67823;
    unitCluster.vertexIDs[14] = 83;
    unitCluster.vertexIDs[15] = 34;

    UnitCluster::SortAndCompressVertexSet(unitCluster.vertexIDs, unitCluster.numVertices);

    EATESTAssert(0 == unitCluster.GetVertexCode(0), ("Vertex index should be 0"));
    EATESTAssert(1 == unitCluster.GetVertexCode(1), ("Vertex index should be 1"));
    EATESTAssert(2 == unitCluster.GetVertexCode(26), ("Vertex index should be 2"));
    EATESTAssert(3 == unitCluster.GetVertexCode(34), ("Vertex index should be 3"));
    EATESTAssert(4 == unitCluster.GetVertexCode(83), ("Vertex index should be 4"));
    EATESTAssert(5 == unitCluster.GetVertexCode(985), ("Vertex index should be 5"));
    EATESTAssert(6 == unitCluster.GetVertexCode(986), ("Vertex index should be 6"));
    EATESTAssert(7 == unitCluster.GetVertexCode(987), ("Vertex index should be 7"));
    EATESTAssert(8 == unitCluster.GetVertexCode(989), ("Vertex index should be 8"));
    EATESTAssert(9 == unitCluster.GetVertexCode(4567), ("Vertex index should be 9"));
    EATESTAssert(10 == unitCluster.GetVertexCode(4652), ("Vertex index should be 10"));
    EATESTAssert(11 == unitCluster.GetVertexCode(9356), ("Vertex index should be 11"));
    EATESTAssert(12 == unitCluster.GetVertexCode(67823), ("Vertex index should be 12"));
}
