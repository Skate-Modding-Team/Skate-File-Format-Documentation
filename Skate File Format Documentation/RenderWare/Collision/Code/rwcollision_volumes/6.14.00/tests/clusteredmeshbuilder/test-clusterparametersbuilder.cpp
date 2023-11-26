// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <rw/collision/libcore.h>
#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/detail/clusterparametersbuilder.h>
#include <rw/collision/clusteredmeshcluster.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;
using namespace rw::collision::meshbuilder;
using namespace rw::collision::meshbuilder::detail;

// Unit tests for clustered mesh builder utilities
class TestClusterParameterBuilder : public tests::TestSuiteBase
{

public:

    virtual void Initialize()
    {
        SuiteName("TestClusterParameterBuilder");

        EATEST_REGISTER("TestInitializeClusterParameters", "Test InitializeClusterParameters", TestClusterParameterBuilder, TestInitializeClusterParameters);
        EATEST_REGISTER("TestSumUnitComponentCountsSingleTriangle", "Test SumUnitComponentCounts", TestClusterParameterBuilder, TestSumUnitComponentCountsSingleTriangle);
        EATEST_REGISTER("TestSumUnitComponentCountsSingleQuad", "Test SumUnitComponentCounts", TestClusterParameterBuilder, TestSumUnitComponentCountsSingleQuad);
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    }

private:

    void TestInitializeClusterParameters();
    void TestSumUnitComponentCountsSingleTriangle();
    void TestSumUnitComponentCountsSingleQuad();

    EA::Allocator::ICoreAllocator * m_allocator;

} TestClusterParameterBuilderSingleton;

/**
Tests the InitializeClusterParameters method
*/
void
TestClusterParameterBuilder::TestInitializeClusterParameters()
{
    // Initialize the UnitCluster
    const uint32_t unitCount = 4;
    uint32_t unitIDs[unitCount] = {0, 1, 2, 3};
    UnitCluster unitCluster;
    unitCluster.clusterID = 0;
    unitCluster.numVertices = unitCount * 3;
    unitCluster.numUnits = unitCount;
    unitCluster.unitIDs = unitIDs;
    unitCluster.compressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;

    // Initialize TriangleSurfaceIDList
    TriangleSurfaceIDList * triangleSurfaceIDs = TriangleSurfaceIDList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleSurfaceIDs, ("TriangleSurfaceIDList should have been allocated"));
    triangleSurfaceIDs->resize(unitCount);
    (*triangleSurfaceIDs)[0] = 0x01;
    (*triangleSurfaceIDs)[1] = 0x01;
    (*triangleSurfaceIDs)[2] = 0x01;
    (*triangleSurfaceIDs)[3] = 0x01;

    // Initialize TriangleGroupIDList
    TriangleGroupIDList * triangleGroupIDs = TriangleGroupIDList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleGroupIDs, ("TriangleGroupIDList should have been allocated"));
    triangleGroupIDs->resize(unitCount);
    (*triangleGroupIDs)[0] = 0x01;
    (*triangleGroupIDs)[1] = 0x01;
    (*triangleGroupIDs)[2] = 0x01;
    (*triangleGroupIDs)[3] = 0x01;

    // Initialize UnitList
    UnitList * units = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    EATESTAssert(units, ("UnitList should have been allocated"));
    units->resize(unitCount);
    (*units)[0].tri0 = 0;
    (*units)[0].type = Unit::TYPE_TRIANGLE;
    (*units)[1].tri0 = 1;
    (*units)[1].type = Unit::TYPE_TRIANGLE;
    (*units)[2].tri0 = 2;
    (*units)[2].type = Unit::TYPE_TRIANGLE;
    (*units)[3].tri0 = 3;
    (*units)[3].type = Unit::TYPE_QUAD;

    // Initialize UnitParameters
    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_GROUPID | rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_EDGEANGLE;
    unitParameters.groupIDSize = 2;
    unitParameters.surfaceIDSize = 2;

    ClusterConstructionParameters constructionParameters;

    ClusterParametersBuilder::InitializeClusterParameters(
        constructionParameters,
        unitCluster,
        *triangleSurfaceIDs,
        *triangleGroupIDs,
        *units,
        unitParameters);

    // Check the state of the ClusterConstructionParameter
    EATESTAssert(12 == constructionParameters.mVertexCount, ("Vertex Count should be 12"));
    EATESTAssert(ClusteredMeshCluster::VERTICES_UNCOMPRESSED == constructionParameters.mVertexCompressionMode, ("Vertex compression mode should be VERTICES_UNCOMPRESSED"));
    EATESTAssert(3 == constructionParameters.mTriangleUnitCount, ("Triangle Unit Count should be 3"));
    EATESTAssert(1 == constructionParameters.mQuadUnitCount, ("Quad Unit Count should be 1"));
    EATESTAssert(13 == constructionParameters.mEdgeCosineCount, ("Edge Cosine Count should be 13"));
    EATESTAssert(4 == constructionParameters.mGroupIDCount, ("Group ID Count should be 8"));
    EATESTAssert(2 == constructionParameters.mGroupIDSize, ("Group ID Size Count should be 2"));
    EATESTAssert(4 == constructionParameters.mSurfaceIDCount, ("Surface ID Count should be 8"));
    EATESTAssert(2 == constructionParameters.mSurfaceIDSize, ("Surface ID Size should be 2"));

    UnitList::Free(m_allocator, units);
    TriangleGroupIDList::Free(m_allocator, triangleGroupIDs);
    TriangleSurfaceIDList::Free(m_allocator, triangleSurfaceIDs);
}


/**
Tests the SumUnitComponentCounts method.
*/
void
TestClusterParameterBuilder::TestSumUnitComponentCountsSingleTriangle()
{
    const uint32_t unitType = Unit::TYPE_TRIANGLE;
    const uint32_t flagsDefault = rw::collision::UNITFLAG_GROUPID | rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_EDGEANGLE;
    const TriangleGroupID groupID = 1;
    const TriangleSurfaceID surfaceID = 1;

    rw::collision::ClusterConstructionParameters parameters;

    ClusterParametersBuilder::SumUnitComponentCounts(
        parameters,
        unitType,
        flagsDefault,
        groupID,
        surfaceID);

    EATESTAssert(1 == parameters.mGroupIDCount, ("Group ID Count should be 1"));
    EATESTAssert(1 == parameters.mSurfaceIDCount, ("Surface ID Count should be 1"));
    EATESTAssert(1 == parameters.mTriangleUnitCount, ("Triangle Unit Count should be 1"));
    EATESTAssert(3 == parameters.mEdgeCosineCount, ("Edge Cosine Count should be 3"));
}

/**
Tests the SumUnitComponentCounts method.
*/
void
TestClusterParameterBuilder::TestSumUnitComponentCountsSingleQuad()
{
    const uint32_t unitType = Unit::TYPE_QUAD;
    const uint32_t flagsDefault = rw::collision::UNITFLAG_GROUPID | rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_EDGEANGLE;
    const TriangleGroupID groupID = 1;
    const TriangleSurfaceID surfaceID = 1;

    rw::collision::ClusterConstructionParameters parameters;

    ClusterParametersBuilder::SumUnitComponentCounts(
        parameters,
        unitType,
        flagsDefault,
        groupID,
        surfaceID);

    EATESTAssert(1 == parameters.mGroupIDCount, ("Group ID Count should be 1"));
    EATESTAssert(1 == parameters.mSurfaceIDCount, ("Surface ID Count should be 1"));
    EATESTAssert(1 == parameters.mQuadUnitCount, ("Quad Unit Count should be 1"));
    EATESTAssert(4 == parameters.mEdgeCosineCount, ("Edge Cosine Count should be 4"));
}

