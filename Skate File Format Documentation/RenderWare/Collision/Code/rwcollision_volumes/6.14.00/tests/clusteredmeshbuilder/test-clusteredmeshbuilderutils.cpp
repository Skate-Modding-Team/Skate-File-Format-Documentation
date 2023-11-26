// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <rw/collision/libcore.h>
#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/vertexcompression.h>
#include <rw/collision/meshbuilder/edgecosines.h>
#include <rw/collision/meshbuilder/edgecodegenerator.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilderutils.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;
using namespace rw::collision::meshbuilder;
using namespace rw::collision::meshbuilder::detail;


// Unit tests for clustered mesh builder utilities
class TestClusteredMeshBuilderUtils : public tests::TestSuiteBase
{

public:

    virtual void Initialize()
    {
        SuiteName("TestClusteredMeshBuilderUtils");

        // GenerateEdgeCode unit test
        EATEST_REGISTER("TestGenerateEdgeCodeMatchedConcaveEdge", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeMatchedConcaveEdge);
        EATEST_REGISTER("TestGenerateEdgeCodeMatchedConvexEdge", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeMatchedConvexEdge);

        EATEST_REGISTER("TestGenerateEdgeCodeUnmatchedConcaveEdge", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeUnmatchedConcaveEdge);
        EATEST_REGISTER("TestGenerateEdgeCodeUnmatchedConvexEdge", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeUnmatchedConvexEdge);

        EATEST_REGISTER("TestGenerateEdgeCodeMatchedForcedConcaveEdge", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeMatchedForcedConcaveEdge);
        EATEST_REGISTER("TestGenerateEdgeCodeUnmatchedForcedConcaveEdge", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeUnmatchedForcedConcaveEdge);

        EATEST_REGISTER("TestGenerateEdgeCodeAboveRangeConcaveTolerance", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeAboveRangeConcaveTolerance);
        EATEST_REGISTER("TestGenerateEdgeCodeBelowRangeConcaveTolerance", "Check encoded edge cosine and flags", TestClusteredMeshBuilderUtils, TestGenerateEdgeCodeBelowRangeConcaveTolerance);

        EATEST_REGISTER("TestCalculateMinimum16BitGranularityForRange", "Check minimum granularity for range", TestClusteredMeshBuilderUtils, TestCalculateMinimum16BitGranularityForRange);
        EATEST_REGISTER("TestDetermineCompressionModeAndOffsetForRange", "Check compression mode and offset", TestClusteredMeshBuilderUtils, TestDetermineCompressionModeAndOffsetForRange);
        EATEST_REGISTER("TestComputeExtendedEdgeCosine", "Check edge cosine", TestClusteredMeshBuilderUtils, TestComputeExtendedEdgeCosine);
        EATEST_REGISTER("TestEdgeCosineToAngleByte", "Check angle byte", TestClusteredMeshBuilderUtils, TestEdgeCosineToAngleByte);
        EATEST_REGISTER("TestEdgeProducesFeaturelessPlane", "Check featureless plane", TestClusteredMeshBuilderUtils, TestEdgeProducesFeaturelessPlane);
        EATEST_REGISTER("TestEdgeDisablesVertex", "Check if edge disables vertex", TestClusteredMeshBuilderUtils, TestEdgeDisablesVertex);
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        rw::collision::InitializeVTables();
    }

private:

    void TestGenerateEdgeCodeMatchedConcaveEdge();
    void TestGenerateEdgeCodeMatchedConvexEdge();

    void TestGenerateEdgeCodeUnmatchedConcaveEdge();
    void TestGenerateEdgeCodeUnmatchedConvexEdge();

    void TestGenerateEdgeCodeMatchedForcedConcaveEdge();
    void TestGenerateEdgeCodeUnmatchedForcedConcaveEdge();

    void TestGenerateEdgeCodeBelowRangeConcaveTolerance();
    void TestGenerateEdgeCodeAboveRangeConcaveTolerance();

    void TestCalculateMinimum16BitGranularityForRange();
    void TestDetermineCompressionModeAndOffsetForRange();
    void TestComputeExtendedEdgeCosine();
    void TestEdgeCosineToAngleByte();
    void TestEdgeProducesFeaturelessPlane();
    void TestEdgeDisablesVertex();

    void TestHowManyBits();

} TestClusteredMeshBuilderUtilsSingleton;

/**
Tests the edge cosine values.
*/
void
TestClusteredMeshBuilderUtils::TestComputeExtendedEdgeCosine()
{
    const rwpmath::Vector3 normalA(1.0f, 0.0f, 0.0f);
    const rwpmath::Vector3 normalB(0.0f, 1.0f, 0.0f);

    const rwpmath::Vector3 edgeVector(0.0f, 0.0f, 1.0f);

    rwpmath::VecFloat edgeCosine = EdgeCosines::ComputeExtendedEdgeCosine(
        normalA,
        normalB,
        edgeVector);

    // Expect an edge cosine of 0
    EATESTAssert(rwpmath::VecFloat(0.0f) == edgeCosine, "Edge cosine should be 2");


    edgeCosine = EdgeCosines::ComputeExtendedEdgeCosine(
        normalA,
        normalB,
        -edgeVector);

    // Expect an edge cosine of 2
    EATESTAssert(rwpmath::VecFloat(2.0f) == edgeCosine, "Edge cosine should be 0");
}


/**
Tests the minimum granularity.
*/
void
TestClusteredMeshBuilderUtils::TestCalculateMinimum16BitGranularityForRange()
{
    const rwpmath::VecFloat xMin(-256.0f);
    const rwpmath::VecFloat xMax(256.0f);
    const rwpmath::VecFloat yMin(-32.0f);
    const rwpmath::VecFloat yMax(32.0f);
    const rwpmath::VecFloat zMin(-64.0f);
    const rwpmath::VecFloat zMax(64.0f);

    rwpmath::VecFloat minimumGranularity = VertexCompression::CalculateMinimum16BitGranularityForRange(
        xMin, xMax,
        yMin, yMax,
        zMin, zMax);

    // Expect an edge cosine of 2
    EATESTAssert(rwpmath::VecFloat(512.0f / 65535.0f) == minimumGranularity, "minimumGranularity should be 1.0f/128.0f");
}


/**
Tests the compression mode and offset.
*/
void
TestClusteredMeshBuilderUtils::TestDetermineCompressionModeAndOffsetForRange()
{
    const int32_t xMin = -256;
    const int32_t xMax = 256;
    const int32_t yMin = -32;
    const int32_t yMax = 32;
    const int32_t zMin = -64;
    const int32_t zMax = 64;

    uint8_t compressionMode = 0u;
    rw::collision::ClusteredMeshCluster::Vertex32 offset;

    VertexCompression::DetermineCompressionModeAndOffsetForRange(
        compressionMode,
        offset,
        xMin, xMax,
        yMin, yMax,
        zMin, zMax);

    // Expect a compression mode of VERTICES_16BIT_COMPRESSED
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED == compressionMode, "compressionMode should be VERTICES_16BIT_COMPRESSED");
    // Expect an x offset of xMin - 1.0f
    EATESTAssert( (xMin - 1.0f) == offset.x, "offset.x should be off - 1.0f");
    // Expect an y offset of yMin - 1.0f
    EATESTAssert( (yMin - 1.0f) == offset.y, "offset.y should be yMin - 1.0f");
    // Expect an z offset of zMin - 1.0f
    EATESTAssert( (zMin - 1.0f) == offset.z, "offset.z should be zMin - 1.0f");
}


/**
Tests the edge cosine value.
*/
void
TestClusteredMeshBuilderUtils::TestEdgeCosineToAngleByte()
{
    rwpmath::VecFloat edgeCosine = 3.0f;
    uint8_t angleByte = ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(edgeCosine);

    // Expect an edge cosine of 2
    EATESTAssert(0u == angleByte, "angleByte should be 26");
}


/**
Tests the GenerateEdgeCode method.
Uses a matched concave extended edge cosine value, and the default concave tolerance.
The expected result should honor the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeMatchedConcaveEdge()
{
    const rwpmath::VecFloat edgeCosine(2.8f);
    const rwpmath::VecFloat concaveTolerance(-1.0f);
    const bool matchedEdge = true;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matchedEdge);

    uint8_t expectedEncodedEdgeData = ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(edgeCosine);

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}

/**
Tests the GenerateEdgeCode method.
Uses a matched convex extended edge cosine value, and the default concave tolerance.
The expected result should honor the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeMatchedConvexEdge()
{
    const rwpmath::VecFloat edgeCosine(0.5f);
    const rwpmath::VecFloat concaveTolerance(-1.0f);
    const bool matchedEdge = true;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matchedEdge);

    uint8_t expectedEncodedEdgeData = ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(edgeCosine);
    expectedEncodedEdgeData |= rw::collision::EDGEFLAG_EDGECONVEX;

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}

/**
Tests the GenerateEdgeCode method.
Uses a unmatched concave extended edge cosine value, and the default concave tolerance.
The expected result should honor the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeUnmatchedConcaveEdge()
{
    const rwpmath::VecFloat edgeCosine(2.5f);
    const rwpmath::VecFloat concaveTolerance(-1.0f);
    const bool matchedEdge = false;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matchedEdge);

    uint8_t expectedEncodedEdgeData = ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(edgeCosine);
    expectedEncodedEdgeData |= rw::collision::EDGEFLAG_EDGEUNMATCHED;

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}

/**
Tests the GenerateEdgeCode method.
Uses a unmatched convex extended edge cosine value, and the default concave tolerance.
The expected result should honor the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeUnmatchedConvexEdge()
{
    const rwpmath::VecFloat edgeCosine(0.5f);
    const rwpmath::VecFloat concaveTolerance(-1.0f);
    const bool matched = false;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matched);

    uint8_t expectedEncodedEdgeData = ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(edgeCosine);
    expectedEncodedEdgeData |= rw::collision::EDGEFLAG_EDGECONVEX |
                               rw::collision::EDGEFLAG_EDGEUNMATCHED;

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}

/**
Tests the GenerateEdgeCode method.
Uses a matched concave extended edge cosine value.
Uses a concave tolerance which should disable the edge.
The expected result should override the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeMatchedForcedConcaveEdge()
{
    const rwpmath::VecFloat edgeCosine(2.0f);
    const rwpmath::VecFloat concaveTolerance(0.1f);
    const bool matched = true;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matched);

    uint8_t expectedEncodedEdgeData = rw::collision::EDGEFLAG_ANGLEZERO;

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}

/**
Tests the GenerateEdgeCode method.
Uses a unmatched concave extended edge cosine value.
Uses a concave tolerance which should disable the edge.
The expected result should override the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeUnmatchedForcedConcaveEdge()
{
    const rwpmath::VecFloat edgeCosine(2.0f);
    const rwpmath::VecFloat concaveTolerance(0.10f);
    const bool matched = false;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matched);

    uint8_t expectedEncodedEdgeData = rw::collision::EDGEFLAG_ANGLEZERO;
    expectedEncodedEdgeData |= rw::collision::EDGEFLAG_EDGEUNMATCHED;

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}


/**
Tests the GenerateEdgeCode method.
Uses a concave tolerance below the allowed range.
The expected result should honor the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeBelowRangeConcaveTolerance()
{
    const rwpmath::VecFloat edgeCosine(3.0f);
    const rwpmath::VecFloat concaveTolerance(-2.0f);
    const bool matched = true;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matched);

    uint8_t expectedEncodedEdgeData = ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(edgeCosine);

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}

/**
Tests the GenerateEdgeCode method.
Uses a concave tolerance above the allowed range.
The expected result should honor the edgecosine value passed in.
*/
void
TestClusteredMeshBuilderUtils::TestGenerateEdgeCodeAboveRangeConcaveTolerance()
{
    const rwpmath::VecFloat edgeCosine(1.0f);
    const rwpmath::VecFloat concaveTolerance(2.0f);
    const bool matched = true;

    uint8_t encodedEdgeData = EdgeCodeGenerator::GenerateEdgeCode(
        edgeCosine,
        concaveTolerance,
        matched);

    uint8_t expectedEncodedEdgeData = ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(edgeCosine);

    EATESTAssert(expectedEncodedEdgeData == encodedEdgeData, "encoded edge cosine and flags is incorrect");
}


/**
Tests the featureless plane.
*/
void
TestClusteredMeshBuilderUtils::TestEdgeProducesFeaturelessPlane()
{
    rwpmath::Vector3 edgeA(1.0f, 0.0f, 0.0f);
    rwpmath::Vector3 edgeB(0.0f, 0.0f, 1.0f);
    rwpmath::Vector3 edgeC(-1.0f, 0.0f, -1.0f);
    rwpmath::VecFloat cosineTolerance(0.05f);

    bool ret = ClusteredMeshBuilderUtils::EdgeProducesFeaturelessPlane(edgeA,
                                                                       edgeB,
                                                                       edgeC,
                                                                       cosineTolerance);

    // Expect a edge to produce a featureless plane
    EATESTAssert( true == ret, "edge should produce featureless plane should be false");
}


/**
Tests whether or not edge disabled vertex
*/
void
TestClusteredMeshBuilderUtils::TestEdgeDisablesVertex()
{
    rwpmath::Vector3 edgeA(1.0f, 0.0f, 0.0f);
    rwpmath::Vector3 edgeB(0.0f, 0.0f, 1.0f);
    rwpmath::Vector3 edgeC(-1.0f, 0.0f, -1.0f);
    rwpmath::Vector3 planeNormal(0.0f, 1.0f, 0.0f);
    rwpmath::VecFloat cosineCoplanarTolerance(0.05f);
    rwpmath::VecFloat cosineTolerance(0.05f);
    rwpmath::VecFloat cosineConcaveTolerance(0.15f);

    bool ret = ClusteredMeshBuilderUtils::EdgeDisablesVertex(edgeA,
                                                             edgeB,
                                                             edgeC,
                                                             planeNormal,
                                                             cosineCoplanarTolerance,
                                                             cosineTolerance,
                                                             cosineConcaveTolerance);

    // Expect edge to disable vertex
    EATESTAssert( true == ret, "edge should produce featureless plane should be false");
}
