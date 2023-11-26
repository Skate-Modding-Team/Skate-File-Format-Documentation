// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>

#include <stdio.h>     // for sprintf()

#include <rw/collision/initialize.h>
#include <rw/collision/aggregatevolume.h>
#include <rw/collision/triangleclusterprocedural.h>

#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/triangleconnector.h>
#include <rw/collision/meshbuilder/edgecodegenerator.h>
#include <rw/collision/meshbuilder/unitlistbuilder.h>
#include <rw/collision/meshbuilder/vertexcompression.h>

#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/unitclusterbuilder.h>
#include <rw/collision/meshbuilder/detail/unitclusterstack.h>
#include <rw/collision/meshbuilder/detail/clusterdatabuilder.h>
#include <rw/collision/meshbuilder/detail/clusterparametersbuilder.h>
#include <rw/collision/meshbuilder/detail/clusteredmeshbuildermethods.h>

#include <rw/collision/clusteredmeshcluster_methods.h>

#include <rw/collision/detail/fpu/triangleclusterprocedural.h>

#include <unit/unit.h>

#include <eaphysics/unitframework/creator.h>
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

namespace EA 
{
namespace Collision
{
namespace Tests
{

    class TestTriangleClusterProcedural : public rw::collision::tests::TestSuiteBase
{
public:     // TestSuite overrides

#define REGISTER_CLUSTER_TEST(M, D) EATEST_REGISTER(#M, D, TestTriangleClusterProcedural, M)

    virtual void Initialize()
    {
        SuiteName("TestTriangleClusterProcedural");

        REGISTER_CLUSTER_TEST(TestGetResoureceDescriptor, "Check GetResoureceDescriptor()");
        REGISTER_CLUSTER_TEST(TestInitialize, "Check Initialize()");
        REGISTER_CLUSTER_TEST(TestGetSizeThis, "Check GetSizeThis()");
		REGISTER_CLUSTER_TEST(TestUpdateThis, "Check UpdateThis()");
        REGISTER_CLUSTER_TEST(TestUpdateWithBBox, "Check UpdateWithBBox()");

        REGISTER_CLUSTER_TEST(TestLineIntersectionQueryThisSingleTriHit, "Check LineIntersectionQueryThis()");
        REGISTER_CLUSTER_TEST(TestLineIntersectionQueryThisMultipleTriHit, "Check LineIntersectionQueryThis()");
        REGISTER_CLUSTER_TEST(TestLineIntersectionQueryThisTriMiss, "Check LineIntersectionQueryThis()");

        REGISTER_CLUSTER_TEST(TestLineIntersectionQueryThisSingleQuadHit, "Check LineIntersectionQueryThis()");
        REGISTER_CLUSTER_TEST(TestLineIntersectionQueryThisMultipleQuadHit, "Check LineIntersectionQueryThis()");
        REGISTER_CLUSTER_TEST(TestLineIntersectionQueryThisQuadMiss, "Check LineIntersectionQueryThis()");

        REGISTER_CLUSTER_TEST(TestBBoxOverlapQueryThisTriHit, "Check TestBBoxOverlapQueryThis()");
        REGISTER_CLUSTER_TEST(TestBBoxOverlapQueryThisTriMiss, "Check TestBBoxOverlapQueryThis()");
        REGISTER_CLUSTER_TEST(TestBBoxOverlapQueryThisArray, "Check TestBBoxOverlapQueryThis()");
        REGISTER_CLUSTER_TEST(TestBBoxOverlapQueryThisQuadOverflow, "Check TestBBoxOverlapQueryThis()");
        REGISTER_CLUSTER_TEST(TestBBoxOverlapQueryThisTriOverflow, "Check TestBBoxOverlapQueryThis()");

        REGISTER_CLUSTER_TEST(TestGetVolumeFromChildIndexTri, "Check GetVolumeFromChildIndex()");
        REGISTER_CLUSTER_TEST(TestGetVolumeFromChildIndexQuad, "Check GetVolumeFromChildIndex()");

        REGISTER_CLUSTER_TEST(TestGetUnitOffsetFromChildIndex, "Check GetUnitOffsetFromChildIndex()");
        REGISTER_CLUSTER_TEST(TestGetTriangleIndexWithinUnitFromChildIndex, "Check GetTriangleOffsetWithinUnitFromChildIndex()");

        REGISTER_CLUSTER_TEST(TestGetVertexIndicesFromChildIndexTri, "Check GetVertexIndicesFromChildIndex() with triangles");
        REGISTER_CLUSTER_TEST(TestGetVertexIndicesFromChildIndexQuad, "Check GetVertexIndicesFromChildIndex() with triangles");

        REGISTER_CLUSTER_TEST(TestHLSerialization, "Check HL Serialization");
        REGISTER_CLUSTER_TEST(TestHLFileSerialization, "Check HL file Serialization");

#if !defined(RWP_NO_VPU_MATH)
        REGISTER_CLUSTER_TEST(TestLLVpuSerialization, "Check LL vpu Serialization");
        REGISTER_CLUSTER_TEST(TestLLVpuFileSerialization, "Check LL vpu file Serialization");
#endif

        REGISTER_CLUSTER_TEST(TestLLFpuSerialization, "Check ll fpu Serialization");
        REGISTER_CLUSTER_TEST(TestLLFpuFileSerialization, "Check LL fpu file Serialization");
    }

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();
        rw::collision::InitializeVTables();
    }

private:

    typedef rw::collision::meshbuilder::EdgeCodeGenerator::TriangleEdgeCodesList TriangleEdgeCodesList;

    typedef rw::collision::meshbuilder::UnitListBuilder::VertexList VertexList;
    typedef rw::collision::meshbuilder::UnitListBuilder::TriangleList TriangleList;
    typedef rw::collision::meshbuilder::UnitListBuilder::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef rw::collision::meshbuilder::UnitListBuilder::TriangleGroupIDList TriangleGroupIDList;
    typedef rw::collision::meshbuilder::UnitListBuilder::UnitList UnitList;
    typedef rw::collision::meshbuilder::UnitListBuilder::IDList IDList;

    typedef rw::collision::meshbuilder::TriangleConnector::TriangleEdgeCosinesList TriangleEdgeCosinesList;
    typedef rw::collision::meshbuilder::TriangleConnector::TriangleNeighborsList TriangleNeighborsList;
    typedef rw::collision::meshbuilder::TriangleConnector::TriangleFlagsList TriangleFlagsList;

    void TestGetResoureceDescriptor();
    void TestInitialize();
    void TestGetSizeThis();
    void TestUpdateThis();
	void TestUpdateWithBBox();

    void TestLineIntersectionQueryThisSingleTriHit();
    void TestLineIntersectionQueryThisMultipleTriHit();
    void TestLineIntersectionQueryThisTriMiss();

    void TestLineIntersectionQueryThisSingleQuadHit();
    void TestLineIntersectionQueryThisMultipleQuadHit();
    void TestLineIntersectionQueryThisQuadMiss();

    void TestBBoxOverlapQueryThisTriHit();
    void TestBBoxOverlapQueryThisTriMiss();
    void TestBBoxOverlapQueryThisArray();
    void TestBBoxOverlapQueryThisQuadOverflow();
    void TestBBoxOverlapQueryThisTriOverflow();

    void TestGetVolumeFromChildIndexTri();
    void TestGetVolumeFromChildIndexQuad();

    void TestGetUnitOffsetFromChildIndex();
    void TestGetTriangleIndexWithinUnitFromChildIndex();\

    void TestGetVertexIndicesFromChildIndexTri();
    void TestGetVertexIndicesFromChildIndexQuad();

    void TestHLSerialization();
    void TestHLFileSerialization();

    void TestLLVpuSerialization();
    void TestLLVpuFileSerialization();

    void TestLLFpuSerialization();
    void TestLLFpuFileSerialization();

    bool CompareTriangleClusterProcedurals(
        rw::collision::TriangleClusterProcedural * tcpA,
        rw::collision::TriangleClusterProcedural * tcpB);

    void CreateTriangles(
        VertexList &vertices,
        TriangleList &triangles,
        const uint32_t xCount,
        const uint32_t yCount,
        const uint32_t zCount);

    void CreateUnits(
        UnitList &units,
        TriangleList &triangles,
        TriangleSurfaceIDList &triangleSurfaceIDs,
        TriangleGroupIDList &triangleGroupIDs,
        TriangleNeighborsList &triangleNeighbors,
        TriangleFlagsList &triangleFlags,
        VertexList &vertices,
        const uint32_t surfaceSize,
        const uint32_t groupSize,
        const bool quads);

    void CreateUnitCluster(
        VertexList **vertices,
        TriangleList **triangles,
        TriangleSurfaceIDList **triangleSurfaceIDs,
        TriangleGroupIDList **triangleGroupIDs,
        TriangleEdgeCodesList **triangleEdgeCodes,
        UnitList **units,
        rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack,
        rw::collision::ClusterConstructionParameters & clusterConstructionParameters,
        rw::collision::UnitParameters & unitParameters,
        const uint32_t xCount,
        const uint32_t yCount,
        const uint32_t zCount,
        const bool quads);

    void AddUnitsToUnitCluster(
        const TriangleList & triangles,
        const UnitList & units,
        rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack);

    void InitializeClusterConstructionParameters(
        rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack,
        TriangleSurfaceIDList & triangleSurfaceIDs,
        TriangleGroupIDList & triangleGroupIDs,
        UnitList & units,
        const rw::collision::UnitParameters & unitParameters,
        rw::collision::ClusterConstructionParameters & clusterConstructionParameters);

    void FinalizeClusteredMeshCluster(
        rw::collision::ClusteredMeshCluster &cluster,
        const VertexList & vertices,
        const TriangleList & triangles,
        const TriangleEdgeCodesList & triangleEdgeCodes,
        const TriangleSurfaceIDList & triangleSurfaceIDs,
        const TriangleGroupIDList & triangleGroupIDs,
        const UnitList & units,
        rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack,
        const rw::collision::UnitParameters & unitParameters,
        const float vertexCompressionGranularity);

    void CreateTriangleClusterProcedural(
        rw::collision::TriangleClusterProcedural ** clusterAgg,
        const uint32_t xCount,
        const uint32_t yCount,
        const uint32_t zCount,
        const bool quads,
        const float vertexCompressionGranularity = 0.0f);

    template <class COMPARISON_TYPE>
    void CheckValue(COMPARISON_TYPE actual, COMPARISON_TYPE expected, const char * msg);

} gTestTCP;


template <class COMPARISON_TYPE>
void
TestTriangleClusterProcedural::CheckValue(COMPARISON_TYPE actual, COMPARISON_TYPE expected, const char * msg)
{
    char str[256];
    sprintf(str, "%s should be %u", msg, expected);
    EATESTAssert(actual == expected, str);
}

template <>
void
TestTriangleClusterProcedural::CheckValue<const rw::collision::AABBox&>(const rw::collision::AABBox& actual, const rw::collision::AABBox& expected, const char * msg)
{
    char str[256];
    sprintf(str, "%s should be Min(%f, %f, %f), Max(%f, %f, %f)", msg, (float)expected.Min().GetX(), (float)expected.Min().GetY(), (float)expected.Min().GetZ(),
                                                                       (float)expected.Max().GetX(), (float)expected.Max().GetY(), (float)expected.Max().GetZ());
    EATESTAssert(actual.Min() == expected.Min() && actual.Max() == expected.Max(), str);
}

template <>
void
TestTriangleClusterProcedural::CheckValue<const rw::collision::TriangleVolume&>(const rw::collision::TriangleVolume& actual, const rw::collision::TriangleVolume& expected, const char * msg)
{
    rwpmath::Vector3 actualV0, actualV1, actualV2;
    actual.GetPoints(actualV0, actualV1, actualV2);

    rwpmath::Vector3 expectedV0, expectedV1, expectedV2;
    expected.GetPoints(expectedV0, expectedV1, expectedV2);

    char str[256];
    sprintf(str, "%s should be V0(%f, %f, %f), V1(%f, %f, %f), V2(%f, %f, %f)", msg, (float)expectedV0.GetX(), (float)expectedV0.GetY(),(float)expectedV0.GetZ(),
                                                                                     (float)expectedV1.GetX(), (float)expectedV1.GetY(),(float)expectedV1.GetZ(),
                                                                                     (float)expectedV2.GetX(), (float)expectedV2.GetY(),(float)expectedV2.GetZ());

    EATESTAssert(actualV0 == expectedV0 && actualV1 == expectedV1 && actualV2 == expectedV2, str);
}

template <>
void
TestTriangleClusterProcedural::CheckValue<const rwpmath::Vector3&>(const rwpmath::Vector3& actual, const rwpmath::Vector3& expected, const char * msg)
{
    char str[256];
    sprintf(str, "%s should be (%f, %f, %f)", msg, (float)expected.GetX(), (float)expected.GetY(), (float)expected.GetZ());
    EATESTAssert(IsSimilar(actual, expected), str);
}

// This method is used to determine whether or not two TriangleClusterProcedurals are the same.
// NOTE: We do not test the data which is platform dependent (e.g. edgeconsines) since the test data
// is mostly likely not generated on the same platform on which the tests are run.
bool
TestTriangleClusterProcedural::CompareTriangleClusterProcedurals(
    rw::collision::TriangleClusterProcedural * tcpA,
    rw::collision::TriangleClusterProcedural * tcpB)
{
    // Check the cluster parameters
    {
        if (tcpA->GetClusterParams().mFlags != tcpB->GetClusterParams().mFlags)
        {
            return false;
        }
        if (tcpA->GetClusterParams().mGroupIdSize != tcpB->GetClusterParams().mGroupIdSize)
        {
            return false;
        }
        if (tcpA->GetClusterParams().mSurfaceIdSize != tcpB->GetClusterParams().mSurfaceIdSize)
        {
            return false;
        }
        if (tcpA->GetClusterParams().mVertexCompressionGranularity != tcpB->GetClusterParams().mVertexCompressionGranularity)
        {
            return false;
        }
    }

    // Check the cluster
    {
        const rw::collision::ClusteredMeshCluster & clusterA = tcpA->GetCluster();
        const rw::collision::ClusteredMeshCluster & clusterB = tcpB->GetCluster();

        if (clusterA.unitCount != clusterB.unitCount)
        {
            return false;
        }
        if (clusterA.unitDataSize != clusterB.unitDataSize)
        {
            return false;
        }
        if (clusterA.unitDataStart != clusterB.unitDataStart)
        {
            return false;
        }
        if (clusterA.normalStart != clusterB.normalStart)
        {
            return false;
        }
        if (clusterA.totalSize != clusterB.totalSize)
        {
            return false;
        }
        if (clusterA.vertexCount != clusterB.vertexCount)
        {
            return false;
        }
        if (clusterA.normalCount != clusterB.normalCount)
        {
            return false;
        }
        if (clusterA.compressionMode != clusterB.compressionMode)
        {
            return false;
        }

        for (uint8_t vertexIndex = 0 ; vertexIndex < clusterA.vertexCount; ++vertexIndex)
        {
            if (clusterA.GetVertex(vertexIndex, tcpA->GetClusterParams().mVertexCompressionGranularity) !=
                clusterB.GetVertex(vertexIndex, tcpB->GetClusterParams().mVertexCompressionGranularity))
            {
                return false;
            }
        }

        // Check the sizeOf
        {
            if (tcpA->GetSizeThis() != tcpB->GetSizeThis())
            {
                return false;
            }
        }
    }

    return true;
}


void
TestTriangleClusterProcedural::TestGetResoureceDescriptor()
{
    rw::collision::ClusterConstructionParameters parameters;
    parameters.mTriangleUnitCount = 16;
    parameters.mVertexCount = 32;

    EA::Physics::SizeAndAlignment resDesc = rw::collision::TriangleClusterProcedural::GetResourceDescriptor(parameters);

    uint32_t expectedSize = sizeof(rw::collision::TriangleClusterProcedural);
    expectedSize = EA::Physics::SizeAlign<uint32_t>(expectedSize, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    expectedSize += rw::collision::ClusteredMeshCluster::GetSize(parameters);

    CheckValue(resDesc.GetSize(), expectedSize, "Size");
}

void
TestTriangleClusterProcedural::TestInitialize()
{
    // TODO : add some tests
}

void
TestTriangleClusterProcedural::TestGetSizeThis()
{
    rw::collision::ClusterConstructionParameters parameters;
    parameters.mTriangleUnitCount = 16;
    parameters.mVertexCount = 16;

    EA::Physics::SizeAndAlignment resDesc = rw::collision::TriangleClusterProcedural::GetResourceDescriptor(parameters);
    EA::Physics::MemoryPtr res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());

    rw::collision::TriangleClusterProcedural * clusterAgg = rw::collision::TriangleClusterProcedural::Initialize(res, parameters);

    uint32_t expectedSize = resDesc.GetSize();

    CheckValue(clusterAgg->GetSizeThis(), expectedSize, "Size");
}

void 
TestTriangleClusterProcedural::TestUpdateWithBBox()
{
	rw::collision::ClusterConstructionParameters parameters;
	parameters.mTriangleUnitCount = 0;
	parameters.mVertexCount = 2;

	EA::Physics::SizeAndAlignment resDesc = rw::collision::TriangleClusterProcedural::GetResourceDescriptor(parameters);
	EA::Physics::MemoryPtr res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());

	rw::collision::TriangleClusterProcedural * clusterAgg = rw::collision::TriangleClusterProcedural::Initialize(res, parameters);

	rw::collision::AABBox expectedAABBox(rwpmath::Vector3(0.0f, 0.0f, 0.0f),
		rwpmath::Vector3(10.0f, 10.0f, 10.0f));

	rw::collision::ClusteredMeshCluster & cluster = clusterAgg->GetCluster();
	cluster.SetVertex(rwpmath::Vector3(0.0f, 0.0f, 0.0f), clusterAgg->GetClusterParams().mVertexCompressionGranularity);
	cluster.SetVertex(rwpmath::Vector3(10.0f, 10.0f, 10.0f), clusterAgg->GetClusterParams().mVertexCompressionGranularity);
	clusterAgg->UpdateWithBBox(expectedAABBox);

	CheckValue<const rw::collision::AABBox&>(clusterAgg->GetBBox(), expectedAABBox, "AABBox");
}


void
TestTriangleClusterProcedural::TestUpdateThis()
{
    rw::collision::ClusterConstructionParameters parameters;
    parameters.mTriangleUnitCount = 0;
    parameters.mVertexCount = 2;

    EA::Physics::SizeAndAlignment resDesc = rw::collision::TriangleClusterProcedural::GetResourceDescriptor(parameters);
    EA::Physics::MemoryPtr res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());

    rw::collision::TriangleClusterProcedural * clusterAgg = rw::collision::TriangleClusterProcedural::Initialize(res, parameters);

    rw::collision::ClusteredMeshCluster & cluster = clusterAgg->GetCluster();
    cluster.SetVertex(rwpmath::Vector3(0.0f, 0.0f, 0.0f), clusterAgg->GetClusterParams().mVertexCompressionGranularity);
    cluster.SetVertex(rwpmath::Vector3(10.0f, 10.0f, 10.0f), clusterAgg->GetClusterParams().mVertexCompressionGranularity);
    clusterAgg->UpdateThis();

    rw::collision::AABBox expectedAABBox(rwpmath::Vector3(0.0f, 0.0f, 0.0f),
                                         rwpmath::Vector3(10.0f, 10.0f, 10.0f));

    CheckValue<const rw::collision::AABBox&>(clusterAgg->GetBBox(), expectedAABBox, "AABBox");
}

void
TestTriangleClusterProcedural::CreateTriangles(
    VertexList &vertices,
    TriangleList &triangles,
    const uint32_t xCount,
    const uint32_t yCount,
    const uint32_t zCount)
{
    typedef rw::collision::meshbuilder::VectorType VectorType;

    uint32_t vertexIndex = 0;
    // Create the vertices
    for (uint32_t yIndex = 0 ; yIndex < yCount ; ++yIndex)
    {
        for (uint32_t zIndex = 0 ; zIndex < zCount ; ++zIndex)
        {
            for (uint32_t xIndex = 0 ; xIndex < xCount ; ++xIndex)
            {
                VectorType v(static_cast<float>(xIndex) * 1.0f,
                             static_cast<float>(yIndex) * 1.0f,
                             static_cast<float>(zIndex) * 1.0f);

                vertices[vertexIndex++] = v;
            }
        }
    }

    uint32_t triangleIndex = 0;
    // Create the triangles
    for (uint32_t yIndex = 0 ; yIndex < yCount ; ++yIndex)
    {
        for (uint32_t zIndex = 0 ; zIndex < zCount - 1 ; ++zIndex)
        {
            for (uint32_t xIndex = 0 ; xIndex < xCount - 1; ++xIndex)
            {
                uint32_t v0 = (xIndex + (zIndex * xCount)) + (yIndex * (xCount * zCount));
                uint32_t v1 = (xIndex + ((zIndex + 1u) * xCount)) + (yIndex * (xCount * zCount));
                uint32_t v2 = ((xIndex + 1u) + (zIndex * xCount)) + (yIndex * (xCount * zCount));
                uint32_t v3 = ((xIndex + 1u) + ((zIndex + 1u) * xCount)) + (yIndex * (xCount * zCount));

                triangles[triangleIndex].vertices[0] = v0;
                triangles[triangleIndex].vertices[1] = v1;
                triangles[triangleIndex++].vertices[2] = v2;

                triangles[triangleIndex].vertices[0] = v1;
                triangles[triangleIndex].vertices[1] = v3;
                triangles[triangleIndex++].vertices[2] = v2;
            }
        }
    }
}


void
TestTriangleClusterProcedural::CreateUnits(
    UnitList &units,
    TriangleList &triangles,
    TriangleSurfaceIDList &triangleSurfaceIDs,
    TriangleGroupIDList &triangleGroupIDs,
    TriangleNeighborsList &triangleNeighbors,
    TriangleFlagsList &triangleFlags,
    VertexList &vertices,
    const uint32_t surfaceSize,
    const uint32_t groupSize,
    const bool quads)
{
    if (quads)
    {
        IDList * compressedUnitIndex = IDList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangles.size(), EA::Allocator::MEM_PERM);
        compressedUnitIndex->resize(triangles.size());

        rw::collision::meshbuilder::UnitListBuilder::BuildUnitListWithQuads(
            units,
            *compressedUnitIndex,
            triangles,
            triangleSurfaceIDs,
            triangleGroupIDs,
            triangleNeighbors,
            triangleFlags,
            vertices,
            surfaceSize,
            groupSize);

        IDList::Free(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), compressedUnitIndex);
    }
    else
    {
        rw::collision::meshbuilder::UnitListBuilder::BuildUnitListWithTriangles(
            units,
            triangles,
            triangleFlags);
    }
}

void
TestTriangleClusterProcedural::AddUnitsToUnitCluster(
    const TriangleList & triangles,
    const UnitList & units,
    rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack)
{
    // Initialize the UnitClusterStack
    unitClusterStack.Initialize(
        EA::Allocator::ICoreAllocator::GetDefaultAllocator(),
        units.size());

    // Get a UnitCluster from the allocator
    rw::collision::meshbuilder::detail::UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    const uint32_t startUnitIndex = 0u;
    const uint32_t numUnitsToAdd = units.size();
    const uint32_t maxVerticesPerUnit = 4u;

    // Add all units to the cluster
    rw::collision::meshbuilder::detail::UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        numUnitsToAdd,
        triangles,
        units,
        maxVerticesPerUnit);
}

void
TestTriangleClusterProcedural::InitializeClusterConstructionParameters(
    rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack,
    TriangleSurfaceIDList & triangleSurfaceIDs,
    TriangleGroupIDList & triangleGroupIDs,
    UnitList & units,
    const rw::collision::UnitParameters & unitParameters,
    rw::collision::ClusterConstructionParameters & clusterConstructionParameters)
{
    // Get the UnitCluster from the UnitClusterStack
    rw::collision::meshbuilder::detail::UnitClusterStack::ClusterIterator it = unitClusterStack.Begin();
    rw::collision::meshbuilder::detail::UnitCluster * unitCluster = *it;

    // Initialize the ClusteredMeshCluster construction parameters
    rw::collision::meshbuilder::detail::ClusterParametersBuilder::InitializeClusterParameters(
        clusterConstructionParameters,
        *unitCluster,
        triangleSurfaceIDs,
        triangleGroupIDs,
        units,
        unitParameters);
}


void
TestTriangleClusterProcedural::FinalizeClusteredMeshCluster(
    rw::collision::ClusteredMeshCluster &cluster,
    const VertexList & vertices,
    const TriangleList & triangles,
    const TriangleEdgeCodesList & triangleEdgeCodes,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitList & units,
    rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack,
    const rw::collision::UnitParameters & unitParameters,
    const float vertexCompressionGranularity)
{
    rw::collision::meshbuilder::detail::UnitClusterStack::ClusterIterator it = unitClusterStack.Begin();
    rw::collision::meshbuilder::detail::UnitCluster * unitCluster = *it;

    rw::collision::meshbuilder::detail::ClusterDataBuilder::Build(
        cluster,
        *unitCluster,
        vertices,
        triangles,
        triangleEdgeCodes,
        triangleSurfaceIDs,
        triangleGroupIDs,
        units,
        unitParameters,
        vertexCompressionGranularity);
}


void
TestTriangleClusterProcedural::CreateUnitCluster(
    VertexList **vertices,
    TriangleList **triangles,
    TriangleSurfaceIDList **triangleSurfaceIDs,
    TriangleGroupIDList **triangleGroupIDs,
    TriangleEdgeCodesList **triangleEdgeCodes,
    UnitList **units,
    rw::collision::meshbuilder::detail::UnitClusterStack & unitClusterStack,
    rw::collision::ClusterConstructionParameters & clusterConstructionParameters,
    rw::collision::UnitParameters & unitParameters,
    const uint32_t xCount,
    const uint32_t yCount,
    const uint32_t zCount,
    const bool quads)
{
    // ClusteredMeshCluster Unit parameters
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_EDGEANGLE;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 0;

    const uint32_t vertexCount = xCount * zCount * yCount;
    const uint32_t triangleCount = ((xCount - 1) * (zCount - 1) * 2) * yCount;

    // Create triangle and vertices
    *vertices = VertexList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), vertexCount, EA::Allocator::MEM_PERM);
    *triangles = TriangleList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    *triangleSurfaceIDs = TriangleSurfaceIDList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    *triangleGroupIDs = TriangleGroupIDList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);

    (*vertices)->resize(vertexCount);
    (*triangles)->resize(triangleCount);
    (*triangleSurfaceIDs)->resize(triangleCount);
    (*triangleGroupIDs)->resize(triangleCount);

    CreateTriangles(
        **vertices,
        **triangles,
        xCount,
        yCount,
        zCount);

    TriangleEdgeCosinesList *triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    TriangleNeighborsList *triangleNeighbors = TriangleNeighborsList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    TriangleFlagsList *triangleFlags = TriangleFlagsList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);

    triangleEdgeCosines->resize(triangleCount);
    triangleNeighbors->resize(triangleCount);
    triangleFlags->resize(triangleCount);

    rw::collision::meshbuilder::TriangleConnector::GenerateTriangleConnectivity(
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *EA::Allocator::ICoreAllocator::GetDefaultAllocator(),
        **vertices,
        **triangles);

    *triangleEdgeCodes = TriangleEdgeCodesList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);
    (*triangleEdgeCodes)->resize(triangleCount);

    rwpmath::VecFloat edgecosConcaveAngleTolerance = 0.0f;
    rw::collision::meshbuilder::EdgeCodeGenerator::GenerateTriangleEdgeCodes(
        **triangleEdgeCodes,
        *triangleEdgeCosines,
        *triangleNeighbors,
        edgecosConcaveAngleTolerance);

    //
    // Create a list of Units
    //

    *units = UnitList::Allocate(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), triangleCount, EA::Allocator::MEM_PERM);

    (*units)->reserve(triangleCount);

    CreateUnits(
        **units,
        **triangles,
        **triangleSurfaceIDs,
        **triangleGroupIDs,
        *triangleNeighbors,
        *triangleFlags,
        **vertices,
        unitParameters.groupIDSize,
        unitParameters.surfaceIDSize,
        quads);

    AddUnitsToUnitCluster(
        **triangles,
        **units,
        unitClusterStack);

    InitializeClusterConstructionParameters(
        unitClusterStack,
        **triangleSurfaceIDs,
        **triangleGroupIDs,
        **units,
        unitParameters,
        clusterConstructionParameters);
}

void
TestTriangleClusterProcedural::CreateTriangleClusterProcedural(
    rw::collision::TriangleClusterProcedural ** clusterAgg,
    const uint32_t xCount,
    const uint32_t yCount,
    const uint32_t zCount,
    const bool quads,
    const float vertexCompressionGranularity)
{
    VertexList *vertices;
    TriangleList *triangles;
    TriangleSurfaceIDList *triangleSurfaceIDs;
    TriangleGroupIDList *triangleGroupIDs;
    TriangleEdgeCodesList *triangleEdgeCodes;
    UnitList *units;
    rw::collision::meshbuilder::detail::UnitClusterStack unitClusterStack;
    rw::collision::ClusterConstructionParameters parameters;

    rw::collision::UnitParameters unitParameters;

    CreateUnitCluster(
        &vertices,
        &triangles,
        &triangleSurfaceIDs,
        &triangleGroupIDs,
        &triangleEdgeCodes,
        &units,
        unitClusterStack,
        parameters,
        unitParameters,
        xCount,
        yCount,
        zCount,
        quads);

    // Initialize the Mesh cluster aggregate
    EA::Physics::SizeAndAlignment resDesc = rw::collision::TriangleClusterProcedural::GetResourceDescriptor(parameters);
    EA::Physics::MemoryPtr res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());

    *clusterAgg = rw::collision::TriangleClusterProcedural::Initialize(
                     res,
                     parameters);

    (*clusterAgg)->SetGroupIdSize(unitParameters.groupIDSize);
    (*clusterAgg)->SetSurfaceIdSize(unitParameters.surfaceIDSize);
    (*clusterAgg)->SetVertexCompressionGranularity(vertexCompressionGranularity);

    // Finalize the ClusteredMeshCluster
    rw::collision::ClusteredMeshCluster & cluster = (*clusterAgg)->GetCluster();
    FinalizeClusteredMeshCluster(
        cluster,
        *vertices,
        *triangles,
        *triangleEdgeCodes,
        *triangleSurfaceIDs,
        *triangleGroupIDs,
        *units,
        unitClusterStack,
        unitParameters,
        vertexCompressionGranularity);

    (*clusterAgg)->UpdateThis();
}


void
TestTriangleClusterProcedural::TestLineIntersectionQueryThisSingleTriHit()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rwpmath::Vector3 lineStart = rwpmath::Vector3(0.25f, 10.0f, 0.25f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(0.25f, -10.0f, 0.25f);

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), 0, 0,resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeLineQuery::GetResourceDescriptor(1u, 1u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), 0, 0,resDesc.GetAlignment());
    rw::collision::VolumeLineQuery *lineQuery = rw::collision::VolumeLineQuery::Initialize(queryRes, 1u, 1u);

    const rw::collision::Volume * vol = aggVol;
    lineQuery->InitQuery(&vol, NULL, 1, lineStart, lineEnd);

    EATESTAssert(!lineQuery->Finished(), "Line query should not have finished.");

    uint32_t numRes = lineQuery->GetAllIntersections();
    EATESTAssert(1 == numRes, "Result count should be 1");
    EATESTAssert(lineQuery->Finished(), "Line query should be finished.");

    rw::collision::VolumeLineSegIntersectResult * results = lineQuery->GetIntersectionResultsBuffer();

    const rw::collision::Volume * intersectedVolume = results[0].v;

    EATESTAssert(intersectedVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE, "Intersected volume type should be triangle");

    EATESTAssert(rwpmath::IsSimilar(results[0].lineParam, 0.5f), "Line param should be 0.5f");
    EATESTAssert(rwpmath::IsSimilar(results[0].normal, rwpmath::Vector3(0.0f, 1.0f, 0.0f)), "Intersection normal should be (0.0f, 1.0f, 0.0f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].position, rwpmath::Vector3(0.25f, 0.0f, 0.25f)), "Intersection point should be (0.25f, 0.0f, 0.25f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].volParam, rwpmath::Vector3(0.25f, 0.25f, 0.0f)), "Intersection volume parameter should be (0.25f, 0.25f, 0.0f)");
    EATESTAssert(results[0].vRef.tag == 1, "Intersection tag should be 1");
    EATESTAssert(results[0].vRef.numTagBits == 5, "Intersection num tag bits should be 5");
}


void
TestTriangleClusterProcedural::TestLineIntersectionQueryThisMultipleTriHit()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 2;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rwpmath::Vector3 lineStart = rwpmath::Vector3(0.25f, 10.0f, 0.25f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(0.25f, -10.0f, 0.25f);

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), 0, 0,resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeLineQuery::GetResourceDescriptor(1u, 1u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), 0, 0,resDesc.GetAlignment());
    rw::collision::VolumeLineQuery *lineQuery = rw::collision::VolumeLineQuery::Initialize(queryRes, 1u, 1u);

    const rw::collision::Volume * vol = aggVol;
    lineQuery->InitQuery(&vol, NULL, 1, lineStart, lineEnd);

    uint32_t numRes = lineQuery->GetAllIntersections();
    EATESTAssert(1 == numRes, "Result count should be 1");
    EATESTAssert(!lineQuery->Finished(), "Line query should not have finished.");

    rw::collision::VolumeLineSegIntersectResult * results = lineQuery->GetIntersectionResultsBuffer();
    const rw::collision::Volume * intersectedVolume = results[0].v;

    EATESTAssert(intersectedVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE, "Intersected volume type should be triangle");
    EATESTAssert(rwpmath::IsSimilar(results[0].lineParam, 0.5f), "Line param should be 0.5f");
    EATESTAssert(rwpmath::IsSimilar(results[0].normal, rwpmath::Vector3(0.0f, 1.0f, 0.0f)), "Intersection normal should be (0.0f, 1.0f, 0.0f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].position, rwpmath::Vector3(0.25f, 0.0f, 0.25f)), "Intersection point should be (0.25f, 0.0f, 0.25f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].volParam, rwpmath::Vector3(0.25f, 0.25f, 0.0f)), "Intersection volume parameter should be (0.25f, 0.25f, 0.0f)");
    EATESTAssert(results[0].vRef.tag == 1, "Intersection tag should be 1");
    EATESTAssert(results[0].vRef.numTagBits == 6, "Intersection num tag bits should be 6");

    numRes = lineQuery->GetAllIntersections();

    EATESTAssert(1 == numRes, "Result count should be 1");
    EATESTAssert(lineQuery->Finished(), "Line query should be finished.");

    results = lineQuery->GetIntersectionResultsBuffer();
    intersectedVolume = results[0].v;

    EATESTAssert(intersectedVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE, "Intersected volume type should be triangle");
    EATESTAssert(rwpmath::IsSimilar(results[0].lineParam, 0.45f), "Line param should be 0.45f");
    EATESTAssert(rwpmath::IsSimilar(results[0].normal, rwpmath::Vector3(0.0f, 1.0f, 0.0f)), "Intersection normal should be (0.0f, 1.0f, 0.0f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].position, rwpmath::Vector3(0.25f, 1.0f, 0.25f)), "Intersection point should be (0.25f, 1.0f, 0.25f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].volParam, rwpmath::Vector3(0.25f, 0.25f, 0.0f)), "Intersection volume parameter should be (0.25f, 0.25f, 0.0f)");
    EATESTAssert(results[0].vRef.tag == 15, "Intersection tag should be 3");
    EATESTAssert(results[0].vRef.numTagBits == 6, "Intersection num tag bits should be 6");
}

void
TestTriangleClusterProcedural::TestLineIntersectionQueryThisTriMiss()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rwpmath::Vector3 lineStart = rwpmath::Vector3(-0.25f, 10.0f, -0.25f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(-0.25f, -10.0f, -0.25f);

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeLineQuery::GetResourceDescriptor(1u, 1u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeLineQuery *lineQuery = rw::collision::VolumeLineQuery::Initialize(queryRes, 1u, 1u);

    const rw::collision::Volume * vol = aggVol;
    lineQuery->InitQuery(&vol, NULL, 1, lineStart, lineEnd);

    EATESTAssert(!lineQuery->Finished(), "Line query should not have finished.");

    uint32_t numRes = lineQuery->GetAllIntersections();
    EATESTAssert(0 == numRes, "Result count should be 0");
    EATESTAssert(lineQuery->Finished(), "Line query should be finished.");
}

void
TestTriangleClusterProcedural::TestLineIntersectionQueryThisSingleQuadHit()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = true;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rwpmath::Vector3 lineStart = rwpmath::Vector3(0.25f, 10.0f, 0.25f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(0.25f, -10.0f, 0.25f);

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeLineQuery::GetResourceDescriptor(1u, 1u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeLineQuery *lineQuery = rw::collision::VolumeLineQuery::Initialize(queryRes, 1u, 1u);

    const rw::collision::Volume * vol = aggVol;
    lineQuery->InitQuery(&vol, NULL, 1, lineStart, lineEnd);

    EATESTAssert(!lineQuery->Finished(), "Line query should not have finished.");

    uint32_t numRes = lineQuery->GetAllIntersections();
    EATESTAssert(1 == numRes, "Result count should be 1");
    EATESTAssert(lineQuery->Finished(), "Line query should be finished.");

    rw::collision::VolumeLineSegIntersectResult * results = lineQuery->GetIntersectionResultsBuffer();

    const rw::collision::Volume * intersectedVolume = results[0].v;

    EATESTAssert(intersectedVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE, "Intersected volume type should be triangle");

    EATESTAssert(rwpmath::IsSimilar(results[0].lineParam, 0.5f), "Line param should be 0.5f");
    EATESTAssert(rwpmath::IsSimilar(results[0].normal, rwpmath::Vector3(0.0f, 1.0f, 0.0f)), "Intersection normal should be (0.0f, 1.0f, 0.0f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].position, rwpmath::Vector3(0.25f, 0.0f, 0.25f)), "Intersection point should be (0.25f, 0.0f, 0.25f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].volParam, rwpmath::Vector3(0.25f, 0.25f, 0.0f)), "Intersection volume parameter should be (0.25f, 0.25f, 0.0f)");
    EATESTAssert(results[0].vRef.tag == 1, "Intersection tag should be 1");
    EATESTAssert(results[0].vRef.numTagBits == 5, "Intersection num tag bits should be 5");
}


void
TestTriangleClusterProcedural::TestLineIntersectionQueryThisMultipleQuadHit()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 2;
    const uint32_t zCount = 2;
    const bool quads = true;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rwpmath::Vector3 lineStart = rwpmath::Vector3(0.25f, 10.0f, 0.25f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(0.25f, -10.0f, 0.25f);

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeLineQuery::GetResourceDescriptor(1u, 1u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeLineQuery *lineQuery = rw::collision::VolumeLineQuery::Initialize(queryRes, 1u, 1u);

    const rw::collision::Volume * vol = aggVol;
    lineQuery->InitQuery(&vol, NULL, 1, lineStart, lineEnd);

    uint32_t numRes = lineQuery->GetAllIntersections();
    EATESTAssert(1 == numRes, "Result count should be 1");
    EATESTAssert(!lineQuery->Finished(), "Line query should not have finished.");

    rw::collision::VolumeLineSegIntersectResult * results = lineQuery->GetIntersectionResultsBuffer();
    const rw::collision::Volume * intersectedVolume = results[0].v;

    EATESTAssert(intersectedVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE, "Intersected volume type should be triangle");
    EATESTAssert(rwpmath::IsSimilar(results[0].lineParam, 0.5f), "Line param should be 0.5f");
    EATESTAssert(rwpmath::IsSimilar(results[0].normal, rwpmath::Vector3(0.0f, 1.0f, 0.0f)), "Intersection normal should be (0.0f, 1.0f, 0.0f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].position, rwpmath::Vector3(0.25f, 0.0f, 0.25f)), "Intersection point should be (0.25f, 0.0f, 0.25f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].volParam, rwpmath::Vector3(0.25f, 0.25f, 0.0f)), "Intersection volume parameter should be (0.25f, 0.25f, 0.0f)");
    EATESTAssert(results[0].vRef.tag == 1, "Intersection tag should be 1");
    EATESTAssert(results[0].vRef.numTagBits == 6, "Intersection num tag bits should be 6");

    numRes = lineQuery->GetAllIntersections();

    EATESTAssert(1 == numRes, "Result count should be 1");
    EATESTAssert(lineQuery->Finished(), "Line query should be finished.");

    results = lineQuery->GetIntersectionResultsBuffer();
    intersectedVolume = results[0].v;

    EATESTAssert(intersectedVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE, "Intersected volume type should be triangle");
    EATESTAssert(rwpmath::IsSimilar(results[0].lineParam, 0.45f), "Line param should be 0.45f");
    EATESTAssert(rwpmath::IsSimilar(results[0].normal, rwpmath::Vector3(0.0f, 1.0f, 0.0f)), "Intersection normal should be (0.0f, 1.0f, 0.0f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].position, rwpmath::Vector3(0.25f, 1.0f, 0.25f)), "Intersection point should be (0.25f, 1.0f, 0.25f)");
    EATESTAssert(rwpmath::IsSimilar(results[0].volParam, rwpmath::Vector3(0.25f, 0.25f, 0.0f)), "Intersection volume parameter should be (0.25f, 0.25f, 0.0f)");
    EATESTAssert(results[0].vRef.tag == 10, "Intersection tag should be 10");
    EATESTAssert(results[0].vRef.numTagBits == 6, "Intersection num tag bits should be 6");
}

void
TestTriangleClusterProcedural::TestLineIntersectionQueryThisQuadMiss()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rwpmath::Vector3 lineStart = rwpmath::Vector3(-0.25f, 10.0f, -0.25f);
    rwpmath::Vector3 lineEnd = rwpmath::Vector3(-0.25f, -10.0f, -0.25f);

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeLineQuery::GetResourceDescriptor(1u, 1u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeLineQuery *lineQuery = rw::collision::VolumeLineQuery::Initialize(queryRes, 1u, 1u);

    const rw::collision::Volume * vol = aggVol;
    lineQuery->InitQuery(&vol, NULL, 1, lineStart, lineEnd);

    EATESTAssert(!lineQuery->Finished(), "Line query should not have finished.");

    uint32_t numRes = lineQuery->GetAllIntersections();
    EATESTAssert(0 == numRes, "Result count should be 0");
    EATESTAssert(lineQuery->Finished(), "Line query should be finished.");
}


void
TestTriangleClusterProcedural::TestBBoxOverlapQueryThisTriHit()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;
    const float vertexCompressionGranularity = 0.1f;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads,
        vertexCompressionGranularity);

    rw::collision::AABBox queryBBox(rwpmath::Vector3(0.0f, -0.5f, 0.0f),
        rwpmath::Vector3(0.25f, 0.5f, 0.25f));

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeBBoxQuery::GetResourceDescriptor(1u, 2u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeBBoxQuery *bboxQuery = rw::collision::VolumeBBoxQuery::Initialize(queryRes, 1u, 2u);

    const rw::collision::Volume * vol = aggVol;
    bboxQuery->InitQuery(&vol, NULL, 1, queryBBox);

    EATESTAssert(!bboxQuery->Finished(), "Line query should not have finished.");

    uint32_t numRes = bboxQuery->GetOverlaps();
    EATESTAssert(2u == numRes, "Result count should be 2");
    EATESTAssert(bboxQuery->Finished(), "Line query should be finished.");

    // Check that the Query BBox has not changed
    CheckValue<const rw::collision::AABBox&>(bboxQuery->m_aabb, queryBBox, "Query BBox");

    rw::collision::VolRef * results = bboxQuery->GetOverlapResultsBuffer();

    const rw::collision::Volume* overlappingVolume1 = results[0].volume;
    const rw::collision::Volume* overlappingVolume2 = results[1].volume;

    EATESTAssert(overlappingVolume1->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");
    EATESTAssert(overlappingVolume2->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");

    const rw::collision::TriangleVolume * triangle1 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume1);
    const rw::collision::TriangleVolume * triangle2 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume2);

    EA::Physics::SizeAndAlignment triResDesc = rw::collision::TriangleVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr tri1Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    EA::Physics::MemoryPtr tri2Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    const rw::collision::TriangleVolume * expectedTriangle1 = rw::collision::TriangleVolume::Initialize( tri1Res, rwpmath::Vector3(0.0f, 0.0f, 0.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f));
    const rw::collision::TriangleVolume * expectedTriangle2 = rw::collision::TriangleVolume::Initialize( tri2Res, rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f));

    CheckValue<const rw::collision::TriangleVolume&>(*triangle1, *expectedTriangle1, "Overlapped Triangle");
    CheckValue<const rw::collision::TriangleVolume&>(*triangle2, *expectedTriangle2, "Overlapped Triangle");

    EATESTAssert(results[0].tag == 1, "Intersection tag should be 1");
    EATESTAssert(results[0].numTagBits == 5, "Intersection num tag bits should be 5");

    EATESTAssert(results[1].tag == 8, "Intersection tag should be 8");
    EATESTAssert(results[1].numTagBits == 5, "Intersection num tag bits should be 5");
}

void
TestTriangleClusterProcedural::TestBBoxOverlapQueryThisTriMiss()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rw::collision::AABBox queryBBox(rwpmath::Vector3(0.0f, 0.5f, 0.0f),
                                    rwpmath::Vector3(0.25f, 1.0f, 0.25f));

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeBBoxQuery::GetResourceDescriptor(1u, 2u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeBBoxQuery *bboxQuery = rw::collision::VolumeBBoxQuery::Initialize(queryRes, 1u, 2u);

    const rw::collision::Volume * vol = aggVol;
    bboxQuery->InitQuery(&vol, NULL, 1, queryBBox);

    EATESTAssert(!bboxQuery->Finished(), "Line query should not have finished.");

    uint32_t numRes = bboxQuery->GetOverlaps();
    EATESTAssert(0u == numRes, "Result count should be 0");
    EATESTAssert(bboxQuery->Finished(), "Line query should be finished.");

    // Check that the Query BBox has not changed
    CheckValue<const rw::collision::AABBox&>(bboxQuery->m_aabb, queryBBox, "Query BBox");
}

void
TestTriangleClusterProcedural::TestBBoxOverlapQueryThisArray()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;
    const float vertexCompressionGranularity = 0.1f;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads,
        vertexCompressionGranularity);

    rw::collision::AABBox queryBBox(rwpmath::Vector3(0.0f, -0.5f, 0.0f),
        rwpmath::Vector3(0.25f, 0.5f, 0.25f));

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeBBoxQuery::GetResourceDescriptor(1u, 2u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeBBoxQuery *bboxQuery = rw::collision::VolumeBBoxQuery::Initialize(queryRes, 1u, 2u);

    const rw::collision::Volume * volArray[5] = {aggVol,aggVol,aggVol,aggVol,aggVol};
    bboxQuery->InitQuery(volArray, NULL, 5, queryBBox);
    EATESTAssert(!bboxQuery->Finished(), "Line query should not have finished.");

    for (uint32_t index = 0u ; index < 5u ; ++index)
    {
        uint32_t numRes = bboxQuery->GetOverlaps();
        EATESTAssert(2u == numRes, "Result count should be 2");

        if (index < 4)
        {
            EATESTAssert(!bboxQuery->Finished(), "Line query should not be finished.");
        }
        else
        {
            EATESTAssert(bboxQuery->Finished(), "Line query should have finished.");
        }

        // Check that the Query BBox has not changed
        CheckValue<const rw::collision::AABBox&>(bboxQuery->m_aabb, queryBBox, "Query BBox");

        rw::collision::VolRef * results = bboxQuery->GetOverlapResultsBuffer();

        const rw::collision::Volume* overlappingVolume1 = results[0].volume;
        const rw::collision::Volume* overlappingVolume2 = results[1].volume;

        EATESTAssert(overlappingVolume1->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");
        EATESTAssert(overlappingVolume2->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");

        const rw::collision::TriangleVolume * triangle1 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume1);
        const rw::collision::TriangleVolume * triangle2 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume2);

        EA::Physics::SizeAndAlignment triResDesc = rw::collision::TriangleVolume::GetResourceDescriptor();
        EA::Physics::MemoryPtr tri1Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
        EA::Physics::MemoryPtr tri2Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
        const rw::collision::TriangleVolume * expectedTriangle1 = rw::collision::TriangleVolume::Initialize( tri1Res, rwpmath::Vector3(0.0f, 0.0f, 0.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f));
        const rw::collision::TriangleVolume * expectedTriangle2 = rw::collision::TriangleVolume::Initialize( tri2Res, rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f));

        CheckValue<const rw::collision::TriangleVolume&>(*triangle1, *expectedTriangle1, "Overlapped Triangle");
        CheckValue<const rw::collision::TriangleVolume&>(*triangle2, *expectedTriangle2, "Overlapped Triangle");

        EATESTAssert(results[0].tag == 1, "Intersection tag should be 1");
        EATESTAssert(results[0].numTagBits == 5, "Intersection num tag bits should be 5");

        EATESTAssert(results[1].tag == 8, "Intersection tag should be 8");
        EATESTAssert(results[1].numTagBits == 5, "Intersection num tag bits should be 5");
    }
}

void
TestTriangleClusterProcedural::TestBBoxOverlapQueryThisQuadOverflow()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 3;
    const bool quads = true;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rw::collision::AABBox queryBBox(
        rwpmath::Vector3(0.1f, -0.5f, 0.1f),
        rwpmath::Vector3(0.9f, 0.5f, 1.9f));

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeBBoxQuery::GetResourceDescriptor(1u, 2u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeBBoxQuery *bboxQuery = rw::collision::VolumeBBoxQuery::Initialize(queryRes, 1u, 3u);

    const rw::collision::Volume * vol = aggVol;
    bboxQuery->InitQuery(&vol, NULL, 1, queryBBox);
    EATESTAssert(!bboxQuery->Finished(), "Line query should not have finished.");

    // Get the first 3 triangles
    uint32_t numRes = bboxQuery->GetOverlaps();
    EATESTAssert(3u == numRes, "Result count should be 3");
    EATESTAssert(!bboxQuery->Finished(), "Line query should not be finished.");

    // Check that the Query BBox has not changed
    CheckValue<const rw::collision::AABBox&>(bboxQuery->m_aabb, queryBBox, "Query BBox");

    rw::collision::VolRef * results = bboxQuery->GetOverlapResultsBuffer();

    const rw::collision::Volume* overlappingVolume1 = results[0].volume;
    const rw::collision::Volume* overlappingVolume2 = results[1].volume;
    const rw::collision::Volume* overlappingVolume3 = results[2].volume;

    EATESTAssert(overlappingVolume1->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");
    EATESTAssert(overlappingVolume2->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");
    EATESTAssert(overlappingVolume3->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");

    const rw::collision::TriangleVolume * triangle1 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume1);
    const rw::collision::TriangleVolume * triangle2 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume2);
    const rw::collision::TriangleVolume * triangle3 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume3);

    EA::Physics::SizeAndAlignment triResDesc = rw::collision::TriangleVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr tri1Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    EA::Physics::MemoryPtr tri2Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    EA::Physics::MemoryPtr tri3Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    const rw::collision::TriangleVolume * expectedTriangle1 = rw::collision::TriangleVolume::Initialize( tri1Res, rwpmath::Vector3(1.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f));
    const rw::collision::TriangleVolume * expectedTriangle2 = rw::collision::TriangleVolume::Initialize( tri2Res, rwpmath::Vector3(0.0f, 0.0f, 0.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f));
    const rw::collision::TriangleVolume * expectedTriangle3 = rw::collision::TriangleVolume::Initialize( tri3Res, rwpmath::Vector3(1.0f, 0.0f, 2.0f), rwpmath::Vector3(1.0f, 0.0f, 1.0f), rwpmath::Vector3(0.0f, 0.0f, 2.0f));

    CheckValue<const rw::collision::TriangleVolume&>(*triangle1, *expectedTriangle1, "Overlapped Triangle");
    CheckValue<const rw::collision::TriangleVolume&>(*triangle2, *expectedTriangle2, "Overlapped Triangle");
    CheckValue<const rw::collision::TriangleVolume&>(*triangle3, *expectedTriangle3, "Overlapped Triangle");

    EATESTAssert(results[0].tag == 33, "Intersection tag should be 33");
    EATESTAssert(results[0].numTagBits == 6, "Intersection num tag bits should be 6");

    EATESTAssert(results[1].tag == 1, "Intersection tag should be 1");
    EATESTAssert(results[1].numTagBits == 6, "Intersection num tag bits should be 6");

    EATESTAssert(results[2].tag == 42, "Intersection tag should be 42");
    EATESTAssert(results[2].numTagBits == 6, "Intersection num tag bits should be 6");

    // Get the next 1 triangle
    numRes = bboxQuery->GetOverlaps();
    EATESTAssert(1u == numRes, "Result count should be 1");
    EATESTAssert(bboxQuery->Finished(), "Line query should have finished.");

    // Check that the Query BBox has not changed
    CheckValue<const rw::collision::AABBox&>(bboxQuery->m_aabb, queryBBox, "Query BBox");

    results = bboxQuery->GetOverlapResultsBuffer();
    const rw::collision::Volume* overlappingVolume4 = results[0].volume;

    EATESTAssert(overlappingVolume4->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");

    const rw::collision::TriangleVolume * triangle4 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume4);

    EA::Physics::MemoryPtr tri4Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    const rw::collision::TriangleVolume * expectedTriangle4 = rw::collision::TriangleVolume::Initialize( tri4Res, rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(0.0f, 0.0f, 2.0f), rwpmath::Vector3(1.0f, 0.0f, 1.0f));

    CheckValue<const rw::collision::TriangleVolume&>(*triangle4, *expectedTriangle4, "Overlapped Triangle");

    EATESTAssert(results[0].tag == 10, "Intersection tag should be 10");
    EATESTAssert(results[0].numTagBits == 6, "Intersection num tag bits should be 6");
}

void
TestTriangleClusterProcedural::TestBBoxOverlapQueryThisTriOverflow()
{
    // Create the mesh cluster aggregate
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    rw::collision::AABBox queryBBox(
        rwpmath::Vector3(0.1f, -0.5f, 0.1f),
        rwpmath::Vector3(0.9f, 0.5f, 0.9f));

    EA::Physics::SizeAndAlignment resDesc = rw::collision::AggregateVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr volRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::AggregateVolume * aggVol = rw::collision::AggregateVolume::Initialize(volRes, clusterAgg);

    resDesc = rw::collision::VolumeBBoxQuery::GetResourceDescriptor(1u, 2u);
    EA::Physics::MemoryPtr queryRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    rw::collision::VolumeBBoxQuery *bboxQuery = rw::collision::VolumeBBoxQuery::Initialize(queryRes, 1u, 1u);

    const rw::collision::Volume * vol = aggVol;
    bboxQuery->InitQuery(&vol, NULL, 1, queryBBox);
    EATESTAssert(!bboxQuery->Finished(), "Line query should not have finished.");

    // Get the first 3 triangles
    uint32_t numRes = bboxQuery->GetOverlaps();
    EATESTAssert(1u == numRes, "Result count should be 1");
    EATESTAssert(!bboxQuery->Finished(), "Line query should not be finished.");

    // Check that the Query BBox has not changed
    CheckValue<const rw::collision::AABBox&>(bboxQuery->m_aabb, queryBBox, "Query BBox");

    rw::collision::VolRef * results = bboxQuery->GetOverlapResultsBuffer();

    const rw::collision::Volume* overlappingVolume1 = results[0].volume;

    EATESTAssert(overlappingVolume1->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");

    const rw::collision::TriangleVolume * triangle1 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume1);

    EA::Physics::SizeAndAlignment triResDesc = rw::collision::TriangleVolume::GetResourceDescriptor();
    EA::Physics::MemoryPtr tri1Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    const rw::collision::TriangleVolume * expectedTriangle1 = rw::collision::TriangleVolume::Initialize( tri1Res, rwpmath::Vector3(0.0f, 0.0f, 0.0f), rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f));

    CheckValue<const rw::collision::TriangleVolume&>(*triangle1, *expectedTriangle1, "Overlapped Triangle");

    EATESTAssert(results[0].tag == 1, "Intersection tag should be 1");
    EATESTAssert(results[0].numTagBits == 5, "Intersection num tag bits should be 6");

    // Get the next 1 triangle
    numRes = bboxQuery->GetOverlaps();
    EATESTAssert(1u == numRes, "Result count should be 1");
    EATESTAssert(bboxQuery->Finished(), "Line query should have finished.");

    // Check that the Query BBox has not changed
    CheckValue<const rw::collision::AABBox&>(bboxQuery->m_aabb, queryBBox, "Query BBox");

    results = bboxQuery->GetOverlapResultsBuffer();
    const rw::collision::Volume* overlappingVolume2 = results[0].volume;

    EATESTAssert(overlappingVolume2->GetType() == rw::collision::VOLUMETYPETRIANGLE, "Overlapped volume type should be triangle");

    const rw::collision::TriangleVolume * triangle2 = static_cast<const rw::collision::TriangleVolume*>(overlappingVolume2);

    EA::Physics::MemoryPtr tri2Res = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(triResDesc.GetSize(), NULL, 0, triResDesc.GetAlignment());
    const rw::collision::TriangleVolume * expectedTriangle2 = rw::collision::TriangleVolume::Initialize( tri2Res, rwpmath::Vector3(0.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 1.0f), rwpmath::Vector3(1.0f, 0.0f, 0.0f));

    CheckValue<const rw::collision::TriangleVolume&>(*triangle2, *expectedTriangle2, "Overlapped Triangle");

    EATESTAssert(results[0].tag == 8, "Intersection tag should be 8");
    EATESTAssert(results[0].numTagBits == 5, "Intersection num tag bits should be 5");
}

void
TestTriangleClusterProcedural::TestGetVolumeFromChildIndexTri()
{
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    // Create a ChildIndex to refer to the 1st triangle
    uint32_t childIndex = 0;

    // Get the Triangle Volume
    rw::collision::TriangleVolume * triangleVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>(*EA::Allocator::ICoreAllocator::GetDefaultAllocator()).New();
    clusterAgg->GetVolumeFromChildIndex(*triangleVolume, childIndex);

    // Check the triangle members
    rwpmath::Vector3 expectedV0(0.0f, 0.0f, 0.0f);
    rwpmath::Vector3 expectedV1(0.0f, 0.0f, 1.0f);
    rwpmath::Vector3 expectedV2(1.0f, 0.0f, 0.0f);

    rwpmath::Vector3 actualV0, actualV1, actualV2;
    triangleVolume->GetPoints(actualV0, actualV1, actualV2);

    CheckValue<const rwpmath::Vector3&>(actualV0, expectedV0, "v0");
    CheckValue<const rwpmath::Vector3&>(actualV1, expectedV1, "v1");
    CheckValue<const rwpmath::Vector3&>(actualV2, expectedV2, "v2");

    CheckValue(triangleVolume->GetGroup(), uint32_t(0), "GroupID");
    CheckValue(triangleVolume->GetSurface(), uint32_t(0), "GroupID");

    // This is the decoded edge cosine of an edge with an angle byte of zero,
    // which is a fully open edge.
    const rwpmath::VecFloat edgeCosOfAngleByteZero(-0.23370051f);
    rwpmath::Vector3 expectedEdgeCos(edgeCosOfAngleByteZero, rwpmath::GetVecFloat_One(), edgeCosOfAngleByteZero);
    CheckValue<const rwpmath::Vector3&>(triangleVolume->GetEdgeCosVector(), expectedEdgeCos, "EdgeCos");

    uint32_t actualFlags = triangleVolume->GetFlags();

    char str[256];

    sprintf(str, "Flag One-Sided should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEONESIDED, str);

    sprintf(str, "Flag Use-EdgeCos should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEUSEEDGECOS, str);

    sprintf(str, "Flag Edge-Zero-Convex should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEEDGE0CONVEX, str);

    // The second edge of the triangle is shared by two co-planar triangles. The convexity/concavity
    // of this edge is undetermined and therefore can not be tested.
    // In practice it is marked as convex on pc/xenon/ps3 and concave on Wii.
    // This is due to numerical errors during the initial edge cosine generation in the cluster build
    // process.

    sprintf(str, "Flag Edge-Two-Convex should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEEDGE2CONVEX, str);

    sprintf(str, "Flag Triangle should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::TRIANGLE, str);
}


void
TestTriangleClusterProcedural::TestGetVolumeFromChildIndexQuad()
{
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = true;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    // Create a ChildIndex to refer to the 2nd triangle in the quad
    uint32_t childIndex = 16;

    // Get the Triangle Volume
    rw::collision::TriangleVolume * triangleVolume = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>(*EA::Allocator::ICoreAllocator::GetDefaultAllocator()).New();
    clusterAgg->GetVolumeFromChildIndex(*triangleVolume, childIndex);

    // Check the triangle members
    rwpmath::Vector3 expectedV0(1.0f, 0.0f, 1.0f);
    rwpmath::Vector3 expectedV1(1.0f, 0.0f, 0.0f);
    rwpmath::Vector3 expectedV2(0.0f, 0.0f, 1.0f);

    rwpmath::Vector3 actualV0, actualV1, actualV2;
    triangleVolume->GetPoints(actualV0, actualV1, actualV2);

    CheckValue<const rwpmath::Vector3&>(actualV0, expectedV0, "v0");
    CheckValue<const rwpmath::Vector3&>(actualV1, expectedV1, "v1");
    CheckValue<const rwpmath::Vector3&>(actualV2, expectedV2, "v2");

    CheckValue(triangleVolume->GetGroup(), uint32_t(0), "GroupID");
    CheckValue(triangleVolume->GetSurface(), uint32_t(0), "GroupID");

    // This is the decoded edge cosine of an edge with an angle byte of zero,
    // which is a fully open edge.
    const rwpmath::VecFloat edgeCosOfAngleByteZero(-0.23370051f);
    rwpmath::Vector3 expectedEdgeCos(edgeCosOfAngleByteZero, rwpmath::GetVecFloat_One(), edgeCosOfAngleByteZero);
    CheckValue<const rwpmath::Vector3&>(triangleVolume->GetEdgeCosVector(), expectedEdgeCos, "EdgeCos");

    char str[256];

    uint32_t actualFlags = triangleVolume->GetFlags();

    sprintf(str, "Flag One-Sided should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEONESIDED, str);

    sprintf(str, "Flag Use-EdgeCos should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEUSEEDGECOS, str);

    sprintf(str, "Flag Edge-Zero-Convex should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEEDGE0CONVEX, str);

    // The second edge of the triangle is shared by two co-planar triangles. The convexity/concavity
    // of this edge is undetermined and therefore can not be tested.
    // In practice it is marked as convex on pc/xenon/ps3 and concave on Wii.
    // This is due to numerical errors during the initial edge cosine generation in the cluster build
    // process.

    sprintf(str, "Flag Edge-Two-Convex should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEEDGE2CONVEX, str);

    sprintf(str, "Flag Triangle should be set");
    EATESTAssert(actualFlags & rw::collision::GPInstance::TRIANGLE, str);
}


void
TestTriangleClusterProcedural::TestGetUnitOffsetFromChildIndex()
{
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    // Create a child index which should refer to the 2nd triangle
    uint32_t childIndex = 7;

    const uint32_t expectedUnitOffset = 7;

    const uint32_t actualUnitOffset = clusterAgg->GetUnitOffsetFromChildIndex(childIndex);

    CheckValue(actualUnitOffset, expectedUnitOffset, "Unit offset");
}


void
TestTriangleClusterProcedural::TestGetTriangleIndexWithinUnitFromChildIndex()
{
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    // Create a child index which refers to the 2nd triangle
    const uint32_t childIndex = 7;

    const uint32_t expectedTriangleIndex = 0;

    const uint32_t actualTriangleIndex = clusterAgg->GetTriangleIndexWithinUnitFromChildIndex(childIndex);

    CheckValue(actualTriangleIndex, expectedTriangleIndex, "Triangle index");
}

void
TestTriangleClusterProcedural::TestGetVertexIndicesFromChildIndexTri()
{
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    // Create a child index which refers to the 2nd triangle
    const uint32_t childIndex = 7;

    uint8_t v0, v1, v2 = 0xFF;

    clusterAgg->GetVertexIndicesFromChildIndex(
        v0,
        v1,
        v2,
        childIndex);

    CheckValue<uint8_t>(v0, 2u, "Triangle Vertex 2");
    CheckValue<uint8_t>(v1, 3u, "Triangle Vertex 3");
    CheckValue<uint8_t>(v2, 1u, "Triangle Vertex 1");
}

void
TestTriangleClusterProcedural::TestGetVertexIndicesFromChildIndexQuad()
{
    rw::collision::TriangleClusterProcedural * clusterAgg;
    const uint32_t xCount = 2;
    const uint32_t yCount = 1;
    const uint32_t zCount = 2;
    const bool quads = true;

    CreateTriangleClusterProcedural(
        &clusterAgg,
        xCount,
        yCount,
        zCount,
        quads);

    // Create a child index which refers to the 2nd triangle
    const uint32_t childIndex = 16;

    uint8_t v0, v1, v2 = 0xFF;

    clusterAgg->GetVertexIndicesFromChildIndex(
        v0,
        v1,
        v2,
        childIndex);

    CheckValue<uint8_t>(v0, 3u, "Triangle Vertex 3");
    CheckValue<uint8_t>(v1, 1u, "Triangle Vertex 1");
    CheckValue<uint8_t>(v2, 2u, "Triangle Vertex 2");
}

void
TestTriangleClusterProcedural::TestHLSerialization()
{
    rw::collision::TriangleClusterProcedural * original;
    const uint32_t xCount = 4;
    const uint32_t yCount = 4;
    const uint32_t zCount = 4;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &original,
        xCount,
        yCount,
        zCount,
        quads);

    rw::collision::TriangleClusterProcedural * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(CompareTriangleClusterProcedurals(original, copied), "Original and high-level serialized copies do not match.");
}


void
TestTriangleClusterProcedural::TestHLFileSerialization()
{
    rw::collision::TriangleClusterProcedural * original;
    const uint32_t xCount = 4;
    const uint32_t yCount = 4;
    const uint32_t zCount = 4;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &original,
        xCount,
        yCount,
        zCount,
        quads);

    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("triangleclusterprocedural");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    rw::collision::TriangleClusterProcedural* copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<rw::collision::TriangleClusterProcedural>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");

    EATESTAssert(CompareTriangleClusterProcedurals(original, copied), "Original and high-level file serialized copies do not match.");
}

#if !defined(RWP_NO_VPU_MATH)

void
TestTriangleClusterProcedural::TestLLVpuSerialization()
{
    rw::collision::TriangleClusterProcedural * original;
    const uint32_t xCount = 4;
    const uint32_t yCount = 4;
    const uint32_t zCount = 4;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &original,
        xCount,
        yCount,
        zCount,
        quads);


    rw::collision::TriangleClusterProcedural* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(CompareTriangleClusterProcedurals(original, copied), "Original and low-level vpu serialized copies do not match.");
}


void
TestTriangleClusterProcedural::TestLLVpuFileSerialization()
{
    rw::collision::TriangleClusterProcedural * original;
    const uint32_t xCount = 4;
    const uint32_t yCount = 4;
    const uint32_t zCount = 4;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &original,
        xCount,
        yCount,
        zCount,
        quads);

    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("triangleclusterprocedural");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    rw::collision::TriangleClusterProcedural* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<rw::collision::TriangleClusterProcedural>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(CompareTriangleClusterProcedurals(original, copied), "Original and low-level vpu file serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)

void
TestTriangleClusterProcedural::TestLLFpuSerialization()
{
    rw::collision::TriangleClusterProcedural * original;
    const uint32_t xCount = 4;
    const uint32_t yCount = 4;
    const uint32_t zCount = 4;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &original,
        xCount,
        yCount,
        zCount,
        quads);

#if !defined(RWP_NO_VPU_MATH)
    rw::collision::TriangleClusterProcedural* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization<rw::collision::TriangleClusterProcedural, rw::collision::detail::fpu::TriangleClusterProcedural>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    rw::collision::TriangleClusterProcedural* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(CompareTriangleClusterProcedurals(original, copied), "Original and low-level fpu serialized copies do not match.");
}


void
TestTriangleClusterProcedural::TestLLFpuFileSerialization()
{
    rw::collision::TriangleClusterProcedural * original;
    const uint32_t xCount = 4;
    const uint32_t yCount = 4;
    const uint32_t zCount = 4;
    const bool quads = false;

    CreateTriangleClusterProcedural(
        &original,
        xCount,
        yCount,
        zCount,
        quads);

    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("triangleclusterprocedural");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<rw::collision::TriangleClusterProcedural, rw::collision::detail::fpu::TriangleClusterProcedural>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<rw::collision::TriangleClusterProcedural>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    rw::collision::TriangleClusterProcedural* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<rw::collision::TriangleClusterProcedural, rw::collision::detail::fpu::TriangleClusterProcedural>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    rw::collision::TriangleClusterProcedural* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<rw::collision::TriangleClusterProcedural>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(CompareTriangleClusterProcedurals(original, copied), "Original and low-level fpu file serialized copies do not match.");
}

}   // namespace Tests
}   // namespace collision
}   // namespace EA
