// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/meshbuilder/clusterbuilder.h>
#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/clusteredmeshcluster.h>

#include "testsuitebase.h" // For TestSuiteBase

class TestClusterBuilder : public rw::collision::tests::TestSuiteBase
{
public:

#define REGISTER_CLUSTERBUILDER_TEST(M, D) EATEST_REGISTER(#M, D, TestClusterBuilder, M)

    virtual void Initialize()
    {
        SuiteName("TestClusterBuilder");

        REGISTER_CLUSTERBUILDER_TEST(TestInitializeClusterParameters, "Testing InitializeClusterParameters");
        REGISTER_CLUSTERBUILDER_TEST(TestBuild, "Testing Build");
    }

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    }

private:

    void TestInitializeClusterParameters();
    void TestBuild();

    EA::Allocator::ICoreAllocator * m_allocator;

} TestClusterBuilderSingleton;

/**
Testing the InitializeClusterParameters method
*/
void
TestClusterBuilder::TestInitializeClusterParameters()
{
    rw::collision::ClusterConstructionParameters parameters;

    const uint32_t numVerticesInCluster = 16u;
    const uint32_t numUnitsInCluster = 14u;
    const uint8_t compressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;

    // Allocate the unit list
    rw::collision::meshbuilder::ClusterBuilder::UnitList * unitList = rw::collision::meshbuilder::ClusterBuilder::UnitList::Allocate(
        m_allocator,
        numUnitsInCluster,
        EA::Allocator::MEM_PERM);
    EATESTAssert(unitList, ("Unit list should have been allocated"));

    unitList->resize(numUnitsInCluster);
    for (uint32_t unitIndex = 0 ; unitIndex < numUnitsInCluster ; ++unitIndex)
    {
        (*unitList)[unitIndex].tri0 = unitIndex;
        (*unitList)[unitIndex].type = rw::collision::meshbuilder::detail::Unit::TYPE_TRIANGLE;
    }

    // Allocate the GroupID List
    rw::collision::meshbuilder::ClusterBuilder::TriangleGroupIDList * groupIDList = rw::collision::meshbuilder::ClusterBuilder::TriangleGroupIDList::Allocate(
        m_allocator,
        numUnitsInCluster,
        EA::Allocator::MEM_PERM);
    EATESTAssert(groupIDList, ("Group ID List should have been allocated"));

    groupIDList->resize(numUnitsInCluster);
    for (uint32_t unitIndex = 0 ; unitIndex < numUnitsInCluster ; ++unitIndex)
    {
        (*groupIDList)[unitIndex] = 0x1234;
    }

    // Allocate the Surface List
    rw::collision::meshbuilder::ClusterBuilder::TriangleSurfaceIDList * surfaceIDList = rw::collision::meshbuilder::ClusterBuilder::TriangleSurfaceIDList::Allocate(
        m_allocator,
        numUnitsInCluster,
        EA::Allocator::MEM_PERM);
    EATESTAssert(surfaceIDList, ("Surface ID List should have been allocated"));

    surfaceIDList->resize(numUnitsInCluster);
    for (uint32_t unitIndex = 0 ; unitIndex < numUnitsInCluster ; ++unitIndex)
    {
        (*surfaceIDList)[unitIndex] = 0x4321;
    }


    rw::collision::UnitParameters unitParameters;
    unitParameters.groupIDSize = 2u;
    unitParameters.surfaceIDSize = 2u;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_GROUPID | rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_EDGEANGLE;

    rw::collision::meshbuilder::ClusterBuilder::InitializeClusterParameters(
        parameters,
        numVerticesInCluster,
        numUnitsInCluster,
        *groupIDList,
        *surfaceIDList,
        *unitList,
        unitParameters,
        compressionMode);

    EATESTAssert(numVerticesInCluster == parameters.mVertexCount, ("Vertex count is incorrect"));
    EATESTAssert(compressionMode == parameters.mVertexCompressionMode, ("Vertex compression is incorrect"));
    EATESTAssert(numUnitsInCluster == parameters.mTriangleUnitCount, ("Triangle unit count is incorrect"));
    EATESTAssert(0u == parameters.mQuadUnitCount, ("Quad unit count is incorrect"));
    EATESTAssert(numUnitsInCluster * 3u == parameters.mEdgeCosineCount, ("Edge cosine count is incorrect"));
    EATESTAssert(numUnitsInCluster == parameters.mGroupIDCount, ("Group ID count is incorrect"));
    EATESTAssert(unitParameters.groupIDSize == parameters.mGroupIDSize, ("Group ID size is incorrect"));
    EATESTAssert(numUnitsInCluster == parameters.mSurfaceIDCount, ("Surface ID count is incorrect"));
    EATESTAssert(unitParameters.surfaceIDSize == parameters.mSurfaceIDSize, ("Surface ID size is incorrect"));
}


/**
Testing the Build method
*/
void
TestClusterBuilder::TestBuild()
{
    // Allocate the cluster
    rw::collision::ClusterConstructionParameters parameters;
    parameters.mVertexCount = 3;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 1;
    parameters.mQuadUnitCount = 0;
    parameters.mEdgeCosineCount = 3;
    parameters.mGroupIDCount = 1;
    parameters.mGroupIDSize = 2;
    parameters.mSurfaceIDCount = 1;
    parameters.mSurfaceIDSize = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);
    void * buffer = m_allocator->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    EATESTAssert(buffer, ("cluster buffer should have been allocated"));
    rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

    // Setup the build parameters
    rw::collision::meshbuilder::ClusterBuilder::BuildParameters buildParameters;
    buildParameters.unitParameters.groupIDSize = static_cast<uint8_t>(parameters.mGroupIDSize);
    buildParameters.unitParameters.surfaceIDSize = static_cast<uint8_t>(parameters.mSurfaceIDSize);
    buildParameters.unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_GROUPID | rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_EDGEANGLE;
    buildParameters.vertexCompressionGranularity = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;

    // Allocate the vertex list
    rw::collision::meshbuilder::ClusterBuilder::VertexList * vertexList = rw::collision::meshbuilder::ClusterBuilder::VertexList::Allocate(
        m_allocator,
        parameters.mVertexCount,
        EA::Allocator::MEM_PERM);
    EATESTAssert(vertexList, ("Vertex list should have been allocated"));
    vertexList->resize(parameters.mVertexCount);
    for (uint32_t vertexIndex = 0 ; vertexIndex < parameters.mVertexCount ; ++vertexIndex)
    {
        (*vertexList)[vertexIndex] = rw::math::fpu::Vector3(0.0f, 1.0f, 2.0f);
    }

    // Allocate the triangle list
    rw::collision::meshbuilder::ClusterBuilder::TriangleList * triangleList = rw::collision::meshbuilder::ClusterBuilder::TriangleList::Allocate(
        m_allocator,
        parameters.mTriangleUnitCount,
        EA::Allocator::MEM_PERM);
    EATESTAssert(triangleList, ("Triangle list should have been allocated"));
    triangleList->resize(parameters.mTriangleUnitCount);
    for (uint32_t triangleIndex = 0 ; triangleIndex < parameters.mTriangleUnitCount ; ++triangleIndex)
    {
        (*triangleList)[triangleIndex].vertices[0] = 0u;
        (*triangleList)[triangleIndex].vertices[1] = 1u;
        (*triangleList)[triangleIndex].vertices[2] = 2u;
    }

    // Allocate the unit list
    rw::collision::meshbuilder::ClusterBuilder::UnitList * unitList = rw::collision::meshbuilder::ClusterBuilder::UnitList::Allocate(
        m_allocator,
        parameters.mTriangleUnitCount,
        EA::Allocator::MEM_PERM);
    EATESTAssert(unitList, ("Unit list should have been allocated"));
    unitList->resize(parameters.mTriangleUnitCount);
    for (uint32_t unitIndex = 0 ; unitIndex < parameters.mTriangleUnitCount ; ++unitIndex)
    {
        (*unitList)[unitIndex].tri0 = unitIndex;
        (*unitList)[unitIndex].type = rw::collision::meshbuilder::detail::Unit::TYPE_TRIANGLE;
    }

    // Allocate the edge code list
    rw::collision::meshbuilder::ClusterBuilder::TriangleEdgeCodesList * edgeCodeList = rw::collision::meshbuilder::ClusterBuilder::TriangleEdgeCodesList::Allocate(
        m_allocator,
        parameters.mTriangleUnitCount,
        EA::Allocator::MEM_PERM);
    EATESTAssert(edgeCodeList, ("edge code list should have been allocated"));
    edgeCodeList->resize(parameters.mTriangleUnitCount);
    for (uint32_t edgeCodesIndex = 0 ; edgeCodesIndex < parameters.mTriangleUnitCount ; ++edgeCodesIndex)
    {
        (*edgeCodeList)[edgeCodesIndex].encodedEdgeCos[0] = 2u;
        (*edgeCodeList)[edgeCodesIndex].encodedEdgeCos[1] = 4u;
        (*edgeCodeList)[edgeCodesIndex].encodedEdgeCos[2] = 6u;
    }

    // Allocate the GroupID List
    rw::collision::meshbuilder::ClusterBuilder::TriangleGroupIDList * groupIDList = rw::collision::meshbuilder::ClusterBuilder::TriangleGroupIDList::Allocate(
        m_allocator,
        parameters.mTriangleUnitCount,
        EA::Allocator::MEM_PERM);
    EATESTAssert(groupIDList, ("Group ID List should have been allocated"));
    groupIDList->resize(parameters.mTriangleUnitCount);
    for (uint32_t index = 0 ; index < parameters.mTriangleUnitCount ; ++index)
    {
        (*groupIDList)[index] = 0x1234;
    }

    // Allocate the Surface List
    rw::collision::meshbuilder::ClusterBuilder::TriangleSurfaceIDList * surfaceIDList = rw::collision::meshbuilder::ClusterBuilder::TriangleSurfaceIDList::Allocate(
        m_allocator,
        parameters.mTriangleUnitCount,
        EA::Allocator::MEM_PERM);
    EATESTAssert(surfaceIDList, ("Surface ID List should have been allocated"));
    surfaceIDList->resize(parameters.mTriangleUnitCount);
    for (uint32_t index = 0 ; index < parameters.mTriangleUnitCount ; ++index)
    {
        (*surfaceIDList)[index] = 0x4321;
    }

    // Specify the cluster offset
    rw::collision::ClusteredMeshCluster::Vertex32 clusterOffset;
    clusterOffset.x = 0;
    clusterOffset.y = 0;
    clusterOffset.z = 0;

    // Build the cluster
    rw::collision::meshbuilder::ClusterBuilder::Build(
        *cluster,
        *m_allocator,
        buildParameters,
        *vertexList,
        *triangleList,
        *unitList,
        *edgeCodeList,
        *surfaceIDList,
        *groupIDList,
        parameters.mVertexCompressionMode,
        clusterOffset);

    // Check the state of the cluster header
    EATESTAssert(1u == cluster->unitCount, "Cluster unit count is incorrect");
    EATESTAssert(11u == cluster->unitDataSize, "Cluster unit data size is incorrect");
    EATESTAssert(3u == cluster->unitDataStart, "Cluster unit data start is incorrect");
    EATESTAssert(3u == cluster->normalStart, "Cluster normal start is incorrect");
    EATESTAssert(75 == cluster->totalSize, "Cluster size is incorrect");
    EATESTAssert(3u == cluster->vertexCount, "Cluster vertex count is incorrect");
    EATESTAssert(0 == cluster->normalCount, "Cluster normal count is incorrect");
    EATESTAssert(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED == cluster->compressionMode, "Cluster compression mode is incorrect");

    // Test the cluster vertices
    rwpmath::Vector3 * vertices = cluster->vertexArray;
    EATESTAssert(rwpmath::Vector3(0.0f, 1.0f, 2.0f) == vertices[0], "The cluster vertex is incorrect");
    EATESTAssert(rwpmath::Vector3(0.0f, 1.0f, 2.0f) == vertices[1], "The cluster vertex is incorrect");
    EATESTAssert(rwpmath::Vector3(0.0f, 1.0f, 2.0f) == vertices[2], "The cluster vertex is incorrect");

    // Test the cluster units
    uint8_t* unitData = reinterpret_cast<uint8_t*>(cluster->vertexArray) + cluster->unitDataStart * 16;
    // Unit header
    EATESTAssert(225u == unitData[0], "The unit header is incorrect");
    // Unit vertices
    EATESTAssert(0 == unitData[1], "The unit vertex is incorrect");
    EATESTAssert(1 == unitData[2], "The unit vertex is incorrect");
    EATESTAssert(2 == unitData[3], "The unit vertex is incorrect");
    // Unit Edge cosines
    EATESTAssert(2 == unitData[4], "The unit edge code is incorrect");
    EATESTAssert(4 == unitData[5], "The unit edge code is incorrect");
    EATESTAssert(6 == unitData[6], "The unit edge code is incorrect");

    // Unit surface and group ID
    EATESTAssert(0x34 == unitData[7], "The unit group Id is incorrect");
    EATESTAssert(0x12 == unitData[8], "The unit group Id is incorrect");
    EATESTAssert(0x21 == unitData[9], "The unit surface Id is incorrect");
    EATESTAssert(0x43 == unitData[10], "The unit surface Id is incorrect");
}