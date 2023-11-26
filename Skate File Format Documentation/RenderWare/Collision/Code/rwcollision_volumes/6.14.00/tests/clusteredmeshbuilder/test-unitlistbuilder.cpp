// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/meshbuilder/unitlistbuilder.h>
#include <rw/collision/meshbuilder/detail/triangleneighborfinder.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision::meshbuilder;
using namespace rw::collision;

class TestUnitListBuilder : public tests::TestSuiteBase
{
public:

#define REGISTER_UNITLISTBUILDER_TEST(M, D) EATEST_REGISTER(#M, D, TestUnitListBuilder, M)

    virtual void Initialize()
    {
        SuiteName("TestUnitListBuilder");

        REGISTER_UNITLISTBUILDER_TEST(TestBuildQuads, "Testing Build Quads");
        REGISTER_UNITLISTBUILDER_TEST(TestBuildTriangles, "Testing Build Triangles");
        REGISTER_UNITLISTBUILDER_TEST(TestBuildQuadsOutOfRange, "Testing Build Quads with out of range indices");
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
        m_unitList = UnitListBuilder::UnitList::Allocate(m_allocator, MAX_UNITS, EA::Allocator::MEM_PERM);
        m_unitList->reserve(MAX_UNITS);
        m_IDList = UnitListBuilder::IDList::Allocate(m_allocator, MAX_TRIANGLES, EA::Allocator::MEM_PERM);
        m_IDList->resize(MAX_TRIANGLES);
        m_triangleList = UnitListBuilder::TriangleList::Allocate(m_allocator, MAX_TRIANGLES, EA::Allocator::MEM_PERM);
        m_triangleList->resize(MAX_TRIANGLES);
        m_triangleGroupIDList = UnitListBuilder::TriangleGroupIDList::Allocate(m_allocator, MAX_TRIANGLES, EA::Allocator::MEM_PERM);
        m_triangleGroupIDList->resize(MAX_TRIANGLES);
        m_triangleSurfaceIDList = UnitListBuilder::TriangleSurfaceIDList::Allocate(m_allocator, MAX_TRIANGLES, EA::Allocator::MEM_PERM);
        m_triangleSurfaceIDList->resize(MAX_TRIANGLES);
        m_triangleNeighborsList = UnitListBuilder::TriangleNeighborsList::Allocate(m_allocator, MAX_TRIANGLES, EA::Allocator::MEM_PERM);
        m_triangleNeighborsList->resize(MAX_TRIANGLES);
        m_triangleFlagsList = UnitListBuilder::TriangleFlagsList::Allocate(m_allocator, MAX_TRIANGLES, EA::Allocator::MEM_PERM);
        m_triangleFlagsList->resize(MAX_TRIANGLES);
        m_vertexList = UnitListBuilder::VertexList::Allocate(m_allocator, MAX_VERTICES, EA::Allocator::MEM_PERM);
        m_vertexList->resize(MAX_VERTICES);

        InitializeTriangleList();
        InitializeTriangleGroupIDs();
        InitializeTriangleSurfaceIDs();
        InitializeTriangleNeighborsList();
        InitializeTriangleFlagsList();
        InitializeVertexList();
    }

    virtual void TeardownSuite()
    {
        UnitListBuilder::VertexList::Free(m_allocator, m_vertexList);
        UnitListBuilder::TriangleFlagsList::Free(m_allocator, m_triangleFlagsList);
        UnitListBuilder::TriangleNeighborsList::Free(m_allocator, m_triangleNeighborsList);
        UnitListBuilder::TriangleSurfaceIDList::Free(m_allocator, m_triangleSurfaceIDList);
        UnitListBuilder::TriangleGroupIDList::Free(m_allocator, m_triangleGroupIDList);
        UnitListBuilder::TriangleList::Free(m_allocator, m_triangleList);
        UnitListBuilder::IDList::Free(m_allocator, m_IDList);
        UnitListBuilder::UnitList::Free(m_allocator, m_unitList);
        tests::TestSuiteBase::TeardownSuite();
    }

private:

    static const uint32_t MAX_TRIANGLES = 8u;
    static const uint32_t MAX_UNITS = MAX_TRIANGLES;
    static const uint32_t MAX_VERTICES = 3u + (MAX_TRIANGLES - 1);

    void TestBuildQuads();
    void TestBuildTriangles();
    void TestBuildQuadsOutOfRange();


    void InitializeTriangleList();
    void InitializeTriangleGroupIDs();
    void InitializeTriangleSurfaceIDs();
    void InitializeTriangleNeighborsList();
    void InitializeTriangleFlagsList();
    void InitializeVertexList();

    void AddOutOfRangeNeighbors();

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
    UnitListBuilder::UnitList * m_unitList;
    UnitListBuilder::IDList * m_IDList;
    UnitListBuilder::TriangleList * m_triangleList;
    UnitListBuilder::TriangleGroupIDList * m_triangleGroupIDList;
    UnitListBuilder::TriangleSurfaceIDList * m_triangleSurfaceIDList;
    UnitListBuilder::TriangleNeighborsList * m_triangleNeighborsList;
    UnitListBuilder::TriangleFlagsList * m_triangleFlagsList;
    UnitListBuilder::VertexList * m_vertexList;

} TestUnitListBuilderSingleton;


void TestUnitListBuilder::InitializeTriangleList()
{
    UnitListBuilder::TriangleList & triangles = *m_triangleList;

    for (uint32_t triangleIndex = 0 ; triangleIndex < MAX_TRIANGLES ; ++triangleIndex)
    {
        if (triangleIndex % 2)
        {
            const uint32_t baseIndex = (triangleIndex / 2u) + 1;
            triangles[triangleIndex].vertices[0] = baseIndex;
            triangles[triangleIndex].vertices[1] = baseIndex + (MAX_TRIANGLES / 2u);
            triangles[triangleIndex].vertices[2] = baseIndex + ((MAX_TRIANGLES / 2u) + 1);
        }
        else
        {
            const uint32_t baseIndex = triangleIndex / 2u;
            triangles[triangleIndex].vertices[0] = baseIndex;
            triangles[triangleIndex].vertices[1] = baseIndex + ((MAX_TRIANGLES / 2u) + 1u);
            triangles[triangleIndex].vertices[2] = baseIndex + 1u;
        }
    }
}

void TestUnitListBuilder::InitializeTriangleGroupIDs()
{
    for (uint32_t triangleIndex = 0 ; triangleIndex < MAX_TRIANGLES ; ++triangleIndex)
    {
        (*m_triangleGroupIDList)[triangleIndex] = 111u;
    }
}

void TestUnitListBuilder::InitializeTriangleSurfaceIDs()
{
    for (uint32_t triangleIndex = 0 ; triangleIndex < MAX_TRIANGLES ; ++triangleIndex)
    {
        (*m_triangleSurfaceIDList)[triangleIndex] = 222u;
    }
}

void TestUnitListBuilder::InitializeTriangleNeighborsList()
{
    UnitListBuilder::TriangleNeighborsList & triangleNeighbors = *m_triangleNeighborsList;

    rw::collision::meshbuilder::detail::TriangleNeighborFinder::InitializeTriangleNeighbors(triangleNeighbors);

    for (uint32_t triangleIndex = 0 ; triangleIndex < MAX_TRIANGLES ; ++triangleIndex)
    {
        if (triangleIndex % 2u)
        {
            triangleNeighbors[triangleIndex].neighbor[0] = triangleIndex - 1u;

            if ((MAX_TRIANGLES - 1) != triangleIndex)
            {
                triangleNeighbors[triangleIndex].neighbor[2] = triangleIndex + 1;
            }
        }
        else
        {
            triangleNeighbors[triangleIndex].neighbor[1] = triangleIndex + 1u;

            if (0 != triangleIndex)
            {
                triangleNeighbors[triangleIndex].neighbor[0] = triangleIndex - 1u;
            }
        }
    }
}

void TestUnitListBuilder::InitializeTriangleFlagsList()
{
    for (uint32_t triangleIndex = 0 ; triangleIndex < MAX_TRIANGLES ; ++triangleIndex)
    {
        (*m_triangleFlagsList)[triangleIndex].enabled = true;
    }
}

void TestUnitListBuilder::InitializeVertexList()
{
    for (uint32_t vertexIndex = 0 ; vertexIndex < MAX_VERTICES ; ++vertexIndex)
    {
        float y = 0.0f;
        float x = 0.0f;
        float z = 0.0f;

        if (vertexIndex < (MAX_VERTICES / 2))
        {
            x = static_cast<float>(vertexIndex);
            z = 0.0f;
        }
        else
        {
            x = static_cast<float>(vertexIndex - (MAX_VERTICES / 2));
            z = 1.0f;
        }

        (*m_vertexList)[vertexIndex] = VectorType(x, y, z);
    }
}

void TestUnitListBuilder::AddOutOfRangeNeighbors()
{
    UnitListBuilder::TriangleNeighborsList & triangleNeighbors = *m_triangleNeighborsList;

    const uint32_t outOfRange = MAX_TRIANGLES + 1u;

    for (uint32_t triangleIndex = 0 ; triangleIndex < MAX_TRIANGLES ; ++triangleIndex)
    {
        if (triangleIndex % 2u)
        {
            triangleNeighbors[triangleIndex].neighbor[1] = outOfRange;

            if ((MAX_TRIANGLES - 1) == triangleIndex)
            {
                triangleNeighbors[triangleIndex].neighbor[2] = outOfRange;
            }
        }
        else
        {
            triangleNeighbors[triangleIndex].neighbor[2] = outOfRange;

            if (0 == triangleIndex)
            {
                triangleNeighbors[triangleIndex].neighbor[0] = outOfRange;
            }
        }
    }
}

// Test a straightforward build of a list of quads
void TestUnitListBuilder::TestBuildQuads()
{
    const uint32_t groupIDSize = 2;
    const uint32_t surfaceIDSize = 2;

    const uint32_t numUnits = UnitListBuilder::BuildUnitListWithQuads(
        *m_unitList,
        *m_IDList,
        *m_triangleList,
        *m_triangleSurfaceIDList,
        *m_triangleGroupIDList,
        *m_triangleNeighborsList,
        *m_triangleFlagsList,
        *m_vertexList,
        surfaceIDSize,
        groupIDSize);

    EATESTAssert(numUnits == 4u, ("Number of units created is incorrect"));
}


// Test a straightforward build of a list of triangles
void TestUnitListBuilder::TestBuildTriangles()
{
    const uint32_t numUnits = UnitListBuilder::BuildUnitListWithTriangles(
        *m_unitList,
        *m_triangleList,
        *m_triangleFlagsList);

    EATESTAssert(numUnits == 8u, ("Number of units created is incorrect"));
}

// Test a build of a list of quads, some of which reference out of range neighbors
void TestUnitListBuilder::TestBuildQuadsOutOfRange()
{
    const uint32_t groupIDSize = 2;
    const uint32_t surfaceIDSize = 2;

    AddOutOfRangeNeighbors();

    const uint32_t numUnits = UnitListBuilder::BuildUnitListWithQuads(
        *m_unitList,
        *m_IDList,
        *m_triangleList,
        *m_triangleSurfaceIDList,
        *m_triangleGroupIDList,
        *m_triangleNeighborsList,
        *m_triangleFlagsList,
        *m_vertexList,
        surfaceIDSize,
        groupIDSize);

    EATESTAssert(numUnits == 4u, ("Number of units created is incorrect"));
}