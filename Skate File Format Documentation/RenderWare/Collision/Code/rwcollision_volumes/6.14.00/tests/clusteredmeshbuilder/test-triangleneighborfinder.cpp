// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/meshbuilder/detail/triangleneighborfinder.h>
#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/common.h>

#include "testsuitebase.h" // For TestSuiteBase

// Unit tests for the triangle neighbor finder

using namespace rw::collision::meshbuilder::detail;
using namespace rw::collision;

class TestTriangleNeighborFinder : public tests::TestSuiteBase
{
public:

    virtual void Initialize()
    {
        SuiteName("TestTriangleNeighborFinder");

        // Standard Tests

        EATEST_REGISTER("TestInitializeTriangleEdgeCosines", "Testing InitializeTriangleEdgeCosines", TestTriangleNeighborFinder, TestInitializeTriangleEdgeCosines);
        EATEST_REGISTER("TestInitializeTriangleNeighbors", "Testing InitializeTriangleNeighbors", TestTriangleNeighborFinder, TestInitializeTriangleNeighbors);
        EATEST_REGISTER("TestInitializeVertexTriangleMap", "Testing InitializeVertexTriangleMap", TestTriangleNeighborFinder, TestInitializeVertexTriangleMap);
        EATEST_REGISTER("TestFindTriangleNeighbors", "Testing FindTriangleNeighbors", TestTriangleNeighborFinder, TestFindTriangleNeighbors);

        // Edge Tests
        EATEST_REGISTER("TestTJunction", "Testing a T Junction", TestTriangleNeighborFinder, TestTJunction);
        EATEST_REGISTER("TestIntersection", "Testing an intersection", TestTriangleNeighborFinder, TestIntersection);
        EATEST_REGISTER("TestEdgeSharedByTwoPairs", "Testing an edge shared by two pairs", TestTriangleNeighborFinder, TestEdgeSharedByTwoPairs);
        EATEST_REGISTER("TestLoopUnmatchedTriangles", "Testing a loop of unmatched triangles", TestTriangleNeighborFinder, TestLoopUnmatchedTriangles);
        EATEST_REGISTER("TestLoopTrianglePairs", "Testing a loop of triangle pairs triangles", TestTriangleNeighborFinder, TestLoopTrianglePairs);
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    }

private:

    void TestInitializeTriangleEdgeCosines();
    void TestInitializeTriangleNeighbors();
    void TestInitializeVertexTriangleMap();
    void TestFindTriangleNeighbors();

    void TestTJunction();
    void TestIntersection();
    void TestEdgeSharedByTwoPairs();
    void TestLoopUnmatchedTriangles();
    void TestLoopTrianglePairs();

    EA::Allocator::ICoreAllocator * m_allocator;

} TestTriangleNeighborFinderSingleton;

/**
Construct a single UnitCluster
*/
void
TestTriangleNeighborFinder::TestInitializeTriangleEdgeCosines()
{
    const uint32_t numTriangles = 256u;
    TriangleEdgeCosinesList * triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCosines, ("TriangleEdgeCosineList should have been allocated"));
    triangleEdgeCosines->resize(numTriangles);

    TriangleNeighborFinder::InitializeTriangleEdgeCosines(*triangleEdgeCosines);

    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        EATESTAssert(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE == (*triangleEdgeCosines)[triangleIndex].edgeCos[0], ("edgeCos should be CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE"));
        EATESTAssert(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE == (*triangleEdgeCosines)[triangleIndex].edgeCos[1], ("edgeCos should be CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE"));
        EATESTAssert(CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE == (*triangleEdgeCosines)[triangleIndex].edgeCos[2], ("edgeCos should be CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE"));
    }

    TriangleEdgeCosinesList::Free(m_allocator, triangleEdgeCosines);
}

void
TestTriangleNeighborFinder::TestInitializeTriangleNeighbors()
{
    const uint32_t numTriangles = 256u;
    TriangleNeighborsList * triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleNeighbors, ("TriangleNeighborList should have been allocated"));
    triangleNeighbors->resize(numTriangles);

    TriangleNeighborFinder::InitializeTriangleNeighbors(*triangleNeighbors);

    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        EATESTAssert(CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH == (*triangleNeighbors)[triangleIndex].neighbor[0], ("neighbor should be CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH"));
        EATESTAssert(CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH == (*triangleNeighbors)[triangleIndex].neighbor[1], ("neighbor should be CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH"));
        EATESTAssert(CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH == (*triangleNeighbors)[triangleIndex].neighbor[2], ("neighbor should be CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH"));
    }

    TriangleNeighborsList::Free(m_allocator, triangleNeighbors);
}

void
TestTriangleNeighborFinder::TestInitializeVertexTriangleMap()
{
    const uint32_t numTriangles = 32u;

    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);

    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        (*triangles)[triangleIndex].vertices[0] = triangleIndex;
        (*triangles)[triangleIndex].vertices[1] = triangleIndex + 1;
        (*triangles)[triangleIndex].vertices[2] = triangleIndex + 2;
    }

    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);

    TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *triangles);

    // Check the vertex iteration
    {
        VertexTriangleMap::VertexIterator vIt = vertexTriangleMap.VerticesBegin();
        const VertexTriangleMap::VertexIterator vItEnd = vertexTriangleMap.VerticesEnd();

        // Check the vertex iteration
        uint32_t vertexIndex = 0;
        while (vItEnd != vIt)
        {
            EATESTAssert(vertexIndex == *vIt, ("Vertex Index is incorrect"));
            ++vertexIndex;
            ++vIt;
        }
    }

    // Check the adjacent triangle iteration
    {
        VertexTriangleMap::VertexIterator vIt = vertexTriangleMap.VerticesBegin();

        // Check the first triangle
        {
            VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
            const VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(*vIt);

            EATESTAssert((*vIt) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert(atItEnd == atIt, ("AdjacentTriangleIteratorshould match AdjacentTriangleIteratorEnd"));
        }

        // Check the second triangle
        ++vIt;
        {
            VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
            const VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(*vIt);

            EATESTAssert((*vIt) - 1 == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert((*vIt) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert(atItEnd == atIt, ("AdjacentTriangleIteratorshould match AdjacentTriangleIteratorEnd"));
        }

        // Check the 2 -> N-2 triangles
        ++vIt;
        ++vIt;
        while (*vIt < (numTriangles - 1))
        {
            VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
            const VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(*vIt);

            EATESTAssert(((*vIt) - 2) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert(((*vIt) - 1) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert((*vIt) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert(atItEnd == atIt, ("AdjacentTriangleIteratorshould match AdjacentTriangleIteratorEnd"));

            ++vIt;
        }

        // Check the N - 2 triangle
        ++vIt;
        {
            VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
            const VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(*vIt);

            EATESTAssert(((*vIt) - 2) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert(((*vIt) - 1) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert(atItEnd == atIt, ("AdjacentTriangleIteratorshould match AdjacentTriangleIteratorEnd"));
        }

        // Check the N - 1 triangle
        ++vIt;
        {
            VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
            const VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(*vIt);

            EATESTAssert(((*vIt) - 2) == *atIt, ("Triangle Index is incorrect"));
            ++atIt;
            EATESTAssert(atItEnd == atIt, ("AdjacentTriangleIteratorshould match AdjacentTriangleIteratorEnd"));
        }
    }

    vertexTriangleMap.Release();
    TriangleList::Free(m_allocator, triangles);
}


void
TestTriangleNeighborFinder::TestFindTriangleNeighbors()
{
    const uint32_t vertexCountX = 2;
    const uint32_t vertexCountZ = 12;
    const uint32_t numTriangles = vertexCountX * (vertexCountZ - 1);

    // Initialize triangles
    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);

    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        if (triangleIndex % 2)
        {
            (*triangles)[triangleIndex].vertices[0] = triangleIndex;
            (*triangles)[triangleIndex].vertices[1] = triangleIndex + 2;
            (*triangles)[triangleIndex].vertices[2] = triangleIndex + 1;
        }
        else
        {
            (*triangles)[triangleIndex].vertices[0] = triangleIndex;
            (*triangles)[triangleIndex].vertices[1] = triangleIndex + 1;
            (*triangles)[triangleIndex].vertices[2] = triangleIndex + 2;
        }
    }

    // Initialize vertices
    VertexList * vertices = VertexList::Allocate(m_allocator, numTriangles * 3, EA::Allocator::MEM_PERM);
    EATESTAssert(vertices, ("VertexList should have been allocated"));
    vertices->resize(numTriangles * 3);

    for (uint32_t vertexIndexX = 0 ; vertexIndexX < vertexCountX ; ++vertexIndexX)
    {
        for (uint32_t vertexIndexZ = 0 ; vertexIndexZ < vertexCountZ ; ++vertexIndexZ)
        {
            (*vertices)[(vertexIndexZ * vertexCountX) + vertexIndexX] = meshbuilder::VectorType(static_cast<float>(vertexIndexX), 0.0f, static_cast<float>(vertexIndexZ));
        }
    }

    // Initialize triangle neighbors
    TriangleNeighborsList * triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleNeighbors, ("TriangleNeighborList should have been allocated"));
    triangleNeighbors->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleNeighbors(*triangleNeighbors);

    // Initialize vertex triangle map
    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);
    TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *triangles);

    // Initialize triangle EdgeCosines
    TriangleEdgeCosinesList * triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCosines, ("TriangleEdgeCosineList should have been allocated"));
    triangleEdgeCosines->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleEdgeCosines(*triangleEdgeCosines);

    // Initialize triangle flags
    TriangleFlagsList * triangleFlags = TriangleFlagsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleFlags, ("TriangleFlagList should have been allocated"));
    triangleFlags->resize(numTriangles);
    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        (*triangleFlags)[triangleIndex].enabled = true;
    }

    // Run the test
    TriangleNeighborFinder::FindTriangleNeighbors(
        *triangles,
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *vertices,
        vertexTriangleMap);

    // Check the first triangle
    EATESTAssert((*triangleNeighbors)[0].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
    EATESTAssert((*triangleNeighbors)[0].neighbor[1] == 1, ("Triangle Neighbor index is incorrect"));
    EATESTAssert((*triangleNeighbors)[0].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));

    EATESTAssert((*triangleEdgeCosines)[0].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[0].edgeCos[1], 1.0f), ("Triangle edge cosine is incorrect"));
    EATESTAssert((*triangleEdgeCosines)[0].edgeCos[2] == CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));

    // Check the N-1 triangles
    for (uint32_t triangleIndex = 1 ; triangleIndex < numTriangles - 1 ; ++triangleIndex)
    {
        if (triangleIndex % 2)
        {
            EATESTAssert((*triangleNeighbors)[triangleIndex].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
            EATESTAssert((*triangleNeighbors)[triangleIndex].neighbor[1] == triangleIndex + 1, ("Triangle Neighbor index is incorrect"));
            EATESTAssert((*triangleNeighbors)[triangleIndex].neighbor[2] == triangleIndex - 1, ("Triangle Neighbor index is incorrect"));

            EATESTAssert((*triangleEdgeCosines)[triangleIndex].edgeCos[0] == CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
            EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[triangleIndex].edgeCos[1], 1.0f), ("Triangle edge cosine is incorrect"));
            EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[triangleIndex].edgeCos[2], 1.0f), ("Triangle edge cosine is incorrect"));
        }
        else
        {
            EATESTAssert((*triangleNeighbors)[triangleIndex].neighbor[0] == triangleIndex - 1, ("Triangle Neighbor index is incorrect"));
            EATESTAssert((*triangleNeighbors)[triangleIndex].neighbor[1] == triangleIndex + 1, ("Triangle Neighbor index is incorrect"));
            EATESTAssert((*triangleNeighbors)[triangleIndex].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));

            EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[triangleIndex].edgeCos[0], 1.0f), ("Triangle edge cosine is incorrect"));
            EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[triangleIndex].edgeCos[1], 1.0f), ("Triangle edge cosine is incorrect"));
            EATESTAssert((*triangleEdgeCosines)[triangleIndex].edgeCos[2] == CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        }
    }

    // Check the last triangle
    EATESTAssert((*triangleNeighbors)[numTriangles - 1].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
    EATESTAssert((*triangleNeighbors)[numTriangles - 1].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
    EATESTAssert((*triangleNeighbors)[numTriangles - 1].neighbor[2] == numTriangles - 2, ("Triangle Neighbor index is incorrect"));

    EATESTAssert((*triangleEdgeCosines)[numTriangles - 1].edgeCos[0] == CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    EATESTAssert((*triangleEdgeCosines)[numTriangles - 1].edgeCos[1] == CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[numTriangles - 1].edgeCos[2], 1.0f), ("Triangle edge cosine is incorrect"));


    TriangleFlagsList::Free(m_allocator, triangleFlags);
    TriangleEdgeCosinesList::Free(m_allocator, triangleEdgeCosines);
    vertexTriangleMap.Release();
    TriangleNeighborsList::Free(m_allocator, triangleNeighbors);
    VertexList::Free(m_allocator, vertices);
    TriangleList::Free(m_allocator, triangles);
}


/**
Tests a t-junction of triangles. Three triangle which share and edge, two in the same place
and another at 90 degrees to the plane. Two triangle at 90 degrees should be neighbored, while the
other triangle in the plane should be unmatched.
*/
void
TestTriangleNeighborFinder::TestTJunction()
{
    const uint32_t numVertices = 5;
    const uint32_t numTriangles = 3;

    // Initialize triangles
    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);

    (*triangles)[0].vertices[0] = 0;
    (*triangles)[0].vertices[1] = 2;
    (*triangles)[0].vertices[2] = 1;

    (*triangles)[1].vertices[0] = 0;
    (*triangles)[1].vertices[1] = 3;
    (*triangles)[1].vertices[2] = 1;

    (*triangles)[2].vertices[0] = 0;
    (*triangles)[2].vertices[1] = 1;
    (*triangles)[2].vertices[2] = 4;


    // Initialize vertices
    VertexList * vertices = VertexList::Allocate(m_allocator, numVertices, EA::Allocator::MEM_PERM);
    EATESTAssert(vertices, ("VertexList should have been allocated"));
    vertices->resize(numVertices);

    (*vertices)[0] = meshbuilder::VectorType(0.0f, 0.0f, 0.0f);
    (*vertices)[1] = meshbuilder::VectorType(0.0f, 0.0f, 1.0f);
    (*vertices)[2] = meshbuilder::VectorType(0.0f, -1.0f, 0.0f);
    (*vertices)[3] = meshbuilder::VectorType(1.0f, 0.0f, 0.0f);
    (*vertices)[4] = meshbuilder::VectorType(-1.0f, 0.0f, 0.0f);

    // Initialize triangle neighbors
    TriangleNeighborsList * triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleNeighbors, ("TriangleNeighborList should have been allocated"));
    triangleNeighbors->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleNeighbors(*triangleNeighbors);

    // Initialize vertex triangle map
    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);
    TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *triangles);

    // Initialize triangle EdgeCosines
    TriangleEdgeCosinesList * triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCosines, ("TriangleEdgeCosineList should have been allocated"));
    triangleEdgeCosines->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleEdgeCosines(*triangleEdgeCosines);

    // Initialize triangle flags
    TriangleFlagsList * triangleFlags = TriangleFlagsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleFlags, ("TriangleFlagList should have been allocated"));
    triangleFlags->resize(numTriangles);
    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        (*triangleFlags)[triangleIndex].enabled = true;
    }

    // Run the test
    TriangleNeighborFinder::FindTriangleNeighbors(
        *triangles,
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *vertices,
        vertexTriangleMap);

    // Check the First Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[0].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[2] == 2, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[0].edgeCos[2], 2.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the Second Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[1].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Third Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[2].neighbor[0] == 0, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[2].edgeCos[0], 2.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    TriangleFlagsList::Free(m_allocator, triangleFlags);
    TriangleEdgeCosinesList::Free(m_allocator, triangleEdgeCosines);
    vertexTriangleMap.Release();
    TriangleNeighborsList::Free(m_allocator, triangleNeighbors);
    VertexList::Free(m_allocator, vertices);
    TriangleList::Free(m_allocator, triangles);
}

/**
Tests the intersection of two planes at 90 degrees. Two triangles in each plane, all four triangles share an
edge. Two triangles should be paired to create a 90 degree concave edge, while the other two should be paired
to create a 270 degree convex edge.
*/
void
TestTriangleNeighborFinder::TestIntersection()
{
    const uint32_t numVertices = 6;
    const uint32_t numTriangles = 4;

    // Initialize triangles
    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);

    (*triangles)[0].vertices[0] = 0;
    (*triangles)[0].vertices[1] = 2;
    (*triangles)[0].vertices[2] = 1;

    (*triangles)[1].vertices[0] = 0;
    (*triangles)[1].vertices[1] = 3;
    (*triangles)[1].vertices[2] = 1;

    (*triangles)[2].vertices[0] = 0;
    (*triangles)[2].vertices[1] = 1;
    (*triangles)[2].vertices[2] = 4;

    (*triangles)[3].vertices[0] = 0;
    (*triangles)[3].vertices[1] = 1;
    (*triangles)[3].vertices[2] = 5;


    // Initialize vertices
    VertexList * vertices = VertexList::Allocate(m_allocator, numVertices, EA::Allocator::MEM_PERM);
    EATESTAssert(vertices, ("VertexList should have been allocated"));
    vertices->resize(numVertices);

    (*vertices)[0] = meshbuilder::VectorType(0.0f, 0.0f, 0.0f);
    (*vertices)[1] = meshbuilder::VectorType(0.0f, 0.0f, 1.0f);
    (*vertices)[2] = meshbuilder::VectorType(0.0f, -1.0f, 0.0f);
    (*vertices)[3] = meshbuilder::VectorType(1.0f, 0.0f, 0.0f);
    (*vertices)[4] = meshbuilder::VectorType(-1.0f, 0.0f, 0.0f);
    (*vertices)[5] = meshbuilder::VectorType(0.0f, 1.0f, 0.0f);

    // Initialize triangle neighbors
    TriangleNeighborsList * triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleNeighbors, ("TriangleNeighborList should have been allocated"));
    triangleNeighbors->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleNeighbors(*triangleNeighbors);

    // Initialize vertex triangle map
    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);
    TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *triangles);

    // Initialize triangle EdgeCosines
    TriangleEdgeCosinesList * triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCosines, ("TriangleEdgeCosineList should have been allocated"));
    triangleEdgeCosines->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleEdgeCosines(*triangleEdgeCosines);

    // Initialize triangle flags
    TriangleFlagsList * triangleFlags = TriangleFlagsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleFlags, ("TriangleFlagList should have been allocated"));
    triangleFlags->resize(numTriangles);
    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        (*triangleFlags)[triangleIndex].enabled = true;
    }

    // Run the test
    TriangleNeighborFinder::FindTriangleNeighbors(
        *triangles,
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *vertices,
        vertexTriangleMap);

    // Check the First Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[0].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[2] == 2, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[0].edgeCos[2], 2.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the Second Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[1].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[2] == 3, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[1].edgeCos[2], 0.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the Third Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[2].neighbor[0] == 0, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[2].edgeCos[0], 2.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Fourth Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[3].neighbor[0] == 1, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[3].edgeCos[0], 0.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    TriangleFlagsList::Free(m_allocator, triangleFlags);
    TriangleEdgeCosinesList::Free(m_allocator, triangleEdgeCosines);
    vertexTriangleMap.Release();
    TriangleNeighborsList::Free(m_allocator, triangleNeighbors);
    VertexList::Free(m_allocator, vertices);
    TriangleList::Free(m_allocator, triangles);
}


/**
Tests two triangle pairs which each describe a 270 degree convex edge, sharing that edge. Two new
pairs should be generated, each describing a 90 degree concave edge.
*/
void
TestTriangleNeighborFinder::TestEdgeSharedByTwoPairs()
{
    const uint32_t numVertices = 6;
    const uint32_t numTriangles = 4;

    // Initialize triangles
    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);

    (*triangles)[0].vertices[0] = 0;
    (*triangles)[0].vertices[1] = 1;
    (*triangles)[0].vertices[2] = 2;

    (*triangles)[1].vertices[0] = 0;
    (*triangles)[1].vertices[1] = 3;
    (*triangles)[1].vertices[2] = 1;

    (*triangles)[2].vertices[0] = 0;
    (*triangles)[2].vertices[1] = 4;
    (*triangles)[2].vertices[2] = 1;

    (*triangles)[3].vertices[0] = 0;
    (*triangles)[3].vertices[1] = 1;
    (*triangles)[3].vertices[2] = 5;

    // Initialize vertices
    VertexList * vertices = VertexList::Allocate(m_allocator, numVertices, EA::Allocator::MEM_PERM);
    EATESTAssert(vertices, ("VertexList should have been allocated"));
    vertices->resize(numVertices);

    (*vertices)[0] = meshbuilder::VectorType(0.0f, 0.0f, 0.0f);
    (*vertices)[1] = meshbuilder::VectorType(0.0f, 0.0f, 1.0f);
    (*vertices)[2] = meshbuilder::VectorType(0.0f, -1.0f, 0.0f);
    (*vertices)[3] = meshbuilder::VectorType(1.0f, 0.0f, 0.0f);
    (*vertices)[4] = meshbuilder::VectorType(-1.0f, 0.0f, 0.0f);
    (*vertices)[5] = meshbuilder::VectorType(0.0f, 1.0f, 0.0f);

    // Initialize triangle neighbors
    TriangleNeighborsList * triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleNeighbors, ("TriangleNeighborList should have been allocated"));
    triangleNeighbors->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleNeighbors(*triangleNeighbors);

    // Initialize vertex triangle map
    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);
    TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *triangles);

    // Initialize triangle EdgeCosines
    TriangleEdgeCosinesList * triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCosines, ("TriangleEdgeCosineList should have been allocated"));
    triangleEdgeCosines->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleEdgeCosines(*triangleEdgeCosines);

    // Initialize triangle flags
    TriangleFlagsList * triangleFlags = TriangleFlagsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleFlags, ("TriangleFlagList should have been allocated"));
    triangleFlags->resize(numTriangles);
    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        (*triangleFlags)[triangleIndex].enabled = true;
    }

    // Run the test
    TriangleNeighborFinder::FindTriangleNeighbors(
        *triangles,
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *vertices,
        vertexTriangleMap);

    // Check the First Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[0].neighbor[0] == 1, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[0].edgeCos[0], 2.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Second Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[1].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[2] == 0, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[1].edgeCos[2], 2.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the Third Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[2].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[2] == 3, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[2].edgeCos[2], 2.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the Fourth Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[3].neighbor[0] == 2, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[3].edgeCos[0], 2.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    TriangleFlagsList::Free(m_allocator, triangleFlags);
    TriangleEdgeCosinesList::Free(m_allocator, triangleEdgeCosines);
    vertexTriangleMap.Release();
    TriangleNeighborsList::Free(m_allocator, triangleNeighbors);
    VertexList::Free(m_allocator, vertices);
    TriangleList::Free(m_allocator, triangles);
}

/**
Tests four triangles sharing a single edge. Each triangle is orientated by rotating the previous triangle by 90 degrees
around the shared edge. No triangles should be neighbored.
*/
void
TestTriangleNeighborFinder::TestLoopUnmatchedTriangles()
{
    const uint32_t numVertices = 6;
    const uint32_t numTriangles = 4;

    // Initialize triangles
    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);

    (*triangles)[0].vertices[0] = 0;
    (*triangles)[0].vertices[1] = 2;
    (*triangles)[0].vertices[2] = 1;

    (*triangles)[1].vertices[0] = 0;
    (*triangles)[1].vertices[1] = 3;
    (*triangles)[1].vertices[2] = 1;

    (*triangles)[2].vertices[0] = 0;
    (*triangles)[2].vertices[1] = 4;
    (*triangles)[2].vertices[2] = 1;

    (*triangles)[3].vertices[0] = 0;
    (*triangles)[3].vertices[1] = 5;
    (*triangles)[3].vertices[2] = 1;

    // Initialize vertices
    VertexList * vertices = VertexList::Allocate(m_allocator, numVertices, EA::Allocator::MEM_PERM);
    EATESTAssert(vertices, ("VertexList should have been allocated"));
    vertices->resize(numVertices);

    (*vertices)[0] = meshbuilder::VectorType(0.0f, 0.0f, 0.0f);
    (*vertices)[1] = meshbuilder::VectorType(0.0f, 0.0f, 1.0f);
    (*vertices)[2] = meshbuilder::VectorType(0.0f, -1.0f, 0.0f);
    (*vertices)[3] = meshbuilder::VectorType(1.0f, 0.0f, 0.0f);
    (*vertices)[4] = meshbuilder::VectorType(-1.0f, 0.0f, 0.0f);
    (*vertices)[5] = meshbuilder::VectorType(0.0f, 1.0f, 0.0f);

    // Initialize triangle neighbors
    TriangleNeighborsList * triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleNeighbors, ("TriangleNeighborList should have been allocated"));
    triangleNeighbors->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleNeighbors(*triangleNeighbors);

    // Initialize vertex triangle map
    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);
    TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *triangles);

    // Initialize triangle EdgeCosines
    TriangleEdgeCosinesList * triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCosines, ("TriangleEdgeCosineList should have been allocated"));
    triangleEdgeCosines->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleEdgeCosines(*triangleEdgeCosines);

    // Initialize triangle flags
    TriangleFlagsList * triangleFlags = TriangleFlagsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleFlags, ("TriangleFlagList should have been allocated"));
    triangleFlags->resize(numTriangles);
    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        (*triangleFlags)[triangleIndex].enabled = true;
    }

    // Run the test
    TriangleNeighborFinder::FindTriangleNeighbors(
        *triangles,
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *vertices,
        vertexTriangleMap);

    // Check the First Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[0].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Second Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[1].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Third Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[2].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Fourth Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[3].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    TriangleFlagsList::Free(m_allocator, triangleFlags);
    TriangleEdgeCosinesList::Free(m_allocator, triangleEdgeCosines);
    vertexTriangleMap.Release();
    TriangleNeighborsList::Free(m_allocator, triangleNeighbors);
    VertexList::Free(m_allocator, vertices);
    TriangleList::Free(m_allocator, triangles);
}


void
TestTriangleNeighborFinder::TestLoopTrianglePairs()
{
    const uint32_t numVertices = 10;
    const uint32_t numTriangles = 8;

    // Initialize triangles
    TriangleList * triangles = TriangleList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangles, ("TriangleList should have been allocated"));
    triangles->resize(numTriangles);

    (*triangles)[0].vertices[0] = 0;
    (*triangles)[0].vertices[1] = 2;
    (*triangles)[0].vertices[2] = 1;

    (*triangles)[1].vertices[0] = 0;
    (*triangles)[1].vertices[1] = 3;
    (*triangles)[1].vertices[2] = 1;

    (*triangles)[2].vertices[0] = 0;
    (*triangles)[2].vertices[1] = 1;
    (*triangles)[2].vertices[2] = 4;

    (*triangles)[3].vertices[0] = 0;
    (*triangles)[3].vertices[1] = 1;
    (*triangles)[3].vertices[2] = 5;

    (*triangles)[4].vertices[0] = 0;
    (*triangles)[4].vertices[1] = 6;
    (*triangles)[4].vertices[2] = 1;

    (*triangles)[5].vertices[0] = 0;
    (*triangles)[5].vertices[1] = 7;
    (*triangles)[5].vertices[2] = 1;

    (*triangles)[6].vertices[0] = 0;
    (*triangles)[6].vertices[1] = 1;
    (*triangles)[6].vertices[2] = 8;

    (*triangles)[7].vertices[0] = 0;
    (*triangles)[7].vertices[1] = 1;
    (*triangles)[7].vertices[2] = 9;

    // Initialize vertices
    VertexList * vertices = VertexList::Allocate(m_allocator, numVertices, EA::Allocator::MEM_PERM);
    EATESTAssert(vertices, ("VertexList should have been allocated"));
    vertices->resize(numVertices);

    (*vertices)[0] = meshbuilder::VectorType(0.0f, 0.0f, 0.0f);
    (*vertices)[1] = meshbuilder::VectorType(0.0f, 0.0f, 1.0f);
    (*vertices)[2] = meshbuilder::VectorType(1.0f, 0.000001f, 0.0f);
    (*vertices)[3] = meshbuilder::VectorType(0.000001f, 1.0f, 0.0f);
    (*vertices)[4] = meshbuilder::VectorType(-0.000001f, 1.0f, 0.0f);
    (*vertices)[5] = meshbuilder::VectorType(-1.0f, 0.000001f, 0.0f);
    (*vertices)[6] = meshbuilder::VectorType(-1.0f, -0.000001f, 0.0f);
    (*vertices)[7] = meshbuilder::VectorType(-0.000001f, -1.0f, 0.0f);
    (*vertices)[8] = meshbuilder::VectorType(0.000001f, -1.0f, 0.0f);
    (*vertices)[9] = meshbuilder::VectorType(1.0f, -0.000001f, 0.0f);

    // Initialize triangle neighbors
    TriangleNeighborsList * triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleNeighbors, ("TriangleNeighborList should have been allocated"));
    triangleNeighbors->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleNeighbors(*triangleNeighbors);

    // Initialize vertex triangle map
    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);
    TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *triangles);

    // Initialize triangle EdgeCosines
    TriangleEdgeCosinesList * triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleEdgeCosines, ("TriangleEdgeCosineList should have been allocated"));
    triangleEdgeCosines->resize(numTriangles);
    TriangleNeighborFinder::InitializeTriangleEdgeCosines(*triangleEdgeCosines);

    // Initialize triangle flags
    TriangleFlagsList * triangleFlags = TriangleFlagsList::Allocate(m_allocator, numTriangles, EA::Allocator::MEM_PERM);
    EATESTAssert(triangleFlags, ("TriangleFlagList should have been allocated"));
    triangleFlags->resize(numTriangles);
    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        (*triangleFlags)[triangleIndex].enabled = true;
    }

    // Run the test
    TriangleNeighborFinder::FindTriangleNeighbors(
        *triangles,
        *triangleEdgeCosines,
        *triangleNeighbors,
        *triangleFlags,
        *vertices,
        vertexTriangleMap);

    // Check the First Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[0].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[0].neighbor[2] == 7, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[0].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[0].edgeCos[2], 3.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the Second Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[1].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[1].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[1].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Third Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[2].neighbor[0] == 5, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[2].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[2].edgeCos[0], 1.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[2].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the Fourth Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[3].neighbor[0] == 4, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[3].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[3].edgeCos[0], 3.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[3].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the fifth Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[4].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[4].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[4].neighbor[2] == 3, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[4].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[4].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[4].edgeCos[2], 3.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the sixth Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[5].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[5].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[5].neighbor[2] == 2, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert((*triangleEdgeCosines)[5].edgeCos[0] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[5].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[5].edgeCos[2], 1.0f), ("Triangle edge cosine is incorrect"));
    }

    // Check the seventh Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[6].neighbor[0] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[6].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[6].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[6].edgeCos[0], CLUSTEREDMESHBUILDER_EDGECOS_OF_UNSHARED_EDGE), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[6].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[6].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    // Check the eight Triangle
    {
        // Neighbors
        EATESTAssert((*triangleNeighbors)[7].neighbor[0] == 0, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[7].neighbor[1] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        EATESTAssert((*triangleNeighbors)[7].neighbor[2] == CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH, ("Triangle Neighbor index is incorrect"));
        // EdgeCosines
        EATESTAssert(rwpmath::IsSimilar((*triangleEdgeCosines)[7].edgeCos[0], 3.0f), ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[7].edgeCos[1] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
        EATESTAssert((*triangleEdgeCosines)[7].edgeCos[2] ==  CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE, ("Triangle edge cosine is incorrect"));
    }

    TriangleFlagsList::Free(m_allocator, triangleFlags);
    TriangleEdgeCosinesList::Free(m_allocator, triangleEdgeCosines);
    vertexTriangleMap.Release();
    TriangleNeighborsList::Free(m_allocator, triangleNeighbors);
    VertexList::Free(m_allocator, vertices);
    TriangleList::Free(m_allocator, triangles);
}
