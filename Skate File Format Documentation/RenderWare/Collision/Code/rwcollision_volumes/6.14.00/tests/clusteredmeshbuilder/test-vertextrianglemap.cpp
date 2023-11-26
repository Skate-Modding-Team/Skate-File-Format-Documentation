// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/meshbuilder/detail/vertextrianglemap.h>

#include "testsuitebase.h" // For TestSuiteBase
#include "random.hpp"

using namespace rw::collision::meshbuilder::detail;

class TestVertexTriangleMap : public rw::collision::tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestVertexTriangleMap");

        EATEST_REGISTER("TestInsert", "Testing Insert", TestVertexTriangleMap, TestInsert);
        EATEST_REGISTER("TestSortAndIndexNonRandom", "Testing SortAndIndex", TestVertexTriangleMap, TestSortAndIndexNonRandom);
        EATEST_REGISTER("TestSortAndIndexPseudoRandom", "Testing SortAndIndex", TestVertexTriangleMap, TestSortAndIndexPseudoRandom);
    }

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    }

private:

    void TestInsert();
    void TestSortAndIndexNonRandom();
    void TestSortAndIndexPseudoRandom();

    EA::Allocator::ICoreAllocator * m_allocator;

} TestVertexTriangleMapSingleton;

/**
Test the insert method
*/
void
TestVertexTriangleMap::TestInsert()
{
    const uint32_t numTriangles = 128u;
    const uint32_t numVertices = 128u;

    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);

    for (uint32_t vertexIndex = 0 ; vertexIndex < numVertices ; ++vertexIndex)
    {
        vertexTriangleMap.Insert(vertexIndex, vertexIndex);
    }

    // Check that each entry has been added by iterating over the sorted and indexed entries
    vertexTriangleMap.SortAndIndex();

    // Check each vertex index
    {
        VertexTriangleMap::VertexIterator vIt = vertexTriangleMap.VerticesBegin();
        const VertexTriangleMap::VertexIterator vItEnd = vertexTriangleMap.VerticesEnd();

        uint32_t vertexIndex = 0u;
        while (vItEnd != vIt)
        {
            EATESTAssert(vertexIndex == *vIt, ("Vertex index is incorrect"));
            ++vertexIndex;
            ++vIt;
        }
    }

    // Check each triangle index
    {
        VertexTriangleMap::VertexIterator vIt = vertexTriangleMap.VerticesBegin();
        const VertexTriangleMap::VertexIterator vItEnd = vertexTriangleMap.VerticesEnd();

        uint32_t triangleIndex = 0u;
        while (vItEnd != vIt)
        {
            VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
            const VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(*vIt);

            EATESTAssert(triangleIndex == *atIt, ("Triangle index is incorrect"));
            ++atIt;
            EATESTAssert(atItEnd == atIt, ("Adjoining Triangle iterator is incorrect"));

            ++triangleIndex;
            ++vIt;
        }
    }

    vertexTriangleMap.Release();
}

/**
Test the SortAndIndex method with non random input
*/
void
TestVertexTriangleMap::TestSortAndIndexNonRandom()
{
    const uint32_t numTriangles = 16u;

    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);

    vertexTriangleMap.Insert(0, 2435);
    vertexTriangleMap.Insert(1, 12);
    vertexTriangleMap.Insert(1, 5125);
    vertexTriangleMap.Insert(2, 16);
    vertexTriangleMap.Insert(2, 16);
    vertexTriangleMap.Insert(2, 627);
    vertexTriangleMap.Insert(3, 73);
    vertexTriangleMap.Insert(4, 848);
    vertexTriangleMap.Insert(5, 62);
    vertexTriangleMap.Insert(6, 9267);
    vertexTriangleMap.Insert(7, 546);
    vertexTriangleMap.Insert(8, 274);
    vertexTriangleMap.Insert(8, 2);
    vertexTriangleMap.Insert(8, 727);
    vertexTriangleMap.Insert(8, 1);
    vertexTriangleMap.Insert(8, 1);

    vertexTriangleMap.SortAndIndex();

    // Check the sorted and indexed entries
    VertexTriangleMap::VertexIterator vIt = vertexTriangleMap.VerticesBegin();

    EATESTAssert(0 == *vIt, ("Vertex Index should be 0"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);

        EATESTAssert(2435 == *atIt, ("Triangle Index should be 2435"));
    }

    ++vIt;
    EATESTAssert(1 == *vIt, ("Vertex Index should be 1"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);

        EATESTAssert(12 == *atIt, ("Triangle Index should be 12"));
        ++atIt;
        EATESTAssert(5125 == *atIt, ("Triangle Index should be 5125"));
    }

    ++vIt;
    EATESTAssert(2 == *vIt, ("Vertex Index should be 2"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);

        EATESTAssert(16 == *atIt, ("Triangle Index should be 16"));
        ++atIt;
        EATESTAssert(16 == *atIt, ("Triangle Index should be 16"));
        ++atIt;
        EATESTAssert(627 == *atIt, ("Triangle Index should be 627"));
    }

    ++vIt;
    EATESTAssert(3 == *vIt, ("Vertex Index should be 3"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
        EATESTAssert(73 == *atIt, ("Triangle Index should be 73"));
    }

    ++vIt;
    EATESTAssert(4 == *vIt, ("Vertex Index should be 4"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
        EATESTAssert(848 == *atIt, ("Triangle Index should be 848"));
    }

    ++vIt;
    EATESTAssert(5 == *vIt, ("Vertex Index should be 5"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
        EATESTAssert(62 == *atIt, ("Triangle Index should be 62"));
    }

    ++vIt;
    EATESTAssert(6 == *vIt, ("Vertex Index should be 6"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
        EATESTAssert(9267 == *atIt, ("Triangle Index should be 9267"));
    }

    ++vIt;
    EATESTAssert(7 == *vIt, ("Vertex Index should be 7"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);
        EATESTAssert(546 == *atIt, ("Triangle Index should be 546"));
    }

    ++vIt;
    EATESTAssert(8 == *vIt, ("Vertex Index should be 8"));
    {
        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(*vIt);

        EATESTAssert(1 == *atIt, ("Triangle Index should be 1"));
        ++atIt;
        EATESTAssert(1 == *atIt, ("Triangle Index should be 1"));
        ++atIt;
        EATESTAssert(2 == *atIt, ("Triangle Index should be 2"));
        ++atIt;
        EATESTAssert(274 == *atIt, ("Triangle Index should be 274"));
        ++atIt;
        EATESTAssert(727 == *atIt, ("Triangle Index should be 727"));
    }

    vertexTriangleMap.Release();
}

/**
Test the SortAndIndex method with pseudo random input
*/
void
TestVertexTriangleMap::TestSortAndIndexPseudoRandom()
{
    const uint32_t numTriangles = 128u;

    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(numTriangles, m_allocator);

    SeedRandom(9u);

    for (uint32_t triangleIndex = 0 ; triangleIndex < numTriangles ; ++triangleIndex)
    {
        vertexTriangleMap.Insert(Random(1, numTriangles * 3u), Random(1u, numTriangles / 2));
    }

    vertexTriangleMap.SortAndIndex();

    // Check the sorted and indexed entries
    VertexTriangleMap::VertexIterator vIt = vertexTriangleMap.VerticesBegin();
    const VertexTriangleMap::VertexIterator vItEnd = vertexTriangleMap.VerticesEnd();

    uint32_t vertexIndex = 0;
    while (vItEnd != vIt)
    {
        EATESTAssert(vertexIndex < *vIt, ("Vertex Index is incorrect"));
        vertexIndex = *vIt;

        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(vertexIndex);
        const VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(vertexIndex);
        uint32_t triangleIndex = 0;
        while (atItEnd != atIt)
        {
            EATESTAssert(triangleIndex <= *atIt, ("Triangle Index is incorrect"));
            triangleIndex = *atIt;
            ++atIt;
        }
        ++vIt;
    }

    vertexTriangleMap.Release();
}
