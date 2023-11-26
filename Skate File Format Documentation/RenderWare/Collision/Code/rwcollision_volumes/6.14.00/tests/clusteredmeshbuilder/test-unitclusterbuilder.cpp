// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/libcore.h>

#include <rw/collision/meshbuilder/detail/unitclusterbuilder.h>
#include <rw/collision/meshbuilder/detail/unitclusterstack.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/types.h>

#include <stdio.h> // for sprintf

#include "testsuitebase.h" // For TestSuiteBase

// Unit tests for the unit cluster builder

using namespace rw::collision::meshbuilder::detail;

class TestUnitClusterBuilder : public rw::collision::tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestUnitClusterBuilder");

        EATEST_REGISTER("TestAddUnitSingleTriangle", "Adding a single triangle unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitSingleTriangle);
        EATEST_REGISTER("TestAddUnitSingleQuad", "Adding a single quad unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitSingleQuad);
        EATEST_REGISTER("TestAddUnitSingleTriangleAndQuad", "Adding a single triangle and quad unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitSingleTriangleAndQuad);
        EATEST_REGISTER("TestAddUnitOverflowTriangle", "Add a single triangle to a full cluster", TestUnitClusterBuilder, TestAddUnitOverflowTriangle);
        EATEST_REGISTER("TestAddUnitOverflowQuad", "Add a single triangle to a full cluster", TestUnitClusterBuilder, TestAddUnitOverflowQuad);
        EATEST_REGISTER("TestAddUnitOverflowMixed","Add a single triangle to a full cluster of mixed units", TestUnitClusterBuilder, TestAddUnitOverflowMixed);

        EATEST_REGISTER("TestAddUnitsToUnitClusterSingleTriangle", "Adding a single triangle unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitsToUnitClusterSingleTriangle);
        EATEST_REGISTER("TestAddUnitsToUnitClusterSingleQuad", "Adding a single quad unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitsToUnitClusterSingleQuad);
        EATEST_REGISTER("TestAddUnitsToUnitClusterSingleTriangleAndQuad", "Adding a single triangle and quad unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitsToUnitClusterSingleTriangleAndQuad);
        EATEST_REGISTER("TestAddUnitsToUnitClusterOverflowTriangle", "Adding a single triangle unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitsToUnitClusterOverflowTriangle);
        EATEST_REGISTER("TestAddUnitsToUnitClusterOverflowQuad", "Adding a single triangle unit to an empty cluster", TestUnitClusterBuilder, TestAddUnitsToUnitClusterOverflowQuad);

        EATEST_REGISTER("TestAddUnitsToUnitClusterAddMiddleUnit", "Adding a single triangle unit from the middle of a list", TestUnitClusterBuilder, TestAddUnitsToUnitClusterAddMiddleUnit);
    }

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();
        m_allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    }

private:

    void TestAddUnitSingleTriangle();
    void TestAddUnitSingleQuad();
    void TestAddUnitSingleTriangleAndQuad();
    void TestAddUnitOverflowTriangle();
    void TestAddUnitOverflowQuad();
    void TestAddUnitOverflowMixed();

    void TestAddUnitsToUnitClusterSingleTriangle();
    void TestAddUnitsToUnitClusterSingleQuad();
    void TestAddUnitsToUnitClusterSingleTriangleAndQuad();
    void TestAddUnitsToUnitClusterOverflowTriangle();
    void TestAddUnitsToUnitClusterOverflowQuad();

    void TestAddUnitsToUnitClusterAddMiddleUnit();

    static const uint32_t m_maxVerticesPerTriangle = 3u;
    static const uint32_t m_maxVerticesPerQuad = 4u;

    EA::Allocator::ICoreAllocator * m_allocator;


} TestUnitClusterBuilderSingleton;

/**
Adding a single triangle unit to an empty cluster.
*/
void
TestUnitClusterBuilder::TestAddUnitSingleTriangle()
{
    // The total number of units
    const uint32_t unitCount = 1u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, 1, EA::Allocator::MEM_PERM);
    triangleList->resize(1);
    Triangle & triangle = (*triangleList)[0];
    triangle.vertices[0] = 0u;
    triangle.vertices[1] = 1u;
    triangle.vertices[2] = 2u;


    // Create a unit list containing 1 triangle unit
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    Unit & unit = (*unitList)[0];
    unit.tri0 = 0u;
    unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
    unit.type = Unit::TYPE_TRIANGLE;
    unit.extraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
    unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST

    bool added = UnitClusterBuilder::AddUnitToCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        0,
        *triangleList,
        *unitList,
        m_maxVerticesPerTriangle);

    // Check the unit has been added
    EATESTAssert(true == added, ("Should have been able to add unit to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 1, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->unitIDs[0] == 0, "UnitCluster should contain unitID 0");
    EATESTAssert(unitCluster->numVertices == 3, "UnitCluster should contain 3 vertices");
    EATESTAssert(unitCluster->vertexIDs[0] == 0, "UnitCluster should contain vertexID 0");
    EATESTAssert(unitCluster->vertexIDs[1] == 1, "UnitCluster should contain vertexID 1");
    EATESTAssert(unitCluster->vertexIDs[2] == 2, "UnitCluster should contain vertexID 2");

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}


/**
Adding a single quad unit to an empty cluster.
*/
void
TestUnitClusterBuilder::TestAddUnitSingleQuad()
{
    // The total number of units
    const uint32_t unitCount = 1u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, 2, EA::Allocator::MEM_PERM);
    triangleList->resize(2);
    for (uint32_t triangleIndex = 0 ; triangleIndex < 2 ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }

    // Create a unit list containing a single quad
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    Unit & unit = (*unitList)[0];
    unit.tri0 = 0u;
    unit.tri1 = 1u;
    unit.type = Unit::TYPE_QUAD;
    unit.extraVertex = 0u; 
    unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST

    bool added = UnitClusterBuilder::AddUnitToCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        0,
        *triangleList,
        *unitList,
        m_maxVerticesPerQuad);

    // Check the unit has been added
    EATESTAssert(true == added, ("Should have been able to add unit to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 1, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->unitIDs[0] == 0, "UnitCluster should contain unitID 0");
    EATESTAssert(unitCluster->numVertices == 4, "UnitCluster should contain 4 vertices");
    EATESTAssert(unitCluster->vertexIDs[0] == 0, "UnitCluster should contain vertexID 0");
    EATESTAssert(unitCluster->vertexIDs[1] == 1, "UnitCluster should contain vertexID 1");
    EATESTAssert(unitCluster->vertexIDs[2] == 2, "UnitCluster should contain vertexID 2");
    EATESTAssert(unitCluster->vertexIDs[3] == 3, "UnitCluster should contain vertexID 3");

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single triangle and quad unit to an empty cluster.
*/
void
TestUnitClusterBuilder::TestAddUnitSingleTriangleAndQuad()
{
    // The total number of units
    const uint32_t unitCount = 2u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, 3, EA::Allocator::MEM_PERM);
    triangleList->resize(3);
    for (uint32_t triangleIndex = 0 ; triangleIndex < 3 ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }

    // Create a unit list containing 10 identical units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    // Triangle Unit
    {
        Unit & unit = (*unitList)[0];
        unit.tri0 = 0u;
        unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
        unit.type = Unit::TYPE_TRIANGLE;
        unit.extraVertex = 0u; // NOT REQUIRED FOR THIS TEST
        unit.edgeOpposingExtraVertex = 0u; // NOT REQUIRED FOR THIS TEST
    }
    // Quad Unit
    {
        Unit & unit = (*unitList)[1];
        unit.tri0 = 1u;
        unit.tri1 = 2u;
        unit.type = Unit::TYPE_QUAD;
        unit.extraVertex = 0u; 
        unit.edgeOpposingExtraVertex = 0u; // NOT REQUIRED FOR THIS TEST
    }

    // Attempt to add the triangle unit to the cluster
    bool added = UnitClusterBuilder::AddUnitToCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        0,
        *triangleList,
        *unitList,
        m_maxVerticesPerTriangle);

    // Check the unit has been added
    EATESTAssert(true == added, ("Should have been able to add unit to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 1, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->unitIDs[0] == 0, "UnitCluster should contain unitID 0");
    EATESTAssert(unitCluster->numVertices == 3, "UnitCluster should contain 4 vertices");
    EATESTAssert(unitCluster->vertexIDs[0] == 0, "UnitCluster should contain vertexID 0");
    EATESTAssert(unitCluster->vertexIDs[1] == 1, "UnitCluster should contain vertexID 1");
    EATESTAssert(unitCluster->vertexIDs[2] == 2, "UnitCluster should contain vertexID 2");

    // Attempt to add the triangle unit to the cluster
    added = UnitClusterBuilder::AddUnitToCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        1,
        *triangleList,
        *unitList,
        m_maxVerticesPerQuad);

    // Check the unit has been added
    EATESTAssert(true == added, ("Should have been able to add unit to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 2, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->unitIDs[1] == 1, "UnitCluster should contain unitID 0");
    EATESTAssert(unitCluster->numVertices == 7, "UnitCluster should contain 4 vertices");
    EATESTAssert(unitCluster->vertexIDs[3] == 3, "UnitCluster should contain vertexID 3");
    EATESTAssert(unitCluster->vertexIDs[4] == 4, "UnitCluster should contain vertexID 4");
    EATESTAssert(unitCluster->vertexIDs[5] == 5, "UnitCluster should contain vertexID 5");
    EATESTAssert(unitCluster->vertexIDs[6] == 6, "UnitCluster should contain vertexID 6");

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single triangle unit to an full cluster, expecting an overflow.
*/
void
TestUnitClusterBuilder::TestAddUnitOverflowTriangle()
{
    // The total number of units
    const uint32_t unitCount = 86;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    triangleList->resize(unitCount);
    for (uint32_t triangleIndex = 0 ; triangleIndex < unitCount ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }


    // Create a unit list containing X triangle Units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    for (uint32_t unitIndex = 0 ; unitIndex < unitCount ; ++unitIndex)
    {
        Unit & unit = (*unitList)[unitIndex];
        unit.tri0 = unitIndex;
        unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
        unit.type = Unit::TYPE_TRIANGLE;
        unit.extraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
        unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
    }

    // Attempt to add enough units to fill the Clusters vertex count
    for (uint32_t unitIndex = 0 ; unitIndex < (unitCount - 1) ; ++unitIndex)
    {
        bool added = UnitClusterBuilder::AddUnitToCluster(
            unitCluster->vertexIDs,
            unitCluster->numVertices,
            unitCluster->unitIDs,
            unitCluster->numUnits,
            unitIndex,
            *triangleList,
            *unitList,
            m_maxVerticesPerTriangle);

        // Check that the unit has been added
        EATESTAssert(true == added, ("Should have been able to add unit to cluster"));

        // Check the state of the Unit Cluster
        char buffer[256];
        sprintf(buffer, "UnitCluster should contain %d unit", unitIndex + 1);
        EATESTAssert(unitCluster->numUnits == (unitIndex + 1), buffer);
        sprintf(buffer, "UnitCluster should contain unitID %d", unitIndex);
        EATESTAssert(unitCluster->unitIDs[unitIndex] == unitIndex, buffer);
        sprintf(buffer, "UnitCluster should contain %d vertices", (unitIndex + 1) * 3);
        EATESTAssert(unitCluster->numVertices == ((unitIndex + 1) * 3), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 0);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 0] == ((unitIndex * 3) + 0), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 1);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 1] == ((unitIndex * 3) + 1), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 2);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 2] == ((unitIndex * 3) + 2), buffer);
    }

    bool added = UnitClusterBuilder::AddUnitToCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        unitCount - 1,
        *triangleList,
        *unitList,
        m_maxVerticesPerTriangle);
    EATESTAssert(false == added, ("Should not have been able to add unit to cluster"));

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single quad unit to an full cluster, expecting an overflow.
*/
void
TestUnitClusterBuilder::TestAddUnitOverflowQuad()
{
    // The total number of units
    const uint32_t unitCount = 128u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    triangleList->resize(unitCount);
    for (uint32_t triangleIndex = 0 ; triangleIndex < unitCount ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }

    // Create a unit list containing X quad units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    for (uint32_t unitIndex = 0 ; unitIndex < unitCount ; ++unitIndex)
    {
        Unit & unit = (*unitList)[unitIndex];
        unit.tri0 = unitIndex;
        unit.tri1 = 64 + unitIndex;
        unit.type = Unit::TYPE_QUAD;
        unit.extraVertex = 0u;
        unit.edgeOpposingExtraVertex = 0u; // NOT REQUIRED FOR THIS TEST
    }

    for (uint32_t unitIndex = 0 ; unitIndex < 63 ; ++unitIndex)
    {
        bool added = UnitClusterBuilder::AddUnitToCluster(
            unitCluster->vertexIDs,
            unitCluster->numVertices,
            unitCluster->unitIDs,
            unitCluster->numUnits,
            unitIndex,
            *triangleList,
            *unitList,
            m_maxVerticesPerQuad);

        // Check that the unit has been added
        EATESTAssert(true == added, ("Should have been able to add unit to cluster"));

        // Check the state of the Unit Cluster
        char buffer[256];
        sprintf(buffer, "UnitCluster should contain %d unit", unitIndex + 1);
        EATESTAssert(unitCluster->numUnits == (unitIndex + 1), buffer);
        sprintf(buffer, "UnitCluster should contain unitID %d", unitIndex);
        EATESTAssert(unitCluster->unitIDs[unitIndex] == unitIndex, buffer);
        sprintf(buffer, "UnitCluster should contain %d vertices", (unitIndex + 1) * 4);
        EATESTAssert(unitCluster->numVertices == ((unitIndex + 1) * 4), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 0);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 4) + 0] == ((unitIndex * 3) + 0), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 1);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 4) + 1] == ((unitIndex * 3) + 1), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 2);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 4) + 2] == ((unitIndex * 3) + 2), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", ((unitIndex + 64) * 3));
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 4) + 3] == (((unitIndex + 64) * 3)), buffer);
    }

    bool added = UnitClusterBuilder::AddUnitToCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        63,
        *triangleList,
        *unitList,
        m_maxVerticesPerQuad);
    EATESTAssert(false == added, ("Should not have been able to add unit to cluster"));

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single quad unit to an full triangle cluster, expecting an overflow.
*/
void
TestUnitClusterBuilder::TestAddUnitOverflowMixed()
{
    // The total number of units
    const uint32_t unitCount = 86;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, unitCount + 1, EA::Allocator::MEM_PERM);
    triangleList->resize(unitCount + 1);
    for (uint32_t triangleIndex = 0 ; triangleIndex < unitCount + 1 ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }


    // Create a unit list containing X triangle units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    for (uint32_t unitIndex = 0 ; unitIndex < unitCount - 1 ; ++unitIndex)
    {
        Unit & unit = (*unitList)[unitIndex];
        unit.tri0 = unitIndex;
        unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
        unit.type = Unit::TYPE_TRIANGLE;
        unit.extraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
        unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
    }

    // Add a single Quad unit to the unit list
    Unit & unit = (*unitList)[unitCount - 1];
    unit.tri0 = unitCount - 1;
    unit.tri1 = unitCount;
    unit.type = Unit::TYPE_QUAD;
    unit.extraVertex = 0u;
    unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST

    // Attempt to add enough units to fill the Clusters vertex count
    for (uint32_t unitIndex = 0 ; unitIndex < (unitCount - 1) ; ++unitIndex)
    {
        bool added = UnitClusterBuilder::AddUnitToCluster(
            unitCluster->vertexIDs,
            unitCluster->numVertices,
            unitCluster->unitIDs,
            unitCluster->numUnits,
            unitIndex,
            *triangleList,
            *unitList,
            m_maxVerticesPerTriangle);

        // Check that the unit has been added
        EATESTAssert(true == added, ("Should have been able to add unit to cluster"));

        // Check the state of the Unit Cluster
        char buffer[256];
        sprintf(buffer, "UnitCluster should contain %d unit", unitIndex + 1);
        EATESTAssert(unitCluster->numUnits == (unitIndex + 1), buffer);
        sprintf(buffer, "UnitCluster should contain unitID %d", unitIndex);
        EATESTAssert(unitCluster->unitIDs[unitIndex] == unitIndex, buffer);
        sprintf(buffer, "UnitCluster should contain %d vertices", (unitIndex + 1) * 3);
        EATESTAssert(unitCluster->numVertices == ((unitIndex + 1) * 3), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 0);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 0] == ((unitIndex * 3) + 0), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 1);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 1] == ((unitIndex * 3) + 1), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 2);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 2] == ((unitIndex * 3) + 2), buffer);
    }

    bool added = UnitClusterBuilder::AddUnitToCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        unitCount - 1,
        *triangleList,
        *unitList,
        m_maxVerticesPerQuad);
    EATESTAssert(false == added, ("Should not have been able to add unit to cluster"));

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}


/**
Adding a single triangle unit to an empty cluster.
*/
void
TestUnitClusterBuilder::TestAddUnitsToUnitClusterSingleTriangle()
{
    // The total number of units
    const uint32_t unitCount = 1u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, 1, EA::Allocator::MEM_PERM);
    triangleList->resize(1);
    Triangle & triangle = (*triangleList)[0];
    triangle.vertices[0] = 0u;
    triangle.vertices[1] = 1u;
    triangle.vertices[2] = 2u;


    // Create a unit list containing 1 triangle unit
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    Unit & unit = (*unitList)[0];
    unit.tri0 = 0u;
    unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
    unit.type = Unit::TYPE_TRIANGLE;
    unit.extraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
    unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST

    const uint32_t startUnitIndex = 0;
    const uint32_t unitAddCount = 1;

    const uint32_t unitsAdded = UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        unitAddCount,
        *triangleList,
        *unitList,
        m_maxVerticesPerTriangle);

    // Check the unit has been added
    EATESTAssert(1 == unitsAdded, ("Should have been able to add unit to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 1, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->unitIDs[0] == 0, "UnitCluster should contain unitID 0");
    EATESTAssert(unitCluster->numVertices == 3, "UnitCluster should contain 3 vertices");
    EATESTAssert(unitCluster->vertexIDs[0] == 0, "UnitCluster should contain vertexID 0");
    EATESTAssert(unitCluster->vertexIDs[1] == 1, "UnitCluster should contain vertexID 1");
    EATESTAssert(unitCluster->vertexIDs[2] == 2, "UnitCluster should contain vertexID 2");

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single quad unit to an empty cluster.
*/
void
TestUnitClusterBuilder::TestAddUnitsToUnitClusterSingleQuad()
{
    // The total number of units
    const uint32_t unitCount = 1u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, 2, EA::Allocator::MEM_PERM);
    triangleList->resize(2);
    for (uint32_t triangleIndex = 0 ; triangleIndex < 2 ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }

    // Create a unit list containing a single quad
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    Unit & unit = (*unitList)[0];
    unit.tri0 = 0u;
    unit.tri1 = 1u;
    unit.type = Unit::TYPE_QUAD;
    unit.extraVertex = 0u; 
    unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST

    const uint32_t startUnitIndex = 0;
    const uint32_t unitAddCount = 1;

    const uint32_t unitsAdded = UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        unitAddCount,
        *triangleList,
        *unitList,
        m_maxVerticesPerQuad);

    // Check the unit has been added
    EATESTAssert(1 == unitsAdded, ("Should have added 1 unit to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 1, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->unitIDs[0] == 0, "UnitCluster should contain unitID 0");
    EATESTAssert(unitCluster->numVertices == 4, "UnitCluster should contain 4 vertices");
    EATESTAssert(unitCluster->vertexIDs[0] == 0, "UnitCluster should contain vertexID 0");
    EATESTAssert(unitCluster->vertexIDs[1] == 1, "UnitCluster should contain vertexID 1");
    EATESTAssert(unitCluster->vertexIDs[2] == 2, "UnitCluster should contain vertexID 2");
    EATESTAssert(unitCluster->vertexIDs[3] == 3, "UnitCluster should contain vertexID 3");

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single triangle and quad unit to an empty cluster.
*/
void
TestUnitClusterBuilder::TestAddUnitsToUnitClusterSingleTriangleAndQuad()
{

    // The total number of units
    const uint32_t unitCount = 2u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, 3, EA::Allocator::MEM_PERM);
    triangleList->resize(3);
    for (uint32_t triangleIndex = 0 ; triangleIndex < 3 ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }

    // Create a unit list containing 10 identical units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    // Triangle Unit
    {
        Unit & unit = (*unitList)[0];
        unit.tri0 = 0u;
        unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
        unit.type = Unit::TYPE_TRIANGLE;
        unit.extraVertex = 0u; // NOT REQUIRED FOR THIS TEST
        unit.edgeOpposingExtraVertex = 0u; // NOT REQUIRED FOR THIS TEST
    }
    // Quad Unit
    {
        Unit & unit = (*unitList)[1];
        unit.tri0 = 1u;
        unit.tri1 = 2u;
        unit.type = Unit::TYPE_QUAD;
        unit.extraVertex = 0u; 
        unit.edgeOpposingExtraVertex = 0u; // NOT REQUIRED FOR THIS TEST
    }

    const uint32_t startUnitIndex = 0;
    const uint32_t unitAddCount = 2;

    const uint32_t unitsAdded = UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        unitAddCount,
        *triangleList,
        *unitList,
        m_maxVerticesPerQuad);

    // Check the unit has been added
    EATESTAssert(2 == unitsAdded, ("Should have added 2 units to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 2, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->numVertices == 7, "UnitCluster should contain 4 vertices");

    EATESTAssert(unitCluster->unitIDs[0] == 0, "UnitCluster should contain unitID 0");
    EATESTAssert(unitCluster->unitIDs[1] == 1, "UnitCluster should contain unitID 0");

    EATESTAssert(unitCluster->vertexIDs[0] == 0, "UnitCluster should contain vertexID 0");
    EATESTAssert(unitCluster->vertexIDs[1] == 1, "UnitCluster should contain vertexID 1");
    EATESTAssert(unitCluster->vertexIDs[2] == 2, "UnitCluster should contain vertexID 2");
    EATESTAssert(unitCluster->vertexIDs[3] == 3, "UnitCluster should contain vertexID 3");
    EATESTAssert(unitCluster->vertexIDs[4] == 4, "UnitCluster should contain vertexID 4");
    EATESTAssert(unitCluster->vertexIDs[5] == 5, "UnitCluster should contain vertexID 5");
    EATESTAssert(unitCluster->vertexIDs[6] == 6, "UnitCluster should contain vertexID 6");

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single triangle unit to a full cluster, expect overflow.
*/
void
TestUnitClusterBuilder::TestAddUnitsToUnitClusterOverflowTriangle()
{
    // The total number of units
    const uint32_t unitCount = 86;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing a single triangle
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    triangleList->resize(unitCount);
    for (uint32_t triangleIndex = 0 ; triangleIndex < unitCount ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }


    // Create a unit list containing X triangle Units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    for (uint32_t unitIndex = 0 ; unitIndex < unitCount ; ++unitIndex)
    {
        Unit & unit = (*unitList)[unitIndex];
        unit.tri0 = unitIndex;
        unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
        unit.type = Unit::TYPE_TRIANGLE;
        unit.extraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
        unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
    }

    const uint32_t startUnitIndex = 0;
    const uint32_t unitAddCount = 86;

    const uint32_t unitsAdded = UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        unitAddCount,
        *triangleList,
        *unitList,
        m_maxVerticesPerTriangle);

    // Check the unit has been added
    EATESTAssert(85 == unitsAdded, ("Should have added 85 units to cluster"));

    // Check the state of the Unit Cluster
    EATESTAssert(unitCluster->numUnits == 85, "UnitCluster should contain 85 units");
    EATESTAssert(unitCluster->numVertices == 255, "UnitCluster should contain 255 vertices");

    for (uint32_t unitIndex = 0 ; unitIndex < 85 ; ++unitIndex)
    {
        char buffer[256];
        sprintf(buffer, "UnitCluster should contain unitID %d", unitIndex);
        EATESTAssert(unitCluster->unitIDs[unitIndex] == unitIndex, buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 0);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 0] == ((unitIndex * 3) + 0), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 1);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 1] == ((unitIndex * 3) + 1), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 2);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 2] == ((unitIndex * 3) + 2), buffer);
    }

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}

/**
Adding a single quad unit to a full cluster, expect overflow.
*/
void
TestUnitClusterBuilder::TestAddUnitsToUnitClusterOverflowQuad()
{
    // The total number of units
    const uint32_t unitCount = 128u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing X triangles
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    triangleList->resize(unitCount);
    for (uint32_t triangleIndex = 0 ; triangleIndex < unitCount ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }

    // Create a unit list containing X quad units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    for (uint32_t unitIndex = 0 ; unitIndex < unitCount ; ++unitIndex)
    {
        Unit & unit = (*unitList)[unitIndex];
        unit.tri0 = unitIndex;
        unit.tri1 = 64 + unitIndex;
        unit.type = Unit::TYPE_QUAD;
        unit.extraVertex = 0u;
        unit.edgeOpposingExtraVertex = 0u; // NOT REQUIRED FOR THIS TEST
    }

    const uint32_t startUnitIndex = 0;
    const uint32_t unitAddCount = 64;

    const uint32_t unitsAdded = UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        unitAddCount,
        *triangleList,
        *unitList,
        m_maxVerticesPerQuad);

    // Check the unit has been added
    EATESTAssert(63 == unitsAdded, ("Should have added 63 units to cluster"));

    // Check the state of the Unit Cluster
    EATESTAssert(unitCluster->numUnits == 63, "UnitCluster should contain 63 unit");
    EATESTAssert(unitCluster->numVertices == 252, "UnitCluster should contain 252 vertices");

    for (uint32_t unitIndex = 0 ; unitIndex < 63 ; ++unitIndex)
    {
        char buffer[256];
        sprintf(buffer, "UnitCluster should contain unitID %d", unitIndex);
        EATESTAssert(unitCluster->unitIDs[unitIndex] == unitIndex, buffer);

        // The vertexIDs should have been sorted so the order in which they appear in the list
        // does not match the order in which they were processed.
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 0);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 0] == ((unitIndex * 3) + 0), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 1);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 1] == ((unitIndex * 3) + 1), buffer);
        sprintf(buffer, "UnitCluster should contain vertexID %d", (unitIndex * 3) + 2);
        EATESTAssert(unitCluster->vertexIDs[(unitIndex * 3) + 2] == ((unitIndex * 3) + 2), buffer);

        // The "quad vertex" IDs should have been sorted and placed at the end of the list, hence
        // the awkward indexing here.
        sprintf(buffer, "UnitCluster should contain vertexID %d", ((((unitIndex+1) * 3) + 189)));
        EATESTAssert(unitCluster->vertexIDs[(unitIndex + 189)] == ((((unitIndex+1) * 3) + 189)), buffer);
    }

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}


/**
Adding a single triangle unit to a cluster from the middle of a unit list.
*/
void
TestUnitClusterBuilder::TestAddUnitsToUnitClusterAddMiddleUnit()
{
    // The total number of units
    const uint32_t unitCount = 5u;

    // Create a UnitClusterStack
    UnitClusterStack unitClusterStack;
    unitClusterStack.Initialize(EA::Allocator::ICoreAllocator::GetDefaultAllocator(), unitCount);

    UnitCluster * unitCluster = unitClusterStack.GetUnitCluster();

    // Create the triangle list containing X triangles
    TriangleList * triangleList = TriangleList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    triangleList->resize(5);
    for (uint32_t triangleIndex = 0 ; triangleIndex < 5 ; ++triangleIndex)
    {
        Triangle & triangle = (*triangleList)[triangleIndex];
        triangle.vertices[0] = (triangleIndex * 3) + 0u;
        triangle.vertices[1] = (triangleIndex * 3) + 1u;
        triangle.vertices[2] = (triangleIndex * 3) + 2u;
    }


    // Create a unit list containing X triangle Units
    UnitList * unitList = UnitList::Allocate(m_allocator, unitCount, EA::Allocator::MEM_PERM);
    unitList->resize(unitCount);
    for (uint32_t unitIndex = 0 ; unitIndex < unitCount ; ++unitIndex)
    {
        Unit & unit = (*unitList)[unitIndex];
        unit.tri0 = unitIndex;
        unit.tri1 = 0u; // NOT REQUIRED FOR THIS TEST
        unit.type = Unit::TYPE_TRIANGLE;
        unit.extraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
        unit.edgeOpposingExtraVertex = 0u;  // NOT REQUIRED FOR THIS TEST
    }

    const uint32_t startUnitIndex = 2;
    const uint32_t unitAddCount = 1;

    const uint32_t unitsAdded = UnitClusterBuilder::AddUnitsToUnitCluster(
        unitCluster->vertexIDs,
        unitCluster->numVertices,
        unitCluster->unitIDs,
        unitCluster->numUnits,
        startUnitIndex,
        unitAddCount,
        *triangleList,
        *unitList,
        m_maxVerticesPerTriangle);

    // Check the unit has been added
    EATESTAssert(1 == unitsAdded, ("Should have been able to add unit to cluster"));

    // Check the state of the Cluster
    EATESTAssert(unitCluster->numUnits == 1, "UnitCluster should contain 1 unit");
    EATESTAssert(unitCluster->unitIDs[0] == 2, "UnitCluster should contain unitID 2");
    EATESTAssert(unitCluster->numVertices == 3, "UnitCluster should contain 3 vertices");
    EATESTAssert(unitCluster->vertexIDs[0] == 6, "UnitCluster should contain vertexID 6");
    EATESTAssert(unitCluster->vertexIDs[1] == 7, "UnitCluster should contain vertexID 7");
    EATESTAssert(unitCluster->vertexIDs[2] == 8, "UnitCluster should contain vertexID 8");

    // Release resources
    m_allocator->Free(unitList);
    m_allocator->Free(triangleList);
    unitClusterStack.Release();
}
