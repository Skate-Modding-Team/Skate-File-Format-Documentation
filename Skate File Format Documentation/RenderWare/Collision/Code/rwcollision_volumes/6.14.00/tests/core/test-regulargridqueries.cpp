// (c) Electronic Arts. All Rights Reserved.
#ifdef _MSC_VER
#pragma warning(disable: 4700)
#endif

#include <new>

#include <EABase/eabase.h>
#include <rw/math/math.h>
#include <eaphysics/base.h>
#include <rw/collision/aabbox.h>
#include <rw/collision/regulargrid.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

#include <unit/unit.h>

using namespace rwpmath;
using namespace rw::collision;


// ***********************************************************************************************************
// Test suite

class TestRegularGridQueries : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestRegularGridQueries");

        // If all of the tests are enabled PS2 will fail the build with Error: Branch out of range
        // This is caused by the size of the test being too large since all of the RegularGrid functions
        // are inlined. Instead we only build a couple of the query tests for PS2.

        EATEST_REGISTER("TestSingleCellBBoxQuery", "TestSingleCellBBoxQuery", TestRegularGridQueries, TestSingleCellBBoxQuery);
        EATEST_REGISTER("Test2x3x4CellBBoxQuery", "Test2x3x4CellBBoxQuery", TestRegularGridQueries, Test2x3x4CellBBoxQuery);

        EATEST_REGISTER("TestSingleCellLineQuery", "TestSingleCellLineQuery", TestRegularGridQueries, TestSingleCellLineQuery);
        EATEST_REGISTER("TestSingleCellSeveralBoxesLineQuery", "TestSingleCellSeveralBoxesLineQuery", TestRegularGridQueries, TestSingleCellSeveralBoxesLineQuery);

        EATEST_REGISTER("TestSingleCellFatLineQuery", "TestSingleCellFatLineQuery", TestRegularGridQueries, TestSingleCellFatLineQuery);
        EATEST_REGISTER("Test2x3x4FatLineQuery", "Test2x3x4CellFatLineQuery", TestRegularGridQueries, Test2x3x4CellFatLineQuery);

        EATEST_REGISTER("TestSingleCellSeveralBoxesFatLineQuery", "TestSingleCellSeveralBoxesFatLineQuery", TestRegularGridQueries, TestSingleCellSeveralBoxesFatLineQuery);
        EATEST_REGISTER("Test2x3x4CellSeveralBoxesFatLineQuery", "Test2x3x4CellSeveralBoxesFatLineQuery", TestRegularGridQueries, Test2x3x4CellSeveralBoxesFatLineQuery);

        EATEST_REGISTER("TestSingleCellSeveralBoxesFatLineQueryNonAxisAligned", "TestSingleCellSeveralBoxesFatLineQueryNonAxisAligned", TestRegularGridQueries, TestSingleCellSeveralBoxesFatLineQueryNonAxisAligned);
        EATEST_REGISTER("Test2x2x1CellSeveralBoxesFatLineQueryNonAxisAligned", "Test2x2x1CellSeveralBoxesFatLineQueryNonAxisAligned", TestRegularGridQueries, Test2x2x1CellSeveralBoxesFatLineQueryNonAxisAligned);
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
    }

    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        tests::TestSuiteBase::TeardownSuite();
    }

private:
    void TestSingleCellBBoxQuery();
    void Test2x3x4CellBBoxQuery();
    void TestBBoxQuery(uint32_t x,uint32_t y,uint32_t z);

    void TestSingleCellLineQuery();
    void TestSingleCellSeveralBoxesLineQuery();

    void TestSingleCellFatLineQuery();
    void Test2x3x4CellFatLineQuery();
    void TestFatLineQuery(uint32_t x,uint32_t y,uint32_t z);

    void TestSingleCellSeveralBoxesFatLineQuery();
    void Test2x3x4CellSeveralBoxesFatLineQuery();
    void TestSeveralBoxesFatLineQuery(uint32_t x, uint32_t y, uint32_t z);

    void TestSingleCellSeveralBoxesFatLineQueryNonAxisAligned();
    void Test2x2x1CellSeveralBoxesFatLineQueryNonAxisAligned();
    void TestSeveralBoxesFatLineQueryNonAxisAligned(uint32_t x, uint32_t y, uint32_t z);

} TestRegularGridQueriesSingleton;



void TestRegularGridQueries::TestSingleCellBBoxQuery()
{
    TestBBoxQuery(1,1,1);
}

void TestRegularGridQueries::Test2x3x4CellBBoxQuery()
{
    TestBBoxQuery(2,3,4);
}

void TestRegularGridQueries::TestBBoxQuery(uint32_t x, uint32_t y, uint32_t z)
{
    const uint32_t maxEntries(2);
    const uint32_t xCells(x);
    const uint32_t yCells(y);
    const uint32_t zCells(z);
    const rw::collision::AABBox extent(Vector3(-1.0f , -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    // These test are testing code paths to make sure they do not crash.
    // TODO: Add checking of internal data structure?
    {
        // Test a box who's min is outside the grids extent and does not overlap.
        {
            const rw::collision::AABBox testBBox(Vector3(3.0f , 3.0f, 3.0f), Vector3(4.0f, 4.0f, 4.0f));
            RegularGrid::BBoxQuery bboxQuery(regularGrid, testBBox);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
        }

        // Test a box who's min plus half cell padding touches the max extent.
        {
            const rw::collision::AABBox testBBox(Vector3(2.0f , 2.0f, 2.0f), Vector3(3.0f, 3.0f, 3.0f));
            RegularGrid::BBoxQuery bboxQuery(regularGrid, testBBox);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
        }

        // Test a box who's min is outside the grid extent but overlaps due to half cell padding.
        {
            const rw::collision::AABBox testBBox(Vector3(1.5f , 1.5f, 1.5f), Vector3(2.5f, 2.5f, 2.5f));
            RegularGrid::BBoxQuery bboxQuery(regularGrid, testBBox);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
        }

        // Test a box who's min is inside the grid extent and max outside.
        {
            const rw::collision::AABBox testBBox(Vector3(0.5f , 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f));
            RegularGrid::BBoxQuery bboxQuery(regularGrid, testBBox);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
        }

        // Test a box that fits inside the grid extent.
        {
            const rw::collision::AABBox testBBox(Vector3(-0.2f , -0.2f, -0.2f), Vector3(0.2f, 0.2f, 0.2f));
            RegularGrid::BBoxQuery bboxQuery(regularGrid, testBBox);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
        }

        // Test a box that is the same size as the grid extent.
        {
            const rw::collision::AABBox testBBox(Vector3(-1.0f , -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
            RegularGrid::BBoxQuery bboxQuery(regularGrid, testBBox);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
        }

        // Test a box that is bigger and contains the grid extent.
        {
            const rw::collision::AABBox testBBox(Vector3(-2.0f , -2.0f, -2.0f), Vector3(2.0f, 2.0f, 2.0f));
            RegularGrid::BBoxQuery bboxQuery(regularGrid, testBBox);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
        }
    }

    {
        {
            // A box overlapping grid extent by almost a full cell size.
            const rw::collision::AABBox boxA(Vector3(0.8f, 0.8f, -0.5f), Vector3(2.79f, 2.79f, 0.5f));
            regularGrid->Insert(0, boxA);

            // Intersection with box.
            {
                const rw::collision::AABBox testBox(Vector3(2.7f, 2.7f, -0.5f), Vector3(3.0f, 3.0f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            regularGrid->Remove(0);
        }


        // Test query box plus overlap is outside of grid extent
        {
            // A box overlapping grid extent.
            const rw::collision::AABBox boxA(Vector3(0.5f, -0.5f, -0.5f), Vector3(1.5f, 0.5f, 0.5f));
            regularGrid->Insert(0, boxA);

            // A box outside of grid extent.
            const rw::collision::AABBox boxB(Vector3(4.0f, -0.5f, -0.5f), Vector3(5.0f, 0.5f, 0.5f));
            regularGrid->Insert(1, boxB);

            // No intersection with either boxes.
            {
                const rw::collision::AABBox testBox(Vector3(2.5f, -0.5f, -0.5f), Vector3(3.5f, 0.5f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box outside of grid extent only.
            {
                const rw::collision::AABBox testBox(Vector3(3.5f, -0.5f, -0.5f), Vector3(4.5f, 0.5f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            regularGrid->Remove(1);
            regularGrid->Remove(0);
        }


        // Test query box plus overlap, overlaps grid extent.
        {
            // A box overlapping grid extent.
            const rw::collision::AABBox boxA(Vector3(0.5f, -0.5f, -0.5f), Vector3(1.5f, 0.5f, 0.5f));
            regularGrid->Insert(0, boxA);

            // A box outside of grid extent.
            const rw::collision::AABBox boxB(Vector3(3.0f, -0.5f, -0.5f), Vector3(4.0f, 0.5f, 0.5f));
            regularGrid->Insert(1, boxB);

            // No intersection with either boxes.
            {
                const rw::collision::AABBox testBox(Vector3(1.2f, 1.0f, -0.5f), Vector3(2.2f, 2.0f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box A
            {
                const rw::collision::AABBox testBox(Vector3(1.2f, 0.2f, -0.5f), Vector3(2.2f, 1.2f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box B
            {
                const rw::collision::AABBox testBox(Vector3(2.2f, 0.2f, -0.5f), Vector3(3.2f, 1.2f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box A & B
            {
                const rw::collision::AABBox testBox(Vector3(1.2f, 0.2f, -0.5f), Vector3(3.2f, 1.2f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            regularGrid->Remove(1);
            regularGrid->Remove(0);
        }


        // Test query box that overlaps grid extent.
        {
            // A box inside grid extent.
            const rw::collision::AABBox boxA(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
            regularGrid->Insert(0, boxA);

            // A box outside of grid extent.
            const rw::collision::AABBox boxB(Vector3(1.5f, -0.5f, -0.5f), Vector3(2.5f, 0.5f, 0.5f));
            regularGrid->Insert(1, boxB);

            // No intersection with either boxes.
            {
                const rw::collision::AABBox testBox(Vector3(0.8f, -0.5f, -0.5f), Vector3(1.2f, 0.5f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box A
            {
                const rw::collision::AABBox testBox(Vector3(0.4f, -0.5f, -0.5f), Vector3(1.2f, 0.5f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box B
            {
                const rw::collision::AABBox testBox(Vector3(0.8f, -0.5f, -0.5f), Vector3(1.6f, 0.5f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box A & B
            {
                const rw::collision::AABBox testBox(Vector3(0.4f, -0.5f, -0.5f), Vector3(1.6f, 0.5f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            regularGrid->Remove(1);
            regularGrid->Remove(0);
        }


        // Test query box that is inside the grid extent.
        {
            // A box inside grid extent.
            const rw::collision::AABBox boxA(Vector3(-0.5f, -0.0f, -0.5f), Vector3(0.0f, 0.5f, 0.5f));
            regularGrid->Insert(0, boxA);

            // A box outside of grid extent.
            const rw::collision::AABBox boxB(Vector3(-1.5f, -1.5f, -0.5f), Vector3(-0.5f, -0.5f, 0.5f));
            regularGrid->Insert(1, boxB);

            // No intersection with either boxes.
            {
                const rw::collision::AABBox testBox(Vector3(0.2f, -0.2f, -0.5f), Vector3(0.8f, 0.2f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box A
            {
                const rw::collision::AABBox testBox(Vector3(-0.2f, -0.2f, -0.5f), Vector3(0.8f, 0.2f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box B
            {
                const rw::collision::AABBox testBox(Vector3(-0.6f, -0.8f, -0.5f), Vector3(0.0f, -0.2f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            // Intersection with box A & B
            {
                const rw::collision::AABBox testBox(Vector3(-0.6f, -0.8f, -0.5f), Vector3(0.2f, 0.2f, 0.5f));
                RegularGrid::BBoxQuery bboxQuery(regularGrid, testBox);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == bboxQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == bboxQuery.GetNext(intersectedBBoxEntry), "");
            }

            regularGrid->Remove(1);
            regularGrid->Remove(0);
        }
    }

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGridQueries::TestSingleCellLineQuery()
{
    const uint32_t maxEntries(2);
    const uint32_t xCells(1);
    const uint32_t yCells(1);
    const uint32_t zCells(1);
    const rw::collision::AABBox extent(Vector3(-1.0f , -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    // These test are testing code paths to make sure they do not crash.
    {
        // Test a empty cell.
        {
            const Vector3 start(-2.0f , 0.0f, 0.0f);
            const Vector3 end(2.0f, 0.0f, 0.0f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }
    }

    {
        // The box is inside the grid extent.
        const rw::collision::AABBox boxA(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
        regularGrid->Insert(0, boxA);

        // + X Axis
        {
            const Vector3 start(-2.0f, 0.0f, 0.0f);
            const Vector3 end(2.0f, 0.0f, 0.0f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // - X Axis
        {
            const Vector3 start(2.0f, 0.0f, 0.0f);
            const Vector3 end(-2.0f, 0.0f, 0.0f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // + Y Axis
        {
            const Vector3 start(0.0f, -2.0f, 0.0f);
            const Vector3 end(0.0f, 2.0f, 0.0f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // - Y Axis
        {
            const Vector3 start(0.0f, 2.0f, 0.0f);
            const Vector3 end(0.0f, -2.0f, 0.0f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // + Z Axis
        {
            const Vector3 start(0.0f, 0.0f, -2.0f);
            const Vector3 end(0.0f, 0.0f, 2.0f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // - Z Axis
        {
            const Vector3 start(0.0f, 0.0f, 2.0f);
            const Vector3 end(0.0f, 0.0f, -2.0f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        regularGrid->Remove(0);
    }

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGridQueries::TestSingleCellSeveralBoxesLineQuery()
{
    const uint32_t maxEntries(3);
    const uint32_t xCells(1);
    const uint32_t yCells(1);
    const uint32_t zCells(1);
    const rw::collision::AABBox extent(Vector3(0.0f , 0.0f, 0.0f), Vector3(7.0f, 3.0f, 3.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    {
        // The boxes are inside the grid extent.
        const rw::collision::AABBox boxA(Vector3(1.0f, 1.0f, 1.0f), Vector3(2.0f, 2.0f, 2.0f));
        const rw::collision::AABBox boxB(Vector3(3.2f, 1.2f, 1.0f), Vector3(3.8f, 1.8f, 2.0f));
        const rw::collision::AABBox boxC(Vector3(5.0f, 1.0f, 1.0f), Vector3(6.0f, 2.0f, 2.0f));

        regularGrid->Insert(0, boxA);
        regularGrid->Insert(1, boxB);
        regularGrid->Insert(2, boxC);

        // + X Axis line
        {
            // A only intersection
            {
                const Vector3 start(0.5f, 1.5f, 1.5f);
                const Vector3 end(2.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B only intersection
            {
                const Vector3 start(2.5f, 1.5f, 1.5f);
                const Vector3 end(4.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // C only intersection
            {
                const Vector3 start(4.5f, 1.5f, 1.5f);
                const Vector3 end(6.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & B intersection
            {
                const Vector3 start(0.5f, 1.5f, 1.5f);
                const Vector3 end(4.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & C intersection
            {
                const Vector3 start(0.5f, 1.9f, 1.5f);
                const Vector3 end(6.5f, 1.9f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B & C intersection
            {
                const Vector3 start(2.5f, 1.5f, 1.5f);
                const Vector3 end(6.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A, B & C intersection
            {
                const Vector3 start(0.5f, 1.5f, 1.5f);
                const Vector3 end(6.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }
        }

        // - X Axis line
        {
            // A only intersection
            {
                const Vector3 start(2.5f, 1.5f, 1.5f);
                const Vector3 end(0.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B only intersection
            {
                const Vector3 start(4.5f, 1.5f, 1.5f);
                const Vector3 end(2.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // C only intersection
            {
                const Vector3 start(6.5f, 1.5f, 1.5f);
                const Vector3 end(4.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & B intersection
            {
                const Vector3 start(4.5f, 1.5f, 1.5f);
                const Vector3 end(0.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & C intersection
            {
                const Vector3 start(6.5f, 1.9f, 1.5f);
                const Vector3 end(0.5f, 1.9f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B & C intersection
            {
                const Vector3 start(6.5f, 1.5f, 1.5f);
                const Vector3 end(2.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A, B & C intersection
            {
                const Vector3 start(6.5f, 1.5f, 1.5f);
                const Vector3 end(0.5f, 1.5f, 1.5f);

                RegularGrid::LineQuery lineQuery(regularGrid, start, end);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }
        }

        regularGrid->Remove(2);
        regularGrid->Remove(1);
        regularGrid->Remove(0);
    }

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGridQueries::TestSingleCellFatLineQuery()
{
    TestFatLineQuery(1,1,1);
}


void TestRegularGridQueries::Test2x3x4CellFatLineQuery()
{
    TestFatLineQuery(2,3,4);
}


void TestRegularGridQueries::TestFatLineQuery(uint32_t x, uint32_t y, uint32_t z)
{
    const uint32_t maxEntries(2);
    const uint32_t xCells(x);
    const uint32_t yCells(y);
    const uint32_t zCells(z);
    const rw::collision::AABBox extent(Vector3(-1.0f , -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    // These test are testing code paths to make sure they do not crash.
    {
        // Test a empty cell.
        {
            const Vector3 start(-2.0f , 0.0f, 0.0f);
            const Vector3 end(2.0f, 0.0f, 0.0f);
            const float fatness(0.2f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }
    }

    {
        // The box is inside the grid extent.
        const rw::collision::AABBox boxA(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
        regularGrid->Insert(0, boxA);

        // + X Axis
        {
            const Vector3 start(-2.0f, 0.0f, 0.0f);
            const Vector3 end(2.0f, 0.0f, 0.0f);
            const float fatness(0.2f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // - X Axis
        {
            const Vector3 start(2.0f, 0.0f, 0.0f);
            const Vector3 end(-2.0f, 0.0f, 0.0f);
            const float fatness(0.2f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // + Y Axis
        {
            const Vector3 start(0.0f, -2.0f, 0.0f);
            const Vector3 end(0.0f, 2.0f, 0.0f);
            const float fatness(0.2f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // - Y Axis
        {
            const Vector3 start(0.0f, 2.0f, 0.0f);
            const Vector3 end(0.0f, -2.0f, 0.0f);
            const float fatness(0.2f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // + Z Axis
        {
            const Vector3 start(0.0f, 0.0f, -2.0f);
            const Vector3 end(0.0f, 0.0f, 2.0f);
            const float fatness(0.2f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        // - Z Axis
        {
            const Vector3 start(0.0f, 0.0f, 2.0f);
            const Vector3 end(0.0f, 0.0f, -2.0f);
            const float fatness(0.2f);
            RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

            uint32_t intersectedBBoxEntry = ~0u;
            EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
            EATESTAssert(0 == intersectedBBoxEntry, "");
            EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
        }

        regularGrid->Remove(0);
    }

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGridQueries::TestSingleCellSeveralBoxesFatLineQuery()
{
    TestRegularGridQueries::TestSeveralBoxesFatLineQuery(1,1,1);
}


void TestRegularGridQueries::Test2x3x4CellSeveralBoxesFatLineQuery()
{
    TestRegularGridQueries::TestSeveralBoxesFatLineQuery(2,3,4);
}


void TestRegularGridQueries::TestSeveralBoxesFatLineQuery(uint32_t x, uint32_t y, uint32_t z)
{
    const uint32_t maxEntries(3);
    const uint32_t xCells(x);
    const uint32_t yCells(y);
    const uint32_t zCells(z);
    const rw::collision::AABBox extent(Vector3(0.0f , 0.0f, 0.0f), Vector3(7.0f, 3.0f, 3.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    {
        // The boxes are inside the grid extent.
        const rw::collision::AABBox boxA(Vector3(1.0f, 1.0f, 1.0f), Vector3(2.0f, 2.0f, 2.0f));
        const rw::collision::AABBox boxB(Vector3(3.2f, 1.2f, 1.0f), Vector3(3.8f, 1.8f, 2.0f));
        const rw::collision::AABBox boxC(Vector3(5.0f, 1.0f, 1.0f), Vector3(6.0f, 2.0f, 2.0f));

        regularGrid->Insert(0, boxA);
        regularGrid->Insert(1, boxB);
        regularGrid->Insert(2, boxC);

        // + X Axis line
        {
            // A only intersection
            {
                const Vector3 start(0.5f, 2.1f, 2.1f);
                const Vector3 end(2.5f, 2.1f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B only intersection
            {
                const Vector3 start(2.5f, 1.9f, 0.9f);
                const Vector3 end(4.5f, 1.9f, 0.9f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // C only intersection
            {
                const Vector3 start(4.5f, 0.9f, 2.1f);
                const Vector3 end(6.5f, 0.9f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & B intersection
            {
                const Vector3 start(0.5f, 1.1f, 0.9f);
                const Vector3 end(4.5f, 1.1f, 0.9f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & C intersection
            {
                const Vector3 start(0.5f, 0.9f, 2.1f);
                const Vector3 end(6.5f, 0.9f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B & C intersection
            {
                const Vector3 start(2.5f, 1.9f, 2.1f);
                const Vector3 end(6.5f, 1.9f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A, B & C intersection
            {
                const Vector3 start(0.5f, 1.9f, 0.9f);
                const Vector3 end(6.5f, 1.9f, 0.9f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }
        }

        // - X Axis line
        {
            // A only intersection
            {
                const Vector3 start(2.5f, 2.1f, 2.1f);
                const Vector3 end(0.5f, 2.1f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B only intersection
            {
                const Vector3 start(4.5f, 1.9f, 0.9f);
                const Vector3 end(2.5f, 1.9f, 0.9f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // C only intersection
            {
                const Vector3 start(6.5f, 0.9f, 2.1f);
                const Vector3 end(4.5f, 0.9f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & B intersection
            {
                const Vector3 start(4.5f, 1.1f, 0.9f);
                const Vector3 end(0.5f, 1.1f, 0.9f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & C intersection
            {
                const Vector3 start(6.5f, 0.9f, 2.1f);
                const Vector3 end(0.5f, 0.9f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B & C intersection
            {
                const Vector3 start(6.5f, 1.9f, 2.1f);
                const Vector3 end(2.5f, 1.9f, 2.1f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A, B & C intersection
            {
                const Vector3 start(6.5f, 1.9f, 0.9f);
                const Vector3 end(0.5f, 1.9f, 0.9f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }
        }

        regularGrid->Remove(2);
        regularGrid->Remove(1);
        regularGrid->Remove(0);
    }

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGridQueries::TestSingleCellSeveralBoxesFatLineQueryNonAxisAligned()
{
    TestSeveralBoxesFatLineQueryNonAxisAligned(1,1,1);
}


void TestRegularGridQueries::Test2x2x1CellSeveralBoxesFatLineQueryNonAxisAligned()
{
    TestSeveralBoxesFatLineQueryNonAxisAligned(2, 2, 1);
}


void TestRegularGridQueries::TestSeveralBoxesFatLineQueryNonAxisAligned(uint32_t x, uint32_t y, uint32_t z)
{
    const uint32_t maxEntries(4);
    const uint32_t xCells(x);
    const uint32_t yCells(y);
    const uint32_t zCells(z);
    const rw::collision::AABBox extent(Vector3(0.0f , 0.0f, 0.0f), Vector3(6.0f, 6.0f, 3.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    {
        // The boxes are inside the grid extent.
        const rw::collision::AABBox boxA(Vector3(1.0f, 1.0f, 1.0f), Vector3(2.0f, 2.0f, 2.0f));
        const rw::collision::AABBox boxB(Vector3(4.0f, 1.0f, 1.0f), Vector3(4.0f, 2.0f, 2.0f));
        const rw::collision::AABBox boxC(Vector3(1.0f, 4.0f, 1.0f), Vector3(2.0f, 5.0f, 2.0f));
        const rw::collision::AABBox boxD(Vector3(4.0f, 4.0f, 1.0f), Vector3(5.0f, 5.0f, 2.0f));

        regularGrid->Insert(0, boxA);
        regularGrid->Insert(1, boxB);
        regularGrid->Insert(2, boxC);
        regularGrid->Insert(3, boxD);

        // + X Axis line
        {
            // A only intersection
            {
                const Vector3 start(0.0f, 0.0f, 0.0f);
                const Vector3 end(3.0f, 3.0f, 3.0f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // B only intersection
            {
                const Vector3 start(3.0f, 3.0f, 3.0f);
                const Vector3 end(6.0f, 0.0f, 0.0f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // C only intersection
            {
                const Vector3 start(0.0f, 6.0f, 0.0f);
                const Vector3 end(3.0f, 3.0f, 3.0f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // D only intersection
            {
                const Vector3 start(3.0f, 3.0f, 3.0f);
                const Vector3 end(6.0f, 6.0f, 0.0f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(3 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // A & D intersection
            {
                const Vector3 start(0.0f, 0.0f, 1.5f);
                const Vector3 end(6.0f, 6.0f, 1.5f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(3 == intersectedBBoxEntry ||0 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(0 == intersectedBBoxEntry ||3 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

            // C & B intersection
            {
                const Vector3 start(0.0f, 6.0f, 1.5f);
                const Vector3 end(6.0f, 0.0f, 1.5f);
                const float fatness(0.2f);
                RegularGrid::LineQuery lineQuery(regularGrid, start, end, fatness);

                uint32_t intersectedBBoxEntry = ~0u;
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(2 == intersectedBBoxEntry || 1 == intersectedBBoxEntry, "");
                EATESTAssert(true == lineQuery.GetNext(intersectedBBoxEntry), "");
                EATESTAssert(1 == intersectedBBoxEntry || 2 == intersectedBBoxEntry, "");
                EATESTAssert(false == lineQuery.GetNext(intersectedBBoxEntry), "");
            }

        }

        regularGrid->Remove(3);
        regularGrid->Remove(2);
        regularGrid->Remove(1);
        regularGrid->Remove(0);
    }

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}

