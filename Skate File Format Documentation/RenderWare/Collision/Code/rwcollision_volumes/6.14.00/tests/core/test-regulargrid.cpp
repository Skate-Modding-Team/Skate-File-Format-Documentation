// (c) Electronic Arts. All Rights Reserved.
#ifdef _MSC_VER
#pragma warning(disable: 4700)
#endif

#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <rw/math/math.h>
#include <rw/collision/libcore.h>
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

class TestRegularGrid : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestRegularGrid");
        EATEST_REGISTER("TestConstructorSingleCell", "TestConstructorSingleCell", TestRegularGrid, TestConstructorSingleCell);
        EATEST_REGISTER("TestInsertRemoveSingleCell", "TestInsertRemoveSingleCell", TestRegularGrid, TestInsertRemoveSingleCell);

        EATEST_REGISTER("TestConstructor2x3x4Cell", "TestConstructor2x3x4Cell", TestRegularGrid, TestConstructor2x3x4Cell);
        EATEST_REGISTER("TestInsertRemove2x3x4Cell", "TestInsertRemove2x3x4Cell", TestRegularGrid, TestInsertRemove2x3x4Cell);
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
    void TestConstructorSingleCell();
    void TestInsertRemoveSingleCell();

    void TestConstructor2x3x4Cell();
    void TestInsertRemove2x3x4Cell();

} TestSpatialMapsSingleton;


void TestRegularGrid::TestConstructorSingleCell()
{
    const uint32_t maxEntries(1);
    const uint32_t xCells(1);
    const uint32_t yCells(1);
    const uint32_t zCells(1);
    const rw::collision::AABBox extent(Vector3(0.0f , 0.0f, 0.0f), Vector3(10.0f, 10.0f, 10.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    // Check class internal are initialized correctly
    EATESTAssert(extent.Min()    == regularGrid->m_extent.Min(), "Min extent does not match" );
    EATESTAssert(extent.Max()    == regularGrid->m_extent.Max(), "Max extent does not match" );
    EATESTAssert(maxEntries      == regularGrid->m_maxEntries,   "Max entries does not match" );
    EATESTAssert((int32_t)xCells == regularGrid->m_xCells,       "X cells does not match" );
    EATESTAssert((int32_t)yCells == regularGrid->m_yCells,       "Y cells does not match" );
    EATESTAssert((int32_t)zCells == regularGrid->m_zCells,       "Z cells does not match" );

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGrid::TestInsertRemoveSingleCell()
{
    const uint32_t maxEntries(3);
    const uint32_t xCells(1);
    const uint32_t yCells(1);
    const uint32_t zCells(1);
    const rw::collision::AABBox extent(Vector3(-1.0f , -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    // Insert a box that is too big for the cell
    const float bigBoxHalfSize(10.0f);
    regularGrid->Insert(0, rw::collision::AABBox(Vector3(-1.0f * bigBoxHalfSize, -1.0f * bigBoxHalfSize, -1.0f * bigBoxHalfSize),
                                  Vector3( 1.0f * bigBoxHalfSize,  1.0f * bigBoxHalfSize,  1.0f * bigBoxHalfSize)));

    // Insert a box that is outside the extent
    regularGrid->Insert(1, rw::collision::AABBox(Vector3(2.0f, 2.0f, 2.0f),
                                  Vector3(3.0f, 3.0f, 3.0f)));

    // Insert a box at that is inside the extent
    const float boxHalfSize(0.1f);
    regularGrid->Insert(2, rw::collision::AABBox(Vector3(-1.0f * boxHalfSize, -1.0f * boxHalfSize, -1.0f * boxHalfSize),
                                  Vector3( 1.0f * boxHalfSize,  1.0f * boxHalfSize,  1.0f * boxHalfSize)));

    // Remove all entries
    for (uint32_t i(0); i < maxEntries; ++i)
    {
        regularGrid->Remove(i);
    }

    // Make sure that the cells have no entries in them
    EATESTAssert(true == regularGrid->m_cells[0].IsEmpty(), "");
    EATESTAssert(true == regularGrid->m_cells[1].IsEmpty(), "");

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGrid::TestConstructor2x3x4Cell()
{
    const uint32_t maxEntries(1);
    const uint32_t xCells(2);
    const uint32_t yCells(3);
    const uint32_t zCells(3);
    const rw::collision::AABBox extent(Vector3(0.0f , 0.0f, 0.0f), Vector3(2.0f, 3.0f, 4.0f));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    // Check class internal are initialized correctly
    EATESTAssert(extent.Min()    == regularGrid->m_extent.Min(), "Min extent does not match" );
    EATESTAssert(extent.Max()    == regularGrid->m_extent.Max(), "Max extent does not match" );
    EATESTAssert(maxEntries      == regularGrid->m_maxEntries,   "Max entries does not match" );
    EATESTAssert((int32_t)xCells == regularGrid->m_xCells,       "X cells does not match" );
    EATESTAssert((int32_t)yCells == regularGrid->m_yCells,       "Y cells does not match" );
    EATESTAssert((int32_t)zCells == regularGrid->m_zCells,       "Z cells does not match" );

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}


void TestRegularGrid::TestInsertRemove2x3x4Cell()
{
    const uint32_t xCells(2);
    const uint32_t yCells(3);
    const uint32_t zCells(4);
    const uint32_t numCells(xCells * yCells * zCells);
    const uint32_t numEntriesPerCell(9);
    const uint32_t maxEntries(numCells * numEntriesPerCell);
    const rw::collision::AABBox extent(Vector3(0.0f , 0.0f, 0.0f),
                        Vector3((Vector3::FloatType)xCells, (Vector3::FloatType)yCells, (Vector3::FloatType)zCells));

    // Create the spatial map
    RegularGrid * regularGrid = EA::Physics::UnitFramework::Creator<RegularGrid>().New(maxEntries, xCells, yCells, zCells, extent);

    uint32_t insertedBBoxCount(0);

    // Insert a box at the center of each cell
    {
        const Vector3 boxDiag(0.1f, 0.1f, 0.1f);

        for (uint32_t z(0); z < zCells; ++z)
        {
            for (uint32_t y(0); y < yCells; ++y)
            {
                for (uint32_t x(0); x < xCells; ++x)
                {
                    Vector3 cellCenter(x + 0.5f, y + 0.5f, z + 0.5f);
                    regularGrid->Insert(insertedBBoxCount++, rw::collision::AABBox(cellCenter - boxDiag, cellCenter + boxDiag));
                }
            }
        }
    }


    // Now place a cube just inside each of the cells corners
    {
        float boxHalfSize(0.2f);

        for (uint32_t z(0); z < zCells; ++z)
        {
            for (uint32_t y(0); y < yCells; ++y)
            {
                for (uint32_t x(0); x < xCells; ++x)
                {
                    const Vector3 shrinkVector(boxHalfSize + 0.001f, boxHalfSize + 0.001f, boxHalfSize + 0.001f);
                    Vector3 cellMin((Vector3::FloatType)x, (Vector3::FloatType)y, (Vector3::FloatType)z);
                    cellMin += shrinkVector;
                    Vector3 cellMax(x + 1.0f, y  + 1.0f, z  + 1.0f);
                    cellMax -= shrinkVector;

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMin.GetX() - boxHalfSize, cellMin.GetY() - boxHalfSize, cellMin.GetZ() - boxHalfSize),
                                               Vector3(cellMin.GetX() + boxHalfSize, cellMin.GetY() + boxHalfSize, cellMin.GetZ() + boxHalfSize)));

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMax.GetX(), cellMin.GetY(), cellMin.GetZ()) - boxHalfSize,
                                               Vector3(cellMax.GetX(), cellMin.GetY(), cellMin.GetZ()) + boxHalfSize));

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMax.GetX() - boxHalfSize, cellMax.GetY() - boxHalfSize, cellMin.GetZ() - boxHalfSize),
                                               Vector3(cellMax.GetX() + boxHalfSize, cellMax.GetY() + boxHalfSize, cellMin.GetZ() + boxHalfSize)));

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMin.GetX() - boxHalfSize, cellMax.GetY() - boxHalfSize, cellMin.GetZ() - boxHalfSize),
                                               Vector3(cellMin.GetX() + boxHalfSize, cellMax.GetY() + boxHalfSize, cellMin.GetZ() + boxHalfSize)));

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMin.GetX() - boxHalfSize, cellMin.GetY() - boxHalfSize, cellMax.GetZ() - boxHalfSize),
                                               Vector3(cellMin.GetX() + boxHalfSize, cellMin.GetY() + boxHalfSize, cellMax.GetZ() + boxHalfSize)));

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMax.GetX() - boxHalfSize, cellMin.GetY() - boxHalfSize, cellMax.GetZ() - boxHalfSize),
                                               Vector3(cellMax.GetX() + boxHalfSize, cellMin.GetY() + boxHalfSize, cellMax.GetZ() + boxHalfSize)));

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMax.GetX() - boxHalfSize, cellMax.GetY() - boxHalfSize, cellMax.GetZ() - boxHalfSize),
                                               Vector3(cellMax.GetX() + boxHalfSize, cellMax.GetY() + boxHalfSize, cellMax.GetZ() + boxHalfSize)));

                    regularGrid->Insert(insertedBBoxCount++,
                                        rw::collision::AABBox(Vector3(cellMin.GetX() - boxHalfSize, cellMax.GetY() - boxHalfSize, cellMax.GetZ() - boxHalfSize),
                                               Vector3(cellMin.GetX() + boxHalfSize, cellMax.GetY() + boxHalfSize, cellMax.GetZ() + boxHalfSize)));
                }
            }
        }
    }

    // Remove all entries
    for (uint32_t i(0); i < insertedBBoxCount; ++i)
    {
        regularGrid->Remove(i);
    }

    // Make sure that all cell are now empty
    EATESTAssert(true == regularGrid->m_cells[0].IsEmpty(), "");

    for (uint32_t i(0); i < numCells; ++i)
    {
        EATESTAssert(true == regularGrid->m_cells[i + 1].IsEmpty(), "");
    }

    // Free the regular grid
    regularGrid->Release();
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(regularGrid);
}

