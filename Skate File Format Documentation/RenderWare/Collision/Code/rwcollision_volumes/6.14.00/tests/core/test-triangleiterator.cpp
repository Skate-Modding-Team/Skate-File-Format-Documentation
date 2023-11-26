// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

File: test-triangleiterator.cpp

Purpose: Test iterator for accessing triangles from clustered mesh clusters.

*/

// Intentionally include code under test first to check standalone header.
#include <rw/collision/clustertriangleiterator.h>

#include <EABase/eabase.h>
#include <unit/unit.h>

#include "testsuitebase.h" // For TestSuiteBase

#include "mock-unit.h"

class TestTriangleIterator : public rw::collision::tests::TestSuiteBase
{
private:

    typedef rw::collision::ClusterTriangleIterator<MockUnit2> TestIterator;
    typedef rw::collision::ClusterTriangleIterator<MockUnitQuad> TestIteratorQuad;

public:     // TestSuite overrides

#define TRIANGLE_ITERATOR_TEST(M, D) EATEST_REGISTER(#M, D, TestTriangleIterator, M)

    virtual void Initialize()
    {
        SuiteName("TestTriangleIterator");
        TRIANGLE_ITERATOR_TEST(TestConstructor, "Check basic constructor");
        TRIANGLE_ITERATOR_TEST(TestConstructorWithOffset, "Check constructor with non-default offset");
        TRIANGLE_ITERATOR_TEST(TestReset, "Check resetting");
        TRIANGLE_ITERATOR_TEST(TestIsValidFromUnit, "Check behavior of IsValid()");
        TRIANGLE_ITERATOR_TEST(TestIsValidAtEnd, "Check behavior of IsValid() when at end");
        TRIANGLE_ITERATOR_TEST(TestIsValidOffset, "Check behavior of IsValid() when overflowing cluster");
        TRIANGLE_ITERATOR_TEST(TestNextOneTriangle, "Check moving to next with single triangle");
        TRIANGLE_ITERATOR_TEST(TestNextOneQuad, "Check moving to next with single quad");
        TRIANGLE_ITERATOR_TEST(TestNextMixed, "Check moving to next with mixed triangles and quads");
        TRIANGLE_ITERATOR_TEST(TestGetTriVertices, "Check getting triangle vertices");
        TRIANGLE_ITERATOR_TEST(TestGetQuadVertices, "Check getting quad vertices");
        TRIANGLE_ITERATOR_TEST(TestGetTriEdgeCosines, "Check getting triangle edge cosines");
        TRIANGLE_ITERATOR_TEST(TestGetQuadEdgeCosines, "Check getting quad edge cosines");
        TRIANGLE_ITERATOR_TEST(TestGetID, "Check getting ID from current unit");
        TRIANGLE_ITERATOR_TEST(TestGetGroupID, "Check getting groupID from current unit");
        TRIANGLE_ITERATOR_TEST(TestGetSurfaceID, "Check getting surfaceID from current unit");
        TRIANGLE_ITERATOR_TEST(TestGetOffset, "Check getting offset for current unit");
        TRIANGLE_ITERATOR_TEST(TestGetNumTrianglesLeftInCurrentUnit, "Check getting the number of triangle left to iterate in the current unit");
        TRIANGLE_ITERATOR_TEST(TestGetTriVertexIndices, "Check getting triangle vertex indices");
        TRIANGLE_ITERATOR_TEST(TestGetQuadVertexIndices, "Check getting quad vertex indices");
        EATEST_REGISTER_SPU("TestTriangleIteratorSPU", "SPU triangle iterator tests", "test-triangleiterator.elf");
    }

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();

        const uint8_t numVertices = 20;
        const uint8_t numTriangles = 2;
        const uint8_t numQuads = 2;

        rw::collision::ClusterConstructionParameters params;
        params.mVertexCount = numVertices;
        params.mVertexCompressionMode = 0;
        params.mTriangleUnitCount = numTriangles;
        params.mQuadUnitCount = numQuads;
        params.mEdgeCosineCount = 0;
        params.mGroupIDCount = 0;
        params.mGroupIDSize = 0;
        params.mSurfaceIDCount = 0;
        params.mSurfaceIDSize = 0;

        EA_ASSERT(rw::collision::ClusteredMeshCluster::GetSize(params) < CLUSTER_MEMORY_SIZE);

        void * memory = mClusterData;

        mCluster = rw::collision::ClusteredMeshCluster::Initialize(
            memory,
            params);

        rw::collision::UnitParameters unitParams;
        unitParams.unitFlagsDefault = 0u;
        unitParams.groupIDSize = 0u;
        unitParams.surfaceIDSize = 0u;

        // Setup first triangle
        mCluster->SetTriangle(
            unitParams,
            0u,
            0u,
            0u,
            1u,
            2u,
            0u,
            0u,
            0u);

        // Setup second triangle
        mCluster->SetTriangle(
            unitParams,
            0u,
            0u,
            7u,
            8u,
            9u,
            0u,
            0u,
            0u);

        // Setup first quad
        mCluster->SetQuad(
            unitParams,
            0u,
            0u,
            0u,
            1u,
            2u,
            3u,
            0u,
            0u,
            0u,
            0u);

        // Setup second quad
        mCluster->SetQuad(
            unitParams,
            0u,
            0u,
            9u,
            8u,
            7u,
            6u,
            0u,
            0u,
            0u,
            0u);

        for (uint8_t v = 0; v < numVertices; ++v)
        {
            float x = float(v);
            mCluster->SetVertex(rwpmath::Vector3(x, 1.0f + x, -x), 0);
        }
    }

    virtual void TeardownSuite()
    {
        mCluster = 0;
        rw::collision::tests::TestSuiteBase::TeardownSuite();
    }

private:

    void TestConstructor()
    {
        TestIterator i(*mCluster, mClusterParams);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(u.mCluster == mCluster, "Should have passed cluster to unit");
        EATESTAssert(u.mClusterParams == &mClusterParams, "Should have passed cluster params to unit");
        EATESTAssert(u.mOffset == 0, "Should have set offset to 0 by default");
        EATESTAssert(u.mData == mCluster->UnitData(), "Should have initialized unit");
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
    }

    void TestConstructorWithOffset()
    {
        // Test unit offset and non zero unit count
        {
            TestIterator i(*mCluster, mClusterParams, 46, 3);
            const TestIterator::UnitType & u = i.GetUnit();
            EATESTAssert(u.mCluster == mCluster, "Should have passed cluster to unit");
            EATESTAssert(u.mClusterParams == &mClusterParams, "Should have passed cluster params to unit");
            EATESTAssert(u.mOffset == 46, "Should have passed offset to unit");
            EATESTAssert(u.mData == mCluster->UnitData() + 46, "Should have initialized unit");
            EATESTAssert(!i.AtEnd(), "Should not be at end initially");
            // Don't test initialization of number of units in this test.
        }

        // Test unit offset and zero unit count
        {
            TestIterator i(*mCluster, mClusterParams, 46, 0);
            const TestIterator::UnitType & u = i.GetUnit();
            EATESTAssert(u.mCluster == mCluster, "Should have passed cluster to unit");
            EATESTAssert(u.mClusterParams == &mClusterParams, "Should have passed cluster params to unit");
            EATESTAssert(u.mOffset == 46, "Should have passed offset to unit");
            EATESTAssert(u.mData == mCluster->UnitData() + 46, "Should have initialized unit");
            EATESTAssert(i.AtEnd(), "Should be at end");
            // Don't test initialization of number of units in this test.
        }
    }

    void TestReset()
    {
        TestIterator i(*mCluster, mClusterParams, 46, 3);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(u.mOffset == 46, "Should have passed offset to unit");
        EATESTAssert(u.mData == mCluster->UnitData() + 46, "Should have initialized unit");
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        
        // Reset somewhere else and we should only be able to iterate over 1 unit
        i.Reset(12, 1);
        EATESTAssert(u.mCluster == mCluster, "Should still have passed cluster to unit");
        EATESTAssert(u.mClusterParams == &mClusterParams, "Should still have passed cluster params to unit");
        EATESTAssert(u.mData == mCluster->UnitData() + 12, "Should have reset unit");
        EATESTAssert(!i.AtEnd(), "Should not be at end after Reset()");
        i.Next();
        EATESTAssert(i.AtEnd(), "Should now be at end");

        // Check we can iterate over 3 units again
        i.Reset(46, 3);
        EATESTAssert(!i.AtEnd(), "Should not be at end again after Reset()");
        i.Next();
        i.Next();
        i.Next();
        EATESTAssert(i.AtEnd(), "Should be at end after 3 Nexts");

        // Check we can reset with no units
        i.Reset(46, 0);
        EATESTAssert(i.AtEnd(), "Should be at end again after Reset()");
    }

    void TestIsValidFromUnit()
    {
        TestIterator i(*mCluster, mClusterParams, mCluster->unitDataSize - MockUnit::DEFAULT_SIZE, 1);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(i.IsValid(), "Should be valid if unit is");
        u.SetValid(false);
        EATESTAssert(!i.IsValid(), "Should not be valid if unit isn't");
    }

    void TestIsValidAtEnd()
    {
        TestIterator i(*mCluster, mClusterParams, mCluster->unitDataSize - MockUnit::DEFAULT_SIZE, 1);
        const TestIterator::UnitType & u = i.GetUnit();
        // Move onto end
        i.Next();
        EATESTAssert(i.AtEnd(), "Should now be at end");
        u.SetValid(true);
        EATESTAssert(!i.IsValid(), "Should not be valid if unit is at end");
    }

    void TestIsValidOffset()
    {
        mCluster->unitDataSize = 50u;
        TestIterator i(*mCluster, mClusterParams, mCluster->unitDataSize - MockUnit::DEFAULT_SIZE, 3);
        EATESTAssert(i.IsValid(), "Should be valid initially");
        const TestIterator::UnitType & u = i.GetUnit();
        // Next will move beyond end of cluster data, despite being told there were 3 units
        u.SetSize(200u);
        i.Next();
        EATESTAssert(i.IsValid(), "Should be valid unless unit is invalid");
        ((TestIterator::UnitType &) u).mValid = false;
        EATESTAssert(!i.IsValid(), "Should not be valid if unit data is beyond end of cluster");
    }

    void TestNextOneTriangle()
    {
        // Setup for 1 units (1 tri)
        TestIterator i(*mCluster, mClusterParams, 0, 1);
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        i.Next();
        EATESTAssert(i.AtEnd(), "Should now be at end after one triangle");
    }

    void TestNextOneQuad()
    {
        // Setup for 1 unit (1 quad)
        TestIteratorQuad i(*mCluster, mClusterParams, 0, 1);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        const uint8_t * firstUnit = u.mData;
        i.Next();
        EATESTAssert(u.mData == firstUnit, "Should still be on first unit");
        EATESTAssert(!i.AtEnd(), "Should not be at end after first triangle from quad");
        i.Next();
        EATESTAssert(u.mData == firstUnit, "Should still be on second unit when reached end");
        EATESTAssert(i.AtEnd(), "Should now be at end after two triangles from quad");
    }

    void TestNextMixed()
    {
        // Setup for 3 units (1 tri and 2 quads)
        TestIterator i(*mCluster, mClusterParams, 0, 3);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        u.SetTriCount(2);    // return a quad for the second unit
        i.Next();
        const uint8_t * firstUnit = u.mData;
        EATESTAssert(!i.AtEnd(), "Should not be at end after first triangle");
        // Move onto second triangle from second unit
        i.Next();
        EATESTAssert(u.mData == firstUnit, "Should still be on second unit");
        EATESTAssert(!i.AtEnd(), "Should still not be at end after first triangle in quad");
        i.Next();
        EATESTAssert(!i.AtEnd(), "Should not be at end after both triangles from first quad");
        // Move onto second triangle from second unit
        i.Next();
        EATESTAssert(u.mData != firstUnit, "Should have moved to third unit");
        EATESTAssert(!i.AtEnd(), "Should still not be at end after first triangle in second quad");
        i.Next();
        EATESTAssert(i.AtEnd(), "Should now be at end after both triangles from second quad");
    }

    void TestGetTriVertices()
    {
        TestIterator i(*mCluster, mClusterParams);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        rwpmath::Vector3 undefined(-1.0f, -1.0f, -1.0f);
        rwpmath::Vector3 v0 = undefined, v1 = undefined, v2 = undefined;
        i.GetVertices(v0, v1, v2);
        EATESTAssert(v0 == mCluster->vertexArray[0], "First corner should be vertex 0");
        EATESTAssert(v1 == mCluster->vertexArray[1], "Second corner should be vertex 1");
        EATESTAssert(v2 == mCluster->vertexArray[2], "Third corner should be vertex 2");
        // Move onto second triangle
        u.SetVertexBase(3); 
        i.Next();
        EATESTAssert(!i.AtEnd(), "Should still not be at end");
        rwpmath::Vector3 w0 = undefined, w1 = undefined, w2 = undefined;
        i.GetVertices(w0, w1, w2);
        EATESTAssert(w0 == mCluster->vertexArray[3], "First corner should be vertex 3");
        EATESTAssert(w1 == mCluster->vertexArray[4], "Second corner should be vertex 4");
        EATESTAssert(w2 == mCluster->vertexArray[5], "Third corner should be vertex 5");
    }

    void TestGetQuadVertices()
    {
        // Create iterator which is offset beyond the first two triangle units.
        TestIteratorQuad i(
            *mCluster,
            mClusterParams,
            8,
            1);

        const TestIterator::UnitType & u = i.GetUnit();
        const uint8_t * firstUnit = u.mData;
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        rwpmath::Vector3 undefined(-1.0f, -1.0f, -1.0f);
        rwpmath::Vector3 v0 = undefined, v1 = undefined, v2 = undefined;
        // MockUnit2 returns 3 + 2*i for vertex i.
        // TriangleIterator returns "second" triangle first (v3,v2,v1)
        i.GetVertices(v0, v1, v2);
        EATESTAssert(v0 == mCluster->vertexArray[3], "First corner should be vertex 9");
        EATESTAssert(v1 == mCluster->vertexArray[2], "Second corner should be vertex 7");
        EATESTAssert(v2 == mCluster->vertexArray[1], "Third corner should be vertex 5");
        // Move onto second triangle from first unit
        i.Next();
        EATESTAssert(u.mData == firstUnit, "Should still be on first unit");
        EATESTAssert(!i.AtEnd(), "Should still not be at end after first triangle in quad");
        rwpmath::Vector3 w0 = undefined, w1 = undefined, w2 = undefined;
        i.GetVertices(w0, w1, w2);
        EATESTAssert(w0 == mCluster->vertexArray[0], "First corner should be vertex 3");
        EATESTAssert(w1 == mCluster->vertexArray[1], "Second corner should be vertex 5");
        EATESTAssert(w2 == mCluster->vertexArray[2], "Third corner should be vertex 7");
    }

    void TestGetTriEdgeCosines()
    {
        TestIterator i(*mCluster, mClusterParams);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        rwpmath::Vector3 e;
        uint32_t flags = i.GetEdgeCosinesAndFlags(e);
        // MockUnit2 returns 3.0f - 2*(i+1) for edge i
        EATESTAssert(e == rwpmath::Vector3(-2.0f, -4.0f, -6.0f), "Edge cosines should be -2,-4,-6");
        EATESTAssert(flags == MockUnit2::FLAGS, "Should return flags from unit");
        // Move onto second triangle
        u.SetVertexBase(3); // Change so MockUnit2 returns 5.0f - 2*(i+1) for edge i
        i.Next();
        EATESTAssert(!i.AtEnd(), "Should still not be at end");
        rwpmath::Vector3 f;
        flags = i.GetEdgeCosinesAndFlags(f);
        EATESTAssert(f == rwpmath::Vector3(1.0f, -1.0f, -3.0f), "Edge cosines should be 1,-1,-3");
        EATESTAssert(flags == MockUnit2::FLAGS, "Should return flags from next unit");
    }

    void TestGetQuadEdgeCosines()
    {
        TestIteratorQuad i(*mCluster, mClusterParams);
        const TestIterator::UnitType & u = i.GetUnit();
        const uint8_t * firstUnit = u.mData;
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        rwpmath::Vector3 e;
        uint32_t flags = i.GetEdgeCosinesAndFlags(e);
        // MockUnit2 returns 4.0f - 2*(i+1) for edge i, tri=1 first
        EATESTAssert(e == rwpmath::Vector3(-1.0f, -3.0f, -5.0f), "Edge cosines should be -1,-3,-5");
        EATESTAssert(flags == MockUnit2::FLAGS + 1, "Should return flags from unit for second tri");
        // Move onto second triangle from first unit
        i.Next();
        EATESTAssert(u.mData == firstUnit, "Should still be on first unit");
        EATESTAssert(!i.AtEnd(), "Should still not be at end after first triangle in quad");
        rwpmath::Vector3 f;
        flags = i.GetEdgeCosinesAndFlags(f);
        // MockUnit2 returns 3.0f - 2*(i+1) for edge i, tri=0 second
        EATESTAssert(f == rwpmath::Vector3(-2.0f, -4.0f, -6.0f), "Edge cosines should be -2,-4,-6");
        EATESTAssert(flags == MockUnit2::FLAGS, "Should return flags from unit");
    }

    void TestGetID()
    {
        TestIterator i(*mCluster, mClusterParams);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        EATESTAssert(i.GetID() == 0x12345678u, "Should return ID from Unit");
        u.SetTriCount(2);    // return a quad for the second unit
        u.SetID(0x87654321u);
        i.Next();
        EATESTAssert(i.GetID() == 0x87654321u, "Should return ID from second unit");
        // Move onto second triangle from first unit
        i.Next();
        EATESTAssert(i.GetID() == 0x87654321u, "Should return same ID");
    }

    void TestGetGroupID()
    {
        TestIterator i(*mCluster, mClusterParams);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        EATESTAssert(i.GetID() == 0x12345678u, "Should return ID from Unit");
        u.SetTriCount(2);    // return a quad for the second unit
        u.SetID(0x87654321u);
        i.Next();
        EATESTAssert(i.GetGroupID() == 0x00004321u, "Should return GroupID from second unit");
        // Move onto second triangle from first unit
        i.Next();
        EATESTAssert(i.GetGroupID() == 0x00004321u, "Should return same GroupID");
    }

    void TestGetSurfaceID()
    {
        TestIterator i(*mCluster, mClusterParams);
        const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        EATESTAssert(i.GetID() == 0x12345678u, "Should return ID from Unit");
        u.SetTriCount(2);    // return a quad for the second unit
        u.SetID(0x87654321u);
        i.Next();
        EATESTAssert(i.GetSurfaceID() == 0x00008765u, "Should return SurfaceID from second unit");
        // Move onto second triangle from first unit
        i.Next();
        EATESTAssert(i.GetSurfaceID() == 0x00008765u, "Should return same SurfaceID");
    }

    void TestGetOffset()
    {
        TestIterator iT(*mCluster, mClusterParams);
        const TestIterator::UnitType & uT = iT.GetUnit();
        EATESTAssert(!iT.AtEnd(), "Should not be at end initially");

        uint32_t offset;
        offset = iT.GetOffset();
        EATESTAssert(0 == offset, "Offset should be zero");

        iT.Next();
        offset = iT.GetOffset();
        EATESTAssert(uT.GetSize() == offset, "Offset should be same size as first unit");

        iT.Next();
        offset = iT.GetOffset();
        EATESTAssert(uT.GetSize() * 2  == offset, "Offset should be twice the size of first unit");

        // Create iterator which is offset beyond the first two triangle units.
        TestIteratorQuad iQ(
            *mCluster,
            mClusterParams,
            8,
            2);
        const TestIteratorQuad::UnitType & uQ = iQ.GetUnit();

        offset = iQ.GetOffset();
        EATESTAssert(uT.GetSize() * 2 == offset, "Offset should be twice the size of first unit");

        iQ.Next();
        offset = iQ.GetOffset();
        EATESTAssert(uT.GetSize() * 2 == offset, "Offset should be twice the size of first unit");

        iQ.Next();
        offset = iQ.GetOffset();
        EATESTAssert(uT.GetSize() * 2 + uQ.GetSize() == offset, "Offset should be twice the size of first plus size of third unit");

        iQ.Next();
        offset = iQ.GetOffset();
        EATESTAssert(uT.GetSize() * 2 + uQ.GetSize() == offset, "Offset should be twice the size of first plus size of third unit");
    }

    void TestGetNumTrianglesLeftInCurrentUnit()
    {
        // No units
        {
            TestIterator i(*mCluster, mClusterParams, 0, 0, 0);
            EATESTAssert(i.AtEnd(), "Should be at end initially");

            uint32_t numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(0 == numTriLeftInCurrentUnit, "Should return 0 triangles");
        }

        // One Unit with one triangle
        {
            TestIterator i(*mCluster, mClusterParams, 0, 1);
            EATESTAssert(!i.AtEnd(), "Should not be at end initially");

            uint32_t numTriLeftInCurrentUnit;

            numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(1 == numTriLeftInCurrentUnit, "Should return 1 triangle");

            i.Next();
            numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(0 == numTriLeftInCurrentUnit, "Should return 1 triangle");
        }

        // One Unit with two triangles
        {
            TestIteratorQuad i(*mCluster, mClusterParams, 0, 1);
            EATESTAssert(!i.AtEnd(), "Should not be at end initially");

            uint32_t numTriLeftInCurrentUnit;

            numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(2 == numTriLeftInCurrentUnit, "Should return 1 triangle");

            i.Next();
            numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(1 == numTriLeftInCurrentUnit, "Should return 1 triangle");

            i.Next();
            numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(0 == numTriLeftInCurrentUnit, "Should return 1 triangle");
        }

        // One Unit with two triangles but only one left to iterate
        {
            TestIteratorQuad i(*mCluster, mClusterParams, 0, 1, 1);
            EATESTAssert(!i.AtEnd(), "Should not be at end initially");

            uint32_t numTriLeftInCurrentUnit;

            numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(1 == numTriLeftInCurrentUnit, "Should return 1 triangle");

            i.Next();
            numTriLeftInCurrentUnit = i.GetNumTrianglesLeftInCurrentUnit();
            EATESTAssert(0 == numTriLeftInCurrentUnit, "Should return 1 triangle");
        }
    }


    void TestGetTriVertexIndices()
    {
        TestIterator i(*mCluster, mClusterParams);
        //const TestIterator::UnitType & u = i.GetUnit();
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        uint8_t v0, v1, v2;
        i.GetVertexIndices(v0, v1, v2);
        // MockUnit2 returns 3 + 2*i for vertex i
        EATESTAssert(v0 == 0, "First corner should be vertex 0");
        EATESTAssert(v1 == 1, "Second corner should be vertex 1");
        EATESTAssert(v2 == 2, "Third corner should be vertex 2");
        // Move onto second triangle
        //u.SetVertexBase(7); // Change so MopckUnit2 returns 7 + 2*i for vertex i
        i.Next();
        EATESTAssert(!i.AtEnd(), "Should still not be at end");
        uint8_t w0, w1, w2;
        i.GetVertexIndices(w0, w1, w2);
        EATESTAssert(w0 == 7, "First corner should be vertex 7");
        EATESTAssert(w1 == 8, "Second corner should be vertex 8");
        EATESTAssert(w2 == 9, "Third corner should be vertex 9");
    }

    void TestGetQuadVertexIndices()
    {
        // Create iterator which is offset beyond the first two triangle units.
        TestIteratorQuad i(
            *mCluster,
            mClusterParams,
            8,
            2);

        const TestIteratorQuad::UnitType & u = i.GetUnit();
        const uint8_t * firstUnit = u.mData;
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        uint8_t v0, v1, v2;
        // TriangleIterator returns "second" triangle first (v3,v2,v1)
        i.GetVertexIndices(v0, v1, v2);
        EATESTAssert(v0 == 3, "First corner should be vertex 3");
        EATESTAssert(v1 == 2, "Second corner should be vertex 2");
        EATESTAssert(v2 == 1, "Third corner should be vertex 1");
        // Move onto second triangle from first unit
        i.Next();
        EATESTAssert(u.mData == firstUnit, "Should still be on first unit");
        EATESTAssert(!i.AtEnd(), "Should still not be at end after first triangle in quad");
        uint8_t w0, w1, w2;
        i.GetVertexIndices(w0, w1, w2);
        EATESTAssert(w0 == 0, "First corner should be vertex 0");
        EATESTAssert(w1 == 1, "Second corner should be vertex 1");
        EATESTAssert(w2 == 2, "Third corner should be vertex 2");
        // Move onto second quad
        i.Next();
        const uint8_t * secondUnit = u.mData;
        // TriangleIterator returns "second" triangle first (v3,v2,v1)
        i.GetVertexIndices(v0, v1, v2);
        EATESTAssert(v0 == 6, "First corner should be vertex 3");
        EATESTAssert(v1 == 7, "Second corner should be vertex 2");
        EATESTAssert(v2 == 8, "Third corner should be vertex 1");
        // Move onto first triangle in second quad
        i.Next();
        EATESTAssert(u.mData == secondUnit, "Should still be on first unit");
        EATESTAssert(!i.AtEnd(), "Should still not be at end after first triangle in quad");
        i.GetVertexIndices(w0, w1, w2);
        EATESTAssert(w0 == 9, "First corner should be vertex 9");
        EATESTAssert(w1 == 8, "Second corner should be vertex 8");
        EATESTAssert(w2 == 7, "Third corner should be vertex 7");
    }

private:

    static const uint32_t CLUSTER_MEMORY_SIZE = 2048;
    static EA_ALIGNED(uint8_t, mClusterData[CLUSTER_MEMORY_SIZE], 16);

    rw::collision::ClusteredMeshCluster * mCluster;
    rw::collision::ClusterParams mClusterParams;

} gTestTriangleIterator;

EA_ALIGNED(uint8_t, TestTriangleIterator::mClusterData[TestTriangleIterator::CLUSTER_MEMORY_SIZE], 16);
