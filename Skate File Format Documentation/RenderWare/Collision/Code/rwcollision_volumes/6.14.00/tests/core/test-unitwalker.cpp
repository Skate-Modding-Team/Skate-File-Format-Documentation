// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

File: test-unitwalker.cpp

Purpose: Test walker for accessing units in clustered mesh clusters.

*/

// Intentionally include code under test first to check standalone header.
#include <rw/collision/clusterunitwalker.h>

#include <EABase/eabase.h>
#include <unit/unit.h>
#include <rw/collision/clusteredmeshcluster.h>

#include "testsuitebase.h" // For TestSuiteBase

#include "mock-unit.h"

class TestUnitWalker : public rw::collision::tests::TestSuiteBase
{
private:

    // We'll run all our tests on a walker that accesses a mock unit class
    typedef rw::collision::ClusterUnitWalker<MockUnit> TestWalker;

public:     // TestSuite overrides

#define UNIT_ITERATOR_TEST(M, D) EATEST_REGISTER(#M, D, TestUnitWalker, M)

    virtual void Initialize()
    {
        SuiteName("TestUnitWalker");
        UNIT_ITERATOR_TEST(TestConstructor, "Check basic constructor");
        UNIT_ITERATOR_TEST(TestConstructorWithOffset, "Check constructor with non-default offset");
        UNIT_ITERATOR_TEST(TestReset, "Check reset");
        UNIT_ITERATOR_TEST(TestIteration, "Check iteration");
        UNIT_ITERATOR_TEST(TestIsValidFromUnit, "Check behavior of IsValid()");
        UNIT_ITERATOR_TEST(TestIsValidAtEnd, "Check behavior of IsValid() when at end");
        UNIT_ITERATOR_TEST(TestIsValidOffset, "Check behavior of IsValid() when overflowing cluster");

        EATEST_REGISTER_SPU("TestUnitWalkerSPU", "SPU unit walker tests", "test-unitwalker.elf");
}

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();

        // We don't read any unit or vertex data from cluster since our MockUnit doesn't, 
        // but we do need some bits of the header stuff
        mCluster.unitCount = 5u;
        mCluster.unitDataSize = 123u;
        mCluster.unitDataStart = 2u;
        // These members shouldn't be used, so we'll set them to something unusual
        mCluster.normalStart = 3u;
        mCluster.totalSize = 723u;
        mCluster.vertexCount = 44u;
        mCluster.normalCount = 196u;
        mCluster.compressionMode = 57u;
        mClusterParams.mVertexCompressionGranularity = 0.0f;
        mClusterParams.mFlags = 33u;
        mClusterParams.mGroupIdSize = 33u;
        mClusterParams.mSurfaceIdSize = 33u;
    }

private:

    void TestConstructor()
    {
        MockUnit unit(mCluster, mClusterParams);
        TestWalker i(unit);
        const TestWalker::UnitType & u = i.GetUnit();
        EATESTAssert(u.mCluster == &mCluster, "Should have passed cluster to unit");
        EATESTAssert(u.mClusterParams == &mClusterParams, "Should have passed cluster params to unit");
        EATESTAssert(u.mOffset == 0, "Should have set offset to 0 by default");
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        EATESTAssert(mCluster.unitCount == 5, "Should have a cluster with 5 units");
        i.Next();
        i.Next();
        i.Next();
        i.Next();
        EATESTAssert(!i.AtEnd(), "Should still not be at end");
        i.Next();
        EATESTAssert(i.AtEnd(), "Should be at end now");
    }

    void TestConstructorWithOffset()
    {
        MockUnit unit(mCluster, mClusterParams, 46);
        TestWalker i(unit, 3);
        const TestWalker::UnitType & u = i.GetUnit();
        EATESTAssert(u.mCluster == &mCluster, "Should have passed cluster to unit");
        EATESTAssert(u.mClusterParams == &mClusterParams, "Should have passed cluster params to unit");
        EATESTAssert(u.mOffset == 46, "Should have passed offset to unit");
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        i.Next();
        i.Next();
        EATESTAssert(!i.AtEnd(), "Should still not be at end");
        i.Next();
        EATESTAssert(i.AtEnd(), "Should be at end now");
    }

    void TestIteration()
    {
        const uint8_t * expectedData = mCluster.UnitData() + 46;
        MockUnit unit(mCluster, mClusterParams, 46);
        TestWalker i(unit, 3);
        const TestWalker::UnitType & u = i.GetUnit();
        EATESTAssert(u.mData == expectedData, "Should not have called Advance()");
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        EATESTAssert(3 == i.GetRemainingUnits(), "Should have 3 remaining units");

        i.Next();
        EATESTAssert(2 == i.GetRemainingUnits(), "Should have 2 remaining units");
        expectedData += u.GetSize();
        EATESTAssert(i.GetUnit().mData == expectedData, "Should have called Advance() after first unit");
        u.SetSize(93);

        i.Next();
        EATESTAssert(1 == i.GetRemainingUnits(), "Should have 1 remaining units");
        expectedData += u.GetSize();
        EATESTAssert(i.GetUnit().mData == expectedData, "Should have called Advance() after second unit");
        EATESTAssert(!i.AtEnd(), "Should still not be at end");

        i.Next();
        EATESTAssert(0 == i.GetRemainingUnits(), "Should have 0 remaining units");
        // Should not move on
        EATESTAssert(i.GetUnit().mData == expectedData, "Should not have called Advance() after final unit");
        EATESTAssert(i.AtEnd(), "Should be at end now");
    }

    void TestReset()
    {
        MockUnit unit(mCluster, mClusterParams, 46);
        TestWalker i(unit, 3);
        const TestWalker::UnitType & u = i.GetUnit();
        EATESTAssert(u.mCluster == &mCluster, "Should have passed cluster to unit");
        EATESTAssert(u.mClusterParams == &mClusterParams, "Should have passed cluster params to unit");
        EATESTAssert(u.mOffset == 46, "Should have passed offset to unit");
        EATESTAssert(!i.AtEnd(), "Should not be at end initially");
        EATESTAssert(3 == i.GetRemainingUnits(), "Should have 3 remaining units");
        i.Next();
        EATESTAssert(2 == i.GetRemainingUnits(), "Should have 2 remaining units");
        i.Next();
        EATESTAssert(1 == i.GetRemainingUnits(), "Should have 1 remaining units");
        EATESTAssert(!i.AtEnd(), "Should still not be at end");
        // Reset to zero and set unit count to 1
        i.Reset(0, 1);
        EATESTAssert(!i.AtEnd(), "Should not be at end after Reset to zero");
        EATESTAssert(u.mOffset == 0, "Should have passed zero offset to unit");
        EATESTAssert(1 == i.GetRemainingUnits(), "Should have 1 remaining units");
        i.Next();
        EATESTAssert(0 == i.GetRemainingUnits(), "Should have 0 remaining units");
        EATESTAssert(i.AtEnd(), "Should be at end now since count adjusted");
        // Reset to non-zero and a new unit count
        i.Reset(46, 3);
        EATESTAssert(!i.AtEnd(), "Should not be at end after Reset");
        EATESTAssert(u.mOffset == 46, "Should have passed new offset to unit");
        EATESTAssert(3 == i.GetRemainingUnits(), "Should have 3 remaining units");
        uint32_t expected = 46 + u.GetSize();
        i.Next();
        EATESTAssert(2 == i.GetRemainingUnits(), "Should have 2 remaining units");
        EATESTAssert(u.mData == mCluster.UnitData() + expected, "Should have moved on again");
        i.Next();
        EATESTAssert(1 == i.GetRemainingUnits(), "Should have 1 remaining units");
        i.Next();
        EATESTAssert(0 == i.GetRemainingUnits(), "Should have 0 remaining units");
        EATESTAssert(i.AtEnd(), "Should be at end now");
    }


    void TestIsValidFromUnit()
    {
        MockUnit unit(mCluster, mClusterParams, mCluster.unitDataSize - MockUnit::DEFAULT_SIZE);
        TestWalker i(unit, 1);
        const TestWalker::UnitType & u = i.GetUnit();
        EATESTAssert(i.IsValid(), "Should be valid if unit is");
        u.SetValid(false);
        EATESTAssert(!i.IsValid(), "Should not be valid if unit isn't");
    }

    void TestIsValidAtEnd()
    {
        MockUnit unit(mCluster, mClusterParams, mCluster.unitDataSize - MockUnit::DEFAULT_SIZE);
        TestWalker i(unit, 1);
        const TestWalker::UnitType & u = i.GetUnit();
        // Move onto end
        i.Next();
        EATESTAssert(i.AtEnd(), "Should now be at end");
        u.SetValid(true);
        EATESTAssert(!i.IsValid(), "Should not be valid if unit is at end");
    }

    void TestIsValidOffset()
    {
        mCluster.unitDataSize = 50u;
        MockUnit unit(mCluster, mClusterParams, mCluster.unitDataSize - MockUnit::DEFAULT_SIZE);
        TestWalker i(unit, 3);
        EATESTAssert(i.IsValid(), "Should be valid initially");
        const TestWalker::UnitType & u = i.GetUnit();
        // Next will move beyond end of cluster data, despite being told there were 3 units
        u.SetSize(200u);
        i.Next();
        EATESTAssert(i.IsValid(), "Should be valid unless unit invalid");
        u.mValid = false;   // validity picked up from unit
        EATESTAssert(!i.IsValid(), "Should not be valid if unit data is beyond end of cluster");
    }

private:

    rw::collision::ClusteredMeshCluster mCluster;
    rw::collision::ClusterParams mClusterParams;

} gTestUnitWalker;
