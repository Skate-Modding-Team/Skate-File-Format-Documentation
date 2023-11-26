// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <rw/collision/libcore.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase


class TestAggregateVolume : public rw::collision::tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestAggregateVolume");

        EATEST_REGISTER("TestGetType", "Test Volume::GetType returns correct type", TestAggregateVolume, TestGetType);
    }

    void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        rw::collision::Volume::InitializeVTable();
    }

    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        rw::collision::tests::TestSuiteBase::TeardownSuite();
    }

private:
    void TestGetType()
    {
        rw::collision::Aggregate aggregate(0, 0);
        const rw::collision::AggregateVolume *volume = EA::Physics::UnitFramework::Creator<rw::collision::AggregateVolume>().New(&aggregate);

        EATESTAssert(rw::collision::VOLUMETYPEAGGREGATE == volume->GetType(),
        "AggregateVolume::GetType() returned incorrect type for aggregate");
        EATESTAssert(rw::collision::VOLUMETYPEAGGREGATE == static_cast<const rw::collision::Volume *>(volume)->GetType(),
            "Volume::GetType() returned incorrect type for aggregate");
    }

} gTestAggregate;
