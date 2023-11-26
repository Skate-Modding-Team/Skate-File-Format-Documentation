// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rwpmath;
using namespace rw::collision;


class TestVolumeSerialization: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestVolumeSerialization");

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of Volume",
                        TestVolumeSerialization, TestHLSerialization);
        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of Volume",
                         TestVolumeSerialization, TestHLFileSerialization);

#if 0 // not implemented
        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of Volume using fpu math",
                        TestVolumeSerialization, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of Volume using fpu math",
                        TestVolumeSerialization, TestLLFpuFileSerialization);
#endif // not implemented
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        Volume::InitializeVTable();
    }

    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        tests::TestSuiteBase::TeardownSuite();
    }

    static Volume * CreateVolume()
    {
        return EA::Physics::UnitFramework::Creator<SphereVolume>().New(5.0f);
    }

    static bool CompareVolumes(const Volume& original, const Volume& copied)
    {
        return (original.GetRadius() == copied.GetRadius());
    }

private:

    void TestHLSerialization();
    void TestHLFileSerialization();

#if 0 // not implemented
    void TestLLFpuSerialization();
    void TestLLFpuFileSerialization();
#endif // not implemented

} TestVolumeSerializationSingleton;


void TestVolumeSerialization::TestHLSerialization()
{
    Volume * original = CreateVolume();
    Volume * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(CompareVolumes(*original, *copied), "Original and high-level serialized copies do not match.");
}


void TestVolumeSerialization::TestHLFileSerialization()
{
    Volume* original = CreateVolume();
    const char* filename = UNITTEST_DATA_FILE("sphere.dat");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    Volume* copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<Volume>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(CompareVolumes(*original, *copied), "Original and high-level file serialized copies do not match.");
}


#if 0 // not implemented

void TestVolumeSerialization::TestLLFpuSerialization()
{
    Volume* original = CreateVolume();

#if !defined(RWP_NO_VPU_MATH)
    Volume* copied = CopyViaLLFpuSerialization<Volume, rw::collision::detail::fpu::Volume>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    Volume* copied = CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(CompareVolumes(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}


void TestVolumeSerialization::TestLLFpuFileSerialization()
{
    Volume* original = CreateVolume();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("sphere");

#if defined(CREATE_SERIALIZATION_TEST_DATA)
#if !defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<Volume, rw::collision::detail::fpu::Volume>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<Volume>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)
#endif // defined(CREATE_SERIALIZATION_TEST_DATA)

#if !defined(RWP_NO_VPU_MATH)
    Volume* copied = LoadLLFpuSerializationFromFile<Volume, rw::collision::detail::fpu::Volume>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    Volume* copied = LoadLLFpuSerializationFromFile<Volume>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(CompareVolumes(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}


#endif // not implemented
