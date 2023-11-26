// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/box.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

#include "volumecompare.h"

using namespace rwpmath;
using namespace rw::collision;


class TestBoxVolume: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestBoxVolume");

        EATEST_REGISTER("TestGetType", "Test Volume::GetType returns correct type", TestBoxVolume, TestGetType);

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of BoxVolume",
                        TestBoxVolume, TestHLSerialization);
        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of BoxVolume",
                         TestBoxVolume, TestHLFileSerialization);


#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuSerialization", "Low-level serialization of BoxVolume using vpu math",
                        TestBoxVolume, TestLLVpuSerialization);
        EATEST_REGISTER("TestLLVpuFileSerialization", "Low-level file serialization of BoxVolume using vpu math",
                        TestBoxVolume, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of BoxVolume using fpu math",
                        TestBoxVolume, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of BoxVolume using fpu math",
                        TestBoxVolume, TestLLFpuFileSerialization);

        EATEST_REGISTER("TestBoxUniformScale", "Test application of uniform scale to BoxVolume",
                        TestBoxVolume, TestBoxUniformScale);
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

    static BoxVolume * CreateBoxVolume()
    {
        return EA::Physics::UnitFramework::Creator<BoxVolume>().New(1.0f, 2.0f, 4.0f);
    }

private:
    void TestHLSerialization();
    void TestHLFileSerialization();

    void TestLLVpuSerialization();
    void TestLLVpuFileSerialization();

    void TestLLFpuSerialization();
    void TestLLFpuFileSerialization();

    void TestBoxUniformScale();

    void TestGetType()
    {
        const BoxVolume *volume = CreateBoxVolume();
        EATESTAssert(rw::collision::VOLUMETYPEBOX == volume->GetType(), "BoxVolume::GetType() returned incorrect type for box");
        EATESTAssert(rw::collision::VOLUMETYPEBOX == static_cast<const Volume *>(volume)->GetType(), "Volume::GetType() returned incorrect type for box");
    }

} TestBoxVolumeSingleton;


void TestBoxVolume::TestHLSerialization()
{
    BoxVolume * original = CreateBoxVolume();
    BoxVolume * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level serialized copies do not match.");
}


void TestBoxVolume::TestHLFileSerialization()
{
    BoxVolume* original = CreateBoxVolume();
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("box");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    BoxVolume* copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<BoxVolume>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level file serialized copies do not match.");
}


#if !defined(RWP_NO_VPU_MATH)

void TestBoxVolume::TestLLVpuSerialization()
{
    BoxVolume* original = CreateBoxVolume();

    BoxVolume* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu serialized copies do not match.");
}


void TestBoxVolume::TestLLVpuFileSerialization()
{
    BoxVolume* original = CreateBoxVolume();
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("box");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    BoxVolume* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<BoxVolume>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu file serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)


void TestBoxVolume::TestLLFpuSerialization()
{
    BoxVolume * original = CreateBoxVolume();

#if !defined(RWP_NO_VPU_MATH)
    BoxVolume* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization<BoxVolume, rw::collision::detail::fpu::BoxVolume>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    BoxVolume* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}


void TestBoxVolume::TestLLFpuFileSerialization()
{
    BoxVolume* original = CreateBoxVolume();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("box");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<BoxVolume, rw::collision::detail::fpu::BoxVolume>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<BoxVolume>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    BoxVolume* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<BoxVolume, rw::collision::detail::fpu::BoxVolume>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    BoxVolume* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<BoxVolume>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}

void TestBoxVolume::TestBoxUniformScale()
{
    Matrix44Affine tm(GetMatrix44Affine_Identity());
    tm.Pos().Set(1.0f, 2.0f, 3.0f);

    float scale = 2.0f;

    // Manually scaled box 1
    BoxVolume* box1 = CreateBoxVolume();
    Matrix44Affine scaledTM(tm);
    scaledTM.Pos() *= scale;
    box1->SetLocalTransform(scaledTM);
    Vector3 dimensions;
    box1->GetDimensions(dimensions);
    box1->SetDimensions(dimensions * scale);
    box1->SetRadius(box1->GetRadius() * scale);

    // Scale box 2 with API
    BoxVolume* box2 = CreateBoxVolume();
    box2->SetLocalTransform(tm);
    box2->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*box1, *box2), "BoxVolume::ApplyUniformScale does not behave as expected.");

    // Scale with API - virtual
    BoxVolume* box3 = CreateBoxVolume();
    box3->SetLocalTransform(tm);
    static_cast<Volume*>(box3)->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*box1, *box3), "Volume::ApplyUniformScale does not behave as expected on BoxVolume.");
}


