// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#if 0 // not implemented
#include <rw/collision/detail/fpu/capsule.h>
#endif // not implemented

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

#include "volumecompare.h"

using namespace rwpmath;
using namespace rw::collision;



class TestCapsuleVolume: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestCapsuleVolume");

        EATEST_REGISTER("TestGetType", "Test Volume::GetType returns correct type", TestCapsuleVolume, TestGetType);

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of CapsuleVolume",
                        TestCapsuleVolume, TestHLSerialization);
        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of CapsuleVolume",
                         TestCapsuleVolume, TestHLFileSerialization);


#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuSerialization", "Low-level serialization of CapsuleVolume using vpu math",
                        TestCapsuleVolume, TestLLVpuSerialization);
        EATEST_REGISTER("TestLLVpuFileSerialization", "Low-level file serialization of CapsuleVolume using vpu math",
                        TestCapsuleVolume, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

#if 0 // not implemented
        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of CapsuleVolume using fpu math",
                        TestCapsuleVolume, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of CapsuleVolume using fpu math",
                        TestCapsuleVolume, TestLLFpuFileSerialization);
#endif // not implemented

        EATEST_REGISTER("TestCapsuleUniformScale", "Test application of uniform scale to CapsuleVolume",
                        TestCapsuleVolume, TestCapsuleUniformScale);

        EATEST_REGISTER("TestCapsuleDisableEndCapAPI", "Test API for disabling end caps on CapsuleVolume",
                        TestCapsuleVolume, TestCapsuleDisableEndCapAPI);
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


    static CapsuleVolume * CreateCapsuleVolume()
    {
        // Capsule with 5.0 halfheight and 1.0 radius
        const float halfHeight = 5.0f;
        const float radius = 1.0f;
        return EA::Physics::UnitFramework::Creator<CapsuleVolume>().New(radius, halfHeight);
    }

private:
    void TestHLSerialization();
    void TestHLFileSerialization();

    void TestLLVpuSerialization();
    void TestLLVpuFileSerialization();

#if 0 // not implemented
    void TestLLFpuSerialization();
    void TestLLFpuFileSerialization();
#endif // not implemented

    void TestCapsuleUniformScale();

    void TestGetType()
    {
        const CapsuleVolume *volume = CreateCapsuleVolume();
        EATESTAssert(rw::collision::VOLUMETYPECAPSULE == volume->GetType(), "CapsuleVolume::GetType() returned incorrect type for capsule");
        EATESTAssert(rw::collision::VOLUMETYPECAPSULE == static_cast<const Volume *>(volume)->GetType(), "Volume::GetType() returned incorrect type for capsule");
    }

    void TestCapsuleDisableEndCapAPI();

} TestCapsuleVolumeSingleton;


void TestCapsuleVolume::TestHLSerialization()
{
    CapsuleVolume * original = CreateCapsuleVolume();
    CapsuleVolume * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level serialized copies do not match.");
}


void TestCapsuleVolume::TestHLFileSerialization()
{
    CapsuleVolume* original = CreateCapsuleVolume();
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("capsule");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    CapsuleVolume* copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<CapsuleVolume>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level file serialized copies do not match.");
}


#if !defined(RWP_NO_VPU_MATH)

void TestCapsuleVolume::TestLLVpuSerialization()
{
    CapsuleVolume* original = CreateCapsuleVolume();

    CapsuleVolume* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu serialized copies do not match.");
}


void TestCapsuleVolume::TestLLVpuFileSerialization()
{
    CapsuleVolume* original = CreateCapsuleVolume();
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("capsule");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    CapsuleVolume* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<CapsuleVolume>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu file serialized copies do not match.");
}
#endif // !defined(RWP_NO_VPU_MATH)



#if 0 // not implemented

void TestCapsuleVolume::TestLLFpuSerialization()
{
    CapsuleVolume* original = CreateCapsuleVolume();

#if !defined(RWP_NO_VPU_MATH)
    CapsuleVolume* copied = CopyViaLLFpuSerialization<CapsuleVolume, rw::collision::detail::fpu::CapsuleVolume>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    CapsuleVolume* copied = CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(CompareCapsuleVolumes(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}


void TestCapsuleVolume::TestLLFpuFileSerialization()
{
    CapsuleVolume* original = CreateCapsuleVolume();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("capsule");

#if defined(CREATE_SERIALIZATION_TEST_DATA)
#if !defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<CapsuleVolume, rw::collision::detail::fpu::CapsuleVolume>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<CapsuleVolume>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)
#endif // defined(CREATE_SERIALIZATION_TEST_DATA)

#if !defined(RWP_NO_VPU_MATH)
    CapsuleVolume* copied = LoadLLFpuSerializationFromFile<CapsuleVolume, rw::collision::detail::fpu::CapsuleVolume>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    CapsuleVolume* copied = LoadLLFpuSerializationFromFile<CapsuleVolume>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(CompareCapsuleVolumes(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}

#endif // not implemented

void TestCapsuleVolume::TestCapsuleUniformScale()
{
    Matrix44Affine tm(GetMatrix44Affine_Identity());
    tm.Pos().Set(1.0f, 2.0f, 3.0f);

    float scale = 2.0f;

    // Manually scaled
    CapsuleVolume* cap1 = CreateCapsuleVolume();
    Matrix44Affine scaledTM(tm);
    scaledTM.Pos() *= scale;
    cap1->SetLocalTransform(scaledTM);
    cap1->SetRadius(cap1->GetRadius() * scale);
    cap1->SetHalfHeight(cap1->GetHalfHeight() * scale);

    // Scale with API
    CapsuleVolume* cap2 = CreateCapsuleVolume();
    cap2->SetLocalTransform(tm);
    cap2->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*cap1, *cap2), "CapsuleVolume::ApplyUniformScale does not behave as expected.");

    // Scale with API - virtual
    CapsuleVolume* cap3 = CreateCapsuleVolume();
    cap3->SetLocalTransform(tm);
    static_cast<Volume*>(cap3)->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*cap1, *cap3), "Volume::ApplyUniformScale does not behave as expected on CapsuleVolume.");
}


void TestCapsuleVolume::TestCapsuleDisableEndCapAPI()
{
    CapsuleVolume* capsule = CreateCapsuleVolume();

    //Force all other flags to be set
    uint32_t otherFlags = 0xFFFFFFFF & ~(VOLUMEFLAG_CAPSULEEND_0_DISABLED + VOLUMEFLAG_CAPSULEEND_1_DISABLED);
    capsule->SetFlags(otherFlags);

    EATESTAssert(!capsule->IsEndCap0Disabled(), "CapsuleVolume is created with End Cap 0 Disabled.");
    EATESTAssert(!capsule->IsEndCap1Disabled(), "CapsuleVolume is created with End Cap 1 Disabled.");

    //Disable End cap 0
    capsule->SetEndCap0Disabled(true);
    EATESTAssert(capsule->IsEndCap0Disabled(), "CapsuleVolume failed to disable End Cap 0");
    EATESTAssert(!capsule->IsEndCap1Disabled(), "CapsuleVolume disabled End Cap 1 by mistake");

    //Check Flags
    uint32_t expectedFlags = capsule->GetFlags() & ~(VOLUMEFLAG_CAPSULEEND_0_DISABLED + VOLUMEFLAG_CAPSULEEND_1_DISABLED);
    EATESTAssert(expectedFlags == otherFlags, "CapsuleVolume flags not what expected.");

    //re-enable End cap 0
    capsule->SetEndCap0Disabled(false);
    EATESTAssert(!capsule->IsEndCap0Disabled(), "CapsuleVolume failed to re-enable End Cap 0");
    EATESTAssert(!capsule->IsEndCap1Disabled(), "CapsuleVolume disabled End Cap 1 by mistake");

    //Disable End cap 1
    capsule->SetEndCap1Disabled(true);
    EATESTAssert(capsule->IsEndCap1Disabled(), "CapsuleVolume failed to disable End Cap 1");
    EATESTAssert(!capsule->IsEndCap0Disabled(), "CapsuleVolume disabled End Cap 0 by mistake");

    //re-enable End cap 1
    capsule->SetEndCap1Disabled(false);
    EATESTAssert(!capsule->IsEndCap1Disabled(), "CapsuleVolume failed to re-enable End Cap 1");
    EATESTAssert(!capsule->IsEndCap0Disabled(), "CapsuleVolume disabled End Cap 0 by mistake");

    //Check other flags again
    expectedFlags = capsule->GetFlags() & ~(VOLUMEFLAG_CAPSULEEND_0_DISABLED + VOLUMEFLAG_CAPSULEEND_1_DISABLED);
    EATESTAssert(expectedFlags == otherFlags, "CapsuleVolume flags not what expected.");

}



