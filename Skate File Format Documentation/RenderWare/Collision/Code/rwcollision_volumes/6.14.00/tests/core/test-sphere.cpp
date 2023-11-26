// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>
#include <rw/collision/libcore.h>
#if 0 // not implemented
#include <rw/collision/detail/fpu/sphere.h>
#endif // not implemented

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

#include "volumecompare.h"

using namespace rwpmath;
using namespace rw::collision;



class TestSphereVolume: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestSphereVolume");

        EATEST_REGISTER("TestGetType", "Test Volume::GetType returns correct type", TestSphereVolume, TestGetType);

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of SphereVolume",
                        TestSphereVolume, TestHLSerialization);
        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of SphereVolume",
                         TestSphereVolume, TestHLFileSerialization);


#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuSerialization", "Low-level serialization of SphereVolume using vpu math",
                        TestSphereVolume, TestLLVpuSerialization);
        EATEST_REGISTER("TestLLVpuFileSerialization", "Low-level file serialization of SphereVolume using vpu math",
                        TestSphereVolume, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

#if 0 // not implemented
        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of SphereVolume using fpu math",
                        TestSphereVolume, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of SphereVolume using fpu math",
                        TestSphereVolume, TestLLFpuFileSerialization);
#endif // not implemented

        EATEST_REGISTER("TestSphereUniformScale", "Test application of uniform scale to SphereVolume",
                        TestSphereVolume, TestSphereUniformScale);

        EATEST_REGISTER("TestSphereProcessingFlags", "Test API for setting/clearing processing flags",
                        TestSphereVolume, TestSphereProcessingFlags);
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


    static SphereVolume * CreateSphereVolume()
    {
        const float radius = 5.0f;
        return EA::Physics::UnitFramework::Creator<SphereVolume>().New(radius);
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

    void TestSphereUniformScale();

    void TestSphereProcessingFlags();

    void TestGetType()
    {
        const SphereVolume *volume = CreateSphereVolume();
        EATESTAssert(rw::collision::VOLUMETYPESPHERE == volume->GetType(), "SphereVolume::GetType() returned incorrect type for sphere");
        EATESTAssert(rw::collision::VOLUMETYPESPHERE == static_cast<const Volume *>(volume)->GetType(), "Volume::GetType() returned incorrect type for sphere");
    }

} TestSphereVolumeSingleton;



void TestSphereVolume::TestHLSerialization()
{
    SphereVolume * original = CreateSphereVolume();
    SphereVolume * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level serialized copies do not match.");
}



void TestSphereVolume::TestHLFileSerialization()
{
    SphereVolume* original = CreateSphereVolume();
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("sphere");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    SphereVolume* copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<SphereVolume>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level file serialized copies do not match.");
}



#if !defined(RWP_NO_VPU_MATH)

void TestSphereVolume::TestLLVpuSerialization()
{
    SphereVolume* original = CreateSphereVolume();

    SphereVolume* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu serialized copies do not match.");
}


void TestSphereVolume::TestLLVpuFileSerialization()
{
    SphereVolume* original = CreateSphereVolume();
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("sphere");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    SphereVolume* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<SphereVolume>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu file serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)



#if 0 // not implemented

void TestSphereVolume::TestLLFpuSerialization()
{
    SphereVolume* original = CreateSphereVolume();

#if !defined(RWP_NO_VPU_MATH)
    SphereVolume* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization<SphereVolume, rw::collision::detail::fpu::SphereVolume>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    SphereVolume* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}


void TestSphereVolume::TestLLFpuFileSerialization()
{
    SphereVolume* original = CreateSphereVolume();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("sphere");

#if defined(CREATE_SERIALIZATION_TEST_DATA)
#if !defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<SphereVolume, rw::collision::detail::fpu::SphereVolume>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<SphereVolume>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)
#endif // defined(CREATE_SERIALIZATION_TEST_DATA)

#if !defined(RWP_NO_VPU_MATH)
    SphereVolume* copied = LoadLLFpuSerializationFromFile<SphereVolume, rw::collision::detail::fpu::SphereVolume>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    SphereVolume* copied = LoadLLFpuSerializationFromFile<SphereVolume>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}


#endif // not implemented


void TestSphereVolume::TestSphereUniformScale()
{
    Matrix44Affine tm(GetMatrix44Affine_Identity());
    tm.Pos().Set(1.0f, 2.0f, 3.0f);

    float scale = 2.0f;

    // Manually scaled
    SphereVolume* sph1 = CreateSphereVolume();
    Matrix44Affine scaledTM(tm);
    scaledTM.Pos() *= scale;
    sph1->SetLocalTransform(scaledTM);
    sph1->SetRadius(sph1->GetRadius() * scale);

    // Scale with API
    SphereVolume* sph2 = CreateSphereVolume();
    sph2->SetLocalTransform(tm);
    sph2->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*sph1, *sph2), "SphereVolume::ApplyUniformScale does not behave as expected.");

    // Scale with API - virtual
    SphereVolume* sph3 = CreateSphereVolume();
    sph3->SetLocalTransform(tm);
    static_cast<Volume*>(sph3)->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*sph1, *sph3), "Volume::ApplyUniformScale does not behave as expected on SphereVolume.");
}


void TestSphereVolume::TestSphereProcessingFlags()
{
    EA::Allocator::ICoreAllocator *alloc = EA::Allocator::ICoreAllocator::GetDefaultAllocator();

    const float radius = 1.0f;
    SphereVolume *sphere = EA::Physics::UnitFramework::Creator<SphereVolume>(*alloc).New(radius);

    // These call generic volume functions that are applicable to all primitive types
    sphere->SetProcessedFlag();
    EATESTAssert(sphere->GetFlags() & VOLUMEFLAG_ISPROCESSED, "SetProcessedFlag() failed");

    sphere->ClearAllProcessedFlags();
    EATESTAssert(!(sphere->GetFlags() & VOLUMEFLAG_ISPROCESSED), "ClearAllProcessedFlags() failed");

    alloc->Free(sphere);
}
