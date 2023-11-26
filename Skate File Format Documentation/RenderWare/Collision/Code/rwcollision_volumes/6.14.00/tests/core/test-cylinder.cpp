// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#if 0 // not implemented
#include <rw/collision/detail/fpu/cylinder.h>
#endif // not implemented

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

#include "volumecompare.h"

using namespace rwpmath;
using namespace rw::collision;


class TestCylinderVolume: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestCylinderVolume");

        EATEST_REGISTER("TestGetType", "Test Volume::GetType returns correct type", TestCylinderVolume, TestGetType);

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of CylinderVolume",
                        TestCylinderVolume, TestHLSerialization);

        EATEST_REGISTER("TestCapsuleBug", "CylinderCapsule collision bug", TestCylinderVolume, TestCapsuleBug);

        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of CylinderVolume",
                         TestCylinderVolume, TestHLFileSerialization);

#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuSerialization", "Low-level serialization of CylinderVolume using vpu math",
                        TestCylinderVolume, TestLLVpuSerialization);
        EATEST_REGISTER("TestLLVpuFileSerialization", "Low-level file serialization of CylinderVolume using vpu math",
                        TestCylinderVolume, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

#if 0 // not implemented
        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of CylinderVolume using fpu math",
                        TestCylinderVolume, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of CylinderVolume using fpu math",
                        TestCylinderVolume, TestLLFpuFileSerialization);
#endif // not implemented

        EATEST_REGISTER("TestCylinderUniformScale", "Test application of uniform scale to CylinderVolume",
                        TestCylinderVolume, TestCylinderUniformScale);
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


    static CylinderVolume * CreateCylinderVolume()
    {
        const float halfHeight = 5.0f;
        const float radius = 1.0f;
        return EA::Physics::UnitFramework::Creator<CylinderVolume>().New(radius, halfHeight);
    }

private:
    void TestCapsuleBug();

    void TestHLSerialization();
    void TestHLFileSerialization();

    void TestLLVpuSerialization();
    void TestLLVpuFileSerialization();

#if 0 // not implemented
    void TestLLFpuSerialization();
    void TestLLFpuFileSerialization();
#endif // not implemented

    void TestCylinderUniformScale();

    void TestGetType()
    {
        const CylinderVolume *volume = CreateCylinderVolume();
        EATESTAssert(rw::collision::VOLUMETYPECYLINDER == volume->GetType(), "CylinderVolume::GetType() returned incorrect type for cylinder");
        EATESTAssert(rw::collision::VOLUMETYPECYLINDER == static_cast<const Volume *>(volume)->GetType(), "Volume::GetType() returned incorrect type for cylinder");
    }

} TestCylinderVolumeSingleton;


/*
Test the Cylinder vs Capsule bug.  discovered by Steve Scholl at mysims.
*/

void TestCylinderVolume::TestCapsuleBug()
{
    Volume v1, v2;

    CapsuleVolume::Initialize(&v1, .3f,.575f);

    CylinderVolume::Initialize(&v2, 2.2115f, 2.08501f);

    v1.SetLocalTransform(Matrix44Affine(0.0f, 0.0f, -1.0f,     -1.0f, 0.0f, 0.0f,     0.0f, 1.0f, 0.0f,     0.0f, 0.0f, 0.0f));

    Matrix44Affine tm1(-0.328261f, 0, -0.944587f,
        0.0f, 1.0f, 0.0f,
        0.944587f, 0, -0.328261f,
        71.8751f, 12.5006f + .995f, 31.007f);

    v2.SetLocalTransform(Matrix44Affine(0.0f, 0.0f, -1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -0.0199032f, 2.2256f, 0.0191081f));

    Matrix44Affine tm2(-0.45519f, -0.00190662f, -0.890392f,
        0.0143886f, 0.999851f, -0.00949681f,
        0.890278f, -0.0171344f, -0.455095f,
        73.2937f, 11.5582f, 33.9036f);

    PrimitivePairIntersectResult result;

    RwpBool ok = rw::collision::detail::PrimitivePairIntersect(result, &v1, &tm1, &v2, &tm2, 2.f);

    EATESTAssert(ok, "PrimitivePairIntersect failed.");
    EATESTAssert(Abs(result.distance - 0.729f) < 0.002f, "wrong distance.");
    EATESTAssert(Magnitude(result.normal - Vector3(0.457079f, 0.00187028f, 0.889424f)) < 0.001f, "wrong normal.");
}




void TestCylinderVolume::TestHLSerialization()
{
    CylinderVolume * original = CreateCylinderVolume();
    CylinderVolume * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level serialized copies do not match.");
}



void TestCylinderVolume::TestHLFileSerialization()
{
    CylinderVolume * original = CreateCylinderVolume();
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("cylinder");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    CylinderVolume * copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<CylinderVolume>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level file serialized copies do not match.");
}


#if !defined(RWP_NO_VPU_MATH)

void TestCylinderVolume::TestLLVpuSerialization()
{
    CylinderVolume* original = CreateCylinderVolume();

    CylinderVolume* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu serialized copies do not match.");
}


void TestCylinderVolume::TestLLVpuFileSerialization()
{
    CylinderVolume * original = CreateCylinderVolume();
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("cylinder");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    CylinderVolume* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<CylinderVolume>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu file serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)


#if 0 // not implemented

void TestCylinderVolume::TestLLFpuSerialization()
{
    CylinderVolume* original = CreateCylinderVolume();

#if !defined(RWP_NO_VPU_MATH)
    CylinderVolume* copied = CopyViaLLFpuSerialization<CylinderVolume, rw::collision::detail::fpu::CylinderVolume>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    CylinderVolume* copied = CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}


void TestCylinderVolume::TestLLFpuFileSerialization()
{
    CylinderVolume* original = CreateCylinderVolume();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("cylinder");

#if defined(CREATE_SERIALIZATION_TEST_DATA)
#if !defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<CylinderVolume, rw::collision::detail::fpu::CylinderVolume>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<CylinderVolume>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)
#endif // defined(CREATE_SERIALIZATION_TEST_DATA)

#if !defined(RWP_NO_VPU_MATH)
    CylinderVolume* copied = LoadLLFpuSerializationFromFile<CylinderVolume, rw::collision::detail::fpu::CylinderVolume>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    CylinderVolume* copied = LoadLLFpuSerializationFromFile<CylinderVolume>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}

#endif // not implemented



void TestCylinderVolume::TestCylinderUniformScale()
{
    Matrix44Affine tm(GetMatrix44Affine_Identity());
    tm.Pos().Set(1.0f, 2.0f, 3.0f);

    float scale = 2.0f;

    // Manually scaled
    CylinderVolume* cyl1 = CreateCylinderVolume();
    Matrix44Affine scaledTM(tm);
    scaledTM.Pos() *= scale;
    cyl1->SetLocalTransform(scaledTM);
    cyl1->SetRadius(cyl1->GetRadius() * scale);
    cyl1->SetInnerRadius(cyl1->GetInnerRadius() * scale);
    cyl1->SetHalfHeight(cyl1->GetHalfHeight() * scale);

    // Scale with API
    CylinderVolume* cyl2 = CreateCylinderVolume();
    cyl2->SetLocalTransform(tm);
    cyl2->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*cyl1, *cyl2), "CylinderVolume::ApplyUniformScale does not behave as expected.");

    // Scale with API - virtual
    CylinderVolume* cyl3 = CreateCylinderVolume();
    cyl3->SetLocalTransform(tm);
    static_cast<Volume*>(cyl3)->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*cyl1, *cyl3), "Volume::ApplyUniformScale does not behave as expected on CylinderVolume.");
}



