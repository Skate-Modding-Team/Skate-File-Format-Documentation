// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <rw/collision/libcore.h>
#if 0 // not implemented
#include <rw/collision/detail/fpu/triangle.h>
#endif // not implemented

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

#include "volumecompare.h"

using namespace rwpmath;
using namespace rw::collision;



class TestTriangleVolume: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestTriangleVolume");

        EATEST_REGISTER("TestGetType", "Test Volume::GetType returns correct type", TestTriangleVolume, TestGetType);

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of TriangleVolume",
                        TestTriangleVolume, TestHLSerialization);
        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of TriangleVolume",
                         TestTriangleVolume, TestHLFileSerialization);


#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuSerialization", "Low-level serialization of TriangleVolume using vpu math",
                        TestTriangleVolume, TestLLVpuSerialization);
        EATEST_REGISTER("TestLLVpuFileSerialization", "Low-level file serialization of TriangleVolume using vpu math",
                        TestTriangleVolume, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

#if 0 // not implemented
        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of TriangleVolume using fpu math",
                        TestTriangleVolume, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of TriangleVolume using fpu math",
                        TestTriangleVolume, TestLLFpuFileSerialization);
#endif // not implemented

        EATEST_REGISTER("TestTriangleUniformScale", "Test application of uniform scale to TriangleVolume",
                        TestTriangleVolume, TestTriangleUniformScale);
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


    static TriangleVolume * CreateTriangleVolume()
    {
        return EA::Physics::UnitFramework::Creator<TriangleVolume>().New(Vector3(0.0f, 0.0f, 0.0f),
                                             Vector3(0.0f, 0.0f, 1.0f),
                                             Vector3(1.0f, 0.0f, 0.0f));
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

    void TestTriangleUniformScale();

    void TestGetType()
    {
        const TriangleVolume *volume = CreateTriangleVolume();
        EATESTAssert(rw::collision::VOLUMETYPETRIANGLE == volume->GetType(), "TriangleVolume::GetType() returned incorrect type for triangle");
        EATESTAssert(rw::collision::VOLUMETYPETRIANGLE == static_cast<const Volume *>(volume)->GetType(), "Volume::GetType() returned incorrect type for triangle");
    }

} TestTriangleVolumeSingleton;



void TestTriangleVolume::TestHLSerialization()
{
    TriangleVolume * original = CreateTriangleVolume();
    TriangleVolume * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level serialized copies do not match.");
}



void TestTriangleVolume::TestHLFileSerialization()
{
    TriangleVolume* original = CreateTriangleVolume();
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("triangle");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    TriangleVolume* copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<TriangleVolume>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and high-level file serialized copies do not match.");
}



#if !defined(RWP_NO_VPU_MATH)

void TestTriangleVolume::TestLLVpuSerialization()
{
    TriangleVolume* original = CreateTriangleVolume();

    TriangleVolume* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu serialized copies do not match.");
}


void TestTriangleVolume::TestLLVpuFileSerialization()
{
    TriangleVolume* original = CreateTriangleVolume();
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("triangle");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    TriangleVolume* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<TriangleVolume>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level vpu file serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)



#if 0 // not implemented

void TestTriangleVolume::TestLLFpuSerialization()
{
    TriangleVolume* original = CreateTriangleVolume();

#if !defined(RWP_NO_VPU_MATH)
    TriangleVolume* copied = CopyViaLLFpuSerialization<TriangleVolume, rw::collision::detail::fpu::TriangleVolume>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    TriangleVolume* copied = CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}



void TestTriangleVolume::TestLLFpuFileSerialization()
{
    TriangleVolume* original = CreateTriangleVolume();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("triangle");

#if defined(CREATE_SERIALIZATION_TEST_DATA)
#if !defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<TriangleVolume, rw::collision::detail::fpu::TriangleVolume>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    SaveLLFpuSerializationToFile<TriangleVolume>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)
#endif // defined(CREATE_SERIALIZATION_TEST_DATA)

#if !defined(RWP_NO_VPU_MATH)
    TriangleVolume* copied = LoadLLFpuSerializationFromFile<TriangleVolume, rw::collision::detail::fpu::TriangleVolume>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    TriangleVolume* copied = LoadLLFpuSerializationFromFile<TriangleVolume>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(rw::collision::unittest::IsSimilar(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}


#endif // not implemented


void TestTriangleVolume::TestTriangleUniformScale()
{
    const float scale = 2.0f;

    // Manually scaled
    TriangleVolume* tri1 = CreateTriangleVolume();
    Vector3 triangleVertices[3];
    tri1->GetPoints(triangleVertices[0], triangleVertices[1], triangleVertices[2], NULL);
    tri1->SetPoints(triangleVertices[0] * scale,
                    triangleVertices[1] * scale,
                    triangleVertices[2] * scale);
    tri1->SetRadius(tri1->GetRadius() * scale);

    // Scale with API
    TriangleVolume* tri2 = CreateTriangleVolume();
    tri2->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*tri1, *tri2), "TriangleVolume::ApplyUniformScale does not behave as expected.");

    // NOTE : We have to reset the flags on tri1 at this point as IsSimilar calls the GetNormal method which alters the flags (using const_cast!)
    tri1->SetFlags(tri1->GetFlags() | VOLUMEFLAG_TRIANGLENORMALISDIRTY);

    // Scale with API - virtual
    TriangleVolume* tri3 = CreateTriangleVolume();
    static_cast<Volume*>(tri3)->ApplyUniformScale(scale);

    EATESTAssert(rw::collision::unittest::IsSimilar(*tri1, *tri3), "Volume::ApplyUniformScale does not behave as expected on TriangleVolume.");
}



