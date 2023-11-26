// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/octree.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rwpmath;
using namespace rw::collision;



// Unit tests for serializing Octrees from archives.
// This package relies on preexisting data files, but can generate it's own by setting the define CREATE_SERIALIZATION_TEST_DATA to 1.


class TestOctree: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestOctree");

        EATEST_REGISTER("TestHLFileSerialization",
                        "High-level file serialization (loading only) of Octree (class serialization version 1)",
                        TestOctree, TestHLFileSerialization);

#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuFileSerialization",
                        "Low-level Vpu file serialization (loading only) of Octree (class serialization version 1)",
                        TestOctree, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

        EATEST_REGISTER("TestLLFpuFileSerialization",
                        "Low-level Fpu file serialization (loading only) of Octree (class serialization version 1)",
                        TestOctree, TestLLFpuFileSerialization);
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
    void TestHLFileSerialization();

    void TestLLVpuFileSerialization();

    void TestLLFpuFileSerialization();

    Octree* CreateOctree();
    bool CheckOctree(Octree* testOctree);

} TestOctreeSingleton;



Octree* TestOctree::CreateOctree()
{
    rw::collision::AABBox octreeExtent;
    octreeExtent.m_min=Vector3(0.0f,0.0f,0.0f);
    octreeExtent.m_max=Vector3(20.0f,20.0f,20.0f);
    Octree * testOctree = EA::Physics::UnitFramework::Creator<Octree>().New(1000u,octreeExtent);

    //Populate Octree
    //10x10x10 entries
    uint32_t n=0;
    for(float x=0.0f; x<20.0f;x+=2.0f)
        for(float y=0.0f; y<20.0f;y+=2.0f)
            for(float z=0.0f; z<20.0f;z+=2.0f)
            {
                rw::collision::AABBox entryBBox;
                entryBBox.m_min=Vector3(x,y,z);
                entryBBox.m_max=Vector3(x+1.0f,y+1.0f,z+1.0f);
                testOctree->Insert(n,entryBBox);
                n++;
            }
            return testOctree;
}


bool TestOctree::CheckOctree(Octree* testOctree)
{
    bool valid = true;
    uint32_t n=0;
    for(float x=0.0f; x<20.0f;x+=2.0f)
        for(float y=0.0f; y<20.0f;y+=2.0f)
            for(float z=0.0f; z<20.0f;z+=2.0f)
            {
                const rw::collision::AABBox *entryBBox = testOctree->GetEntryBBox(n);
                valid = valid && (entryBBox->m_min==Vector3(x,y,z));
                valid = valid && (entryBBox->m_max==Vector3(x+1.0f,y+1.0f,z+1.0f));
                n++;
            }    
            return valid;
}


void TestOctree::TestHLFileSerialization()
{
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("octree");

    Octree* testOctree = CreateOctree();    
    EA::Physics::UnitFramework::SaveHLSerializationToFile(*testOctree, filename);

    Octree * copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<Octree>(filename);

    EATESTAssert(copied, "Failed high level file serialization (loading only).");
    EATESTAssert(CheckOctree(copied), "Failed high level file serialization (loading only).");
}


#if !defined(RWP_NO_VPU_MATH)
void TestOctree::TestLLVpuFileSerialization()
{
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("octree");

    Octree* testOctree = CreateOctree();    
    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*testOctree, filename);

    Octree* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<Octree>(filename);

    EATESTAssert(copied, "Failed low level vpu file serialization (loading only).");
    EATESTAssert(CheckOctree(copied), "Failed low level vpu file serialization (loading only).");
}
#endif //!RWP_NO_VPU_MATH


void TestOctree::TestLLFpuFileSerialization()
{
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("octree");

    Octree* testOctree = CreateOctree();    
#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<Octree, rw::collision::detail::fpu::Octree>(*testOctree, filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<Octree>(*testOctree, filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    Octree* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<Octree, rw::collision::detail::fpu::Octree>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    Octree* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<Octree>(filename);
#endif // defined(RWP_NO_VPU_MATH)


    EATESTAssert(copied, "Failed low level fpu file serialization (loading only).");
    EATESTAssert(CheckOctree(copied), "Failed low level fpu level file serialization (loading only).");
}
