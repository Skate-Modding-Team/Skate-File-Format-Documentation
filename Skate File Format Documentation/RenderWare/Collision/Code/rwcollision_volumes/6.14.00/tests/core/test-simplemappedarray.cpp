// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/simplemappedarray.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rwpmath;
using namespace rw::collision;



class TestSimpleMappedArray: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestSimpleMappedArray");

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of SimpleMappedArray",
                        TestSimpleMappedArray, TestHLSerialization);
        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of SimpleMappedArray",
                         TestSimpleMappedArray, TestHLFileSerialization);

#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuSerialization", "Low-level serialization of SimpleMappedArray using vpu math",
                        TestSimpleMappedArray, TestLLVpuSerialization);
        EATEST_REGISTER("TestLLVpuFileSerialization", "Low-level file serialization of SimpleMappedArray using vpu math",
                        TestSimpleMappedArray, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of SimpleMappedArray using fpu math",
                        TestSimpleMappedArray, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of SimpleMappedArray using fpu math",
                        TestSimpleMappedArray, TestLLFpuFileSerialization);

        EATEST_REGISTER("TestVolumeIteration", "VolumeIteration of SimpleMappedArray using GetNextVolume",
                        TestSimpleMappedArray, TestVolumeIteration);

        EATEST_REGISTER("TestBBoxQuery", "Calling BBox Query on a SimpleMappedArray",
                        TestSimpleMappedArray, TestBBoxQuery);

        EATEST_REGISTER("TestBBoxQueryOutOfPrimSpace", "Calling BBox Query on a SimpleMappedArray without enough prim ref space",
                        TestSimpleMappedArray, TestBBoxQueryOutOfPrimSpace);

        EATEST_REGISTER("TestBBoxQueryOutOfStackSpace", "Calling BBox Query on a SimpleMappedArray without a big enough query stack",
                        TestSimpleMappedArray, TestBBoxQueryOutOfStackSpace);

        EATEST_REGISTER("TestUniformScale", "Test uniform scaling of simple mapped array",
                        TestSimpleMappedArray, TestUniformScale);
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


    static SimpleMappedArray * CreateSimpleMappedArray()
    {
        SimpleMappedArray * mappedArray = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(1u);
        BoxVolume::Initialize(EA::Physics::MemoryPtr(mappedArray->GetVolume(0u)), 1.0f, 2.0f, 4.0f);
        mappedArray->UpdateThis();
        return mappedArray;
    }

    static bool CompareBoxVolumes(const BoxVolume& original, const BoxVolume& copied)
    {
        Vector3 inDimensions;
        Vector3 outDimensions;
        original.GetDimensions(inDimensions);
        copied.GetDimensions(outDimensions);
        return (inDimensions == outDimensions);
    }

    static bool CompareSimpleMappedArrays(const SimpleMappedArray& original, const SimpleMappedArray& copied)
    {
        const rw::collision::AABBox originalBBox = original.GetBBox();
        const rw::collision::AABBox copiedBBox = copied.GetBBox();
        if (!rwpmath::IsSimilar(originalBBox.m_min, copiedBBox.m_min) ||
            !rwpmath::IsSimilar(originalBBox.m_max, copiedBBox.m_max))
        {
            return false;
        }

        const BoxVolume* originalVol = static_cast<const BoxVolume*>(original.GetVolume(0));
        const BoxVolume* copiedVol   = static_cast<const BoxVolume*>(copied.GetVolume(0));
        if (!originalVol || !copiedVol)
        {
            return false;
        }

        return CompareBoxVolumes(*originalVol, *copiedVol);
    }


private:

    void TestHLSerialization();
    void TestHLFileSerialization();

    void TestLLVpuSerialization();
    void TestLLVpuFileSerialization();

    void TestLLFpuSerialization();
    void TestLLFpuFileSerialization();

    void TestVolumeIteration();

    void TestBBoxQuery();
    void TestBBoxQueryOutOfPrimSpace();
    void TestBBoxQueryOutOfStackSpace();

    void TestUniformScale();

} TestSimpleMappedArraySingleton;




void TestSimpleMappedArray::TestHLSerialization()
{
    SimpleMappedArray * original = CreateSimpleMappedArray();
    SimpleMappedArray * copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(CompareSimpleMappedArrays(*original, *copied), "Original and high-level serialized copies do not match.");
}


void TestSimpleMappedArray::TestHLFileSerialization()
{
    SimpleMappedArray* original = CreateSimpleMappedArray();
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("simplemappedarray");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    SimpleMappedArray * copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<SimpleMappedArray>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(CompareSimpleMappedArrays(*original, *copied), "Original and high-level file serialized copies do not match.");
}


#if !defined(RWP_NO_VPU_MATH)

void TestSimpleMappedArray::TestLLVpuSerialization()
{
    SimpleMappedArray* original = CreateSimpleMappedArray();

    SimpleMappedArray* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(CompareSimpleMappedArrays(*original, *copied), "Original and low-level vpu serialized copies do not match.");
}


void TestSimpleMappedArray::TestLLVpuFileSerialization()
{
    SimpleMappedArray* original = CreateSimpleMappedArray();
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("simplemappedarray");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    SimpleMappedArray* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<SimpleMappedArray>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(CompareSimpleMappedArrays(*original, *copied), "Original and low-level vpu file serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)



void TestSimpleMappedArray::TestLLFpuSerialization()
{
    SimpleMappedArray * original = CreateSimpleMappedArray();

#if !defined(RWP_NO_VPU_MATH)
    SimpleMappedArray* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization<SimpleMappedArray, rw::collision::detail::fpu::SimpleMappedArray>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    SimpleMappedArray* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(CompareSimpleMappedArrays(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}


void TestSimpleMappedArray::TestLLFpuFileSerialization()
{
    SimpleMappedArray* original = CreateSimpleMappedArray();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("simplemappedarray");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<SimpleMappedArray, rw::collision::detail::fpu::SimpleMappedArray>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<SimpleMappedArray>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    SimpleMappedArray* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<SimpleMappedArray, rw::collision::detail::fpu::SimpleMappedArray>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    SimpleMappedArray* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<SimpleMappedArray>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(CompareSimpleMappedArrays(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}


void TestSimpleMappedArray::TestVolumeIteration()
{
    // Create a simple mapped array
    const uint32_t numVolumes(16);
    SimpleMappedArray * simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount(); ++i)
    {
        rw::collision::SphereVolume::Initialize(&simpleMappedArray->GetVolumeArray()[i], static_cast<float>(i));
    }

    // Iterate the simple mapped arrays volumes
    uint32_t countedVolumes(0);
    SimpleMappedArray::VolumeWalker volumeWalker(&*simpleMappedArray);
    while (!volumeWalker.Finished())
    {
        EATESTAssert(volumeWalker.IsValid(), "VolumeIterator is not valid");
        EATESTAssert(VOLUMETYPESPHERE == volumeWalker->GetType(), "Volume is not of type VOLUMETYPESPHERE");
        EATESTAssert(static_cast<float>(countedVolumes) == volumeWalker->GetRadius(), "Volumes radius is incorrect");

        ++countedVolumes;
        ++volumeWalker;
    }

    EATESTAssert(numVolumes == countedVolumes, "Volume counts do not match");
    EATESTAssert(volumeWalker.IsValid(), "VolumeIterator is valid but should be invalid");
}

void TestSimpleMappedArray::TestBBoxQuery()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 5;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Create a simple mapped array
    const uint32_t numVolumes(4);
    SimpleMappedArray * simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount(); ++i)
    {
        SphereVolume::Initialize(&simpleMappedArray->GetVolumeArray()[i], static_cast<float>(i));
    }

    simpleMappedArray->Update();

    AggregateVolume * volume(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(&*simpleMappedArray));

    const Volume *volArray[1] = { &*volume };

    rwpmath::Matrix44Affine identityMatrix = rwpmath::GetMatrix44Affine_Identity();
    rw::collision::AABBox volBBox;
    volume->GetBBox(&identityMatrix, TRUE, volBBox);
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    RwpBool finished = simpleMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);

    EATESTAssert(FALSE != finished, "BBoxOverlapQuery didn't complete when there was enough space to complete");
    EATESTAssert(numVolumes == p_VBBQ->m_primNext, "BBoxOverlapQuery returned the wrong number of results");
}

void TestSimpleMappedArray::TestBBoxQueryOutOfPrimSpace()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 5;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Create a simple mapped array
    const uint32_t numVolumes(12);
    SimpleMappedArray * simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount(); ++i)
    {
        SphereVolume::Initialize(&simpleMappedArray->GetVolumeArray()[i], static_cast<float>(i));
    }

    simpleMappedArray->Update();

    AggregateVolume * volume(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(&*simpleMappedArray));

    const Volume *volArray[1] = { &*volume };

    rwpmath::Matrix44Affine identityMatrix = rwpmath::GetMatrix44Affine_Identity();
    rw::collision::AABBox volBBox;
    volume->GetBBox(&identityMatrix, TRUE, volBBox);
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    uint32_t numResults = 0;

    RwpBool finished = simpleMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_primNext == RESBUFFERSIZE, "BBoxOverlapQuery returned the wrong number of results");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE,
                 "BBoxOverlapQuery didn't not flag result buffer overflow");
    numResults += p_VBBQ->m_primNext;
    p_VBBQ->m_primNext = 0;

    finished = simpleMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_primNext == RESBUFFERSIZE, "BBoxOverlapQuery returned the wrong number of results");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE,
                 "BBoxOverlapQuery didn't not flag result buffer overflow");
    numResults += p_VBBQ->m_primNext;
    p_VBBQ->m_primNext = 0;

    finished = simpleMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE != finished, "BBoxOverlapQuery hasn't completed when it should have");
    numResults += p_VBBQ->m_primNext;
    EATESTAssert(numVolumes == numResults, "BBoxOverlapQuery returned incorrect total number of results");

}

void TestSimpleMappedArray::TestBBoxQueryOutOfStackSpace()
{
    const uint32_t STACKSIZE = 5;
    const uint32_t RESBUFFERSIZE = 1;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Create a very basic simple mapped array to instance into the mapped array of aggregates
    SimpleMappedArray * embeddedMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(1u));
    SphereVolume::Initialize(&embeddedMappedArray->GetVolumeArray()[0], 1.0f);
    embeddedMappedArray->Update();

    // Create a simple mapped array of aggregates
    const uint32_t numVolumes(12);
    SimpleMappedArray * simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount(); ++i)
    {
        AggregateVolume::Initialize(EA::Physics::MemoryPtr(&simpleMappedArray->GetVolumeArray()[i]), &*embeddedMappedArray);
    }
    simpleMappedArray->Update();

    AggregateVolume * volume(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(&*simpleMappedArray));

    const Volume *volArray[1] = { &*volume };

    rwpmath::Matrix44Affine identityMatrix = rwpmath::GetMatrix44Affine_Identity();
    rw::collision::AABBox volBBox;
    volume->GetBBox(&identityMatrix, TRUE, volBBox);
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    uint32_t numResults = 0;

    RwpBool finished = simpleMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_stackNext == STACKSIZE, "BBoxOverlapQuery has the wrong number of entries on the stack");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE,
                 "BBoxOverlapQuery didn't not flag stack overflow");
    numResults += p_VBBQ->m_stackNext;
    p_VBBQ->m_stackNext = 0;

    finished = simpleMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_stackNext == STACKSIZE, "BBoxOverlapQuery has the wrong number of entries on the stack");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE,
                 "BBoxOverlapQuery didn't not flag stack overflow");
    numResults += p_VBBQ->m_stackNext;
    p_VBBQ->m_stackNext = 0;

    finished = simpleMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE != finished, "BBoxOverlapQuery hasn't completed when it should have");
    numResults += p_VBBQ->m_stackNext;
    EATESTAssert(numVolumes == numResults, "BBoxOverlapQuery returned incorrect total number of results");

}


void TestSimpleMappedArray::TestUniformScale()
{
    EA::Allocator::ICoreAllocator *alloc = EA::Allocator::ICoreAllocator::GetDefaultAllocator();

    Matrix44Affine mtx(GetMatrix44Affine_Identity());

    SimpleMappedArray* sma1 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>(*alloc).New(2u);
    SphereVolume *sph1 = SphereVolume::Initialize(sma1->GetVolume(0u), 1.0f);
    mtx = sph1->GetLocalTransform();
    mtx.Pos().X() = 1.0f;
    sph1->SetLocalTransform(mtx);
    SphereVolume *sph2 = SphereVolume::Initialize(sma1->GetVolume(1u), 2.0f);
    mtx = sph2->GetLocalTransform();
    mtx.Pos().Y() = 2.0f;
    sph2->SetLocalTransform(mtx);
    sma1->Update();
    rw::collision::AABBox bbox1 = sma1->GetBBox();

    // Two instances of sma1 and another sphere
    SimpleMappedArray* sma2 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>(*alloc).New(3u);
    AggregateVolume *agg1 = AggregateVolume::Initialize(sma2->GetVolume(0u), sma1);
    mtx = agg1->GetLocalTransform();
    mtx.Pos().Z() = 3.0f;
    agg1->SetLocalTransform(mtx);
    AggregateVolume *agg2 = AggregateVolume::Initialize(sma2->GetVolume(1u), sma1);
    mtx = agg2->GetLocalTransform();
    mtx.Pos().X() = -1.0f;
    agg2->SetLocalTransform(mtx);
    SphereVolume *sph3 = SphereVolume::Initialize(sma2->GetVolume(2u), 3.0f);
    mtx = sph3->GetLocalTransform();
    mtx.Pos().Y() = -2.0f;
    sph3->SetLocalTransform(mtx);
    sma2->Update();
    rw::collision::AABBox bbox2 = sma2->GetBBox();

    // Two instances of sma2 and another sphere
    SimpleMappedArray* sma3 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>(*alloc).New(3u);
    AggregateVolume *agg3 = AggregateVolume::Initialize(sma3->GetVolume(0u), sma2);
    mtx = agg3->GetLocalTransform();
    mtx.Pos().Z() = -3.0f;
    agg3->SetLocalTransform(mtx);
    AggregateVolume *agg4 = AggregateVolume::Initialize(sma3->GetVolume(1u), sma2);
    mtx = agg4->GetLocalTransform();
    mtx.Pos().X() = 1.0f;
    agg4->SetLocalTransform(mtx);
    SphereVolume *sph4 = SphereVolume::Initialize(sma3->GetVolume(2u), 4.0f);
    mtx = sph4->GetLocalTransform();
    mtx.Pos().Y() = 2.0f;
    sph4->SetLocalTransform(mtx);
    sma3->Update();

    // Try and trip up the scaling function with some "dirty" processing flags
    sma3->ClearAllProcessedFlags();
    sph4->SetProcessedFlag();

    const float scale = 2.0f;
    // Test scaling of everything - except those things marked as processed
    sma3->ApplyUniformScale(scale, true);

    EATESTAssert(IsSimilar(sph1->GetRadius(), 1.0f*scale), "SimpleMappedArray::ApplyUniformScale failed");
    EATESTAssert(IsSimilar(sph2->GetRadius(), 2.0f*scale), "SimpleMappedArray::ApplyUniformScale failed");
    EATESTAssert(IsSimilar(sph3->GetRadius(), 3.0f*scale), "SimpleMappedArray::ApplyUniformScale failed");
    // "sph4" radius should be original radius as processed flag was set prior to scale operation
    EATESTAssert(IsSimilar(sph4->GetRadius(), 4.0f), "SimpleMappedArray::ApplyUniformScale failed");
    EATESTAssert(IsSimilar(sma1->GetBBox().Min(), bbox1.Min()*scale),"SimpleMappedArray::ApplyUniformScale failed");
    EATESTAssert(IsSimilar(sma1->GetBBox().Max(), bbox1.Max()*scale),"SimpleMappedArray::ApplyUniformScale failed");
    EATESTAssert(IsSimilar(sma2->GetBBox().Min(), bbox2.Min()*scale),"SimpleMappedArray::ApplyUniformScale failed");
    EATESTAssert(IsSimilar(sma2->GetBBox().Max(), bbox2.Max()*scale),"SimpleMappedArray::ApplyUniformScale failed");

    // The following tests have been removed as they do not account for the fact that a child of the aggregate volume
    // i.e. sph4 was not scaled because its process flag was set before the scale function was used
    //EATESTAssert(IsSimilar(sma3->GetBBox().Min(), bbox3.Min()*scale),"SimpleMappedArray::ApplyUniformScale failed");
    //EATESTAssert(IsSimilar(sma3->GetBBox().Max(), bbox3.Max()*scale),"SimpleMappedArray::ApplyUniformScale failed");

    // Test scaling of only components marked un-processed (sph1 in this case)
    const bool respectProcessingFlags = true;
    sma1->ClearAllProcessedFlags();
    sph2->SetProcessedFlag();
    sma1->ApplyUniformScale(scale, respectProcessingFlags);
    EATESTAssert(IsSimilar(sph1->GetRadius(), 1.0f*scale*scale), "SimpleMappedArray::ApplyUniformScale failed");
    EATESTAssert(IsSimilar(sph2->GetRadius(), 2.0f*scale), "SimpleMappedArray::ApplyUniformScale failed");

    alloc->Free(sma3);
    alloc->Free(sma2);
    alloc->Free(sma1);
}
