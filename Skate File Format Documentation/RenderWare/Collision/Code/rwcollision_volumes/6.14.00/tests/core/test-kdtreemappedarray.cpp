// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/kdtreebuilder.h>
#include <rw/collision/detail/fpu/kdtreemappedarray.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

#include "volumecompare.h"

using namespace rwpmath;
using namespace rw::collision;

class TestKDTreeMappedArray: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestKDTreeMappedArray");

        EATEST_REGISTER("TestHLSerialization", "High-level serialization of KDTreeMappedArray",
                        TestKDTreeMappedArray, TestHLSerialization);
        EATEST_REGISTER("TestHLFileSerialization", "High-level file serialization of KDTreeMappedArray",
                         TestKDTreeMappedArray, TestHLFileSerialization);


#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestLLVpuSerialization", "Low-level serialization of KDTreeMappedArray using vpu math",
                        TestKDTreeMappedArray, TestLLVpuSerialization);
        EATEST_REGISTER("TestLLVpuFileSerialization", "Low-level file serialization of KDTreeMappedArray using vpu math",
                        TestKDTreeMappedArray, TestLLVpuFileSerialization);
#endif // !defined(RWP_NO_VPU_MATH)

        EATEST_REGISTER("TestLLFpuSerialization", "Low-level serialization of KDTreeMappedArray using fpu math",
                        TestKDTreeMappedArray, TestLLFpuSerialization);
        EATEST_REGISTER("TestLLFpuFileSerialization", "Low-level file serialization of KDTreeMappedArray using fpu math",
                        TestKDTreeMappedArray, TestLLFpuFileSerialization);

        EATEST_REGISTER("TestBBoxQuery", "Calling BBox Query on a KDTreeMappedArray",
                        TestKDTreeMappedArray, TestBBoxQuery);

        EATEST_REGISTER("TestBBoxQueryOutOfPrimSpace", "Calling BBox Query on a KDTreeMappedArray without enough prim ref space",
                        TestKDTreeMappedArray, TestBBoxQueryOutOfPrimSpace);

        EATEST_REGISTER("TestBBoxQueryOutOfStackSpace", "Calling BBox Query on a KDTreeMappedArray without a big enough query stack",
                        TestKDTreeMappedArray, TestBBoxQueryOutOfStackSpace);

        EATEST_REGISTER("TestUniformScale", "Test uniform scaling of kdtree mapped array",
                        TestKDTreeMappedArray, TestUniformScale);
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

    // Create KDTree mapped array from a (possibly temporary) input volume array
    static KDTreeMappedArray * CreateKDTreeMappedArray(const Volume *inputVolumes, uint32_t numVolumes)
    {
        EA::Allocator::ICoreAllocator *alloc = EA::Allocator::ICoreAllocator::GetDefaultAllocator();

        uint32_t splitThreshold = 3;

        // Get BBox array
        AABBoxU* bboxes = reinterpret_cast<AABBoxU *>(alloc->Alloc(sizeof(AABBoxU) * numVolumes, 0, 0, AABBoxU::Vector3Type::Alignment)); 
        for (uint32_t i=0; i<numVolumes; ++i)
        {
            rw::collision::AABBox bbox;
            inputVolumes[i].GetBBox(0, true, bbox);
            bboxes[i].m_min = rw::math::fpu::Vector3(bbox.Min());
            bboxes[i].m_max = rw::math::fpu::Vector3(bbox.Max());
        }

        // Build kdtree
        rw::collision::KDTreeBuilder kdTreeBuilder(*alloc);
        kdTreeBuilder.BuildTree(numVolumes, bboxes, splitThreshold);

        // Can now allocate KDTreeMappedArray
        uint32_t numBranchNodes = kdTreeBuilder.GetNumBranchNodes();
        rw::collision::AABBox totalBBox = kdTreeBuilder.GetRootBBox();
        KDTreeMappedArray* kma = EA::Physics::UnitFramework::Creator<KDTreeMappedArray>().New(numVolumes, numBranchNodes, totalBBox);

        // Create the child volumes using order of objects defined by kd tree
        const uint32_t * entryIndices = kdTreeBuilder.GetSortedEntryIndices();
        Volume *kmaVolumes = kma->GetVolumeArray();
        for (uint32_t i = 0; i < numVolumes; ++i)
        {
            kmaVolumes[i] = inputVolumes[entryIndices[i]];
        }

        // Initialize the kdtree
        kdTreeBuilder.InitializeRuntimeKDTree(kma->GetKDTreeMap());
        kma->Update();

        alloc->Free(bboxes);

        return kma;
    }


    static KDTreeMappedArray * CreateKDTreeMappedArray()
    {
        // Create array of boxes
        const uint32_t numVolumes = 5;
        Volume volumes[numVolumes];
        for (uint32_t i=0; i<5; ++i)
        {
            float x = float(i+5);
            Vector3 boxMin(-x, -x, 2.0f*x - 1.0f);
            Vector3 boxMax( x,  x, 2.0f*x + 1.0f);
            Vector3 halfDimensions = GetVecFloat_Half() * (boxMax - boxMin);
            Vector3 center = GetVecFloat_Half() * (boxMax + boxMin);
            BoxVolume *box = BoxVolume::Initialize(volumes + i, halfDimensions);
            Matrix44Affine mtx(box->GetLocalTransform());
            mtx.Pos() = center;
            box->SetLocalTransform(mtx);
        }

        return CreateKDTreeMappedArray(volumes, numVolumes);
    }

    static bool CompareBoxVolumes(const BoxVolume& original, const BoxVolume& copied)
    {
        Vector3 inDimensions;
        Vector3 outDimensions;
        original.GetDimensions(inDimensions);
        copied.GetDimensions(outDimensions);
        return (inDimensions == outDimensions);
    }

    static bool CompareKDTreeMappedArrays(const KDTreeMappedArray& original, const KDTreeMappedArray& copied)
    {
        const rw::collision::AABBox originalBBox = original.GetBBox();
        const rw::collision::AABBox copiedBBox = copied.GetBBox();
        if (!rwpmath::IsSimilar(originalBBox.m_min, copiedBBox.m_min) ||
            !rwpmath::IsSimilar(originalBBox.m_max, copiedBBox.m_max))
        {
            return false;
        }
        
        bool valid = true;
        for(uint16_t i = 0; i<5; i++){
        const BoxVolume* originalVol = static_cast<const BoxVolume*>(original.GetVolume(i));
        const BoxVolume* copiedVol   = static_cast<const BoxVolume*>(copied.GetVolume(i));
        if (!originalVol || !copiedVol)
        {
            return false;
        }

        valid = valid && CompareBoxVolumes(*originalVol, *copiedVol);
        }
        return valid;
    }


private:
    void TestHLSerialization();
    void TestHLFileSerialization();

    void TestLLVpuSerialization();
    void TestLLVpuFileSerialization();

    void TestLLFpuSerialization();
    void TestLLFpuFileSerialization();

    void TestBBoxQuery();
    void TestBBoxQueryOutOfPrimSpace();
    void TestBBoxQueryOutOfStackSpace();

    void TestUniformScale();

} TestKDTreeMappedArraySingleton;





void TestKDTreeMappedArray::TestHLSerialization()
{
    KDTreeMappedArray* original = CreateKDTreeMappedArray();
    KDTreeMappedArray* copied = EA::Physics::UnitFramework::CopyViaHLSerialization(*original);
    EATESTAssert(copied, "Failed copy via high-level serialization.");
    EATESTAssert(CompareKDTreeMappedArrays(*original, *copied), "Original and high-level serialized copies do not match.");
}



void TestKDTreeMappedArray::TestHLFileSerialization()
{
    KDTreeMappedArray* original = CreateKDTreeMappedArray();
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("kdtreemappedarray");

    EA::Physics::UnitFramework::SaveHLSerializationToFile(*original, filename);

    KDTreeMappedArray* copied = EA::Physics::UnitFramework::LoadHLSerializationFromFile<KDTreeMappedArray>(filename);

    EATESTAssert(copied, "Failed copy via high-level file serialization.");
    EATESTAssert(CompareKDTreeMappedArrays(*original, *copied), "Original and high-level file serialized copies do not match.");
}



#if !defined(RWP_NO_VPU_MATH)

void TestKDTreeMappedArray::TestLLVpuSerialization()
{
    KDTreeMappedArray* original = CreateKDTreeMappedArray();

    KDTreeMappedArray* copied = EA::Physics::UnitFramework::CopyViaLLVpuSerialization(*original);

    EATESTAssert(copied, "Failed copy via low-level vpu serialization.");
    EATESTAssert(CompareKDTreeMappedArrays(*original, *copied), "Original and low-level vpu serialized copies do not match.");
}


void TestKDTreeMappedArray::TestLLVpuFileSerialization()
{
    KDTreeMappedArray* original = CreateKDTreeMappedArray();
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("kdtreemappedarray");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*original, filename);

    KDTreeMappedArray* copied = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<KDTreeMappedArray>(filename);

    EATESTAssert(copied, "Failed copy via low-level vpu file serialization.");
    EATESTAssert(CompareKDTreeMappedArrays(*original, *copied), "Original and low-level vpu file serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)



void TestKDTreeMappedArray::TestLLFpuSerialization()
{
    KDTreeMappedArray* original = CreateKDTreeMappedArray();

#if !defined(RWP_NO_VPU_MATH)
    KDTreeMappedArray* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization<KDTreeMappedArray, rw::collision::detail::fpu::KDTreeMappedArray>(*original);
#else // if defined(RWP_NO_VPU_MATH)
    KDTreeMappedArray* copied = EA::Physics::UnitFramework::CopyViaLLFpuSerialization(*original);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu serialization.");
    EATESTAssert(CompareKDTreeMappedArrays(*original, *copied), "Original and low-level fpu serialized copies do not match.");
}


void TestKDTreeMappedArray::TestLLFpuFileSerialization()
{
    KDTreeMappedArray* original = CreateKDTreeMappedArray();
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("kdtreemappedarray");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<KDTreeMappedArray, rw::collision::detail::fpu::KDTreeMappedArray>(*original, filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<KDTreeMappedArray>(*original, filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    KDTreeMappedArray* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<KDTreeMappedArray, rw::collision::detail::fpu::KDTreeMappedArray>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    KDTreeMappedArray* copied = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<KDTreeMappedArray>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(copied, "Failed copy via low-level fpu file serialization.");
    EATESTAssert(CompareKDTreeMappedArrays(*original, *copied), "Original and low-level fpu file serialized copies do not match.");
}


void TestKDTreeMappedArray::TestBBoxQuery()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 5;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Create a kdtree mapped array
    const uint32_t numVolumes(4);

    // Build a set of bboxes to generate our kdtree
    AABBoxU *kdtreeBBoxes = new AABBoxU[numVolumes];
    for (uint32_t i(0); i < numVolumes; ++i)
    {
        float r = static_cast<float>(i);
        kdtreeBBoxes[i].Set(rw::math::fpu::Vector3(-r, -r, -r), rw::math::fpu::Vector3(r, r, r));
    }

    KDTreeBuilder kdtreeBuilder(*EA::Allocator::ICoreAllocator::GetDefaultAllocator());
    kdtreeBuilder.BuildTree(numVolumes, kdtreeBBoxes, numVolumes);

    delete [] kdtreeBBoxes;

    KDTreeMappedArray * kdtreeMappedArray(EA::Physics::UnitFramework::Creator<KDTreeMappedArray>().New(numVolumes, kdtreeBuilder.GetNumBranchNodes(), kdtreeBuilder.GetRootBBox()));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < kdtreeMappedArray->GetVolumeCount(); ++i)
    {
        SphereVolume::Initialize(&kdtreeMappedArray->GetVolumeArray()[i], static_cast<float>(kdtreeBuilder.GetSortedEntryIndices()[i]));
    }

    kdtreeMappedArray->Update();

    AggregateVolume * volume(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(&*kdtreeMappedArray));

    const Volume *volArray[1] = { &*volume };

    rwpmath::Matrix44Affine identityMatrix = rwpmath::GetMatrix44Affine_Identity();
    rw::collision::AABBox volBBox;
    volume->GetBBox(&identityMatrix, TRUE, volBBox);
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    RwpBool finished = kdtreeMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);

    EATESTAssert(FALSE != finished, "BBoxOverlapQuery didn't complete when there was enough space to complete");
    EATESTAssert(numVolumes == p_VBBQ->m_primNext, "BBoxOverlapQuery returned the wrong number of results");
}

void TestKDTreeMappedArray::TestBBoxQueryOutOfPrimSpace()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 5;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Create a kdtree mapped array
    const uint32_t numVolumes(12);

    // Build a set of bboxes to generate our kdtree
    AABBoxU *kdtreeBBoxes = new AABBoxU[numVolumes];
    for (uint32_t i(0); i < numVolumes; ++i)
    {
        float r = static_cast<float>(i);
        kdtreeBBoxes[i].Set(rw::math::fpu::Vector3(-r, -r, -r), rw::math::fpu::Vector3(r, r, r));
    }

    KDTreeBuilder kdtreeBuilder(*EA::Allocator::ICoreAllocator::GetDefaultAllocator());
    kdtreeBuilder.BuildTree(numVolumes, kdtreeBBoxes, numVolumes);

    delete [] kdtreeBBoxes;

    KDTreeMappedArray* kdtreeMappedArray(EA::Physics::UnitFramework::Creator<KDTreeMappedArray>().New(numVolumes, kdtreeBuilder.GetNumBranchNodes(), kdtreeBuilder.GetRootBBox()));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < kdtreeMappedArray->GetVolumeCount(); ++i)
    {
        SphereVolume::Initialize(&kdtreeMappedArray->GetVolumeArray()[i], static_cast<float>(kdtreeBuilder.GetSortedEntryIndices()[i]));
    }

    kdtreeMappedArray->Update();

    AggregateVolume* volume(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(&*kdtreeMappedArray));

    const Volume *volArray[1] = { &*volume };

    rwpmath::Matrix44Affine identityMatrix = rwpmath::GetMatrix44Affine_Identity();
    rw::collision::AABBox volBBox;
    volume->GetBBox(&identityMatrix, TRUE, volBBox);
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    uint32_t numResults = 0;

    RwpBool finished = kdtreeMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_primNext == RESBUFFERSIZE, "BBoxOverlapQuery returned the wrong number of results");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE,
                 "BBoxOverlapQuery didn't not flag result buffer overflow");
    numResults += p_VBBQ->m_primNext;
    p_VBBQ->m_primNext = 0;

    finished = kdtreeMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_primNext == RESBUFFERSIZE, "BBoxOverlapQuery returned the wrong number of results");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE,
                 "BBoxOverlapQuery didn't not flag result buffer overflow");
    numResults += p_VBBQ->m_primNext;
    p_VBBQ->m_primNext = 0;

    finished = kdtreeMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE != finished, "BBoxOverlapQuery hasn't completed when it should have");
    numResults += p_VBBQ->m_primNext;
    EATESTAssert(numVolumes == numResults, "BBoxOverlapQuery returned incorrect total number of results");

}

void TestKDTreeMappedArray::TestBBoxQueryOutOfStackSpace()
{
    const uint32_t STACKSIZE = 5;
    const uint32_t RESBUFFERSIZE = 1;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Create a very basic simple mapped array to instance into the mapped array of aggregates
    SimpleMappedArray* embeddedMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(1u));
    SphereVolume::Initialize(&embeddedMappedArray->GetVolumeArray()[0], 1.0f);
    embeddedMappedArray->Update();

    // Create a kdtree mapped array of aggregates
    const uint32_t numVolumes(12);

    // Build a set of bboxes to generate our kdtree (note all the same since we're cloning the same volume)
    AABBoxU *kdtreeBBoxes = new AABBoxU[numVolumes];
    for (uint32_t i(0); i < numVolumes; ++i)
    {
        float r = 1.0f;
        kdtreeBBoxes[i].Set(rw::math::fpu::Vector3(-r, -r, -r), rw::math::fpu::Vector3(r, r, r));
    }

    KDTreeBuilder kdtreeBuilder(*EA::Allocator::ICoreAllocator::GetDefaultAllocator());
    kdtreeBuilder.BuildTree(numVolumes, kdtreeBBoxes, numVolumes);

    delete [] kdtreeBBoxes;

    KDTreeMappedArray * kdtreeMappedArray(EA::Physics::UnitFramework::Creator<KDTreeMappedArray>().New(numVolumes, kdtreeBuilder.GetNumBranchNodes(), kdtreeBuilder.GetRootBBox()));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < kdtreeMappedArray->GetVolumeCount(); ++i)
    {
        AggregateVolume::Initialize(EA::Physics::MemoryPtr(&kdtreeMappedArray->GetVolumeArray()[i]), &*embeddedMappedArray);
    }

    kdtreeMappedArray->Update();

    AggregateVolume * volume(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(&*kdtreeMappedArray));

    const Volume *volArray[1] = { &*volume };

    rwpmath::Matrix44Affine identityMatrix = rwpmath::GetMatrix44Affine_Identity();
    rw::collision::AABBox volBBox;
    volume->GetBBox(&identityMatrix, TRUE, volBBox);
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    uint32_t numResults = 0;

    RwpBool finished = kdtreeMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_stackNext == STACKSIZE, "BBoxOverlapQuery has the wrong number of entries on the stack");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE,
                 "BBoxOverlapQuery didn't not flag stack overflow");
    numResults += p_VBBQ->m_stackNext;
    p_VBBQ->m_stackNext = 0;

    finished = kdtreeMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE == finished, "BBoxOverlapQuery completed when it shouldn't have");
    EATESTAssert(p_VBBQ->m_stackNext == STACKSIZE, "BBoxOverlapQuery has the wrong number of entries on the stack");
    EATESTAssert(p_VBBQ->GetFlags() & VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE,
                 "BBoxOverlapQuery didn't not flag stack overflow");
    numResults += p_VBBQ->m_stackNext;
    p_VBBQ->m_stackNext = 0;

    finished = kdtreeMappedArray->BBoxOverlapQueryThis(p_VBBQ, &identityMatrix);
    EATESTAssert(FALSE != finished, "BBoxOverlapQuery hasn't completed when it should have");
    numResults += p_VBBQ->m_stackNext;
    EATESTAssert(numVolumes == numResults, "BBoxOverlapQuery returned incorrect total number of results");
}


void TestKDTreeMappedArray::TestUniformScale()
{
    KDTreeMappedArray *kma_ref = CreateKDTreeMappedArray();
    KDTreeMappedArray *kma1 = CreateKDTreeMappedArray();

    // Make temporary volume array of aggregates
    const uint32_t numVolumes = 10;
    Volume vols[numVolumes];
    for (uint32_t i=0; i < numVolumes; ++i)
    {
        AggregateVolume *aggVol = AggregateVolume::Initialize(vols + i, kma1);
        Matrix44Affine mtx(aggVol->GetLocalTransform());
        mtx.Pos().X() = float(10*i);
        aggVol->SetLocalTransform(mtx);
    }

    // Bake into a kdtree mapped array
    KDTreeMappedArray *kma2 = CreateKDTreeMappedArray(vols, numVolumes);
    Vector3 kma2_PreScale_VolPos[numVolumes];
    for (uint32_t i = 0; i < numVolumes; ++i)
    {
        kma2_PreScale_VolPos[i] = kma2->GetVolume(uint16_t(i))->GetLocalTransform().Pos();
    }

    const float scale = 2.0f;
    const rw::collision::AABBox bboxBeforeScale = kma2->GetBBox();

    kma2->ClearAllProcessedFlags();
    kma2->ApplyUniformScale(scale, true);

    KDTree *kdt1 = kma1->GetKDTreeMap();
    KDTree *kdt_ref = kma_ref->GetKDTreeMap();

    EATESTAssert(IsSimilar(kma2->GetBBox().Min(), scale*bboxBeforeScale.Min()), "BBox scaled incorrectly");
    EATESTAssert(IsSimilar(kma2->GetBBox().Max(), scale*bboxBeforeScale.Max()), "BBox scaled incorrectly");

    EATESTAssert(IsSimilar(kma1->GetBBox().Min(), scale*kma_ref->GetBBox().Min()), "BBox scaled incorrectly");
    EATESTAssert(IsSimilar(kma1->GetBBox().Max(), scale*kma_ref->GetBBox().Max()), "BBox scaled incorrectly");

    EATESTAssert(IsSimilar(kdt1->GetBBox().Min(), scale*kdt_ref->GetBBox().Min()), "KDTree bbox scaled incorrectly");
    EATESTAssert(IsSimilar(kdt1->GetBBox().Max(), scale*kdt_ref->GetBBox().Max()), "KDTree bbox scaled incorrectly");

    for (uint32_t ib=0; ib < kdt1->GetNumBranchNodes(); ++ib)
    {
        EATESTAssert(IsSimilar(kdt1->m_branchNodes[ib].m_extents[0], scale*kdt_ref->m_branchNodes[ib].m_extents[0]), "KDTree extents scaled in correctly");
        EATESTAssert(IsSimilar(kdt1->m_branchNodes[ib].m_extents[1], scale*kdt_ref->m_branchNodes[ib].m_extents[1]), "KDTree extents scaled in correctly");
    }

    for (uint32_t i=0; i < kma1->GetVolumeCount(); ++i)
    {
        Volume volume = *kma_ref->GetVolume(uint16_t(i));
        volume.ClearAllProcessedFlags();
        volume.ApplyUniformScale(scale, true);
        EATESTAssert(rw::collision::unittest::IsSimilar(*kma1->GetVolume(uint16_t(i)), volume), "Volumes scaled incorrectly");
    }

    for (uint32_t i=0; i < kma2->GetVolumeCount(); ++i)
    {
        EATESTAssert(IsSimilar(kma2->GetVolume(uint16_t(i))->GetLocalTransform().Pos(), scale*kma2_PreScale_VolPos[i]), "Transforms scaled incorrectly");
    }
}
