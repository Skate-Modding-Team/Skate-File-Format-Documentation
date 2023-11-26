// (c) Electronic Arts. All Rights Reserved.

#include <rw/collision/kdtreewithsubtrees.h>
#include <rw/collision/detail/fpu/kdtreewithsubtrees.h>

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rwpmath;
using namespace rw::collision;


// Unit tests for KDTreeWithSubTrees.
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.
// The serialization tests do not check the values inside the clustered meshes other than
// relying on the asserted IsValid method called after serialization.

class TestKDTreeWithSubTrees: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestKDTreeWithSubTrees");

#define KDTREE_WITH_SUBTREES_TEST(F, D) EATEST_REGISTER(#F, D, TestKDTreeWithSubTrees, F)

        KDTREE_WITH_SUBTREES_TEST(TestCreate, "Test creating");
        KDTREE_WITH_SUBTREES_TEST(TestCreateFpu, "Test creating fpu version");
        KDTREE_WITH_SUBTREES_TEST(TestKDSubTreeAssignment, "Test assigning KDSubTrees");

        KDTREE_WITH_SUBTREES_TEST(TestHLLoad, "Test High-level serialization (loading only) of KDTreeWithSubTrees");
#if !defined(RWP_NO_VPU_MATH)
        KDTREE_WITH_SUBTREES_TEST(TestLLVpuLoad, "Test Low-level serialization (loading only) of KDTreeWithSubTrees");
#endif // !defined(RWP_NO_VPU_MATH)
        KDTREE_WITH_SUBTREES_TEST(TestLLFpuLoad, "Test Low-level fpu-layout serialization (loading only) of KDTreeWithSubTrees");
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

    void TestCreate();
    void TestCreateFpu();
    void TestKDSubTreeAssignment();

    void TestHLLoad();
    void TestLLVpuLoad();
    void TestLLFpuLoad();

} TestKDTreeWithSubTreesSingleton;


void TestKDTreeWithSubTrees::TestKDSubTreeAssignment()
{
    // Create a kdtree with no subtrees
    uint32_t numBranchNodes = 20;
    KDTreeWithSubTrees::ObjectDescriptor od(numBranchNodes, 0);
    KDTreeWithSubTrees * kdtree = EA::Physics::UnitFramework::Creator<KDTreeWithSubTrees>().New(od);
    EATESTAssert(kdtree->GetNumKDSubTrees() == 0, "Should have no subtrees");

    KDSubTree subtrees[4];
    kdtree->SetKDSubTrees(subtrees, 4);
    EATESTAssert(kdtree->GetNumKDSubTrees() == 4, "Should have 4 subtrees now");
    for (uint32_t c = 0; c < 4; ++c)
    {
        EATESTAssert(&kdtree->GetKDSubTree(c) == &subtrees[c], "Should have each subtree set");
    }
    KDSubTree moresubtrees[4];
    kdtree->SetKDSubTrees(moresubtrees, 4);
    EATESTAssert(kdtree->GetNumKDSubTrees() == 4, "Should have 4 subtrees now");
    for (uint32_t c = 0; c < 4; ++c)
    {
        EATESTAssert(&kdtree->GetKDSubTree(c) == &moresubtrees[c], "Should have each subtree changed");
    }

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(kdtree);
}


void TestKDTreeWithSubTrees::TestHLLoad()
{
    // Load a mesh with subtrees
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("kdtreewithsubtrees");

    KDTreeWithSubTrees* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<KDTreeWithSubTrees>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->GetNumKDSubTrees() > 0, "Should have loaded at least one subtree.");

}


#if !defined(RWP_NO_VPU_MATH)
void TestKDTreeWithSubTrees::TestLLVpuLoad()
{
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("kdtreewithsubtrees");

    KDTreeWithSubTrees* loadedHLTree = EA::Physics::UnitFramework::LoadHLSerializationFromFile<KDTreeWithSubTrees>(UNITTEST_HL_SERIALIZED_DATA_FILE("kdtreewithsubtrees"));
    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*loadedHLTree, filename);

    KDTreeWithSubTrees* loaded = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<KDTreeWithSubTrees>(filename);

    EATESTAssert(loaded, "Failed low level vpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level vpu file serialization (loading only).");
    EATESTAssert(loaded->GetNumKDSubTrees() > 0, "Should have loaded at least one subtree.");
}
#endif // !defined(RWP_NO_VPU_MATH)


void TestKDTreeWithSubTrees::TestLLFpuLoad()
{
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("kdtreewithsubtrees");

    KDTreeWithSubTrees* loadedHLTree = EA::Physics::UnitFramework::LoadHLSerializationFromFile<KDTreeWithSubTrees>(UNITTEST_HL_SERIALIZED_DATA_FILE("kdtreewithsubtrees"));
#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<KDTreeWithSubTrees, rw::collision::detail::fpu::KDTreeWithSubTrees>(*loadedHLTree, filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<KDTreeWithSubTrees>(*loadedHLTree, filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    KDTreeWithSubTrees* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<KDTreeWithSubTrees, rw::collision::detail::fpu::KDTreeWithSubTrees>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    KDTreeWithSubTrees* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<KDTreeWithSubTrees>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(loaded, "Failed low level fpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level fpu file serialization (loading only).");
    EATESTAssert(loaded->GetNumKDSubTrees() > 0, "Should have loaded at least one subtree.");
}


void TestKDTreeWithSubTrees::TestCreate()
{
    // Make these sizes as awkward as possible to stress the alignment code
    uint32_t maxSubtrees = 9;
    uint32_t numBranchNodes = 19;
    rw::collision::AABBox bbox(Vector3(-1.0f, -2.0f, -3.0f), Vector3(1.0f, 0.2f, -1.5f));

    KDTreeWithSubTrees::ObjectDescriptor withoutOD(numBranchNodes, 0);
    EATESTAssert(numBranchNodes == withoutOD.mMaxBranchNodes, "Should have given number of nodes");
    EATESTAssert(0 == withoutOD.mMaxSubTrees, "Should have no subtrees");
    EA::Physics::SizeAndAlignment withoutRD = KDTreeWithSubTrees::GetResourceDescriptor(withoutOD);

    KDTreeWithSubTrees::ObjectDescriptor withOD(numBranchNodes, maxSubtrees);
    EATESTAssert(numBranchNodes == withOD.mMaxBranchNodes, "Should have given number of nodes");
    EATESTAssert(maxSubtrees == withOD.mMaxSubTrees, "Should have expected number of subtrees");
    EA::Physics::SizeAndAlignment withRD = KDTreeWithSubTrees::GetResourceDescriptor(withOD);

    uint32_t withoutSize = withoutRD.GetSize();
    uint32_t withSize = withRD.GetSize();
    EATESTAssert(withSize > withoutSize, "Should be bigger with subtrees");
    EATESTAssert(withSize >= withoutSize + maxSubtrees*sizeof(KDSubTree), "Should be a lot bigger with subtrees");

    {
        const KDTreeWithSubTrees * without = EA::Physics::UnitFramework::Creator<KDTreeWithSubTrees>().New(withoutOD);
        EATESTAssert(without->GetNumKDSubTrees() == 0, "Should have no subtrees");
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free((void*)without);
    }

    {
        const KDTreeWithSubTrees * with = EA::Physics::UnitFramework::Creator<KDTreeWithSubTrees>().New(withOD);
        EATESTAssert(with->GetNumKDSubTrees() == maxSubtrees, "Should have expected number of subtrees");
        for (uint32_t c = 0; c < maxSubtrees; ++c)
        {
            EATESTAssert(with->GetKDSubTree(c).GetNumBranchNodes() == 0, "Each subtree should be empty");
        }

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free((void*)with);
    }
}


void TestKDTreeWithSubTrees::TestCreateFpu()
{
    uint32_t maxSubtrees = 10;
    uint32_t numBranchNodes = 20;
    rw::collision::detail::fpu::AABBox bbox;
    bbox.m_min = rw::math::fpu::Vector3(-1.0f, -2.0f, -3.0f);
    bbox.m_max = rw::math::fpu::Vector3(1.0f, 0.2f, -1.5f);

    rw::collision::detail::fpu::KDTreeWithSubTrees::ObjectDescriptor withoutOD(numBranchNodes, 0);
    EATESTAssert(numBranchNodes == withoutOD.mMaxBranchNodes, "Should have given number of nodes");
    EATESTAssert(0 == withoutOD.mMaxSubTrees, "Should have no subtrees");
    EA::Physics::SizeAndAlignment withoutRD = rw::collision::detail::fpu::KDTreeWithSubTrees::GetResourceDescriptor(withoutOD);

    rw::collision::detail::fpu::KDTreeWithSubTrees::ObjectDescriptor withOD(numBranchNodes, maxSubtrees);
    EATESTAssert(numBranchNodes == withOD.mMaxBranchNodes, "Should have given number of nodes");
    EATESTAssert(maxSubtrees == withOD.mMaxSubTrees, "Should have expected number of subtrees");
    EA::Physics::SizeAndAlignment withRD = rw::collision::detail::fpu::KDTreeWithSubTrees::GetResourceDescriptor(withOD);

    uint32_t withoutSize = withoutRD.GetSize();
    uint32_t withSize = withRD.GetSize();
    EATESTAssert(withSize > withoutSize, "Should be bigger with subtrees");
    EATESTAssert(withSize >= withoutSize + maxSubtrees*sizeof(rw::collision::detail::fpu::KDSubTree), "Should be a lot bigger with subtrees");

    {
        const rw::collision::detail::fpu::KDTreeWithSubTrees * without = EA::Physics::UnitFramework::Creator<rw::collision::detail::fpu::KDTreeWithSubTrees>().New(withoutOD);
        EATESTAssert(without->GetNumKDSubTrees() == 0, "Should have no subtrees");
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free((void*)without);
    }

    {
        const rw::collision::detail::fpu::KDTreeWithSubTrees * with = EA::Physics::UnitFramework::Creator<rw::collision::detail::fpu::KDTreeWithSubTrees>().New(withOD);
        EATESTAssert(with->GetNumKDSubTrees() == maxSubtrees, "Should have expected number of subtrees");
        for (uint32_t c = 0; c < maxSubtrees; ++c)
        {
            // No public accessors to test here since only required for serialization
            // EATESTAssert(with->GetKDSubTree(c).GetNumBranchNodes() == 0, "Each subtree should be empty");
        }
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free((void*)with);
    }
}
