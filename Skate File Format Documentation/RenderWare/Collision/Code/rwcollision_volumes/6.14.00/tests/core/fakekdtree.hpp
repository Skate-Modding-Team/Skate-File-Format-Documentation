// (c) Electronic Arts. All Rights Reserved.
#if !defined(RW_COLLISION_UNITTEST_FAKEKDTREE_H)
#define RW_COLLISION_UNITTEST_FAKEKDTREE_H

#include <coreallocator/icoreallocator_interface.h>

namespace rw
{
namespace collision
{
namespace unittest
{

// This class mirrors KDTree but allows the members to be modified by users.
// This class is required because KDTrees cannot be created at run-time.
// IMPORTANT: any changes to the KDTree layout must be mirrored here to keep the unittests working
struct FakeKDTree
{
    struct NodeRef
    {
        uint32_t    m_content;
        uint32_t    m_index;
    };

    struct BranchNode
    {
        uint32_t    m_parent;
        uint32_t    m_axis;
        NodeRef     m_childRefs[2];
        float       m_extents[2];
    };

    BranchNode *          m_branchNodes;
    uint32_t              m_numBranchNodes;
    uint32_t              m_numEntries;
    rw::collision::AABBox m_bbox;
}
;

// create a kdtree within a unit bbox centred on the origin of width 0.5
// this kd tree has only a single leaf node
inline rw::collision::KDTree*
GetKDTreeWithNoBranchNodes()
{
    void* fakeKdtreeBuffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sizeof(FakeKDTree), 0, 0, 16);
    FakeKDTree* fakeKdtree = new (fakeKdtreeBuffer) FakeKDTree;
    fakeKdtree->m_bbox = rw::collision::AABBox(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f);
    fakeKdtree->m_numEntries = 1;
    fakeKdtree->m_numBranchNodes = 0;
    fakeKdtree->m_branchNodes = 0;
    rw::collision::KDTree* kdtree = reinterpret_cast<rw::collision::KDTree*>(fakeKdtree);
    return kdtree;
}

// create a kdtree within a unit bbox centred on the origin of width 0.5
// this kd tree has a single branch node with 2 leaf nodes
inline rw::collision::KDTree*
GetKDTreeWithSingleBranchNode()
{
    void* fakeKdtreeBuffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sizeof(FakeKDTree), 0, 0, 16);
    FakeKDTree* fakeKdtree = new (fakeKdtreeBuffer) FakeKDTree;
    fakeKdtree->m_bbox = rw::collision::AABBox(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f);
    fakeKdtree->m_numEntries = 3;
    fakeKdtree->m_numBranchNodes = 1;

    FakeKDTree::BranchNode *branchNodes = new FakeKDTree::BranchNode[1];
    branchNodes[0].m_parent = 0;
    branchNodes[0].m_axis = 0;
    branchNodes[0].m_childRefs[0].m_content = 1;
    branchNodes[0].m_childRefs[0].m_index = 0;
    branchNodes[0].m_childRefs[1].m_content = 2;
    branchNodes[0].m_childRefs[1].m_index = 1;
    branchNodes[0].m_extents[0] = 0.0f;
    branchNodes[0].m_extents[1] = 0.0f;

    fakeKdtree->m_branchNodes = branchNodes;
    KDTree* kdtree = reinterpret_cast<KDTree*>(fakeKdtree);
    return kdtree;
}


// create a kdtree within a unit bbox centred on the origin of width 0.5
// this kd tree has a 3 branch nodes
inline rw::collision::KDTree*
GetKDTreeWithBranchNodes()
{
    void* fakeKdtreeBuffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sizeof(FakeKDTree), 0, 0, 16);
    FakeKDTree* fakeKdtree = new (fakeKdtreeBuffer) FakeKDTree;
    fakeKdtree->m_bbox = rw::collision::AABBox(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f);
    fakeKdtree->m_numEntries = 10;
    fakeKdtree->m_numBranchNodes = 3;

    FakeKDTree::BranchNode *branchNodes = new FakeKDTree::BranchNode[3];

    branchNodes[0].m_parent = 0;
    branchNodes[0].m_axis = 0;
    branchNodes[0].m_childRefs[0].m_content = rwcKDTREE_BRANCH_NODE;
    branchNodes[0].m_childRefs[0].m_index = 1;
    branchNodes[0].m_childRefs[1].m_content = rwcKDTREE_BRANCH_NODE;
    branchNodes[0].m_childRefs[1].m_index = 2;
    branchNodes[0].m_extents[0] = 0.0f;
    branchNodes[0].m_extents[1] = 0.0f;

    branchNodes[1].m_parent = 0;
    branchNodes[1].m_axis = 1;
    branchNodes[1].m_childRefs[0].m_content = 1;
    branchNodes[1].m_childRefs[0].m_index = 0;
    branchNodes[1].m_childRefs[1].m_content = 2;
    branchNodes[1].m_childRefs[1].m_index = 1;
    branchNodes[1].m_extents[0] =  0.0f;
    branchNodes[1].m_extents[1] = -0.1f;

    branchNodes[2].m_parent = 0;
    branchNodes[2].m_axis = 1;
    branchNodes[2].m_childRefs[0].m_content = 3;
    branchNodes[2].m_childRefs[0].m_index = 3;
    branchNodes[2].m_childRefs[1].m_content = 4;
    branchNodes[2].m_childRefs[1].m_index = 6;
    branchNodes[2].m_extents[0] = 0.1f;
    branchNodes[2].m_extents[1] = 0.0f;

    fakeKdtree->m_branchNodes = branchNodes;
    rw::collision::KDTree* kdtree = reinterpret_cast<rw::collision::KDTree*>(fakeKdtree);
    return kdtree;
}

// free the memory used to set up the kdtree
inline void
FreeKDTree(rw::collision::KDTree*& kdtree)
{
    FakeKDTree *fakeKdtree = reinterpret_cast<FakeKDTree*>(kdtree);
    delete [] fakeKdtree->m_branchNodes;
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(fakeKdtree);
    kdtree = 0;
}

// this class holds onto a kdtree and frees it when the class is destructed
class KDTreeHolder
{
public:
    KDTreeHolder(rw::collision::KDTree* kdtree)
     : m_kdtree(kdtree)
    {
    }

    ~KDTreeHolder()
    {
        FreeKDTree(m_kdtree);
    }

private:
    // disallow copy and assignment
    KDTreeHolder(const KDTreeHolder&);
    KDTreeHolder& operator=(const KDTreeHolder&);

    rw::collision::KDTree* m_kdtree;
};


} // namespace rw
} // namespace collision
} // namespace unittest

#endif // RW_COLLISION_UNITTEST_FAKEKDTREE_H
