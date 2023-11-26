// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwckdtree.cpp

 Purpose: KDTree spatial map.

 */

// ***********************************************************************************************************
// Includes

#include "rw/collision/aabbox.h"
#include "rw/collision/kdtree.h"
#include "stdio.h"


using namespace rwpmath;

namespace rw
{
namespace collision
{

/**
\internal

This is an internal structure that is used to associate the parent id and bbox of a kdtree node with it
while traversing the kdtree with a Traversal.
*/
struct ValidityCheckNodeData
{
    uint32_t parent; ///< the index of the parent branch node
    AABBox   bbox;   ///< the bounding box of the node which is the union of the bounding boxes of all its children.
};


// ***********************************************************************************************************
//                                          rw::collision::KDTree CLASS
// ***********************************************************************************************************

/**
\brief Check validity of KDTree.

This is only available in debug library.
The KDTree is traversed and each branch node is checked to:
\li ensure the parent index is correct.
\li ensure the branch nodes are sorted in left-to-right depth first traversal order
\li ensure the split axis number is valid
\li ensure the bbox of the child is within the split region defined by the branch
\li ensure leaf entry indices are sorted in left-to-right traversal order
\li ensure total leaf entry count it correct
\li ensure total branch node count is correct

\return TRUE if KDTree is internally consistent.
 */
RwpBool
KDTree::IsValid() const
{
    // Check for KDSubTree index first
    if (m_numBranchNodes>0 && m_branchNodes[0].m_parent!= 0)
    {
        EAPHYSICS_MESSAGE("KDTree root Branchnode (node %d) is not its parent (node %d)- Could be KDSubTree",
            0, m_branchNodes[0].m_parent);
        printf("KDTree root Branchnode (node %d) is not its parent (node %d)- Could be KDSubTree",
            0, m_branchNodes[0].m_parent);
        return FALSE;
    }

    RwpBool isValid = TRUE;
    
    ValidityCheckNodeData curData;
    curData.parent = 0;
    curData.bbox = m_bbox;

    KDTree::Traversal<ValidityCheckNodeData> traversal(this, curData);

    uint32_t leafEntryCountCheck = 0;
    uint32_t lastLeafEntryIndex = 0;
    uint32_t branchIndexCheck = 0;

    while (traversal.PopNode(curData))
    {
        if (traversal.CurrentNodeIsBranch())
        {
            uint32_t branchIndex = traversal.GetBranchIndex();
            const KDTree::BranchNode &branch = m_branchNodes[branchIndex];

            // Check branch index
            if (branchIndex != branchIndexCheck)
            {
                EAPHYSICS_MESSAGE("Branch node index %d is invalid (referenced from node %d).",
                    branchIndex, curData.parent);
                printf("Branch node index %d is invalid (referenced from node %d).",
                    branchIndex, curData.parent);
                return FALSE;
            }
            branchIndexCheck++;

            // Parent
            if (branch.m_parent != curData.parent)
            {
                EAPHYSICS_MESSAGE("Branch node %d has invalid parent index.", branchIndex);
                printf("Branch node %d has invalid parent index.", branchIndex);
                isValid = FALSE;
            }

            // Axis
            if (branch.m_axis > 2)
            {
                EAPHYSICS_MESSAGE("Branch node %d has invalid split axis %d.", branchIndex, branch.m_axis);
                printf("Branch node %d has invalid split axis %d.", branchIndex, branch.m_axis);
                isValid = FALSE;
            }
            else
            {
                // Check that child extents are contained
                if (static_cast<float>(curData.bbox.Min().GetComponent((uint16_t)branch.m_axis)) >
                        math::Min(branch.m_extents[0], branch.m_extents[1]))
                {
                    EAPHYSICS_MESSAGE("Branch node %d does not completely enclose its left child extent.", branchIndex);
                    printf("Branch node %d does not completely enclose its left child extent.\n", branchIndex);
                    printf("left child extent = %f \n Branch node extent = %f \n", static_cast<float>(curData.bbox.Min().GetComponent((uint16_t)branch.m_axis)), math::Min(branch.m_extents[0], branch.m_extents[1]));                    
                    isValid = FALSE;
                }

                if (static_cast<float>(curData.bbox.Max().GetComponent((uint16_t)branch.m_axis)) <
                        math::Max(branch.m_extents[0], branch.m_extents[1]))
                {
                    EAPHYSICS_MESSAGE("Branch node %d does not completely enclose its right child extent.", branchIndex);
                    printf("Branch node %d does not completely enclose its right child extent.\n", branchIndex);
                    printf("right child extent = %f \n Branch node extent = %f \n", static_cast<float>(curData.bbox.Max().GetComponent((uint16_t)branch.m_axis)), math::Max(branch.m_extents[0], branch.m_extents[1]));                    
                    isValid = FALSE;
                }
            }

            ValidityCheckNodeData childData;
            childData.parent = branchIndex;

            // Push right
            childData.bbox = curData.bbox;
            childData.bbox.m_min.SetComponent((uint16_t)branch.m_axis, branch.m_extents[1]);
            traversal.PushChildNode(1, childData);

            // Push left
            childData.bbox = curData.bbox;
            childData.bbox.m_max.SetComponent((uint16_t)branch.m_axis, branch.m_extents[0]);
            traversal.PushChildNode(0, childData);
        }
        else
        {

            uint32_t first, count;
            traversal.GetLeafNodeEntries(first, count);

            if (count > 0)
            {
                if (!(first >= lastLeafEntryIndex))
                {
                    EAPHYSICS_MESSAGE("Invalid leaf entry index (referenced from node %d).", curData.parent);
                    printf("Invalid leaf entry index (referenced from node %d).", curData.parent);
                    isValid = FALSE;
                }

                // For increasing but non-contiguous indices (eg as in ClusteredMesh)
                // the last entry index will be greater than this, hence the >= test above
                lastLeafEntryIndex = first + count;
                leafEntryCountCheck += count;
            }
        }
    }

    if (leafEntryCountCheck != GetNumEntries())
    {
        EAPHYSICS_MESSAGE("Sum of leaf entry counts does not match actual number of entries");
        printf("Sum of leaf entry counts does not match actual number of entries");
        isValid = FALSE;
    }

    return isValid;
}

} // namespace collision
} // namespace rw
