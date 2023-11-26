// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: rwckdsubtree.cpp

Purpose: KDSubTree spatial map.

*/

// ***********************************************************************************************************
// Includes

#include <rw/collision/kdsubtree.h>

namespace rw
{
    namespace collision
    {
        /**
        \brief
        Check validity of KDSubTree.

        The KDSubTree is traversed and each branch node is checked to:
        \li ensure the parent index is correct.
        \li ensure the branch nodes are sorted in left-to-right depth first traversal order
        \li ensure the split axis number is valid
        \li ensure the bbox of the child is within the split region defined by the branch
        \li ensure leaf entry indices are sorted in left-to-right traversal order
        \li ensure total leaf entry count it correct
        \li ensure total branch node count is correct

        \return TRUE if KDSubTree is internally consistent.
        */
        RwpBool
            KDSubTree::IsValid() const
        {
            RwpBool isValid = TRUE;

            ValidityCheckNodeData curData;
            curData.parent = 0;
            curData.bbox = m_bbox;

            KDTreeBase::Traversal<ValidityCheckNodeData> traversal(this, curData, m_branchNodeOffset);

            uint32_t leafEntryCountCheck = 0;
            uint32_t lastLeafEntryIndex = 0;
            uint32_t branchIndexCheck = 0;

            while (traversal.PopNode(curData))
            {
                if (traversal.CurrentNodeIsBranch())
                {
                    uint32_t branchIndex = traversal.GetBranchIndex();
                    const KDTreeBase::BranchNode &branch = m_branchNodes[branchIndex];

                    // Check branch index
                    if (branchIndex != branchIndexCheck)
                    {
                        EAPHYSICS_MESSAGE("Branch node index %d is invalid (referenced from node %d).",
                            branchIndex, curData.parent);
                        return FALSE;
                    }
                    branchIndexCheck++;

                    // Parent
                    if (branch.m_parent-m_branchNodeOffset != curData.parent && branchIndex!=0)
                    {
                        EAPHYSICS_MESSAGE("Branch node %d has invalid parent index.", branchIndex);
                        isValid = FALSE;
                    }

                    // Axis
                    if (branch.m_axis > 2)
                    {
                        EAPHYSICS_MESSAGE("Branch node %d has invalid split axis %d.", branchIndex, branch.m_axis);
                        isValid = FALSE;
                    }
                    else
                    {
                        // Check that child extents are contained
                        if(branch.m_childRefs[0].m_content!=0) // LeftChild is not Empty Leaf (Empty leaves appear to have incorrect extents set...)
                        {
                            // Check Left child Extent is within BBox 
                            if (static_cast<float>(curData.bbox.Min().GetComponent((uint16_t)branch.m_axis)) > branch.m_extents[0] &&
                                static_cast<float>(curData.bbox.Max().GetComponent((uint16_t)branch.m_axis)) < branch.m_extents[0])
                            {
                                EAPHYSICS_MESSAGE("Branch node %d does not completely enclose its left child extent.", branchIndex);
                                isValid = FALSE;
                            }
                        }

                        if(branch.m_childRefs[1].m_content!=0) // RightChild is not Empty Leaf (Empty leaves appear to have incorrect extents set...)
                        {
                            // Check Right child Extent is within BBox 
                            if (static_cast<float>(curData.bbox.Min().GetComponent((uint16_t)branch.m_axis)) > branch.m_extents[1] &&
                                static_cast<float>(curData.bbox.Max().GetComponent((uint16_t)branch.m_axis)) < branch.m_extents[1])                                
                            {
                                EAPHYSICS_MESSAGE("Branch node %d does not completely enclose its right child extent.", branchIndex);
                                isValid = FALSE;
                            }
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
                isValid = FALSE;
            }

            return isValid;
        }
    } // namespace collision
} // namespace rw
