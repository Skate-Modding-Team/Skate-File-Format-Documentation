// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREE_BB_QUERY_BASE_H
#define PUBLIC_RW_COLLISION_KDTREE_BB_QUERY_BASE_H

/*************************************************************************************************************

File: kdtreebboxquerybase.h

Purpose: Base class for KDTree AABB queries.

*/

#include "rw/collision/common.h"
#include "rw/collision/kdtreebase.h"

namespace rw
{
namespace collision
{
 // *******************************************************************************************************
//                                      KDTree::BBoxQuery CLASS
// *******************************************************************************************************

/**
The base class for all bounding box queries that operate on a kd tree
*/
class KDTreeBBoxQueryBase
{
public:
    KDTreeBBoxQueryBase();
    KDTreeBBoxQueryBase(const KDTreeBase *kdtree, const AABBox &bbox,  const uint32_t branchIndexOffset = 0, const uint32_t defaultEntry = 0);

    /// Access the copy of the bounding passed to the constructor
    const AABBox & GetBBox() const;

protected:

    void ProcessBranchNode();

private:

    AABBox          m_bbox;                          ///<The AABB to query against
    AABBoxU         m_bboxFpu;                       ///<The AABB to query against in Fpu form

protected:

    const KDTreeBase *m_kdtree;                      ///<KDTree we're querying
    uint32_t        m_stack[rwcKDTREE_STACK_SIZE];   ///< Stack for recursive tree traversal
    uint32_t        m_top;                           ///< The index of the top of the stack
    uint32_t        m_branchIndexOffset;             ///< Start offset into branchnode array

    // Contiguous block of results from leaf node
    uint32_t        m_resultCount;                   ///< The number of results
    uint32_t        m_nextEntry;                     ///< The next result

};

/**
Construct a KDTree bounding box query.
*/
RW_COLLISION_FORCE_INLINE KDTreeBBoxQueryBase::KDTreeBBoxQueryBase():m_kdtree(0)
{
}



/**

Construct a KDTree bounding box query.

 The next result may be obtained using the GetNext method.
\param kdtree Spatial map to be queried
\param bbox The query bounding box
\see KDTree::BBoxQuery::GetNext
*/
RW_COLLISION_FORCE_INLINE
KDTreeBBoxQueryBase::KDTreeBBoxQueryBase(const KDTreeBase *kdtree,
                                         const AABBox &bbox,
                                         const uint32_t branchIndexOffset /*=0*/,
                                         const uint32_t defaultEntry /*=0*/)
                                         : m_bbox(bbox),
                                         m_kdtree(kdtree),
                                         m_branchIndexOffset(branchIndexOffset),
                                         m_resultCount(0),
                                         m_nextEntry(defaultEntry)
{
    if (kdtree->m_numBranchNodes > 0)
    {
        m_stack[0] = branchIndexOffset; // Start at root
        m_top = 1;
    }
    else
    {
        // Treat as single leaf
        m_resultCount = kdtree->m_numEntries;
        m_top = 0;
    }
    // Keep a copy of the bounding box as floats for fast access in ProcessBranchNode().
    // We could replace the original m_bbox, but this might break existing code which accesses it.
    // We would ideally access these floats from the original, but this requires a union within the AABBox.
    // We don't strictly need to keep this copy if we're not using VPU_MATH, but its simpler to do so.
    m_bboxFpu.m_min = static_cast<rw::math::fpu::Vector3>(bbox.Min());
    m_bboxFpu.m_max = static_cast<rw::math::fpu::Vector3>(bbox.Max());
}

RW_COLLISION_FORCE_INLINE
const AABBox & KDTreeBBoxQueryBase::GetBBox() const
{
    return m_bbox;
}



/**
\internal

\brief
Process node at top of stack. Branch nodes are pushed onto the stack, and
leaf nodes are added to the results set.

This works slightly differently to the line query because line has a
direction and the query tries to process the nearest branch first. That's
not true here where we can sometimes join two leaf nodes as a single
slice of the entry array (left followed by right).

\note
This implementation has been optimized to reduce the number of load hit stores on Xenon that
occur when calling this method in a tight loop, mainly due to the obvious implementation
writing results back into member variables and then needing to read them back in.
We use floating point comparison, avoiding costs converting to vector registers.
A significant performance improvement would be to go to fixed point.

See http://docs.ea.com/RWPhysicsDev:KDBtree#Reducing_Load_Hit_Stores_during_existing_traversal
for a further step that hasn't been performed to use parameters rather than member variables altogether
because it made the calling code, GetNext(), significantly more complex.

Also, see http://docs.ea.com/RWPhysicsDev:KDBtree#Reducing_branching_during_existing_traversal 
for a semi-branchless variant of this, that also had little benefit.
*/
RW_COLLISION_FORCE_INLINE void
KDTreeBBoxQueryBase::ProcessBranchNode()
{
    EA_ASSERT(m_kdtree);
    EA_ASSERT(m_top > 0);

    // Writes to the stack won't alias with reads from the node array
    uint32_t * EA_RESTRICT stack = m_stack;
    const KDTreeBase::BranchNode * EA_RESTRICT nodes = m_kdtree->m_branchNodes;

    uint32_t top = m_top;
    const KDTreeBase::BranchNode & node = nodes[stack[--top] - m_branchIndexOffset];
    uint32_t resultCount = 0;
    uint32_t nextEntry = m_nextEntry;
    const int axis = int(node.m_axis);

    // Right child
    const float max_axis = m_bboxFpu.Max().GetComponent(axis);
	EA_ASSERT(rwpmath::GetFloat(m_bbox.m_max.GetComponent(axis)) == max_axis); // check m_bbox hasn't been changed
    if (max_axis >= node.m_extents[1])
    {
        if (rwcKDTREE_BRANCH_NODE == node.m_childRefs[1].m_content)
        {
            EA_ASSERT(top < rwcKDTREE_STACK_SIZE);
            stack[top++] = node.m_childRefs[1].m_index;
        }
        else
        {
            resultCount += node.m_childRefs[1].m_content;
            nextEntry = node.m_childRefs[1].m_index;
        }
    }

    // Left child
    const float min_axis = m_bboxFpu.Min().GetComponent(axis);
	EA_ASSERT(rwpmath::GetFloat(m_bbox.m_min.GetComponent(axis)) == min_axis); // check m_bbox hasn't been changed
    if (min_axis <= node.m_extents[0])
    {
        if (rwcKDTREE_BRANCH_NODE == node.m_childRefs[0].m_content)
        {
            EA_ASSERT(top < rwcKDTREE_STACK_SIZE);
            stack[top++] = node.m_childRefs[0].m_index;
        }
        else
        {
            resultCount += node.m_childRefs[0].m_content;
            nextEntry = node.m_childRefs[0].m_index; // Right leaf entries follow on
        }
    }

    // Only update member variables when we've completely finished updating them to avoid LHS.
    m_top = top;
    m_resultCount = resultCount;
    m_nextEntry = nextEntry;
}

}
}
#endif
