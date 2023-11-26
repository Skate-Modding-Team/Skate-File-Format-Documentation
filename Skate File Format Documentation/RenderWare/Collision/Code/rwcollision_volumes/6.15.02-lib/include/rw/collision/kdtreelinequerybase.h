// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREE_LINE_QUERY_BASE_H
#define PUBLIC_RW_COLLISION_KDTREE_LINE_QUERY_BASE_H

/*************************************************************************************************************

File: kdtreelinequerybase.h

Purpose: Base class for KDTree line queries.

*/


#include "rw/collision/common.h"

#if defined(RWC_KDTREELINEQUERYBASE_OPT) && (defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU))
#include "rw/math/version.h"
#if RW_MATH_VERSION >= RW_MATH_CREATE_VERSION_NUMBER( 1, 0, 2 )
#include "rw/math/vpu/mask3.h"
#include "rw/math/vpu/mask3_operation.h"
#include "rw/math/vpu/vector_intrinsic_operation.h"
#endif // RW_MATH_VERSION >= RW_MATH_CREATE_VERSION_NUMBER( 1, 0, 2 )
#endif // defined(RWC_KDTREELINEQUERYBASE_OPT) && (defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU))

#include "rw/collision/kdtreebase.h"

namespace rw
{
namespace collision
{


// *******************************************************************************************************
//                                      KDTree::LineQuery CLASS
// *******************************************************************************************************

/**
\brief This is the base class for all line queries that operate on a kd tree
*/

class KDTreeLineQueryBase
{
public:
    KDTreeLineQueryBase(const KDTreeBase *kdtree,
                    rwpmath::Vector3::InParam start,
                    rwpmath::Vector3::InParam end,
                    const float fatness = 0.0f,
                    const uint32_t branchIndexOffset = 0,
                    const uint32_t defaultEntry = 0);
    
    void   ProcessBranchNode();

    /**
    \brief Used to cache tree nodes and relevant line segment parameters for later processing.

    \importlib rwccore
    */
    struct EA_PREFIX_ALIGN(RWMATH_VECTOR4_ALIGNMENT) StackElement
    {
        KDTreeBase::NodeRef     m_nodeRef;
        float   m_pa;
        float   m_pb;
    } EA_POSTFIX_ALIGN(RWMATH_VECTOR4_ALIGNMENT);

    const KDTreeBase       *m_kdtree;                        ///< Spatial map to be queried
    AALineClipper       m_lineClipper;                    ///< Parametric line

    StackElement        m_stack[rwcKDTREE_STACK_SIZE];    ///< Stack for hierarchy traversal 
    uint32_t            m_top;                            ///< next free stack index
    uint32_t            m_branchIndexOffset;              ///< Start offset into branchnode array


    uint32_t            m_leafCount;                    ///< number of entries in the next batch
    uint32_t            m_nextEntry;                    ///< index of the first entry in the next batch
};


/**
\internal
\brief Constructor for a line query.

\param kdtree      The KDTree spatial map to query against.
\param start    Start point of the line.
\param end      End point of the line.
*/
RW_COLLISION_FORCE_INLINE 
KDTreeLineQueryBase::KDTreeLineQueryBase(const KDTreeBase *kdtree,
                                         rwpmath::Vector3::InParam start,
                                         rwpmath::Vector3::InParam end,
                                         const float fatness /* = 0.0f*/,
                                         const uint32_t branchIndexOffset /* = 0*/,
                                         const uint32_t defaultEntry /* = 0*/)
                                         :   m_kdtree(kdtree),
                                         m_lineClipper(start, end, rwpmath::Vector3(fatness, fatness, fatness), kdtree->m_bbox),
                                         m_branchIndexOffset(branchIndexOffset),
                                         m_leafCount(0),
                                         m_nextEntry(defaultEntry)
{
    m_stack[0].m_pa = 0.0f;
    m_stack[0].m_pb = 1.0f;
    if (!m_lineClipper.ClipToAABBox(m_stack[0].m_pa, m_stack[0].m_pb, m_kdtree->m_bbox))
    {
        // Line does not overlap extent of KDTree.
        m_top = 0;
    }
    else if (kdtree->m_numBranchNodes > 0)
    {
        // Start at root
        m_stack[0].m_nodeRef.m_content = rwcKDTREE_BRANCH_NODE;
        m_stack[0].m_nodeRef.m_index = branchIndexOffset;
        m_top = 1;
    }
    else
    {
        // Consider tree as single leaf
        m_leafCount = kdtree->m_numEntries;
        m_top = 0;
    }

}


/**
\internal

Processes current branch node in stack.

This pops the node from the top of the stack, and it \b must be a branch node.  Then this
tests the two children nodes and if the line intersects either, they are push onto the stack.
*/
RW_COLLISION_FORCE_INLINE void
KDTreeLineQueryBase::ProcessBranchNode()
{
#if defined(RWC_KDTREELINEQUERYBASE_OPT) && (defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU))

    const StackElement& cur = m_stack[--m_top];
    EA_ASSERT(cur.m_nodeRef.m_content == rwcKDTREE_BRANCH_NODE);
    KDTree::BranchNode &node = m_kdtree->m_branchNodes[cur.m_nodeRef.m_index-m_branchIndexOffset];

    // Clip to child regions
    // clip the line to (p0, p1) at the extents specifie
    // this should be safe from aliasing as we are only ever reading from node
    const rwpmath::Vector4& nodeVector0 = *(reinterpret_cast<const rwpmath::Vector4*>(&node));
    const rwpmath::Vector4& nodeVector1 = *(reinterpret_cast<const rwpmath::Vector4*>(&node) + 1);
    // this is sensitive to changing the member order in KDTree::BranchNode
    rwpmath::VecFloat extents0 = nodeVector1.Z();
    rwpmath::VecFloat extents1 = nodeVector1.W();
    const rwpmath::Vector4& lineClipperData = m_lineClipper.m_swizzled_data[node.m_axis];
    rwpmath::VecFloat origin = lineClipperData.X();
    rwpmath::VecFloat recip  = lineClipperData.Z();
    rwpmath::VecFloat pad    = lineClipperData.W();
    rwpmath::VecFloat p0 = (extents0 + pad - origin) * recip;
    rwpmath::VecFloat p1 = (extents1 - pad - origin) * recip;

    // determine (pNear, pFar) from (p0, p1) based on distance from start of line
    // ??? can we just do pNear = Min(p0,p1), pFar = Max(p0, p1)
    bool farBranch = (recip > rwpmath::GetVecFloat_Zero());
    rwpmath::MaskScalar farBranchMask(farBranch);
    rwpmath::VecFloat pFar = rwpmath::Select(farBranchMask, p1, p0);
    rwpmath::VecFloat pNear = rwpmath::Select(farBranchMask, p0, p1);

    // the child refs have been extracted from the kdtree node as Vector4s
    // the childRefs0 will be in the z,w coordinate of nodeVector0
    // the childRefs1 will be in the x,y coordinate of nodeVector1
    // todo: swap usage of rwmath_vpu_detail_GetVectorPermuteConstantI for VPL from rwmath 1.0.4
    rwpmath::VectorIntrinsic permuteZ0W0Z0W0 =
        (rwpmath::VectorIntrinsic)rwmath_vpu_detail_GetVectorPermuteConstantI(rw::math::vpu::detail::Z0,
                                                                               rw::math::vpu::detail::W0,
                                                                               rw::math::vpu::detail::Z0,
                                                                               rw::math::vpu::detail::W0);
    rwpmath::Vector4 childRefsVector0 =
        (rw::math::vpu::VectorIntrinsic)rwmath_vpu_detail_VecPerm(nodeVector0, nodeVector0, permuteZ0W0Z0W0);
    rwpmath::Vector4 childRefsVector1 = nodeVector1;
    // determine which set of child refs is the closest/furthest from the start of the line
    rwpmath::Vector4 farElementVector =
        rwpmath::Select(farBranchMask, childRefsVector1, childRefsVector0);
    rwpmath::Vector4 nearElementVector =
        rwpmath::Select(farBranchMask, childRefsVector0, childRefsVector1);

    const rwpmath::Vector4& curVector = reinterpret_cast<const rwpmath::Vector4&>(cur);
    rwpmath::VecFloat pa = curVector.Z();
    rwpmath::VecFloat pb = curVector.W();

    rwpmath::VectorIntrinsic* firstElement  = reinterpret_cast<rwpmath::VectorIntrinsic*>(m_stack+m_top);
    rwpmath::VectorIntrinsic* secondElement = reinterpret_cast<rwpmath::VectorIntrinsic*>(m_stack+m_top+1);

    // update the clipped line points of the far child
    farElementVector.Z() = rwpmath::Max(pa, pFar);
    farElementVector.W() = pb;
    bool farBranchWithinBounds = (pb > pFar);

    // update the clipped line points of the near child
    nearElementVector.Z() = pa;
    nearElementVector.W() = rwpmath::Min(pb, pNear);
    bool nearBranchWithinBounds = (pa < pNear);

    // if the near and far element are within the appropriate bounds we want to place the far element
    // then the near element on the stack
    rwpmath::Vector4 firstElementVector =
        rwpmath::Select(rwpmath::MaskScalar(farBranchWithinBounds), farElementVector, nearElementVector);
    *firstElement = firstElementVector.GetVector();
    *secondElement = nearElementVector.GetVector();
    m_top += farBranchWithinBounds + nearBranchWithinBounds;

#else

    uint32_t top = m_top-1;
    StackElement * EA_RESTRICT stack = &m_stack[0];

    // Load the current stack register into registers to avoid load-hit-stores on Xenon.
    const StackElement & cur = stack[top];
    const float pa = cur.m_pa;
    const float pb = cur.m_pb;
    const uint32_t index = cur.m_nodeRef.m_index - m_branchIndexOffset;
    EA_ASSERT(cur.m_nodeRef.m_content == rwcKDTREE_BRANCH_NODE);
    const KDTreeBase::BranchNode &node = m_kdtree->m_branchNodes[index];

    const int32_t axis = int32_t(node.m_axis);

    // Clip to child regions
    const float origin = m_lineClipper.m_origin.GetComponent(axis);
    const float pad    = m_lineClipper.m_padding.GetComponent(axis);
    const float recip  = m_lineClipper.m_recip.GetComponent(axis);
    const float p0 = (node.m_extents[0] + pad - origin) * recip;
    const float p1 = (node.m_extents[1] - pad - origin) * recip;

    const uint32_t farBranch = m_lineClipper.m_farBranch[axis];
    const float pfar  = (farBranch != 0) ? p1 : p0;
    const float pnear = (farBranch != 0) ? p0 : p1;

    if (pb > pfar)                                    // hard-to-avoid branch on fpu comparison
    {
        EA_ASSERT(top < rwcKDTREE_STACK_SIZE);
        stack[top].m_nodeRef = node.m_childRefs[farBranch];
        stack[top].m_pa = rwpmath::Max(pa, pfar);     // all fpu
        stack[top].m_pb = pb;
        top++;
    }

    const uint32_t nearBranch = uint32_t(!farBranch);
    if (pa < pnear)                                   // hard-to-avoid branch on fpu comparison
    {
        EA_ASSERT(top < rwcKDTREE_STACK_SIZE);
        stack[top].m_nodeRef = node.m_childRefs[nearBranch];
        stack[top].m_pa = pa;
        stack[top].m_pb = rwpmath::Min(pb, pnear);    // all fpu
        top++;
    }

    m_top = top;

#endif
}

}
}
#endif
