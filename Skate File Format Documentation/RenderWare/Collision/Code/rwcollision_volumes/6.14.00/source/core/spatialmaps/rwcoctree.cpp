// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwoctree.cpp

 Purpose: Octree based spatial map.

 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/octree.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{



#define OCTREE_INBOX_GENERATION_COMPONENT ((rwOCTREE_INBOX_SCALE - 1.0f) * 0.5f)
static const Vector3 rwc_OctreeInboxGenerationVector(OCTREE_INBOX_GENERATION_COMPONENT, OCTREE_INBOX_GENERATION_COMPONENT, OCTREE_INBOX_GENERATION_COMPONENT);
 


/**
\internal

\brief

\param v0   First vector.
\param v1   Second vector.

\return Bits 0, 1 and 2 indicate whether the x, y, and z components of the first vector
are greater than those of the second.
*/
/**
\internal

\brief

\param v0   First vector.
\param v1   Second vector.

\return Bits 0, 1 and 2 indicate whether the x, y, and z components of the first vector
are greater than those of the second.
*/
static inline uint32_t
Vector3Gt(Vector3::InParam v0, Vector3::InParam v1)
{
    uint32_t result = (uint32_t)(static_cast<float>(v0.GetX()) > static_cast<float>(v1.GetX())) | ((static_cast<float>(v0.GetY()) > static_cast<float>(v1.GetY())) << 1) | ((static_cast<float>(v0.GetZ()) > static_cast<float>(v1.GetZ())) << 2);

    return result;
}

/**
\internal

\brief
Find overlaps of a box with octree node children.




\param box      Box to test for overlaps.
\param inBox    Inner box representing extents of child octants.

\return         Bits 0-7 are set if box overlaps corresponding child.
*/
static inline uint32_t
NodeBBoxGetChildOverlaps(const AABBox &box, const AABBox &inBox)
{
    uint32_t overlaps = 0xFF;

    if (box.Min().GetX() > inBox.Max().GetX()) 
    {
        overlaps &= 0xAA; // 10101010 - remove lo X overlaps
    }
    else if (box.Max().GetX() < inBox.Min().GetX()) 
    {
        overlaps &= 0x55; // 01010101 - remove hi X overlaps
    }

    if (box.Min().GetY() > inBox.Max().GetY()) 
    {
        overlaps &= 0xCC; // 11001100 - remove lo Y overlaps
    }
    else if (box.Max().GetY() < inBox.Min().GetY()) 
    {
        overlaps &= 0x33; // 00110011 - remove hi Y overlaps
    }

    if (box.Min().GetZ() > inBox.Max().GetZ()) 
    {
        overlaps &= 0xF0; // 11110000 - remove lo Z overlaps
    }
    else if (box.Max().GetZ() < inBox.Min().GetZ()) 
    {
        overlaps &= 0x0F; // 00001111 - remove hi Z overlaps
    }

    return overlaps;
}

/**
\internal

\brief
Get bbox of child in octree node.

\param childBox     Receives resulting child bounding box.
\param nodeBox      Bounding box of node.
\param inBox        Inner box representing extents of child octants.
\param child        Child index. Bits 0,1,2 are set for high X,Y,Z regions.
*/
static inline void
NodeBBoxGetChildBBox(AABBox &childBox, const AABBox &nodeBox, const AABBox &inBox, const uint32_t child)
{
    if (child & 1)
    {
        // Hi X
        childBox.m_min.SetX(inBox.m_min.GetX());
        childBox.m_max.SetX(nodeBox.m_max.GetX());
    }
    else
    {
        childBox.m_min.SetX(nodeBox.m_min.GetX());
        childBox.m_max.SetX(inBox.m_max.GetX());
    }

    if (child & 2)
    {
        // Hi Y
        childBox.m_min.SetY(inBox.m_min.GetY());
        childBox.m_max.SetY(nodeBox.m_max.GetY());}
    else
    {
        childBox.m_min.SetY(nodeBox.m_min.GetY());
        childBox.m_max.SetY(inBox.m_max.GetY());
    }

    if (child & 4)
    {
        // Hi Z
        childBox.m_min.SetZ(inBox.m_min.GetZ());
        childBox.m_max.SetZ(nodeBox.m_max.GetZ());
    }
    else
    {
        childBox.m_min.SetZ(nodeBox.m_min.GetZ());
        childBox.m_max.SetZ(inBox.m_max.GetZ());
    }

}

/**
\internal
\brief
Generate the 'InBox' for a given AABBox, using a constant scale vector.
\param bbox AABBox to generate 'InBox' from.
\return 'InBox' bounding box
*/
static inline AABBox
GenerateInBox(const AABBox &bbox)
{
    Vector3 offset = Mult((bbox.m_max - bbox.m_min), rwc_OctreeInboxGenerationVector);
    AABBox inBox(bbox.m_min - offset, bbox.m_max + offset);

    return inBox;
}

/**
\internal

\brief
Find child in node that contains a bbox.

\param childBox     Receives child bounding box.
\param nodeBox      Nodes bounding box.
\param bbox         BBox to test.

\return             Index of child region (bits 0,1,2 indicate hi X,Y,Z region).
*/
static uint32_t
FindChildContainingBBox(AABBox &childBox, const AABBox &nodeBox, const AABBox &bbox)
{
    AABBox inBox = GenerateInBox(nodeBox);

    // Amounts by which object is inside hi and lo regions
    Vector3 inHi(bbox.Min() - inBox.Min());
    Vector3 inLo(inBox.Max() - bbox.Max());

    // No child if object sticks out of both hi/lo regions on any axis
    if (   ((static_cast<float>(inLo.GetX()) < 0) && (static_cast<float>(inHi.GetX()) < 0)) 
        || ((static_cast<float>(inLo.GetY()) < 0) && (static_cast<float>(inHi.GetY()) < 0))
        || ((static_cast<float>(inLo.GetZ()) < 0) && (static_cast<float>(inHi.GetZ()) < 0)))
    {
        return rwOCTREE_NO_CHILD;
    }

    // Identify correct child region. Sometimes bbox can be completely inside multiple regions, in
    // which case this will return the best region based on the center position of the bbox.
    uint32_t child = Vector3Gt(inHi, inLo);

    NodeBBoxGetChildBBox(childBox, nodeBox, inBox, child);

    return child;
}


/**
\internal

\brief
Tests whether a bounding box is completely contained by any octants of a node. The octants
overlap so some boxes can be contained by more than one.

\param bbox     BBox to be tested.
\param nodeBox  BBox of node.

\return TRUE if bbox can be contained.
*/
static inline RwpBool 
BBoxInCorner(const AABBox &bbox, const AABBox &nodeBox)
{
    AABBox inBox = GenerateInBox(nodeBox);

    // Test whether box is completely inside hi and lo regions for each axis
    uint32_t  inHi = Vector3Gt(bbox.Min(), inBox.Min());
    uint32_t  inLo = Vector3Gt(inBox.Max(), bbox.Max());

    return static_cast<RwpBool>((inHi|inLo) == 7);
}


// ***********************************************************************************************************
//                                                Octree CLASS
// ***********************************************************************************************************

/**
\internal

\brief
Split a leaf and form a new branch node. Redistribute the list of leaf entries to the children of
the new node.




\param iParent          Index of parent node (containing child leaf to be split).
\param iChildOfParent   Index within the parent node of child leaf to split.
\param nodeBBox         Bounding box of the leaf node.
*/
void 
Octree::SplitLeaf(uint32_t   iParent,
                  uint32_t   iChildOfParent,
                  const AABBox &nodeBBox)
{    
    Octree::Node *parent = &m_nodes[iParent];

    // Grab list of entries before overwriting with child node reference
    uint32_t iEntry = parent->LeafEntries(iChildOfParent);

    // Allocate new node
    EA_ASSERT(m_nodeFreeList != rwOCTREE_END_OF_LIST);
    uint32_t iNode = m_nodeFreeList;
    Octree::Node *node = &m_nodes[iNode];
    m_nodeFreeList = node->NextFree();

    // Init new node
    node->Init(iParent, iChildOfParent);
    parent->SetChildNode(iChildOfParent, iNode);

    // Redistribute entries
    while (iEntry != rwOCTREE_END_OF_LIST)
    {
        Octree::Entry *entry = &m_entries[iEntry];

        // Grab next object before inserting this one into new list
        uint32_t iNextEntry = entry->Next();

        // Can entry be pushed into a corner?
        if (entry->CornerFlag())
        {
            uint32_t   iChild;
#if defined(EA_COMPILER_GNUC)
            AABBox     childBBox = nodeBBox;    // avoid potentially unintialized variable warnings from gcc
#else
            AABBox     childBBox;
#endif

            // Find which corner
            iChild = FindChildContainingBBox(childBBox, nodeBBox, m_bboxes[iEntry]);
            EA_ASSERT(iChild != rwOCTREE_NO_CHILD);
            RwpBool inCorner = BBoxInCorner(m_bboxes[iEntry], childBBox);
            AddEntryToLeaf(iEntry, iNode, iChild, inCorner);
        }
        else
        {
            AddEntryToNode(iEntry, iNode);
        }

        iEntry = iNextEntry;
    }    

}


/**
\brief
Insert an entry into the octree with a particular index. The index must not already be in use.
It is up to the caller to manage which indices are free.

\param iEntry   Index of entry to be inserted.
\param bbox     The bounding box of the entry.
*/
void
Octree::Insert(uint32_t iEntry,
               const AABBox &bbox)
{
    EA_ASSERT(iEntry < m_maxEntries);

    // Set stored bbox
    m_bboxes[iEntry] = bbox;

    // Walk down tree and find where this entry belongs
    uint32_t iNode = 0;
    uint32_t iChild = rwOCTREE_NO_CHILD;
    AABBox curBox = m_extent;
    Octree::Node *node = &m_nodes[iNode];

    if (curBox.Contains(m_bboxes[iEntry]))
    {
        AABBox childBox;

        while((iChild = FindChildContainingBBox(childBox, curBox, m_bboxes[iEntry])) != rwOCTREE_NO_CHILD)
        {
            curBox = childBox;
            if (node->ChildIsLeaf(iChild))
            {
                break;
            }

            iNode = node->ChildNode(iChild);
            node = &m_nodes[iNode];
        }
    }
#ifdef EA_DEBUG
    else
    {
        static uint32_t mc=0;
        if (++mc < 20)
        {
            EAPHYSICS_MESSAGE("Performance Warning: Object %u is outside octree bounding box.", iEntry);
        }
    }
#endif

    if (iChild == rwOCTREE_NO_CHILD)
    {
        AddEntryToNode(iEntry, iNode);
    }
    else
    {
        RwpBool inCorner = BBoxInCorner(m_bboxes[iEntry], curBox);
        AddEntryToLeaf(iEntry, iNode, iChild, inCorner);
        if (node->PushCount(iChild) > rwOCTREE_SPLIT_THRESHOLD && m_nodeFreeList != rwOCTREE_END_OF_LIST)
        {
            SplitLeaf(iNode, iChild, curBox);
        }
    }

}


/**
\brief
Remove an entry from the octree.

\param iEntry  Index of entry to be removed.
*/
void 
Octree::Remove(uint32_t iEntry)
{
    EA_ASSERT(iEntry < m_maxEntries);

    uint32_t iNode, iChild;
    uint16_t *ref;

    Octree::Entry *entry = &m_entries[iEntry];

    entry->GetNode(iNode, iChild);

    Octree::Node *node = &m_nodes[iNode];

    // Get list of entries
    if (iChild == rwOCTREE_NO_CHILD)
    {
        ref = &node->m_stuckEntries;
    }
    else
    {
        ref = &node->m_childRefs[iChild];

        if (entry->CornerFlag())
        {
            node->DecPushCount(iChild, this);
        }
    }

    // Search list for ourself
    while (*ref != iEntry)
    {
        EA_ASSERT(*ref != rwOCTREE_END_OF_LIST);
        ref = &m_entries[*ref].m_next;
    }
    
    // Remove ourself from list
    *ref = (uint16_t)m_entries[*ref].Next();

    // Test if node (not root) is now empty
    if ((iNode != 0) && (node->StuckEntries() == rwOCTREE_END_OF_LIST))
    {
        for (uint32_t i=0; i<8; i++)
        {
            if ((!node->ChildIsLeaf(i)) || (node->LeafEntries(i) != rwOCTREE_END_OF_LIST))
            {
                return;
            }
        }

        // Empty - free the node
        m_nodes[node->Parent()].InitLeaf(node->ChildOfParent());
        node->InitFree(m_nodeFreeList);
        m_nodeFreeList = iNode;
    }

}

void Octree::ComputeConstructionMetrics(Octree::Node& node, Octree::ConstructionMetrics& metrics, uint32_t level) const
{
    //Record a found branch node.
    ++metrics.numberBranchNodes;

    //Get the maximum level visited
    metrics.maxLevel = rwpmath::Max(metrics.maxLevel, level);

    //collect number of stuck entries at this level
    uint32_t stuckID =  node.StuckEntries();
    while (stuckID != rwOCTREE_END_OF_LIST)
    {
        ++metrics.numberStuckEntries;
        stuckID = m_entries[stuckID].m_next;
    }

    //collect metrics for each child
    for(uint16_t childIndex = 0; childIndex != 8; childIndex++)
    {
        uint32_t childNodeIndex = node.ChildNode(childIndex);        

        //if the child is a leaf node need to count how many entries it has
        if(node.ChildIsLeaf(childIndex))
        {
            //Record a leaf node found
            ++metrics.numberLeaves;

            //Record empty leaf nodes
            if(childNodeIndex == rwOCTREE_END_OF_LIST)
            {
                ++metrics.numberEmptyLeaves;
            }

            //Record the number of entries in the leaf node
            while (childNodeIndex != rwOCTREE_END_OF_LIST)
            {
                ++metrics.numberLeafEntries;
                childNodeIndex = m_entries[childNodeIndex].m_next;
            }
        }

        //if the child is a node we process the node
        else 
        {            
            //Process metrics for child node           
            ComputeConstructionMetrics(m_nodes[childNodeIndex], metrics, level+1);
        }        
    }
}

/**
\brief
Compute the construction metrics for the octree
\return the computed Octree::ConstructionMetrics.
*/
Octree::ConstructionMetrics Octree::ComputeConstructionMetrics() const
{
    Octree::ConstructionMetrics metrics;

    //Start at the top of the tree,     
    //process child nodes and collect metrics
    ComputeConstructionMetrics(m_nodes[0], metrics, 1);

    return metrics;
}

/**
\brief
Return the memory requirements of an octree container

\param maxEntries    Maximum number of entries in the octree container.

\return the EA::Physics::SizeAndAlignment
*/
EA::Physics::SizeAndAlignment
Octree::GetResourceDescriptor(uint32_t maxEntries, const AABBox &/*extent*/)
{
    EA_ASSERT(maxEntries <= rwOCTREE_MAX_ENTRIES);

    EA_ASSERT(EA::Physics::SizeAlign<uint32_t>(sizeof(AABBox), rwOCTREE_BBOX_ALIGNMENT) == sizeof(AABBox));
    EA_ASSERT(EA::Physics::SizeAlign<uint32_t>(sizeof(Octree::Node), rwOCTREE_NODE_ALIGNMENT) == sizeof(Octree::Node));

    uint32_t size = 0;

    // Base struct
    size += sizeof(Octree);

    // Entry bboxes
    size = EA::Physics::SizeAlign<uint32_t>(size, rwOCTREE_BBOX_ALIGNMENT);
    size += maxEntries * sizeof(AABBox);

    // Nodes
    size = EA::Physics::SizeAlign<uint32_t>(size, rwOCTREE_NODE_ALIGNMENT);
    size += rwOCTREE_MAX_NODES(maxEntries) * sizeof(Octree::Node);

    // Entries
    size += maxEntries * sizeof(Octree::Entry);
    
    return EA::Physics::SizeAndAlignment(size, rwOCTREE_ALIGNMENT);
}

EA::Physics::SizeAndAlignment
Octree::GetResourceDescriptor(const ObjectDescriptor & objDesc)
{
    return Octree::GetResourceDescriptor(objDesc.m_maxEntries, objDesc.m_extent);
}


/**
\brief
Initialize the octree datastructure.

\param resource The EA::Physics::MemoryPtr that the Octree is initialized into.
\param maxEntries Maximum number of entries. This value must be the same as that
passed to the GetSize function.
\param extent The volume of space covered by the far field. (Note this is currently
extended to be a cube internally, for efficient operation.)
*/
Octree*
Octree::Initialize(const EA::Physics::MemoryPtr& resource,
                   uint32_t maxEntries, 
                   const AABBox &extent)
{
    Octree* octree = new (resource.GetMemory()) Octree(maxEntries, extent);
    return octree;
}

Octree*
Octree::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor &objDesc)
{
    Octree* octree = new (resource.GetMemory()) Octree(objDesc.m_maxEntries, objDesc.m_extent);
    return octree;
}

/**
\brief
Destruct the octree datastructure.
*/
void
Octree::Release()
{
}

/**
\brief Initialize the octree datastructure.

\param maxEntries   Maximum number of entries. This value must be the same as that
                    passed to the GetSize function.
\param extent       The volume of space covered by the far field. (Note this is currently
                    extended to be a cube internally, for efficient operation.)
*/
Octree::Octree(uint32_t maxEntries, 
               const AABBox &extent)
{
    EA_ASSERT(maxEntries <= rwOCTREE_MAX_ENTRIES);

    m_maxEntries = maxEntries;
    m_maxNodes = rwOCTREE_MAX_NODES(maxEntries);

    // Setup pointers for entry bboxes, nodes and entry data
    uintptr_t addr = (uintptr_t)this;

    addr += sizeof(Octree);

    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwOCTREE_BBOX_ALIGNMENT);
    m_bboxes = reinterpret_cast<AABBox *>(addr);
    addr += m_maxEntries * sizeof(AABBox);

    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwOCTREE_NODE_ALIGNMENT);
    m_nodes = reinterpret_cast<Octree::Node *>(addr);
    addr += m_maxNodes * sizeof(Octree::Node);

    m_entries = reinterpret_cast<Octree::Entry *>(addr);

    // Force extent to be a cube. This minimizes the likelihood of entries not being completely
    // contained by any octants of a node (despite overlaps) and getting stuck too low down in the tree.
    rwpmath::Vector3 center = (extent.Max() + extent.Min()) * rwpmath::VecFloat(0.5f);
    rwpmath::Vector3 diag   = (extent.Max() - extent.Min()) * rwpmath::VecFloat(0.5f);
    float maxDiag = Max((float)diag.GetX(), (float)diag.GetY(), (float)diag.GetZ());
    diag.Set(maxDiag, maxDiag, maxDiag);
    m_extent.m_min = center - diag;
    m_extent.m_max = center + diag;
        
    // Initialize root of tree
    m_nodes[0].Init(0, 0);

    // Initialize unused nodes
    uint32_t i;
    for (i=1; i<(m_maxNodes-1); i++)
    {
        m_nodes[i].InitFree(i+1);
    }
    m_nodes[m_maxNodes-1].InitFree(rwOCTREE_END_OF_LIST);
    m_nodeFreeList = 1;

}

/**
\brief
Initialize an octree bounding box query. This will return all octree entries that overlap
the bounding box. Use the GetNext method to iterate through the results.

\param octree   Pointer to the octree.
\param bbox     Bounding box region to be queried.
*/
Octree::BBoxQuery::BBoxQuery(const Octree *octree,
                             const AABBox  &bbox)
:   m_octree(octree),
    m_bbox(bbox)
{
    // Add root of tree as first node on stack
    m_stack[0].m_bb = m_octree->m_extent;
    m_stack[0].m_node = 0;
    m_top = 1;

    // Set up iterator in finished state, ready to pop the node off the stack
    m_curResult = -1;
    m_nextEntry = rwOCTREE_END_OF_LIST;

}


/**
\internal

\brief
Process node at top of stack, filling the results buffer ready for iteration over
its leaf child nodes. Child branch nodes will be added to the stack for later processing.
*/
void
Octree::BBoxQuery::ProcessNode()
{
    EA_ASSERT(m_top > 0);

    uint32_t    entryList;

    // Pop stack
    Octree::BBoxQuery::StackElement cur = m_stack[--m_top];
    Octree::Node *node = &m_octree->m_nodes[cur.m_node];

    m_curResult = -1;

    // Objects stuck in node
    if ((entryList = node->StuckEntries()) != rwOCTREE_END_OF_LIST)
    {
        m_results[++m_curResult] = (uint16_t)entryList;
    }

    /* Check overlaps with children */
    AABBox      inBox;
    inBox = GenerateInBox(cur.m_bb);

    uint32_t overlaps = NodeBBoxGetChildOverlaps(m_bbox, inBox);

    for (uint32_t i=0; overlaps; i++, overlaps>>=1)
    {
        if (overlaps & 1)
        {
            if (node->ChildIsLeaf(i))
            {
                if ((entryList = node->LeafEntries(i)) != rwOCTREE_END_OF_LIST)
                {
                    m_results[++m_curResult] = uint16_t(entryList);
                }
            }
            else
            {
                /* Add non-terminal child node to stack */
                EA_ASSERT(m_top < rwOCTREE_NODE_STACK_SIZE);
                NodeBBoxGetChildBBox(m_stack[m_top].m_bb, cur.m_bb, inBox, i);
                m_stack[m_top].m_node = uint16_t(node->ChildNode(i));
                m_top++;
            }
        }
    }

}


/**
\brief
Initialize an octree line query. This can be used to find all entries whose bounding box
intersect the line. Use the GetNext method to find the next result.

\param octree   Pointer to the octree.
\param start    Position of the start of the line.
\param end      Position of the end of the line.
\param fatness  Optional fatness of the line (equivalent to swept box).
 */
Octree::LineQuery::LineQuery(const Octree *octree,
                             rwpmath::Vector3::InParam start,
                             rwpmath::Vector3::InParam end,
                             const float fatness)
:   m_octree(octree),
    m_clipper(start, end, Vector3(fatness, fatness, fatness), octree->m_extent),
    m_recipPad(Abs(Mult(m_clipper.m_padding, m_clipper.m_recip))),
    m_swap(   (uint32_t)((static_cast<float>(m_clipper.m_delta.GetX()) < 0))
            | ((static_cast<float>(m_clipper.m_delta.GetY()) < 0) << 1)
            | ((static_cast<float>(m_clipper.m_delta.GetZ()) < 0) << 2))
{
    /**
    Start at root of octree, with a bounding box transformed to "line space" such that the line 
    goes from (0,0,0) to (1,1,1). We never actually need to know the node box coordinates in world space,
    only that they touch the line. Child boxes are the same fraction of the parent in either space.
    
    Note that this can produce very large/small numbers (10^10) in the case where lines are aligned
    with some axes. This is intentional, and the numbers should never go out of range. Internally, a
    slightly skewed line is used with enough padding so that we never miss any intersections.

    The transformation of the bbox flips some of the octant addresses. The m_swap value encodes this
    information. A child 'i' in 111 space corresponds to child 'i ^ m_swap' in the original space.
    */

    AABBox    bb;
    bb.m_min = Mult((m_octree->m_extent.m_min - rwpmath::Vector3(m_clipper.m_origin)) , rwpmath::Vector3(m_clipper.m_recip));
    bb.m_max = Mult((m_octree->m_extent.m_max - rwpmath::Vector3(m_clipper.m_origin)) , rwpmath::Vector3(m_clipper.m_recip));
    m_stack[0].m_bb.m_min = Min(bb.m_min, bb.m_max);
    m_stack[0].m_bb.m_max = Max(bb.m_min, bb.m_max);
    m_stack[0].m_p[0] = 0.0f;
    m_stack[0].m_p[1] = 1.0f;
    m_stack[0].m_node = 0;
    m_top = 1;

    // Set up iterator in finished state, ready to pop the node off the stack
    m_curResult = -1;
    m_nextEntry = rwOCTREE_END_OF_LIST;

}


/**
\internal

\brief
Process node at top of line query stack, filling the results buffer ready for iteration over
its leaf child nodes. Child branch nodes will be added to the stack for later processing.
*/
void
Octree::LineQuery::ProcessNode()
{
    EA_ASSERT(m_top > 0);

    uint32_t    entryList;

    // Pop stack
    Octree::LineQuery::StackElement cur = m_stack[--m_top];
    Octree::Node *node = &m_octree->m_nodes[cur.m_node];

    m_curResult = -1;

    // Entries stuck in node
    if ((entryList = node->StuckEntries()) != rwOCTREE_END_OF_LIST)
    {
        m_results[++m_curResult].m_node = uint16_t(entryList);
        m_results[m_curResult].m_p[0] = cur.m_p[0];
        m_results[m_curResult].m_p[1] = cur.m_p[1];
    }

    /* Check overlaps with children */
    AABBox inBox = GenerateInBox(cur.m_bb);

    Vector3 padMin = inBox.m_min - m_recipPad;
    Vector3 padMax = inBox.m_max + m_recipPad;

    float pc[8][2];
    
    pc[0][0] = cur.m_p[0];
    pc[1][0] = Max(cur.m_p[0], (float)padMin.GetX());
    pc[2][0] = Max(cur.m_p[0], (float)padMin.GetY());
    pc[4][0] = Max(cur.m_p[0], (float)padMin.GetZ());
    pc[3][0] = Max(pc[1][0], (float)padMin.GetY());
    pc[6][0] = Max(pc[2][0], (float)padMin.GetZ());
    pc[5][0] = Max(pc[4][0], (float)padMin.GetX());
    pc[7][0] = Max(pc[3][0], (float)padMin.GetZ());

    pc[7][1] = cur.m_p[1];
    pc[6][1] = Min(cur.m_p[1], (float)padMax.GetX());
    pc[5][1] = Min(cur.m_p[1], (float)padMax.GetY());
    pc[3][1] = Min(cur.m_p[1], (float)padMax.GetZ());
    pc[4][1] = Min(pc[6][1], (float)padMax.GetY());
    pc[1][1] = Min(pc[5][1], (float)padMax.GetZ());
    pc[2][1] = Min(pc[3][1], (float)padMax.GetX());
    pc[0][1] = Min(pc[4][1], (float)padMax.GetZ());

    for (uint32_t i=0; i<8; i++)
    {
        // Get original child position
        uint32_t child = i ^ m_swap;

        if (pc[i][0] <= pc[i][1])
        {
            if (node->ChildIsLeaf(child))
            {
                if ((entryList = node->LeafEntries(child)) != rwOCTREE_END_OF_LIST)
                {
                    m_results[++m_curResult].m_node = uint16_t(entryList);
                    m_results[m_curResult].m_p[0] = pc[i][0];
                    m_results[m_curResult].m_p[1] = pc[i][1];
                }
            }
            else
            {
                /* Add non-terminal child node to stack */
                EA_ASSERT(m_top < rwOCTREE_NODE_STACK_SIZE);
                NodeBBoxGetChildBBox(m_stack[m_top].m_bb, cur.m_bb, inBox, i);
                m_stack[m_top].m_p[0] = pc[i][0];
                m_stack[m_top].m_p[1] = pc[i][1];
                m_stack[m_top].m_node = uint16_t(node->ChildNode(child));
                m_top++;
            }
        }
    }

}


} // namespace collision
} // namespace rw
