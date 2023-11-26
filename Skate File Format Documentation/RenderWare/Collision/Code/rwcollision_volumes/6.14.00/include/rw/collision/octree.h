// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_OCTREE_HPP
#define PUBLIC_RW_OCTREE_HPP

#include "rw/collision/common.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/aalineclipper.h"

namespace rw
{
    namespace collision
    {
        // forward declare Octree in the rw::collision namespace so that we can
        // use it in the EA_SERIALIZATION_CLASS_VERSION macro
        class Octree;
    } // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::Octree, 1)

namespace rw
{
namespace collision
{

/**
\internal
Note: 0xffff is not a valid entry index (it's used for end of list etc)
*/
#define rwOCTREE_MAX_ENTRIES            0xffff
/**
\internal
*/
#define rwOCTREE_MAX_NODES(maxEntries)  (1 + (maxEntries)/2)
/**
\internal
*/

//Alignment is set at 32 rather than 16 so as to avoid running over two cache lines on PS2

#if !defined(RWP_NO_VPU_MATH)
#define rwOCTREE_ALIGNMENT              32
#else
#define rwOCTREE_ALIGNMENT              4
#endif
/**
\internal
*/
#if !defined(RWP_NO_VPU_MATH)
#define rwOCTREE_NODE_ALIGNMENT         32
#else
#define rwOCTREE_NODE_ALIGNMENT         4
#endif
/**
\internal
*/
#if !defined(RWP_NO_VPU_MATH)
#define rwOCTREE_BBOX_ALIGNMENT         32
#else
#define rwOCTREE_BBOX_ALIGNMENT         4
#endif

/**
\internal
*/
#define rwOCTREE_END_OF_LIST            0xffff
/**
\internal
*/
#define rwOCTREE_NO_CHILD               0xff

/**
\internal
Size of stack for FF octree traversal - allow depth of 30
*/
#define rwOCTREE_NODE_STACK_SIZE        (1+7*30)

/**
\internal
Threshold for splitting octree nodes
*/
#define rwOCTREE_SPLIT_THRESHOLD        3

/**
\brief
Octree node children overlap by a fixed fraction of the node bbox. The extents of the
children are therefore represented by a small box at the center. The size of this
relative to the node box is given by rwOCTREE_INBOX_SCALE.
 */
#define rwOCTREE_INBOX_SCALE            (0.2f)


// ***********************************************************************************************************
//                                               Octree CLASS
// ***********************************************************************************************************

/**
\brief
Dynamic octree based spatial map.

\importlib rwccore
*/
class Octree
{
public:

    // *******************************************************************************************************
    //                                          Octree::ConstructionMetrics STRUCT
    // *******************************************************************************************************
    struct ConstructionMetrics
    {
        uint32_t numberLeaves;                     ///< The number of leaves that are stored
        uint32_t numberBranchNodes;                ///< The number of nodes that are branches (including root node)
        uint32_t numberStuckEntries;               ///< The number of entries stored in branch nodes
        uint32_t numberLeafEntries;                ///< The number of entries stored in leaf nodes
        uint32_t numberEmptyLeaves;                ///< The number of leaves that store no entries                     
        uint32_t maxLevel;                         ///< The maximum depth of the tree (root node is at level = 1) 

        ConstructionMetrics()
            : numberLeaves(0)
            , numberBranchNodes(0)
            , numberStuckEntries(0)
            , numberLeafEntries(0)
            , numberEmptyLeaves(0)          
            , maxLevel(0)        
        {        
        }
    };

    // *******************************************************************************************************
    //                                          Octree::Entry CLASS
    // *******************************************************************************************************

    /**
    \brief Octree entry.

    \importlib rwccore
    */
    struct Entry
    {
        // Group this entry belongs to
        uint16_t m_group;

        // Next entry in list
        uint16_t m_next;

        // Index of octree node we belong to
        uint16_t m_node;

        // Child leaf of node we belong to, or rwOCTREE_NO_CHILD if a stuck entry
        uint8_t  m_child;

        // =1 if we could be pushed into a sub-child
        uint8_t  m_corner;

        /**
        \internal
        */
        void Init(uint32_t node, uint32_t child, RwpBool inCorner)
        {
            m_group = 0;
            m_next = rwOCTREE_END_OF_LIST;
            m_node = uint16_t(node);
            m_child = uint8_t(child);
            m_corner = uint8_t(inCorner);
        }

        /**
        \internal
        */
        void GetNode(uint32_t &node, uint32_t &child) const
        {
            node  = m_node;
            child = m_child;
        }

        /**
        \internal
        */
        RwpBool CornerFlag() const
        {
            return m_corner;
        }

        /**
        \internal
        */
        void SetNext(const uint32_t next)
        {
            m_next = uint16_t(next);
        }

        /**
        \internal
        */
        uint32_t Next() const
        {
            return m_next;
        }

        /**
        \internal
        */
        void SetGroup(const uint32_t group)
        {
            m_group = uint16_t(group);
        }

        /**
        \internal
        */
        uint32_t Group() const
        {
            return m_group;
        }

        // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
        // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
        template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_group);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_next);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_node);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_child);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_corner);
        }
    };

    // *******************************************************************************************************
    //                                          Octree::Node CLASS
    // *******************************************************************************************************

    /**
    \internal
    \brief
    Octree branch node.

    This is 32 bytes in size and aligned to 32 bytes on some platforms to minimize cache misses.
    Children are numbered 0 to 7. Bits 0,1,2 of the child address are set for the high
    X,Y,Z regions respectively.

    \importlib rwccore
    */
    struct EA_PREFIX_ALIGN(rwOCTREE_NODE_ALIGNMENT) Node
    {
        // Index of parent, also used as freelist next.
        uint16_t m_parent;

        // Which child of parent we are (0-7).
        uint16_t m_childOfParent;

        // Start of list of entries that can't be pushed into a child.
        uint16_t m_stuckEntries;

        // Bits 0-7 indicate whether corresponding child is leaf.
        uint16_t m_childTypes;

        // Index of child node, or start of list of leaf entries
        uint16_t m_childRefs[8];

        // Count of number of objects in leaf that could be pushed into child nodes.
        uint8_t  m_pushCounts[8];

        /**
        \internal
        */
        void InitFree(uint32_t next)
        {
            m_parent = uint16_t(next);
        }

        /**
        \internal
        */
        uint32_t NextFree() const
        {
            return m_parent;
        }

        /**
        \internal
        */
        void InitLeaf(uint32_t child)
        {
            m_childRefs[child] = rwOCTREE_END_OF_LIST;
            m_pushCounts[child] = 0;
            m_childTypes = static_cast<uint16_t>(m_childTypes | (1<<child));
        }

        /**
        \internal
        */
        void Init(uint32_t parent, uint32_t childOfParent)
        {
            m_parent = uint16_t(parent);
            m_childOfParent = uint8_t(childOfParent);
            m_stuckEntries = rwOCTREE_END_OF_LIST;
            m_childTypes = 0;
            for (uint32_t i=0; i<8; i++)
            {
                InitLeaf(i);
            }
        }

        /**
        \internal
        */
        uint32_t Parent() const
        {
            return m_parent;
        }

        /**
        \internal
        */
        uint32_t ChildOfParent() const
        {
            return m_childOfParent;
        }

        /**
        \internal
        */
        void SetStuckEntries(uint32_t firstEntry)
        {
            m_stuckEntries = uint16_t(firstEntry);
        }

        /**
        \internal
        */
        uint32_t StuckEntries() const
        {
            return m_stuckEntries;
        }

        /**
        \internal
        */
        void SetLeafEntries(uint32_t child, uint32_t entry)
        {
            m_childRefs[child] = uint16_t(entry);
        }

        /**
        \internal
        */
        uint32_t LeafEntries(uint32_t child) const
        {
            return m_childRefs[child];
        }

        /**
        \internal
        */
        uint32_t PushCount(uint32_t child) const
        {
            return m_pushCounts[child];
        }

        /**
        \internal
        */
        void IncPushCount(uint32_t child)
        {
            if (m_pushCounts[child] < 255)
            {
                m_pushCounts[child]++;
            }
        }

        /**
        \internal
        */
        void DecPushCount(uint32_t child, Octree *tree)
        {
            if (m_pushCounts[child] != 255)
            {
                m_pushCounts[child]--;
                return;
            }

            // Saturated, so do fresh count
            uint32_t count = 0;
            uint32_t i = m_childRefs[child];
            while (i != rwOCTREE_END_OF_LIST && count < 255)
            {
                if (tree->m_entries[i].CornerFlag())
                {
                    count++;
                }
                i = tree->m_entries[i].Next();
            }

            m_pushCounts[child] = uint8_t(count);
        }

        /**
        \internal
        */
        void SetChildNode(uint32_t child, uint32_t node)
        {
            m_childRefs[child] = uint16_t(node);
            m_childTypes = static_cast<uint16_t>(m_childTypes & ~(1<<child));
        }

        /**
        \internal
        */
        uint32_t ChildNode(uint32_t child) const
        {
            return m_childRefs[child];
        }

        /**
        \internal
        */
        RwpBool ChildIsLeaf(uint32_t child) const
        {
            return static_cast<RwpBool>((m_childTypes >> child) & 1);
        }

        // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
        // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
        template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_parent);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_childOfParent);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_stuckEntries);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_childTypes);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_childRefs, 8);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_pushCounts, 8);
        }

    } EA_POSTFIX_ALIGN(rwOCTREE_NODE_ALIGNMENT);

    // *******************************************************************************************************
    //                                          Octree::ObjectDescriptor CLASS
    // *******************************************************************************************************


    struct ObjectDescriptor
    {
        ObjectDescriptor(uint32_t maxEntries,
            const rw::collision::AABBox& extent)
        {
            m_extent = extent;
            m_maxEntries = maxEntries;

        }

        ObjectDescriptor()
        {
            m_maxEntries = 0;
            m_extent = AABBox(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
        }

        uint32_t m_maxEntries;
        rw::collision::AABBox m_extent;

        // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
        // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
        template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_maxEntries);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_extent);
        }
    };



    // *******************************************************************************************************
    //          Octree CLASS members
    // *******************************************************************************************************

    // Outer extent
    AABBox              m_extent;

    // Max number of entries
    uint32_t            m_maxEntries;

    // Max number of branch nodes
    uint32_t            m_maxNodes;

    // Head of node freelist
    uint32_t            m_nodeFreeList;

    // Branch nodes
    Node               *m_nodes;

    // Entry array
    Entry              *m_entries;

    // Entry bbox array
    AABBox             *m_bboxes;

private:

    void
    SplitLeaf(uint32_t iParent,
              uint32_t iChildOfParent,
              const AABBox &nodeBBox);

    /**
    \internal
    */
    void
    AddEntryToLeaf(uint32_t iEntry, uint32_t iNode, uint32_t iChild, RwpBool inCorner)
    {
        Octree::Node  *node  = &m_nodes[iNode];
        Octree::Entry *entry = &m_entries[iEntry];

        // Prepend to list
        entry->Init(iNode, iChild, inCorner);
        entry->SetNext(node->LeafEntries(iChild));
        node->SetLeafEntries(iChild, iEntry);

        // Can the object be pushed deeper?
        if (inCorner)
        {
            node->IncPushCount(iChild);
        }

    }

    /**
    \internal
    */
    void
    AddEntryToNode(uint32_t iEntry, uint32_t iNode)
    {
        Octree::Node  *node  = &m_nodes[iNode];
        Octree::Entry *entry = &m_entries[iEntry];

        // Prepend to list
        entry->Init(iNode, rwOCTREE_NO_CHILD, FALSE);
        entry->SetNext(node->StuckEntries());
        node->SetStuckEntries(iEntry);

    }


    //  The constructor is private, use Initialize instead.
    Octree(uint32_t maxObjs, const AABBox &extent);

    void ComputeConstructionMetrics(Octree::Node& node, ConstructionMetrics& metrics, uint32_t level) const;

public:

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const;

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t maxObjs, const AABBox &extent);

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(const ObjectDescriptor & objDesc);

    static Octree*
    Initialize(const EA::Physics::MemoryPtr& resource, uint32_t maxObjs, const AABBox &extent);

    static Octree*
    Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor &objDesc);

    void
    Release();

    void
    Insert(uint32_t index, const AABBox &bbox);

    /**
    \brief Update an octree entry's bounding box.

    \param index   Index of octree entry.
    \param bbox    New bounding box.
    */
    void
    Update(uint32_t index, const AABBox &bbox)
    {
        Remove(index);
        Insert(index, bbox);
    }

    void
    Remove(uint32_t index);

    /**
    \brief Retrieve an octree entry's bounding box.

    \param index    Index of octree entry.
    \return Pointer to the bounding box. This cannot be modified.
    */
    const AABBox *
    GetEntryBBox(uint32_t index) const
    {
        return &m_bboxes[index];
    }

    /// Compute the construction metrics for the tree
    ConstructionMetrics ComputeConstructionMetrics() const;

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar.TrackInternalPointer(m_nodes);
        ar.TrackInternalPointer(m_entries);
        ar.TrackInternalPointer(m_bboxes);

        ar & EA_SERIALIZATION_NAMED_VALUE(m_extent);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_maxEntries);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_maxNodes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_nodeFreeList);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_nodes, m_maxNodes);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_entries, m_maxEntries);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_bboxes, m_maxEntries);


    }


    // *******************************************************************************************************
    //                                      Octree::LineQuery CLASS
    // *******************************************************************************************************

    /**
    \brief Query object to find all entries in an octree whose bounding box intersects a line.

    \importlib rwccore
    */
    class LineQuery
    {
    public:
        /**
        \internal
        \importlib rwccore
        */
        struct StackElement
        {
            AABBox    m_bb;        ///< BBox of node
            float   m_p[2];      ///< Pair of line clip parameters in node
            uint16_t  m_node;      ///< Node index
        };

    private:

        /**
        \internal
        \importlib rwccore
        */
        struct Result
        {
            float   m_p[2];
            uint16_t  m_node;
        };

        // Octree we're querying
        const Octree     *m_octree;

        // Line clipper object
        AALineClipper     m_clipper;

        // Padding amount along each axis in units of line parameter
        rwpmath::Vector3 m_recipPad;

        // Bits 0,1,2 indicate whether line X,Y,Z component is negative
        uint32_t          m_swap;

        // Stack for recursive tree traversal (top is next free)
        StackElement      m_stack[rwOCTREE_NODE_STACK_SIZE];
        uint32_t          m_top;

        // Batches of results (enough space for each child of a node plus the 'stuck' entries)
        Result            m_results[9];

        // Index of current result (-1 means no more results).
        int32_t           m_curResult;

        // Next entry in leaf node list (terminated by rwOCTREE_END_OF_LIST).
        uint32_t          m_nextEntry;

        /**
        \internal
        */
        void
        ProcessNode();

    public:

        LineQuery(const Octree *octree,
                  rwpmath::Vector3::InParam start,
                  rwpmath::Vector3::InParam end,
                  const float fatness = 0.0f);

        /**
        \brief

        Find next octree entry from the leaf nodes that are intersected by the line. This will
        return more results than the GetNext function which goes on to further test whether the
        entry's bounding box intersects the line.

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        RwpBool
        GetNextInIntersectedNodes(uint32_t &entry)
        {
            entry = m_nextEntry;
            if (entry == rwOCTREE_END_OF_LIST)
            {
                m_curResult--;
                while (m_curResult < 0)
                {
                    if (m_top == 0)
                    {
                        // No more nodes to process - end of query
                        return FALSE;
                    }
                    else
                    {
                        // Process node at top of stack to fill results
                        ProcessNode();
                    }
                }

                // Results never contain empty lists
                entry = m_results[m_curResult].m_node;
            }

            m_nextEntry = m_octree->m_entries[entry].Next();

            return TRUE;
        }

        /**
        \brief Find next octree entry whose bounding box intersects the query line.

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        RwpBool
        GetNext(uint32_t &entry)
        {
            while (GetNextInIntersectedNodes(entry))
            {
                float pa = m_results[m_curResult].m_p[0];
                float pb = m_results[m_curResult].m_p[1];
                if (m_clipper.ClipToAABBox(pa, pb, m_octree->m_bboxes[entry]))
                {
                    return TRUE;
                }
            }

            return FALSE;
        }

        /**
        \brief
        Modifies the end clip point during iteration over results of an octree line query. This
        will eliminate, from the iteration process, any nodes of the octree that lie further
        along the line than the given point.

        \param endVal  End clip parameter (should lie between 0 and 1).
        */
        void
        ClipEnd(float endVal)
        {
            uint32_t  i, count;

            // Clip results
            count = static_cast<uint32_t>(m_curResult + 1);
            m_curResult = -1;
            for (i=0; i < count; i++)
            {
                // Is the segment in range?
                if (m_results[i].m_p[0] <= endVal)
                {
                    m_curResult++;
                    m_results[m_curResult] = m_results[i];
                    m_results[m_curResult].m_p[1] = rwpmath::Min(m_results[m_curResult].m_p[1], endVal);
                }
            }

            // Clip nodes
            count = m_top;
            m_top = 0;
            for (i=0; i < count; i++)
            {
                // Is the segment in range?
                if (m_stack[i].m_p[0] <= endVal)
                {
                    m_stack[m_top] = m_stack[i];
                    m_stack[m_top].m_p[1] = rwpmath::Min(m_stack[m_top].m_p[1], endVal);
                    m_top++;
                }
            }

        }
    };


    // *******************************************************************************************************
    //                                      Octree::BBoxQuery CLASS
    // *******************************************************************************************************

    /**
    \brief
    Query object that may be used to find all entries in an octree whose bounding boxes overlap a given
    query box.

    \importlib rwccore
    */
    class BBoxQuery
    {
    public:
        /**
        \internal

        \importlib rwccore
         */
        struct StackElement
        {
            AABBox      m_bb;      ///< BBox of node
            uint16_t    m_node;    ///< Node index
        };

    private:
        // Octree we're querying
        const Octree           *m_octree;

        // Query BBox
        AABBox                  m_bbox;

        // Stack for recursive tree traversal
        StackElement            m_stack[rwOCTREE_NODE_STACK_SIZE];
        uint32_t                m_top;

        // Batches of results (8 children, plus stuck entries)
        uint16_t                m_results[9];

        // Current result (-1 if no more results).
        int32_t                 m_curResult;

        // Current entry in leaf node list (terminated by rwOCTREE_END_OF_LIST).
        uint32_t                m_nextEntry;

        /**
        \internal
         */
        void
        ProcessNode();

    public:

        BBoxQuery(const Octree *octree,
                  const AABBox  &bbox);

        /**
        \brief
        Find next octree entry from the leaf nodes that are intersected by the query box. This will
        be cheaper but return more results than the GetNext function which goes on to further test
        specifically whether the entry's bounding box is intersected.

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        RwpBool
        GetNextInIntersectedNodes(uint32_t &entry)
        {
            entry = m_nextEntry;
            if (entry == rwOCTREE_END_OF_LIST)
            {
                m_curResult--;
                while (m_curResult < 0)
                {
                    if (m_top == 0)
                    {
                        // No more nodes to process - end of query
                        return FALSE;
                    }
                    else
                    {
                        // Process node at top of stack, filling results.
                        ProcessNode();
                    }
                }

                // Results never contain empty lists
                entry = m_results[m_curResult];
            }

            m_nextEntry = m_octree->m_entries[entry].Next();

            return TRUE;
        }

        /**
        \brief Find next octree entry whose bounding box overlaps the query box.

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        RwpBool
        GetNext(uint32_t &entry)
        {
            while (GetNextInIntersectedNodes(entry))
            {
                if (m_bbox.Overlaps(m_octree->m_bboxes[entry]))
                {
                    return TRUE;
                }
            }

            return FALSE;
        }
    };

};


// ***********************************************************************************************************
// Inline Methods

inline const Octree::ObjectDescriptor Octree::GetObjectDescriptor() const
{
    return ObjectDescriptor(m_maxEntries, m_extent);
}


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_OCTREE_HPP
