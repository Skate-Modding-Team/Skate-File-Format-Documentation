// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREEBASE_H
#define PUBLIC_RW_COLLISION_KDTREEBASE_H

/*************************************************************************************************************

 File: kdtreebase.h

 Purpose: KDTree spatial map.
 */


#include "rw/collision/common.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/aalineclipper.h"

namespace rw
{
namespace collision
{
//Forward declarations, these now live in their own headers
class KDTreeLineQuery;
class KDTreeBBoxQuery;


/**
\internal
Alignment requirements for the memory of a KDTree object.
*/
#ifdef RWP_NO_VPU_MATH
#define rwcKDTREE_ALIGNMENT     4
#else
#define rwcKDTREE_ALIGNMENT     16
#endif

/**
\internal
Maximum depth of a KDTree.
*/
#define rwcKDTREE_MAX_DEPTH     32


/**
\internal
Maximum stack size for hierarchy traversal of the KDTree.
*/
#define rwcKDTREE_STACK_SIZE    rwcKDTREE_MAX_DEPTH + 1


/**
\internal
This is the special value of m_content for a KDTree node that is a branch (not a leaf).
*/
#define rwcKDTREE_BRANCH_NODE   0xffffffff


/**
\internal
This is the special value of m_index for a KDTree node that is invalid.
*/
#define rwcKDTREE_INVALID_INDEX 0xffffffff



// ***********************************************************************************************************
//                                           rw::collision::KDTreeBase CLASS
// ***********************************************************************************************************

/**
\brief
The KDTree is spatial map to support efficient query of axis-aligned bounding boxes.

The kd-tree is a family of algorithms for using a binary tree to index a multi-dimensional space.
For a general description of kd-tree algorithms see http://en.wikipedia.org/wiki/Kd-tree.
Each branch node (internal node) of the kd-tree splits the data along one dimension.
A generic kd-tree requires each entry to have single key, which is a point in space.  And each branch node
in a generic kd-tree defines a plane perpendicular to one of the axes that splits the children into two
disjoint sets.

\par Implementation
This KDTree implementation is three dimensional, adaptive, and supports overlapping children.
It is adaptive because each branch node plane is chosen by a heuristic in order
to balance the KDTree.  This KDTree supports overlapping children so that each entry can have a region of
keys.  A region of keys is an axis-aligned bounding box.  With overlapping children, each branch node
actually defines two parallel axis-aligned planes which divide space into three regions: the left child
region, the right child region, and the overlapping region that belongs to both children.

\par Usage
The KDTree does not support dynamic run-time insert and delete.  You create the KDTree in batch using the
Conditioning Pipeline class GraphKDTree.   The GraphKDTree::Build method constructs a temporary internal
representation of the KDTree.  And the GraphKDTree::InitializeRuntimeKDTree method converts the temporary
representation into a run-time KDTree object that you can query.

One of the parameters to the GraphKDTree::Build method is splitThreshold.  This is the maximum number of
entries that will be held in a leaf node.  The leaf nodes of the KDTree are actually buckets can hold
many entries.  For convenience, the Build method "sorts" the input entries so that entries that are in
the same leaf will have consecutive indices.  Then each leaf node has only two numbers, the index of the
first entry (m_index) and the number of entries (m_content).  Therefore, when you query the KDTree, for
example using LineQuery::GetNext, the indices returned are refering to the sorted entry list, not the
original list of entries.  To convert a sorted index to the original index, you need to apply the mapping
that is returned by GraphKDTree::GetSortedEntryIndices.

The KDTree is used by other collision classes: KDTreeMappedArray, ClusteredMesh, and TriangleKDTreeProcedural.

\see GraphKDTree::Build, GraphKDTree::InitializeRuntimeKDTree, GraphKDTree::GetSortedEntryIndices
\see NodeKDTreeMappedArrayCreate, NodeClusteredMeshCreate, NodeTriangleKDTreeProceduralCreate

\importlib rwccore
 */
class KDTreeBase
{
private:

public:
    
    /**
    Default constructor for KDTreeBase

    Note: The class data members are left uninitialized.
    */
    KDTreeBase() {}     

    // *******************************************************************************************************
    //                                        rw::collision::KDTree::Node CLASS
    // *******************************************************************************************************

    /**
    \internal
    A node reference is a reference to a child node.

    Each branch node has two NodeRef, one for each child.
    If the child is another branch node, then the m_content has the special value rwcKDTREE_BRANCH_NODE
    and the m_index is the index of the child node.   Otherwise, the m_content is the number of entries
    and the m_index is the first entry.

    \importlib rwccore
    */
    struct NodeRef
    {
        uint32_t    m_content;     ///< rwcKDTREE_BRANCH_NODE or number of entries in a leaf node.
        uint32_t    m_index;       ///< Index of branch node or start index of entries in leaf node.

        // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
        // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
        template <class Archive>
            void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_content);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_index);

        }
    };


    /**
    \internal
    The branch node is an internal node of a KDTree.

    The branch node has the index of its parent (the parent is zero if this is the root) and two child
    NodeRefs.  The branch node puts each entry into left or right child based on its coordinate range in
    a specific axis.  The left child contains entries with coordinate less than m_extents[1] and the right
    child contains entries with coordinate greater than m_extents[0].

    \importlib rwccore
    */
    struct BranchNode
    {
        uint32_t    m_parent;       ///< index of the parent branch node, or index of self, if this is the root.
        uint32_t    m_axis;         ///< axis id, 0=x, 1=y, and 2=z indicating the axis along which this branch divides its children.
        NodeRef     m_childRefs[2]; ///< references to the two children, each of which may be a branch node or a leaf node.
        float     m_extents[2];   ///< location of the branch planes.

        // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
        // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
        template <class Archive>
            void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_parent);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_axis);

            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_childRefs, 2);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_extents, 2);

        }
    };

    BranchNode *      m_branchNodes;    ///< Array of branch nodes (self indexing for hierarchical structure)
    uint32_t          m_numBranchNodes; ///< Size of node array
    uint32_t          m_numEntries;     ///< Total number of 'entries' referenced by leaf nodes of tree
    AABBox            m_bbox;           ///< Outer extent of the kd tree contents


    RwpBool
        IsValid() const;

    /**
    \brief
    Release
    */
    void
    Release()
    {
    }


    /**
    Gets the number of branch nodes in the KDTree.


    Note that number of leaf nodes is one more than the number of branch nodes.
    \return The number of branch nodes in the KDTree.
    */
    uint32_t
    GetNumBranchNodes() const
    {
        return m_numBranchNodes;
    }

    /**
    Gets the number of entries indexed by the KDTree.


    Each leaf node will reference a number of entries between zero and splitThreshold.
    \return The number of entries indexed by the KDTree.
    */
    uint32_t
    GetNumEntries() const
    {
        return m_numEntries;
    }


    /**
    Gets the axis-aligned bounding box that contains all the entries in the KDTree.
    \return the AABB of the KDTree.
    */
    const rw::collision::AABBox &
    GetBBox() const
    {
        return m_bbox;
    }

    /* These typedefs are here because the queries used to live inside this class.  They were moved into
       their own headers, but the following were added for backwards compatibility */
    typedef KDTreeLineQuery LineQuery;
    typedef KDTreeBBoxQuery BBoxQuery;


// Class below produces "alignment of a member was sensitive to packing" warning
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4121)
#endif

    /**
    \internal
    \brief
    Class for generalized depth first traversal of KDTree.

    This class provides a push-down stack so that you can traverse a KDTree using a loop instead of using
    recursion.  As you descend to the KDTree, you push the child node as well as any other data type
    that you want to associate with each node during the traversal.

    \par Usage
    The following example traverses the tree to find the average depth of the leaves.
    The traversal data is an integer which is the depth of the node (root is zero).
    The traversal constructor creates a stack and pushes the root onto the stack.
    The first call to PopNode removes the root from the stack, and if the root node is a branch node, the
    two children of the root are pushed onto the stack, first the right child, then the left.
    The second iteration of the while loop pops the left child off the stack, and so on.
    \code
    uint32_t total_depth = 0;
    uint32_t depth;

    KDTree::Traversal<uint32_t> traversal(kdtree, 0);

    while (traversal.PopNode(depth))
    {
        if (traversal.CurrentNodeIsBranch())
        {
            // push the left child last so that it is explored first.
            traversal.PushChildNode(1, depth + 1);
            traversal.PushChildNode(0, depth + 1);
        }
        else
        {
            // find the sum of the depths of all the leaf nodes
            total_depth += depth;
        }
    }
    float average_depth = 1.0f * total_depth / kdtree->GetNumBranchNodes();
    \endcode

    \importlib rwccore
    */
    template<typename NODEDATA>
    class Traversal
    {
    public:
        /**
        \internal
        This class is the records that are pushed and popped on the stack.

        \importlib rwccore
        */
        class StackValue
        {
        public:
            KDTreeBase::NodeRef   m_nodeRef;                    ///< reference to the node, can be a branch node or a leaf.
            NODEDATA            m_data;                        ///< the user defined data associated with the node.
        };

        const KDTreeBase       *m_kdtree;                        ///< m_kdtree The KDTree that is being traversed

        StackValue                m_stack[rwcKDTREE_STACK_SIZE];    ///< The stack use to traverse the tree.
        uint32_t                m_top;                            ///< The next free stack entry (top of stack)
        uint32_t                m_branchIndexOffset;            ///< Start offset into branchnode array

        KDTreeBase::NodeRef     m_cur;                            ///< The last node popped off the stack.


        /**
        \internal

        \brief Constructor. Starts traversal at root of tree.

        This creates an stack and pushes the root of the kdtree onto the stack with the specified data.

        \param tree KDTree to be traversed.
        \param data Special node data corresponding to the root.
        \param offset BranchNode offset (for use with KDSubTrees)
        */
        Traversal(const KDTreeBase *tree, const NODEDATA &data, const uint32_t offset = 0)
            : m_kdtree(tree), m_top(0), m_branchIndexOffset(offset)
        {
            Reset(data);
        }


        /**
        \internal

        \brief Reset traversal at root of tree.

        When you call Reset, the traversal stack is emptied and the root node is pushed onto the stack
        with the specified data.   The current node is invalidated so that you cannot call
        other methods, such as PushChildNode and GetBranchIndex, until you first call PopNode.

        \param data User defined node data associated with the root.

        \see KDTreeBase::Traversal::PopNode
        */
        void
        Reset(const NODEDATA &data)
        {
            m_stack[0].m_nodeRef.m_content =
                (m_kdtree->m_numBranchNodes > 0) ? rwcKDTREE_BRANCH_NODE : m_kdtree->m_numEntries;
            m_stack[0].m_nodeRef.m_index = m_branchIndexOffset;
            m_stack[0].m_data = data;
            m_top = 1;
            m_cur.m_content = 0;
            m_cur.m_index = rwcKDTREE_INVALID_INDEX;
        }


        /**
        Pop node from stack.

        This might be a branch or leaf.  The PopNode also sets the "current" node pointer to
        the node that was popped off so that calls to the other methods, such as PushChildNode and
        GetBranchIndex, apply to the "current" node.
        \param data Receives special node data.
        */
        RwpBool
        PopNode(NODEDATA &data)
        {
            if (m_top > 0)
            {
                data = m_stack[--m_top].m_data;
                m_cur = m_stack[m_top].m_nodeRef;
                return TRUE;
            }
            return FALSE;
        }


        /**
        \internal
        \brief Push child node of the current node onto the stack.

        This does not change the current node.
        You must call PopNode before calling this method so that the current node is set.
        \param idx Index of child (0 is left, 1 is right).
        \param data Special node data for this child.
        */
        void
        PushChildNode(uint32_t idx, const NODEDATA &data)
        {
            EA_ASSERT_MSG(m_top < rwcKDTREE_STACK_SIZE, ("Stack overflow."));
            m_stack[m_top].m_nodeRef = m_kdtree->m_branchNodes[GetBranchIndex()].m_childRefs[idx];
            m_stack[m_top++].m_data = data;
        }


        /**
        \internal
        \brief Tests if the current node is a branch node.


        You must call PopNode before calling this method so that the current node is set.
        \return TRUE if current node (last popped from stack) is a branch node.
        \see KDTreeBase::Traversal::PopNode
        */
        RwpBool
        CurrentNodeIsBranch() const
        {
            EA_ASSERT_MSG(m_cur.m_index != rwcKDTREE_INVALID_INDEX,
                ("Current node is not valid.  You must call PopNode."));
            return static_cast<RwpBool>(m_cur.m_content == rwcKDTREE_BRANCH_NODE);
        }


        /**
        \internal
        \brief Gets the index of the branch node that was most recently popped off the traversal stack.


        The current node must be a branch node.
        You must call PopNode before calling this method so that the current node is set.
        \return Index of node in internal KDTree branch node array.
        \see KDTreeBase::Traversal::PopNode, KDTreeBase::Traversal::CurrentNodeIsBranch
        */
        uint32_t
        GetBranchIndex() const
        {
            EA_ASSERT(CurrentNodeIsBranch());
            return m_cur.m_index-m_branchIndexOffset;
        }


        /**
        \internal
        \brief
        Returns slice of entry array contained in the leaf node.

        The current node must be a leaf node.
        You must call PopNode before calling this method so that the current node is set.
        \param first Receives start index or reference to entry.
        \param count Receives entry count.
        \see KDTreeBase::Traversal::PopNode
        */
        void
        GetLeafNodeEntries(uint32_t &first, uint32_t &count) const
        {
            EA_ASSERT(!CurrentNodeIsBranch());
            first = m_cur.m_index;
            count = m_cur.m_content;
        }
    };

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    /// Serialize data members, but not structural members
    template <class Archive>
    void SerializeData(Archive& ar, const uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numBranchNodes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numEntries);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
    }

protected:

    /// Memory layout constructor - no other data initialized.
    KDTreeBase(BranchNode * branchNodes)
        : m_branchNodes(branchNodes)
    {        
    }

    /// Full constructor
    KDTreeBase(uint32_t numBranchNodes,
        uint32_t numEntries,
        const rw::collision::AABBox &bbox,
        BranchNode * branchNodes)
        : m_branchNodes(branchNodes)
        , m_numBranchNodes(numBranchNodes)
        , m_numEntries(numEntries)
        , m_bbox(bbox)
    {
    }
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_KDTREEBASE_H

