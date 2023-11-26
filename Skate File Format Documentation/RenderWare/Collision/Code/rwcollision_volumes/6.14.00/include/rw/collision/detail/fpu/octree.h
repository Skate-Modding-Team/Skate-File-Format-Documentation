// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_OCTREEFPU_HPP
#define PUBLIC_RW_OCTREEFPU_HPP

/*************************************************************************************************************

 File: rwoctree.hpp

 Purpose: Octree based spatial map.
 */


#include "rw/collision/common.h"
#include "rw/collision/detail/fpu/aabbox.h"
#include "rw/collision/aalineclipper.h"
#include "rw/collision/octree.h"

namespace rw
{
    namespace collision
    {
        namespace detail
        {
            namespace fpu
            {
                // forward declare Octree in the rw::collision namespace so that we can
                // use it in the EA_SERIALIZATION_CLASS_VERSION macro
                class Octree;
            }
        }

    } // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::Octree, 1)

namespace rw
{
    namespace collision
    {
        namespace detail
        {
            namespace fpu
            {


#define rwOCTREEFPU_ALIGNMENT              4
#define rwOCTREEFPU_NODE_ALIGNMENT         4
#define rwOCTREEFPU_BBOX_ALIGNMENT         4


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
\brief Dynamic octree based spatial map.

\importlib rwccore
*/
class Octree
{
public:

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
    \brief    Octree branch node.

    This is 32 bytes in size and aligned to 32 bytes on some platforms to minimize cache misses.
    Children are numbered 0 to 7. Bits 0,1,2 of the child address are set for the high
    X,Y,Z regions respectively.

    \importlib rwccore
    */
    struct Node
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

    };

    // *******************************************************************************************************
    //                                          Octree::ObjectDescriptor CLASS
    // *******************************************************************************************************


    struct ObjectDescriptor
    {
        ObjectDescriptor(uint32_t maxEntries, const AABBox& extent)
        {
            m_extent = extent;
            m_maxEntries = maxEntries;
            
        }

        ObjectDescriptor()
        {
            m_maxEntries = 0;
            m_extent.m_max = rw::math::fpu::GetVector3_Zero();
            m_extent.m_min = rw::math::fpu::GetVector3_Zero();
        }
        
        uint32_t m_maxEntries;

        AABBox m_extent;

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

    /**
    \internal
    */

    //  The constructor is private, use Initialize instead.
    inline Octree(uint32_t maxObjs, const AABBox /*&extent*/);

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

    static void
    Release();

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

};


// ***********************************************************************************************************
// Inline Methods

inline const Octree::ObjectDescriptor Octree::GetObjectDescriptor() const
{
    return ObjectDescriptor(m_maxEntries,
                            m_extent);
}

inline void
Octree::Release()
{
}

inline EA::Physics::SizeAndAlignment
Octree::GetResourceDescriptor(const ObjectDescriptor & objDesc)
{
    EA_ASSERT(objDesc.m_maxEntries <= rwOCTREE_MAX_ENTRIES);

    EA_ASSERT(EA::Physics::SizeAlign<uint32_t>(sizeof(AABBox), rwOCTREEFPU_BBOX_ALIGNMENT) == sizeof(AABBox));
    EA_ASSERT(EA::Physics::SizeAlign<uint32_t>(sizeof(Octree::Node), rwOCTREEFPU_NODE_ALIGNMENT) == sizeof(Octree::Node));

    uint32_t size = 0;

    // Base struct
    size += sizeof(Octree);

    // Entry bboxes
    size = EA::Physics::SizeAlign<uint32_t>(size, rwOCTREEFPU_BBOX_ALIGNMENT);
    size += objDesc.m_maxEntries * sizeof(AABBox);

    // Nodes
    size = EA::Physics::SizeAlign<uint32_t>(size, rwOCTREEFPU_NODE_ALIGNMENT);
    size += rwOCTREE_MAX_NODES(objDesc.m_maxEntries) * sizeof(Octree::Node);

    // Entries
    size += objDesc.m_maxEntries * sizeof(Octree::Entry);

    return EA::Physics::SizeAndAlignment(size, rwOCTREEFPU_ALIGNMENT);
}

inline Octree*
Octree::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor &objDesc)
{
    Octree* octree = new (resource.GetMemory()) Octree(objDesc.m_maxEntries, objDesc.m_extent);
    return octree;
}

inline Octree::Octree(uint32_t maxEntries, 
               const AABBox /*&extent*/)
{
    EA_ASSERT(maxEntries <= rwOCTREE_MAX_ENTRIES);

    m_maxEntries = maxEntries;
    m_maxNodes = rwOCTREE_MAX_NODES(maxEntries);

    // Setup pointers for entry bboxes, nodes and entry data
    uintptr_t addr = (uintptr_t)this;

    addr += sizeof(Octree);

    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwOCTREEFPU_BBOX_ALIGNMENT);
    m_bboxes = reinterpret_cast<AABBox *>(addr);
    addr += m_maxEntries * sizeof(AABBox);

    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwOCTREEFPU_NODE_ALIGNMENT);
    m_nodes = reinterpret_cast<Octree::Node *>(addr);
    addr += m_maxNodes * sizeof(Octree::Node);

    m_entries = reinterpret_cast<Octree::Entry *>(addr);
}

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_OCTREE_HPP
