// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEMAPPEDARRAY_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEMAPPEDARRAY_H

/*************************************************************************************************************

 File: rwckdtreemappedarray.hpp

 Purpose: Array of volumes with a kdtree spatial map.
 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/kdtreemappedarray.h"
#include "rw/collision/detail/fpu/mappedarray.h"
#include "rw/collision/detail/fpu/kdtree.h"
#include "rw/collision/detail/fpu/aabbox.h"


#define rwcKDTREEMAPPEDARRAYALIGNMENT rwcVOLUMEALIGNMENT

namespace rw
{
    namespace collision
    {
        namespace detail
        {
            namespace fpu
            {
                // Forward declare KDTreeMappedArray in the rw::collision namespace so
                // that we can use it in the EA_SERIALIZATION_CLASS_* macros
                class KDTreeMappedArray;
            } // namespace fpu
        } // namespace detail  
    } // namespace collision
} // namespace rw

// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::KDTreeMappedArray, 1)

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{



class KDTreeMappedArray;



// ***********************************************************************************************************
//                                       rw::collision::KDTreeMappedArray CLASS
// ***********************************************************************************************************

/**
\brief

The Aggregate type consists of an array of Volumes, spatially indexed using a KD tree.
This can be built using the NodeKDTreeMappedArrayCreate conditioning pipeline node.

\see rw::collision::conditioning::MappedArray
\see rw::collision::Aggregate

\importlib rwccore
 */
class KDTreeMappedArray : public MappedArray
{
protected:

public:
    void Release();
    struct ObjectDescriptor;
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(uint32_t numVols, uint32_t numNodes, const AABBox &bbox,
                                const rw::collision::Aggregate::VTable *vTable = &sm_vTable,
                                uint32_t classSize = sizeof(KDTreeMappedArray));
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);
    static KDTreeMappedArray* Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc);
    static KDTreeMappedArray* Initialize(const EA::Physics::MemoryPtr& resource,
                                uint32_t numVols,
                                uint32_t numNodes,
                                const AABBox &bbox,
                                rw::collision::Aggregate::VTable *vTable = &sm_vTable,
                                uint32_t classSize = sizeof(KDTreeMappedArray));
    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const;

    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        // Chain serialize down to base class
        ar & EA::Serialization::MakeNamedValue(*static_cast<MappedArray*>(this), "MappedArray");

        ar.TrackInternalPointer(m_map);
        ar & EA_SERIALIZATION_NAMED_VALUE(*m_map);

        if(ar.IsLoading())
        {
            m_vTable = &sm_vTable;
            EA_ASSERT(m_vTable != NULL);
        }

    }

private:

    static rw::collision::Aggregate::VTable sm_vTable;
    KDTree *m_map;
protected:  // avoid warnings about unused private data
    uint32_t padkdtma[3];
};


struct KDTreeMappedArray::ObjectDescriptor
{
    ObjectDescriptor(uint32_t numVols, uint32_t numNodes, const rw::collision::detail::fpu::AABBox& bbox)
    {
        m_numVols = numVols;
        m_numNodes = numNodes;
        m_bbox = bbox;
    }

    ObjectDescriptor()
    {
        m_numVols = 0;
        m_numNodes = 0;
        m_bbox = AABBox();
    }

    uint32_t m_numVols;
    uint32_t m_numNodes;
    rw::collision::detail::fpu::AABBox m_bbox;

    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numVols);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numNodes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
    }
};



inline KDTreeMappedArray *
KDTreeMappedArray::Initialize(const EA::Physics::MemoryPtr& resource,
                              uint32_t numVols,
                              uint32_t numNodes,
                              const AABBox  &bbox,
                              rw::collision::Aggregate::VTable * /*vTable*/,
                              uint32_t classSize)
{
    KDTreeMappedArray* kdtreeMappedArray = reinterpret_cast<KDTreeMappedArray*>(resource.GetMemory());

    // we need to initialize the layout so that the volumes are serialized into the correct position
    uintptr_t addr = (uintptr_t)(kdtreeMappedArray);

    //Class structure
    addr += classSize;

    //Set the pointer to the data for the volumes
    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcVOLUMEALIGNMENT);
    kdtreeMappedArray->m_volumes = (Volume *)addr;
    addr += numVols*sizeof(Volume);

    //set the ptr to the spatial map 
    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcKDTREE_ALIGNMENT);
    kdtreeMappedArray->m_map = KDTree::Initialize((void *)addr, numNodes, numVols, bbox);

    return kdtreeMappedArray;
}

inline void KDTreeMappedArray::Release()
{

}

inline KDTreeMappedArray * KDTreeMappedArray::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc)
{
    return (Initialize(resource, objDesc.m_numVols, objDesc.m_numNodes, objDesc.m_bbox));
}

inline EA::Physics::SizeAndAlignment
KDTreeMappedArray::GetResourceDescriptor(uint32_t numVols, uint32_t numNodes,
                                         const AABBox &/*bbox*/,
                                         const rw::collision::Aggregate::VTable * /*vTable*/,
                                         uint32_t /*classSize*/)
{
    uint32_t size = 0;

    // Class data
    size = sizeof(KDTreeMappedArray);

    // Volume array
    size = EA::Physics::SizeAlign<uint32_t>(size, rwcVOLUMEALIGNMENT);
    size += numVols*sizeof(Volume);

    // Spatialmap
    EA::Physics::SizeAndAlignment kdTreeResDesc = KDTree::GetResourceDescriptor(numNodes, 0, AABBox());
    size = EA::Physics::SizeAlign(size, kdTreeResDesc.GetAlignment());
    size += kdTreeResDesc.GetSize();
    return(EA::Physics::SizeAndAlignment(size, rwcKDTREEMAPPEDARRAYALIGNMENT));
}


inline EA::Physics::SizeAndAlignment KDTreeMappedArray::GetResourceDescriptor(const ObjectDescriptor & objDesc)
{
    return (GetResourceDescriptor(objDesc.m_numVols, objDesc.m_numNodes, objDesc.m_bbox));
}


// Return the information needed to allocate this object when deserializing
inline const KDTreeMappedArray::ObjectDescriptor KDTreeMappedArray::GetObjectDescriptor() const
{
    return ObjectDescriptor(m_numVolumes, m_map->m_numBranchNodes, m_AABB);
}

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_KDTREEMAPPEDARRAY_H
