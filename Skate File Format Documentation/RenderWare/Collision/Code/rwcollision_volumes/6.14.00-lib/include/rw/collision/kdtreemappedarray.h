// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREEMAPPEDARRAY_H
#define PUBLIC_RW_COLLISION_KDTREEMAPPEDARRAY_H

/*************************************************************************************************************

 File: rwckdtreemappedarray.hpp

 Purpose: Array of volumes with a kdtree spatial map.
 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/mappedarray.h"
#include "rw/collision/kdtree.h"
#include "rw/collision/aabbox.h"

namespace rw
{
namespace collision
{
    // Forward declare KDTreeMappedArray in the rw::collision namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class KDTreeMappedArray;
} // namespace collision
} // namespace rw

// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::KDTreeMappedArray, "rw::collision::KDTreeMappedArray")

namespace rw
{
namespace collision
{

class KDTreeMappedArray;
class VolumeBBoxQuery;
class VolumeLineQuery;


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
    // Classes that inherit should call Initialize
    KDTreeMappedArray(uint32_t numVols, VTable *vTable, uint32_t classSize);

public:

    // Interface
    KDTree *
    GetKDTreeMap()
    {
        return m_map;
    }

    // Initialization and Release functions
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t numVols, uint32_t numNodes, const rw::collision::AABBox &bbox,
                          const VTable *vTable = &sm_vTable,
                          uint32_t classSize = sizeof(KDTreeMappedArray));

    static KDTreeMappedArray *
    Initialize(const EA::Physics::MemoryPtr& resource,
               uint32_t numVols,
               uint32_t numNodes,
               const rw::collision::AABBox &bbox,
               VTable *vTable = &sm_vTable,
               uint32_t classSize = sizeof(KDTreeMappedArray));

    static KDTreeMappedArray *
    Initialize(void *ptr,
               uint32_t numVols,
               uint32_t numNodes,
               const rw::collision::AABBox &bbox,
               VTable *vTable = &sm_vTable,
               uint32_t classSize = sizeof(KDTreeMappedArray));

    void
    Release();

    // Validity check
    RwpBool
    IsValid() const;

    // Functions used to fill in vtable
    uint32_t
    GetSizeThis() const;

    void
    UpdateThis(void);

    RwpBool
    LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                             const rwpmath::Matrix44Affine *tm);

    RwpBool
    BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                         const rwpmath::Matrix44Affine *tm);

    void ApplyUniformScale(float scale, bool useProcessedFlags = false);

    struct ObjectDescriptor;

    static KDTreeMappedArray * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc);
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const;

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
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
    KDTree *m_map;
    static VTable   sm_vTable;

    // This padding is here to avoid relying on compiler specific padding behaviour
protected:  // avoid warnings about unused private data
#if 4 == EA_PLATFORM_PTR_SIZE
    uint32_t padkdtma[3];
#elif 8 == EA_PLATFORM_PTR_SIZE
    uint32_t padkdtma[2];
#endif
};


struct KDTreeMappedArray::ObjectDescriptor
{
    ObjectDescriptor(uint32_t numVols, uint32_t numNodes, const rw::collision::AABBox& bbox)
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
    rw::collision::AABBox m_bbox;

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numVols);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numNodes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
    }
};


inline KDTreeMappedArray * KDTreeMappedArray::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc)
{
    return Initialize(resource, objDesc.m_numVols, objDesc.m_numNodes, objDesc.m_bbox);
}


inline EA::Physics::SizeAndAlignment KDTreeMappedArray::GetResourceDescriptor(const ObjectDescriptor & objDesc)
{
    return GetResourceDescriptor(objDesc.m_numVols, objDesc.m_numNodes, objDesc.m_bbox);
}


// Return the information needed to allocate this object when deserializing
inline const KDTreeMappedArray::ObjectDescriptor KDTreeMappedArray::GetObjectDescriptor() const
{
    return ObjectDescriptor(m_numVolumes, m_map->GetNumBranchNodes(), m_AABB);
}


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_KDTREEMAPPEDARRAY_H
