// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_SIMPLEMAPPEDARRAY_H
#define PUBLIC_RW_COLLISION_SIMPLEMAPPEDARRAY_H

/*************************************************************************************************************

 File: rwcsimplemappedarray.hpp

 Purpose: Simple array of volumes (no spatial map).
 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/mappedarray.h"

#define rwcSIMPLEMAPPEDARRAYALIGNMENT rwcVOLUMEALIGNMENT

namespace rw
{
namespace collision
{
    // Forward declare SimpleMappedArray in the rw::collision namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class SimpleMappedArray;
} // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::SimpleMappedArray, 2)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::SimpleMappedArray, "rw::collision::SimpleMappedArray")

namespace rw
{
namespace collision
{
    class SimpleMappedArray;
    class VolumeLineQuery;
    class VolumeBBoxQuery;

// ***********************************************************************************************************
//                                       rw::collision::SimpleAggregate CLASS
// ***********************************************************************************************************

/**
\brief
A simple type of  rw::collision::Aggregate consisting of an array of volumes.

During intersection queries, all entries in a SimpleMappedArray are tested. No spatialmap is
used to speedup the query. This has the advantage that the volumes in the array can be procedurally
moved or modified in some way, without having to rebuild any indexing structure. On the other hand, large
volume arrays can not be queried efficiently.

\importlib rwccore
*/
class SimpleMappedArray : public MappedArray
{
protected:
    // Derived class should use Initialize().
    /**
    \internal
    */
    SimpleMappedArray(uint32_t numVols,
                     VTable *vTable,
                     uint32_t classSize);

public:
    /**
    \internal
    */
    uint32_t
    GetSizeThis() const;

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t numVols,
                          const VTable *vTable = &sm_vTable,
                          uint32_t classSize = sizeof(SimpleMappedArray));

    static SimpleMappedArray *
    Initialize(const EA::Physics::MemoryPtr& resource,
               uint32_t numVols,
               VTable *vTable = &sm_vTable,
               uint32_t classSize = sizeof(SimpleMappedArray));

    void
    Release();

    // Functions used to fill in vtable
    /**
    \internal
    */
    void
    UpdateThis(void);

    /**
    \internal
    */
    RwpBool
    LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                              const rwpmath::Matrix44Affine *tm);
    /**
    \internal
    */
    RwpBool
    BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                         const rwpmath::Matrix44Affine *tm);

    struct ObjectDescriptor;

    static SimpleMappedArray * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc);
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const;

    /**
    \internal
    */
    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
        // Serialize base class
        ar & EA::Serialization::MakeNamedValue(*static_cast<MappedArray*>(this), "MappedArray");

        if (version >= 2)
        {
            // Register the individual volume pointers so that they can be referenced by external objects.
            // This is required if SMA is used just as a container for volumes that are referenced directly.
            for(uint32_t i = 0; i < m_numVolumes; i ++)
            {
                ar.RegisterAddress(&(m_volumes[i])); 
            }
        }

        if(ar.IsLoading())
        {
            m_vTable = &sm_vTable;
            EA_ASSERT(m_vTable != NULL);
        }

    }

private:

    static VTable   sm_vTable;

};


struct SimpleMappedArray::ObjectDescriptor
{
    ObjectDescriptor(uint32_t numVols)
    {
        m_numVols = numVols;
    }

    ObjectDescriptor()
    {
        m_numVols = 0;
    }

    uint32_t m_numVols;

    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numVols);
    }
};


inline SimpleMappedArray *
SimpleMappedArray::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc)
{
    return (Initialize(resource, objDesc.m_numVols));
}


inline EA::Physics::SizeAndAlignment
SimpleMappedArray::GetResourceDescriptor(const ObjectDescriptor & objDesc)
{
    return (GetResourceDescriptor(objDesc.m_numVols));
}


inline const SimpleMappedArray::ObjectDescriptor SimpleMappedArray::GetObjectDescriptor() const
{
    return ObjectDescriptor(m_numVolumes);
}


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_SIMPLEMAPPEDARRAY_H
