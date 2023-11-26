// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_SIMPLEMAPPEDARRAY_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_SIMPLEMAPPEDARRAY_H

/*************************************************************************************************************

 File: simplemappedarray.h

 Purpose:
 */

#include "rw/collision/common.h"
#include "rw/collision/simplemappedarray.h"

#include "mappedarray.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{
    // Forward declare SimpleMappedArray in the rw::collision::detail::fpu namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class SimpleMappedArray;
} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::SimpleMappedArray, 2)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::detail::fpu::SimpleMappedArray, "rw::collision::SimpleMappedArray")

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/** \brief This class mimics the layout of rw::collision::SimpleMappedArray when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::SimpleMappedArray
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::SimpleMappedArray it is possible to
 * convert between the two using the Serialization framework. As this class also implements the
 * ObjectDescriptor/EA::Physics::SizeAndAlignment framework so HLSerializable can also be used.
 *
 * Changes to data members in rw::collision::SimpleMappedArray or its serialization function should be
 * mirrored in this class.
 */
class SimpleMappedArray : public MappedArray
{
public:
    typedef rw::collision::SimpleMappedArray::ObjectDescriptor ObjectDescriptor;

    const ObjectDescriptor GetObjectDescriptor() const
    {
        return ObjectDescriptor(m_numVolumes);
    }

    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor& objDesc)
    {
        EA_COMPILETIME_ASSERT(rwcSIMPLEMAPPEDARRAYALIGNMENT >= rwcVOLUMEALIGNMENT);
        uint32_t size = EA::Physics::SizeAlign<uint32_t>(sizeof(SimpleMappedArray), rwcVOLUMEALIGNMENT);
        size += objDesc.m_numVols * sizeof(Volume);
        return EA::Physics::SizeAndAlignment(size, rwcSIMPLEMAPPEDARRAYALIGNMENT);
    }

    static SimpleMappedArray * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor& /*objDesc*/)
    {
        SimpleMappedArray* simpleMappedArray = reinterpret_cast<SimpleMappedArray*>(resource.GetMemory());

        // we need to initialize the layout so that the volumes are serialized into the correct position
        void* ptr = EA::Physics::MemAlign(reinterpret_cast<void*>(simpleMappedArray + 1), rwcVOLUMEALIGNMENT);
        simpleMappedArray->m_volumes = reinterpret_cast<Volume *>(ptr);

        return simpleMappedArray;
    }

    void Release()
    {
    }

    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
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
    }
};



} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_SIMPLEMAPPEDARRAY_H
