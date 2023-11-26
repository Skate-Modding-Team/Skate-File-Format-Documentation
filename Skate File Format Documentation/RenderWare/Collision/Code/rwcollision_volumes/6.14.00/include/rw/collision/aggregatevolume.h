// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_AGGREGATEVOLUME_H
#define PUBLIC_RW_COLLISION_AGGREGATEVOLUME_H

/*************************************************************************************************************

 File: rwcaggregatevolume.hpp

 Purpose: A short description of the file.
 */


#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"

namespace rw
{
namespace collision
{

class Aggregate;
class AggregateVolume;


extern Volume::VTable globalAggregateVolumeVTable;


/**
\brief The AggregateVolume represents any volume type that can have compound shape.

 The volume types such as SphereVolume and BoxVolume are the simple primitive collision shapes.
The aggregate volume is a compound shape made up of one or more of the simple volume types in a rigid
configuration.  For example, you can define an aggregate volume for a chair by using boxes for the seat
and back of the chair, and capsules for the legs of the chair.  However, you would not use an aggregate
volume to define a ragdoll, because the parts of a ragdoll need to move around relative to one another.

The AggregateVolume class is a subclass of Volume, and it contains a pointer to an aggregate object.
The Aggregate is the virtual base class for all types of aggregate shapes.
The AggregateVolume implements the volume methods mostly by delegating them to the aggregate object.
The aggregate object is often large, containing hundreds of simple volume types.  Many aggregate
volumes can share the same aggregate object.

\see Aggregate

\importlib rwccore
*/
class AggregateVolume : public Volume
{
protected:
    /**
    Constructor for the AggregateVolume.
    */
    AggregateVolume(Aggregate *agg)
    : Volume(rw::collision::VOLUMETYPEAGGREGATE)
    {
        SetAggregate(agg);
    }

    /**
    Constructor for the AggregateVolume.
    */
    AggregateVolume()
    : Volume(rw::collision::VOLUMETYPEAGGREGATE)
    {
    }

public:
    /**
    Gets the resource requirements of the volume
    \param agg Aggregate of the volume. optional
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(Aggregate * /*agg*/ = 0)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    static AggregateVolume *
    Initialize(const EA::Physics::MemoryPtr & memoryResource, Aggregate *agg);

    /**
        Sets the aggregate object that provides the implementation for this volume.


    The AggregateVolume implements the volume methods mostly by delegating them to the aggregate object.
    The aggregate object can shared by many aggregate volumes.
    \param agg a pointer to a valid aggregate object.
    \see AggregateVolume::GetAggregate
    */
    void
    SetAggregate(Aggregate *agg)
    {
        EA_ASSERT_MSG(agg, ("The aggregate pointer cannot be NULL."));
        *reinterpret_cast<Aggregate**>(&aggregateData.agg) = agg;
    }

    /**
        Gets the aggregate object that provides the implementation for this volume.


    The AggregateVolume implements the volume methods mostly by delegating them to the aggregate object.
    The aggregate object can shared by many aggregate volumes.
    \return A pointer to an aggregate object.
    \see AggregateVolume::SetAggregate
    */
    Aggregate *
    GetAggregate() const
    {
        return aggregateData.agg;
    }

    RwpBool
    GetBBox(const rwpmath::Matrix44Affine *tm, RwpBool tight, AABBox &bBox) const;

    rwpmath::Vector3
    GetBBoxDiag() const;

    RwpBool
    CreateGPInstance(GPInstance &instance, const rwpmath::Matrix44Affine *tm) const;

    struct ObjectDescriptor
    {
        template <class Archive>
        void Serialize(Archive &/*ar*/, uint32_t /*version*/)
        {
        }
    };

    static AggregateVolume *
    Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & /*objDesc*/)
    {
        return (Initialize(resource));
    }

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(const ObjectDescriptor & /*objDesc*/)
    {
        return (GetResourceDescriptor());
    }

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() 
    {
        return ObjectDescriptor();
    }

    /**
    */
    void Release() {}

    void ClearAllProcessedFlags();

    void ApplyUniformScale(float scale, bool useProcessedFlags = false);

private:
    static AggregateVolume *
    Initialize(const EA::Physics::MemoryPtr& resource);
};


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_AGGREGATEVOLUME_H
