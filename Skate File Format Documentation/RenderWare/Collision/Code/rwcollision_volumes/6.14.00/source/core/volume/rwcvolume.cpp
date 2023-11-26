// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcvolume.cpp

 Purpose: Implementation of Volume, the base class for all the collision objects

 */

// ***********************************************************************************************************
// Includes

#include <new>

#if !defined(EA_PLATFORM_PS3_SPU)
#include "rw/collision/aggregate.h"
#endif // !defined(EA_PLATFORM_PS3_SPU)
#include "rw/collision/sphere.h"
#include "rw/collision/capsule.h"
#include "rw/collision/triangle.h"
#include "rw/collision/box.h"
#include "rw/collision/cylinder.h"
#include "rw/collision/aggregatevolume.h"


namespace rw
{
namespace collision
{


GPInstance::VolumeMethods GPInstance::sVolumeMethods[GPInstance::NUMINTERNALTYPES] =
    {
        { NULL },                                                         // GPInstance::NULL
        {   static_cast<GPInstance::GetBBoxFn>(&GPSphere::GetBBox)  },    // GPInstance::SPHERE
        {   static_cast<GPInstance::GetBBoxFn>(&GPCapsule::GetBBox)  },   // GPInstance::CAPSULE
        {   static_cast<GPInstance::GetBBoxFn>(&GPTriangle::GetBBox)  },  // GPInstance::TRIANGLE
        {   static_cast<GPInstance::GetBBoxFn>(&GPBox::GetBBox)  },       // GPInstance::BOX
        {   static_cast<GPInstance::GetBBoxFn>(&GPCylinder::GetBBox)  }   // GPInstance::CYLINDER
    };


/* This could be filled in with the actual function pointers */
Volume::VTable *Volume::vTableArray[rw::collision::VOLUMETYPENUMINTERNALTYPES];
 
// The following is not required for an SPU build
#if       !defined(EA_PLATFORM_PS3_SPU)

// ***********************************************************************************************************
// Structs + Unions + Classes


// ***********************************************************************************************************
// VolumeInitializeCollision

/**
\brief Initializes the volume class vtable array

 You must call this function one time only for any application that will use volumes.
Usually you would call this during application initialization.

\return TRUE always.
*/
RwpBool
Volume::InitializeVTable()
{
    
    /* Fill in the internal types by hand */
    vTableArray[rw::collision::VOLUMETYPENULL] = 0;
#if       defined(EA_PLATFORM_PS3_SPU)
    vTableArray[rw::collision::VOLUMETYPEAGGREGATE] = 0;
#else  // defined(EA_PLATFORM_PS3_SPU)
    vTableArray[rw::collision::VOLUMETYPEAGGREGATE] = &globalAggregateVolumeVTable;
#endif // defined(EA_PLATFORM_PS3_SPU)
    vTableArray[rw::collision::VOLUMETYPESPHERE] = &globalSphereVTable;
    vTableArray[rw::collision::VOLUMETYPECAPSULE] = &globalCapsuleVTable;
    vTableArray[rw::collision::VOLUMETYPETRIANGLE] = &globalTriangleVTable;
    vTableArray[rw::collision::VOLUMETYPEBOX] = &globalBoxVTable;
    vTableArray[rw::collision::VOLUMETYPECYLINDER]  = &globalCylinderVTable;

    return TRUE;
}

/**
\brief
Release the volume class and reset the vtable array

 This only resets the volume class vtable list to zeros so that you cannot create any more
volumes.   No memory is released by this method.

\return TRUE always.
*/
RwpBool
Volume::ReleaseVTable()
{
    /* Fill in the internal types by hand */
    vTableArray[rw::collision::VOLUMETYPENULL] = 0;
    vTableArray[rw::collision::VOLUMETYPESPHERE] = 0;
    vTableArray[rw::collision::VOLUMETYPECAPSULE] = 0;
    vTableArray[rw::collision::VOLUMETYPETRIANGLE] = 0;
    vTableArray[rw::collision::VOLUMETYPEBOX] = 0;
    vTableArray[rw::collision::VOLUMETYPECYLINDER] = 0;
#if       !defined(EA_PLATFORM_PS3_SPU)
    vTableArray[rw::collision::VOLUMETYPEAGGREGATE] = 0;
#endif // !defined(EA_PLATFORM_PS3_SPU)

    return TRUE;
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

/**
Initializes a Volume at the given memory location.
\param resource Memory resource
\return a pointer to the new volume
*/
Volume *
Volume::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & /*objDesc*/)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return(new (resource.GetMemory()) Volume());
}

} // namespace collision
} // namespace rw
