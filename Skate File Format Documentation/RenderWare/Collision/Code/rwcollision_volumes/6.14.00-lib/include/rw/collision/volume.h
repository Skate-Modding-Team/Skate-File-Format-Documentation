// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_VOLUME_H
#define PUBLIC_RW_COLLISION_VOLUME_H

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/deprecated/gpinstance.h"
#include "rw/collision/deprecated/feature.h"

namespace rw
{
namespace collision
{
    // Forward declare Volume in the rw::collision namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class Volume;
} // namespace collision
} // namespace rw

// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::Volume, "rw::collision::Volume")
namespace rw
{
namespace collision
{

class VolumeLineQuery;
class VolumeBBoxQuery;

/// \deprecated No longer used
struct Interval;

/**
\brief Refers to a specific volume primitive and its corresponding world transform.

A list of pairs of \e  VolRef is the output of the pair refinement process in GetPrimitiveBBoxOverlaps, and
the input to the collision testing process in the PrimitiveBatchIntersect function.

\see VolRefPair
\see VolumeVolumeQuery::GetPrimitiveBBoxOverlaps, PrimitiveBatchIntersect

\importlib rwccore
*/
struct VolRef
{
    rwpmath::Matrix44Affine tmContents; ///< Private storage that may be used for the transform
    AABBox                  bBox;       ///< Bounding box of the volume (not defined for VolumeLineQueries or Volume::LineSegIntersect)
    const Volume            *volume;    ///< Primitive volume. This may be a temporary instance so don't hold on to the pointer
    rwpmath::Matrix44Affine *tm;        ///< Transform of the primitive volume (concatenation of transforms of parent aggregates)
    uint32_t                tag;        ///< Identifies where the primitive resides in an aggregate hierarchy (see Aggregate::GetChildTagFromTag)
    uint8_t                 numTagBits; ///< Number of bits used for the tag
};

/**
\internal
\brief A pair of VolRefs.

The \e  VolRefPair is the data type output by the bbox overlap refinement code and input to the
batch primitive collision testing code.
\see VolumeVolumeQuery::GetPrimitiveBBoxOverlaps, PrimitiveBatchIntersect

\importlib rwccore
*/
struct VolRefPair
{
    rw::collision::VolRef *vRef1;
    rw::collision::VolRef *vRef2;
};

/**
\internal
\brief A one-to-many relation between \e  VolRef objects.

This is used when testing 1xN collision primitive volumes.  In other words, one collision primitive is
tested against many other primitives.

\importlib rwccore
*/
struct VolRef1xN
{
    rw::collision::VolRef *vRef1;     ///< the primary volume primitive
    uint32_t    vRefsNCount;          ///< number of other volumes
    RwpBool      volumesSwapped;       ///< if true the pairs are swapped, thus Nx1 instead of 1xN.
    rw::collision::VolRef *vRefsN[1]; ///< \brief an array of pointers to volume primitives against which
                                      ///  the primary volume primitive is paired.
};


/**
\internal
Memory alignment requirement for initialization of the \e  Volume object
*/
#ifndef rwcVOLUMEALIGNMENT
#if defined(RWCROSS) && defined(RWP_NO_VPU_MATH)
// workaround a bug in rwmath where rw::math::fpu::Vector3::Alignment incorrectly has a value of 1 in cross builds
#define rwcVOLUMEALIGNMENT  (4)
#else // if !defined(RWCROSS) || !defined(RWP_NO_VPU_MATH)
#define rwcVOLUMEALIGNMENT  (rwpmath::Vector3::Alignment)
#endif // !defined(RWCROSS) || !defined(RWP_NO_VPU_MATH)
#endif


/**
\brief Holds the result for the Volume::LineSegIntersect functions and VolumeLineQuery.

If a Volume::LineSegIntersect function has been called, then the \e v member just points to the volume queried. Otherwise if the result is
for a VolumeLineQuery, the \e v pointer refers to the top level volume in the array submitted to the query. If you are querying aggregates such as
a ClusteredMesh, then to get at the information for the particular primitive/triangle that has been collided with, look at the \e vRef member.
For queries against a hierarchy of nested aggregates, this will refer to the leaf node primitive.

\importlib rwccore
*/
struct VolumeLineSegIntersectResult
{
    uint32_t          inputIndex;   ///< \brief Index Input Volume intersecting line segment in the input volumes array.
                                    ///< This can be useful if the caller wants to know index in the input array for example to know whrere the volume is taken from, like the Part
                                    ///< Note that the value will be set only if input volumes array is used in the function returning this result
                                    ///< For any new implemented custom procedural will need to change their line query code to write the new value in inputIndex
    const Volume      *v;           ///< Input Volume intersecting line segment
    rwpmath::Vector3  position;     ///< Intersection point in world space
    rwpmath::Vector3  normal;       ///< Normal at intersection point
    rwpmath::Vector3  volParam;     ///< Parametric location of intersection on the volume.
    float             lineParam;    ///< Parametric location of intersection on the line segment.
    VolRef            vRef;         ///< Aggregate element reference 
};

/**
\brief
Enumeration of the current volume types.

\hideinitializer
*/
enum VolumeType
{
    /* If you modify this list then update Volume::InitializeVTable function */
    VOLUMETYPENULL     = GPInstance::UNUSED,    ///< Dummy null volume type
    VOLUMETYPESPHERE   = GPInstance::SPHERE,    ///< Sphere volume type \see rw::collision::SphereVolume
    VOLUMETYPECAPSULE  = GPInstance::CAPSULE,   ///< Capsule volume type \see rw::collision::CapsuleVolume
    VOLUMETYPETRIANGLE = GPInstance::TRIANGLE,  ///< Triangle volume type \see rw::collision::TriangleVolume
    VOLUMETYPEBOX      = GPInstance::BOX,       ///< Box volume type \see rw::collision::BoxVolume
    VOLUMETYPECYLINDER = GPInstance::CYLINDER,  ///< Cylinder volume type \see rw::collision::CylinderVolume
    VOLUMETYPEAGGREGATE,                        ///< Aggregate volume type \see rw::collision::AggregateVolume
    VOLUMETYPECONVEXHULL = 8,                   ///< ConvexHull volume type \see EA::collision::ConvexHull
    VOLUMETYPEHEIGHTFIELD = 9,                  ///< Heightfield volume type \see EA::collision::HeightField in eacollision_heightfield
    VOLUMETYPERESERVED10 = 10,
    VOLUMETYPERESERVED11 = 11,
    VOLUMETYPERESERVED12 = 12,
    VOLUMETYPERESERVED13 = 13,
    VOLUMETYPERESERVED14 = 14,
    VOLUMETYPECUSTOM = 15,                      ///< Custom volume type

    VOLUMETYPENUMINTERNALTYPES,

    VOLUMETYPEFORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};


/**
Enumeration for volume flags.

*/
enum VolumeFlag
{
    //Generic Volume Flags
    VOLUMEFLAG_ISENABLED             = 0x0001, ///< indicating that collision with this volume is enabled.  \see Volume::IsEnabled, Volume::SetEnabled
    VOLUMEFLAG_TRIANGLENORMALISDIRTY = 0x0002, ///< indicating the stored normal is not valid.
    VOLUMEFLAG_ISPROCESSED           = 0x0004, ///< indicating that volume has been processed (i.e. scaled)

    //Capsule Specific Flags
    VOLUMEFLAG_CAPSULEEND_0_DISABLED = GPInstance::FLAG_TRIANGLEVERT0DISABLE,  ///< Flag if Capsule End 0 Has Been Disabled. Deliberately reusing GPInstance enumeration
    VOLUMEFLAG_CAPSULEEND_1_DISABLED = GPInstance::FLAG_TRIANGLEVERT1DISABLE,  ///< Flag if Capsule End 1 Has Been Disabled. Deliberately reusing GPInstance enumeration

    //Triangle Specific Flags
    VOLUMEFLAG_TRIANGLEONESIDED      = GPInstance::FLAG_TRIANGLEONESIDED,  ///< contact with back face of triangle is culled.
    VOLUMEFLAG_TRIANGLEEDGE0CONVEX   = GPInstance::FLAG_TRIANGLEEDGE0CONVEX,  ///< the edge from p0 to p1 is convex.
    VOLUMEFLAG_TRIANGLEEDGE1CONVEX   = GPInstance::FLAG_TRIANGLEEDGE1CONVEX,  ///< the edge from p1 to p2 is convex.
    VOLUMEFLAG_TRIANGLEEDGE2CONVEX   = GPInstance::FLAG_TRIANGLEEDGE2CONVEX,  ///< the edge from p2 to p0 is convex.
    VOLUMEFLAG_TRIANGLEUSEEDGECOS    = GPInstance::FLAG_TRIANGLEUSEEDGECOS,  ///< use the edgecos values to limit the permissible edge contact normal.
    VOLUMEFLAG_TRIANGLEVERT0DISABLE  = GPInstance::FLAG_TRIANGLEVERT0DISABLE,  ///< disable collisions with vertex 0.
    VOLUMEFLAG_TRIANGLEVERT1DISABLE  = GPInstance::FLAG_TRIANGLEVERT1DISABLE,  ///< disable collisions with vertex 1.
    VOLUMEFLAG_TRIANGLEVERT2DISABLE  = GPInstance::FLAG_TRIANGLEVERT2DISABLE,  ///< disable collisions with vertex 2.

    VOLUMEFLAG_TRIANGLEDEFAULT       = VOLUMEFLAG_ISENABLED + GPInstance::FLAG_TRIANGLEDEFAULT,  ///< default flags for a new triangle: two-sided and all edges fully convex.

    VOLUMEFLAG_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};


/**
\brief
Base class for collidable geometries

A collision volume is a general interface to a collidable geometry and
its transform relative to a parent frame of reference. The collidable geometry can
either be a simple primitive or an aggregate or other collidable geometries.

With regards to the volume transform; the world space orientation of a volume,
which belongs to a part, is volume.transform * part.transform. Or in general terms
the volume's orientation in its parent's parent frame is volume.transform * parent.transform.
For a more in-depth explination of this point, and transforms in general, see:
http://docs.ea.com/RWPhysics:Introduction_to_Frames#Example_from_EAPhysics_part_.2F_volume_.2F_body

All volumes are the same size.  The subclasses of volume do not add any extra data to the class.  The
volume has a fixed size area (anonymous union) in which the subclass can store the type specific data.
The members of the type specific union are:

\li sphereData specific data for the sphere volume type
\li capsuleData specific data for the capsule volume type
\li boxData specific data for the box volume type
\li triangleData specific data for the triangle volume type
\li aggregateData specific data for aggregate volume type

Volume has a size of 96 bytes on 32-bit platforms and
112 bytes on 64-bit platforms.

\see AggregateVolume, SphereVolume, TriangleVolume, BoxVolume, CapsuleVolume

\importlib rwccore
*/
class Volume
{
public:

    /**
    Pointer to a function to compute the bounding box of a specific volume type.
    \see Volume::GetBBox
    */
    typedef RwpBool      (Volume::*GetBBoxFn)(const rwpmath::Matrix44Affine *tm,
                                             RwpBool tight, AABBox &bBox) const;

    typedef rwpmath::Vector3      (Volume::*GetBBoxDiagFn)() const;
    
    /**
    \deprecated This is deprecated functionality
    */
    typedef RwpBool      (Volume::*GetIntervalFn)(rwpmath::Vector3::InParam dir,
                                                 Interval &interval) const;

    /**
    \deprecated This is deprecated functionality
    */
    typedef RwpBool      (Volume::*GetMaximumFeatureFn)(RwpBool ccw,
                                                       rwpmath::Vector3::InParam dir,
                                                       Feature &feature) const;
    /**
    Pointer to a function to get the general primitive instance data of a specific volume type.
    \see Volume::CreateGPInstance
    */
    typedef RwpBool      (Volume::*CreateGPInstanceFn)(GPInstance &instance,
                                                      const rwpmath::Matrix44Affine *tm) const;

    /**
    Pointer to a function to test line intersection of a specific volume type.
    \see Volume::LineSegIntersect
    */
    typedef RwpBool      (Volume::*LineSegIntersectFn)(rwpmath::Vector3::InParam pt1,
                                                      rwpmath::Vector3::InParam pt2,
                                                      const rwpmath::Matrix44Affine *tm,
                                                      VolumeLineSegIntersectResult &result,
                                                      const float fatness) const;

    /**
    Pointer to a function to release data pertinant to the volume type.
    \see Volume::Release
    */
    typedef void        (Volume::*ReleaseFn)();

    /**
    Pointer to a function to get the moments of a volume. Only user defined volume types that require
    correct ineria need implement this.
    \see Volume::GetMoments
    */
    typedef RwpBool      (Volume::*GetMomentsFn)(rwpmath::Matrix44 &moments) const;

    /**
    Callback function that is used by GetAsTrianglesFn
    */
    typedef void        (TriangleCallback)(void *context,
                                           rwpmath::Vector3::InParam v0,
                                           rwpmath::Vector3::InParam v1,
                                           rwpmath::Vector3::InParam v2);

    /**
    Gets the volume as triangles. The triangleCallback callback is called per triangle.
    */
    typedef void        (Volume::*GetAsTrianglesFn)(void * context, TriangleCallback triangleCallback) const;

    /**
    Clear all volume processed flags
    */
    typedef void        (Volume::*ClearAllProcessedFlagsFn)();

    /**
    Apply uniform scale to volume
    */
    typedef void        (Volume::*ApplyUniformScaleFn)(float scale, bool useProcessedFlags);


    /**

    \brief
    Constructs a volume.

    This is provided only for allocation convenience (eg embedding a volume inside a class
    or declaring an array). The Initialize method of a subclass must be called in order for the
    data to be valid. The Volume system is designed so that all subclasses are the same size
    and alignment.
    */
    Volume()
        : volumeType(0)
        , radius(0.f)
        , groupID(0)
        , surfaceID(0)
        , m_flags(0)
    {
#if (EA_PLATFORM_PTR_SIZE == 8)
        padding[0] = m_padding[0] = m_padding[1] = 0 ;
#endif
    }


protected:
    /**

    \brief
    Construct a new volume.

    To create a volume you must call the Initialize method of a subclass.
    \param type The volume type
    \param r The radius or fatness of the volume, returned from GetRadius()
    */
    Volume(rw::collision::VolumeType type, float r = 0.0f)
    {
        EA_ASSERT(Volume::vTableArray[type]);
        volumeType = type;    
        radius = r;
        groupID = 0;
        surfaceID = 0;
        m_flags = VOLUMEFLAG_ISENABLED;
        transform = rwpmath::GetMatrix44Affine_Identity();
    }

public:

    /**
    */
    void Release()
    {
        (this->*(Volume::vTableArray[volumeType]->release))();
    }

    rw::collision::VolumeType
    GetType() const;

    const char *
    GetTypeName();

    const float &
    GetRadius() const;

    void
    SetRadius(float rad);

    uint32_t
    GetGroup() const;

    void
    SetGroup(uint32_t group);

    uint32_t
    GetSurface() const;

    void
    SetSurface(uint32_t surface);

    RwpBool
    IsEnabled() const;

    void
    SetEnabled(RwpBool whetherEnabled);

    uint32_t
    GetFlags() const;

    void
    SetFlags(uint32_t newflags);

    void
    SetProcessedFlag();

    void
    ClearProcessedFlag();

    void
    ClearAllProcessedFlags();

    void
    ApplyUniformScale(float scale, bool useProcessedFlags = false);

    const rwpmath::Matrix44Affine *
    GetRelativeTransform() const;

    rwpmath::Matrix44Affine *
    GetRelativeTransform();

    void SetLocalTransform(rwpmath::Matrix44Affine::InParam localTransform);

    rwpmath::Matrix44Affine GetLocalTransform() const;

    RwpBool
    GetBBox(const rwpmath::Matrix44Affine *tm,
        RwpBool tight, AABBox &bBox) const;

    rwpmath::Vector3
        GetBBoxDiag() const;

    RwpBool
    CreateGPInstance(GPInstance &instance, const rwpmath::Matrix44Affine *tm) const;

    RwpBool
    LineSegIntersect(const rwpmath::Vector3 & pt1,
                    const rwpmath::Vector3 & pt2,
                    const rwpmath::Matrix44Affine *tm,
                    VolumeLineSegIntersectResult &result,
                    const float fatness = 0.0f) const;

    RwpBool
    GetMoments(rwpmath::Matrix44 &moments) const;

    void
    GetAsTriangles(void * context, TriangleCallback triangleCallback) const;

    /**
    Contains info needed for object allocation usually used when deserializing.
    */
    struct ObjectDescriptor
    {
        template <class Archive>
        void Serialize(Archive & /*ar*/, uint32_t /*version*/)
        {
        }
    };

    /**
    Returns the resource descriptor for the Volume object type.
    \return the Volume resource descriptor.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(const ObjectDescriptor & /*objDesc*/)
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    /**
    Returns the information needed for object allocation.
    \return the information needed to allocate this object when deserializing
    */
    const ObjectDescriptor GetObjectDescriptor() 
    {
        return ObjectDescriptor();
    }

    static Volume *
    Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & /*objDesc*/);

    /**
    \internal
    \brief
    Volume virtual function VTable in a memdump friendly format.
    \importlib rwccore
    */
    struct VTable
    {
        // this should be consistent with Volume::volumeType
        rw::collision::VolumeType     typeID;  ///< the id number for the volume type
        GetBBoxFn           getBBox;           ///< pointer to a function to compute the bounding box
        GetBBoxDiagFn       getBBoxDiag;
        GetIntervalFn       getInterval;       ///< not used
        GetMaximumFeatureFn getMaximumFeature; ///< not used
        CreateGPInstanceFn  createGPInstance;  ///< a pointer to a function to create the generalized primitive instance data.
        LineSegIntersectFn  lineSegIntersect;  ///< a pointer to a function to get the intersection of a line with the volume.
        ReleaseFn           release;           ///< a pointer to a function to release any structures pertinant to the volume.
        const char          *name;             ///< a printable name of the volume type, such as "SphereVolume"
        uint32_t            flags;             ///< reserved for future use.
        GetMomentsFn        getMoments;        ///< a pointer to a function to get the moments of the volume.
        GetAsTrianglesFn    getAsTriangles;    ///< a pointer to a function to get the volume as triangles.
        ClearAllProcessedFlagsFn      clearAllProcessedFlags; ///< a pointer to a function to clear all volume processed flags
        ApplyUniformScaleFn           applyUniformScale;      ///< a pointer to a function to apply uniform scaling to volume
    };

    static VTable *vTableArray[rw::collision::VOLUMETYPENUMINTERNALTYPES];

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        EA_ASSERT(vTableArray != NULL);

        ar & EA_SERIALIZATION_NAMED_VALUE(groupID);
        ar & EA_SERIALIZATION_NAMED_VALUE(surfaceID);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_flags);
        ar & EA_SERIALIZATION_NAMED_VALUE(radius);
        ar & EA_SERIALIZATION_NAMED_VALUE(transform);

        // We used to serialize volume type from the vtable, and we'll keep the same
        // naming to avoid needing to update the version (which breaks serializing derived classes)
        ar & ::EA::Serialization::MakeNamedValue(volumeType, "vTable");

        switch (volumeType)
        {
        case rw::collision::VOLUMETYPECAPSULE:
            ar & EA_SERIALIZATION_NAMED_VALUE(capsuleData.hh);
            break;
        case rw::collision::VOLUMETYPECYLINDER:
            ar & EA_SERIALIZATION_NAMED_VALUE(cylinderData.hh);
            ar & EA_SERIALIZATION_NAMED_VALUE(cylinderData.innerRadius);

            break;
        case rw::collision::VOLUMETYPETRIANGLE:
            ar & EA_SERIALIZATION_NAMED_VALUE(triangleData.edgeCos0);
            ar & EA_SERIALIZATION_NAMED_VALUE(triangleData.edgeCos1);
            ar & EA_SERIALIZATION_NAMED_VALUE(triangleData.edgeCos2);
            break;
        case rw::collision::VOLUMETYPEBOX:
            ar & EA_SERIALIZATION_NAMED_VALUE(boxData.hx);
            ar & EA_SERIALIZATION_NAMED_VALUE(boxData.hy);
            ar & EA_SERIALIZATION_NAMED_VALUE(boxData.hz);
            break;
        case rw::collision::VOLUMETYPEAGGREGATE:
            ar.TrackPointer(aggregateData.agg);
            break;
        case VOLUMETYPECONVEXHULL:
        case VOLUMETYPECUSTOM:
            ar.TrackPointer(customData.data);
            ar & EA_SERIALIZATION_NAMED_VALUE(customData.type);
            break;
        default:
            break;
        }

    }

    // Collision primitive engine
    static RwpBool
    InitializeVTable(void);

    static RwpBool
    ReleaseVTable(void);

protected:
    // Local transform
    mutable rwpmath::Matrix44Affine transform;  ///< relative transform from the volume to its parent.
                                                 ///  For example, the parent could be an aggregate volume
                                                 ///  or a physics PartDefinition.

    // Memdump friendly virtual functions
    uint32_t volumeType;		///< type of volume. This type is used to fetch the table of function pointers to common methods that all volume types must support.
#if (EA_PLATFORM_PTR_SIZE == 8)
    uint32_t padding[1];
#endif

    // Space for type specific data
    // This structure has a size of 12 bytes on 32-bit platforms, 16 bytes on 64-bit platforms.
    // This structure has an alignment equal to the platform pointer alignment.
    union
    {
        AggregateSpecificData   aggregateData;          // ptr
        SphereSpecificData      sphereData;             // ptr
        CapsuleSpecificData     capsuleData;            // 1 x float
        TriangleSpecificData    triangleData;           // 3 x float
        BoxSpecificData         boxData;                // 3 x float
        CylinderSpecificData    cylinderData;           // 2 x float
        CustomSpecificData      customData;             // 1 x ptr, 1 x uint32_t
#if (EA_PLATFORM_PTR_SIZE == 4)
        uint32_t                paddingData[3];         // explicit padding to 12 bytes on 32-bit platforms
#elif (EA_PLATFORM_PTR_SIZE == 8)
        uint32_t                paddingData[4];         // explicit padding to 16 bytes on 64-bit platforms
#endif // defined(EA_PLATFORM_PTR_SIZE == 8)
    };

    // Radius or fatness of primitive
    float       radius;                       ///< Radius or fatness of primitive

    // Identifier for group collision culling
    uint32_t      groupID;                       ///< Identifier for group collision culling

    // Identifier for physics material or other applications
    uint32_t      surfaceID;                     ///< Identifier for physics material or other applications

    // Volume flags
    uint32_t      m_flags;                       ///< Volume flags

#if (EA_PLATFORM_PTR_SIZE == 8)
    uint32_t      m_padding[2];                  // explicit padding to pad to 16 byte alignment on 64-bit architectures
#endif // (EA_PLATFORM_PTR_SIZE == 8)
};


// ***********************************************************************************************************
// Inline Functions

/**
\brief
Get the type of this particular collision volume.

A volume can be either be a simple primitive volume type or an aggregate
volume. An aggregate volume is a single type with acccess to an aggregate. There are
many different types of aggregate.

\return The volume type.
*/
inline rw::collision::VolumeType
Volume::GetType() const
{
    return static_cast<rw::collision::VolumeType>(volumeType);
}

/**
\brief
Get the name of the volume type.

\return Pointer to a string containing the name of the volume type
*/
inline const char *
Volume::GetTypeName()
{
    return Volume::vTableArray[volumeType]->name;
}

/**
\brief
Get the volumes radius.

The radius of a volume is also its fatness. In the case of a sphere or
capsule, the radius is the basic geometric radius. In the case of a box or triangle, the
radius is a constant fatness added to the underlying geometry.

\return The volumes radius.
*/
inline const float &
Volume::GetRadius() const
{
    return  radius ;
}

/**
\brief
Set the volumes radius.

The radius of a volume is also its fatness. In the case of a sphere or
capsule, the radius is the basic geometric radius. In the case of a box or triangle, the
radius is a constant fatness added to the underlying geometry.

\param rad The volume radius.
*/
inline void
Volume::SetRadius(float rad)
{
    EA_ASSERT(rad >= 0.0f);
    radius = rad;
}

/**
\brief
Get the Volumes group number.

Grouping is the mechanism for doing large scale culling between sets of volumes
during collision queries.

\return The group number
*/
inline uint32_t
Volume::GetGroup() const
{
    return  groupID ;
}

/**
\brief
Set the Volumes group number.

Grouping is the mechanism for doing large scale culling between sets of volumes
during collision queries.

\param group The group number.
*/
inline void
Volume::SetGroup(uint32_t group)
{
    groupID = group;
}

/**
\brief
Get the Volumes surface id number.

You can assign each primitive volume its own surface id, and this value can be used for any game
specific effects.  For example, you can use the surface id to set the material properties (friction
and restitution) of a contact between two volumes.
\return The surface id number
\see Volume::SetSurface
*/
inline uint32_t
Volume::GetSurface() const
{
    return  surfaceID ;
}

/**
\brief
Set the surface id of the volume.

Set the Volumes surface type number.  The surface type is to determine friction and restitution
properties in physics.  It can be used for other applications also.
\param surface The surface type number.
\see Volume::GetSurface
*/
inline void
Volume::SetSurface(uint32_t surface)
{
    surfaceID = surface;
}

/**
\brief
Get the Volume's enabled flag.

When the volume is not enabled, it is ignored for collision purposes.
Volumes can be enabled or disabled using SetEnabled method.
\return TRUE if the volume is enabled for collision.
\see Volume::SetEnabled
*/
inline RwpBool
Volume::IsEnabled() const
{
    return static_cast<RwpBool>((m_flags & VOLUMEFLAG_ISENABLED) != 0 );
}

/**
\brief
Enables or disables the volume.

When the volume is not enabled, it is ignored for collision purposes.
\param whetherEnabled you set this to true to enable collision for this volume, and false to disable.
\see Volume::IsEnabled
*/
inline void
Volume::SetEnabled(RwpBool whetherEnabled)
{
    if ( whetherEnabled )
    {
        m_flags |= VOLUMEFLAG_ISENABLED;
    }
    else
    {
        m_flags &= ~VOLUMEFLAG_ISENABLED;
    }
}

/**
\brief Get the volume flags.

The flags control various collision properties of the volume.  Mostly it is used for triangles.
\see VolumeFlags, Volume::IsEnabled
*/
inline uint32_t
Volume::GetFlags() const
{
    return m_flags;
}


/**
\brief Set the volume flags.

The flags control various collision properties of the volume.  Mostly it is used for triangles.
\see VolumeFlags, Volume::SetEnabled
*/
inline void
Volume::SetFlags(uint32_t newflags)
{
    m_flags = newflags;
}


/**
\brief Set the volume processed flag.

Sets the processed flag. This will not set aggregate processed flags.

\see VolumeFlags, Volume::SetEnabled, ClearAllProcessedFlags
*/
inline void
Volume::SetProcessedFlag()
{
    // Set processed flag
    m_flags |= VOLUMEFLAG_ISPROCESSED;
}


/**
\internal
\brief Clears processed flag of this volume only
Note: This will not clear aggregate flags
\see ClearAllProcessedFlags
*/
inline void
Volume::ClearProcessedFlag()
{
    // Clear volume processed flag
    m_flags &= ~VOLUMEFLAG_ISPROCESSED;
}


/**
\brief Clears all processed flags of current volume and associated volumes

Clears the volume processed flag. Forwards call for aggregate to handle 
child flag clearing.
\see VolumeFlags, Volume::SetEnabled, SetProcessedFlag
*/
inline void
Volume::ClearAllProcessedFlags()
{
    ClearProcessedFlag();

    // Clear aggregate processed flags
    if (Volume::vTableArray[volumeType]->clearAllProcessedFlags)
    {
        (this->*(Volume::vTableArray[volumeType]->clearAllProcessedFlags))();
    }
}


/**
\brief Apply uniform scaling to the volume.

Applies uniform scaling to the volume by calling corresponding scale virtual function

\param scale The scale factor to apply to the mapped array
\param useProcessedFlags Flag to specify whether to use or ignore processed flags
\see ClearAllProcessedFlags, SetProcessedFlag
*/
inline void
Volume::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    EA_ASSERT(scale > 0.0f);

    if (Volume::vTableArray[volumeType]->applyUniformScale)
    {
        (this->*(Volume::vTableArray[volumeType]->applyUniformScale))(scale, useProcessedFlags);
    }
}


/**
\brief
Get the Volumes relative transform.
\deprecated This method is deprecated - use the GetLocalTransform method instead.

This is the transform between this volume and its parent reference frame. In
a mapped array aggregate, this is the relative of each component volume.

\return A const pointer to the relative transform
*/
inline const rwpmath::Matrix44Affine *
Volume::GetRelativeTransform() const
{
    rwcDEPRECATED("Please use GetLocalTransform instead.");
    return &transform;
}


/**
\brief
Get the Volumes relative transform.
\deprecated This method is deprecated - use the GetLocalTransform/SetLocalTransform
methods instead.

This is the transform between this volume and its parent reference frame. In
a mapped array aggregate, this is the relative of each component volume.

\return A pointer to the relative transform.
*/
inline rwpmath::Matrix44Affine *
Volume::GetRelativeTransform()
{
    rwcDEPRECATED("Please use GetLocalTransform/SetLocalTransform instead.");
    return &transform;
}


/**
\brief
Set the Volumes local transform.

This is the transform between this volume and its parent reference frame. In
a mapped array aggregate, this is the relative of each component volume.

This method should not be called on TriangleVolumes as the local transform is
used to store the vertices of the triangle in parent space.
*/
inline void
Volume::SetLocalTransform(rwpmath::Matrix44Affine::InParam localTransform)
{
    EA_ASSERT_MSG(volumeType != VOLUMETYPETRIANGLE, "SetLocalTransform should not be called on TriangleVolumes.");
    transform = localTransform;
}


/**
\brief
Get the Volumes local transform.

This is the transform between this volume and its parent reference frame. In
a mapped array aggregate, this is the relative of each component volume.

This method should not be called on TriangleVolumes as the local transform is
used to store the triangle vertices rather than a transform. To get the triangle
vertices the TriangleVolume::GetPoints API should be used.

\return the local transform of the volume.
*/
inline rwpmath::Matrix44Affine
Volume::GetLocalTransform() const
{
    EA_ASSERT_MSG(volumeType != VOLUMETYPETRIANGLE, "GetLocalTransform should not be called on TriangleVolumes.");
    return transform;
}


/**
\brief
Returns an axis aligned bounding box for the volume.

The bounding box of the volume is transformed by the volume relative transform and by the input parent
transform if it is not NULL.  The transformations may translate and/or rotate the bbox which may increase
the size of the bbox.

\param tm optional input parent transform of this volumes parent frame.  May be NULL.
\param tight Get the smallest bounding around the volume. A tight bounding box is calculated around
the rotated volume and can incur a performance penalty. A non-tight bounding box is fitted around the
transformed bounding box calculated from the non-rotated volume.
\param bBox    output reference to a structure to contain the new bounding box.

\return TRUE if successful, FALSE if the volume does not have a bounding box.
*/
inline RwpBool
Volume::GetBBox(const rwpmath::Matrix44Affine *tm,
        RwpBool tight, AABBox &bBox) const
{
	return (this->*(Volume::vTableArray[volumeType]->getBBox))(tm, tight, bBox);
}


RW_COLLISION_FORCE_INLINE rwpmath::Vector3
Volume::GetBBoxDiag() const
{
    return (this->*(Volume::vTableArray[volumeType]->getBBoxDiag))();
}

/*
\toedit
\classinternal
Create the generalized primitive instance data.


This precomputes some data that is used for the generalized primitive intersection test.
The face directions and edge directions are computed in world coordinates,
the radius, and several virtual function pointers are loaded into the instance structure.
\param instance output  generalized primitive instance data.
\param tm parent transformation to applied to the volume.  May be NULL.
\return TRUE on success.
*/
inline RwpBool
Volume::CreateGPInstance(GPInstance &instance, const rwpmath::Matrix44Affine *tm) const
{
    return (this->*(Volume::vTableArray[volumeType]->createGPInstance))(instance, tm);
}

/**
\brief
Directly perform a line test against this volume.

\param pt1 Start point of the line segment
\param pt2 End point of the line segment
\param tm World transform of this volumes parent frame. Passing NULL implies identity.
\param result Structure to contain the intersection result.
\param fatness The fatness of the line

The function directly calls the specific volume intersect test with the input line segment.
It is only supported for primitives and will assert if this volume is an
aggregate. If you need to test a line segment against an aggregate volume then
use the  rw::collision::VolumeLineQuery mechanism which can handle both primitives and aggregates.

\return TRUE if the volume is intersected by the line segment.
*/
inline RwpBool
Volume::LineSegIntersect(const rwpmath::Vector3 & pt1,
                    const rwpmath::Vector3 & pt2,
                    const rwpmath::Matrix44Affine *tm,
                    VolumeLineSegIntersectResult &result,
                    const float fatness) const
{
    EA_ASSERT_MSG(Volume::vTableArray[volumeType]->lineSegIntersect, ("This is not a Primitive volume"));
    if (!this->IsEnabled())
    {
        return FALSE;
    }
    return (this->*(Volume::vTableArray[volumeType]->lineSegIntersect))(pt1, pt2, tm, result, fatness);
}


/**
Get the moments for the volume.

http://globaltechdocs.ea.com/RWPhysicsDev:VolumeMoments

\return TRUE if the volume has moments.
*/
inline RwpBool
Volume::GetMoments(rwpmath::Matrix44 &moments) const
{
    if (Volume::vTableArray[volumeType]->getMoments)
    {
        return (this->*(Volume::vTableArray[volumeType]->getMoments))(moments);
    }

    return FALSE;
}


/**
Gets the volume as a set of triangles. The triangleCallback is called for every triangle.
*/
inline void
Volume::GetAsTriangles(void * context, TriangleCallback triangleCallback) const
{
    if (Volume::vTableArray[volumeType]->getAsTriangles)
    {
        (this->*(Volume::vTableArray[volumeType]->getAsTriangles))(context, triangleCallback);
    }
}


/* ===================== FRACTION =================================================================
Note, the following struct and method should be removed after linequery are converted to use the primitives/features packages.
*/

/**
\internal
\brief A fraction.

 This class is used to store a ratio, and the performance advantage is that we can postpone
the divide operation until it is necessary, and until then, fractions can be compared, tested for zero,
and so on.

\importlib rwccore
*/
struct Fraction
{
    float    num;
    float    den;
};

/**
\internal

\brief
Compare two fractions and return true if the first is less than the second.
Note the denominators must have the same sign.
\param f1 a fraction
\param f2 another fraction
\return true if the fraction f1 is less than f2.  Thus, f1.num * f2.den < f2.num * f1.den.
*/
inline RwpBool
FracLT(const Fraction &f1, const Fraction &f2)
{
    EA_ASSERT(f1.den * f2.den > 0.0f);
    return static_cast<RwpBool>(f1.num * f2.den < f2.num * f1.den);
}


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_VOLUME_H
