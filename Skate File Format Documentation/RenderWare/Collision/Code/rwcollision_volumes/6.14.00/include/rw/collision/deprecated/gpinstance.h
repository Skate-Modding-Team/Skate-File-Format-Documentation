// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DERECATED_GPINSTANCE_H
#define PUBLIC_RW_COLLISION_DERECATED_GPINSTANCE_H

#include "rw/collision/common.h"
#include "rw/collision/aabbox.h"


namespace rw
{
namespace collision
{


// Enabling RW_COLLISION_GP_INSTANCE_PACKED is not a win on xbox2, first guess is the lack of embedded function pointers
// causing a performance hit. It's also slower because of access to packed data members.
//
// PAB: TODO: Improve packed GPInstance accessors - e.g. use unaligned store instead of element-wise conversion operator for
// assigning Vector3's into Vector3U_32's on PS3 PPU (also possible on xbox2).
//
//#define RW_COLLISION_GP_INSTANCE_PACKED // Expect to use for SPU processed instances

#if !defined(EA_PLATFORM_PS3_PPU) && !defined(EA_PLATFORM_PS3_SPU)
    //#define RW_COLLISION_GP_INSTANCE_PACKED  // Expect to use for SPU processed instances
    #define RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS
#endif


#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    #define rw_COLLISION_GP_INSTANCE_VOLUME_METHODS mVolumeMethods         // embedded in instance structure
#else // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    #define rw_COLLISION_GP_INSTANCE_VOLUME_METHODS sVolumeMethods[Type()] // global method table
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)


/**
\deprecated GPInstance is no longer used internally and will be removed from a subsequent release.
\brief Legacy structure for instance of a "general primitive" - a convex object with fatness.

Currently we support four kinds of primitives: sphere, capsule, triangle (with optional "fatness")
and box, also with optional "fatness".
Sphere and capsule are internally implemented as respectively a point and a line segment with "fatnesses"
equal to their radii.

Each instance declares its face normals and its edge directions - these will be used to generate
the best separating axis between a pair of primitives.

Given two GPinstances, the best separating axis is the axis with minimum overlap among the candidate
axes.  The candidate separating axes are the face vectors of both instances, and all possible non-zero edge
vector cross products between the two instances.   For example, a box instance has three face directions
and three edge directions.  The candidate axes between two boxes is 3 + 3 faces directions and 3 x 3
edge combinations, for a total of 6 + 9 = 15 candidates.  But some of the edge cross products might be
zero which reduces the number of candidates.

\importlib rwccore
*/
struct GPInstance
{
    /**
    \brief Enumeration of the current GPInstance types.

    \hideinitializer
      */
    enum VolumeType
    {
        /* If you modify this list then update GPInstance::sVolumeMethods */
        UNUSED = 0,  ///< Dummy null instanced primtive type
        SPHERE,      ///< Sphere instanced primtive type /see rw::collision::GPSphere
        CAPSULE,     ///< Capsule instanced primtive type /see rw::collision::GPCapsule
        TRIANGLE,    ///< Triangle instanced primtive type /see rw::collision::GPTriangle
        BOX,         ///< Box instanced primtive type /see rw::collision::GPBox
        CYLINDER,    ///< Cylinder instanced primtive type /see rw::collision::GPCylinder

        NUMINTERNALTYPES,

        FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
    };

    /**
    \brief Enumeration for GPInstance flags.
      */
    enum VolumeFlag
    {
        FLAG_TRIANGLEONESIDED     = 0x0010, ///< contact with back face of triangle is culled.
        FLAG_TRIANGLEEDGE0CONVEX  = 0x0020, ///< the edge from p0 to p1 is convex.
        FLAG_TRIANGLEEDGE1CONVEX  = 0x0040, ///< the edge from p1 to p2 is convex.
        FLAG_TRIANGLEEDGE2CONVEX  = 0x0080, ///< the edge from p2 to p0 is convex.
        FLAG_TRIANGLEUSEEDGECOS   = 0x0100, ///< use the edge cosine values to limit the permissible edge contact normal.
        FLAG_TRIANGLEVERT0DISABLE = 0x0200, ///< disable collisions with vertex 0.
        FLAG_TRIANGLEVERT1DISABLE = 0x0400, ///< disable collisions with vertex 1.
        FLAG_TRIANGLEVERT2DISABLE = 0x0800, ///< disable collisions with vertex 2.

        /** \brief default flags for a new triangle: two-sided and all edges fully convex.
          */
        FLAG_TRIANGLEDEFAULT      = FLAG_TRIANGLEUSEEDGECOS + FLAG_TRIANGLEEDGE0CONVEX
                                                            + FLAG_TRIANGLEEDGE1CONVEX
                                                            + FLAG_TRIANGLEEDGE2CONVEX,
        FLAG_TRIANGLEOLDMASK      = FLAG_TRIANGLEONESIDED   + FLAG_TRIANGLEEDGE0CONVEX
                                                            + FLAG_TRIANGLEEDGE1CONVEX
                                                            + FLAG_TRIANGLEEDGE2CONVEX,

        FLAG_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
    };

    // volume specific method definitions
    typedef void (GPInstance::*GetBBoxFn)          ( AABBox &bbox ) const;

    // vTable for volume specific methods
    /**
    \importlib rwccore
    */
    struct VolumeMethods
    {
        GetBBoxFn           mGetBBox;
    };

    // combined vTable for all primitives
    static VolumeMethods sVolumeMethods[GPInstance::NUMINTERNALTYPES];

    // result from GetContactPoints API, sufficient information to generate potential contact constraints
    /**
    \importlib rwccore
    */
    struct ContactPoints
    {
        struct PointPair
        {
            rwpmath::Vector3 p1;
            rwpmath::Vector3 p2;
        };

        uintptr_t volumeTag1;
        uintptr_t volumeTag2;
        uint32_t userTag1;
        uint32_t userTag2;
        uint32_t numPoints;
        rwpmath::Vector3 normal;
        PointPair pointPairs[16];
    };

#if defined(RW_COLLISION_GP_INSTANCE_PACKED)

    // Generic Primitive data                     Offset
    rw::math::fpu::Vector3U_32 mPos;            // 0x00
    float mFatness;                           // 0x0c

    rw::math::fpu::Vector3U_32 mFaceNormal0;    // 0x10
    uint32_t  mFlags;                           // 0x1c

    rw::math::fpu::Vector3U_32 mFaceNormal1;    // 0x20
    uint8_t   mVolumeType;                      // 0x2c
    uint8_t   mNumFaceNormals;                  // 0x2d
    uint8_t   mNumEdgeDirections;               // 0x2e
    uint8_t   mPad0;                            // 0x2f

    rw::math::fpu::Vector3U_32 mFaceNormal2;    // 0x30
    uint32_t  mUserTag;                         // 0x3c

    rw::math::fpu::Vector3U_32 mEdgeDirection0; // 0x40
    float mEdgeData0;                         // 0x4c

    rw::math::fpu::Vector3U_32 mEdgeDirection1; // 0x50
    float mEdgeData1;                         // 0x5c

    rw::math::fpu::Vector3U_32 mEdgeDirection2; // 0x60
    float mEdgeData2;                         // 0x6c

    // dimensions are expected to be contiguous
    rw::math::fpu::Vector3U_32 mDimensions;     // 0x70
    uintptr_t mVolumeTag;                       // 0x7c

#else //defined(RW_COLLISION_GP_INSTANCE_PACKED)

    // Generic Primitive data
    rwpmath::Vector3 mPos;               ///< the center of the shape.
    rwpmath::Vector3 mFaceNormals[3];    ///< an array of face direction vectors
    rwpmath::Vector3 mEdgeDirections[3]; ///< an array of edge direction vectors
    rwpmath::Vector3 mDimensions;        ///<
    uintptr_t mVolumeTag;                ///< the identifier of this volume primitive in an aggregate hierarchy
    float mFatness;                     ///< radius of the shape
    uint32_t mUserTag;                    ///<
    uint8_t mNumFaceNormals;              ///< number of entries in the \e  face_normals array
    uint8_t mNumEdgeDirections;           ///< number of entries in the \e  edge_directions array
    VolumeType mVolumeType;               ///<
    uint32_t mFlags;                      ///<

    float mEdgeData[3];

#endif //defined(RW_COLLISION_GP_INSTANCE_PACKED)


#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    VolumeMethods mVolumeMethods;
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

#if defined(EA_DEBUG)
    RW_COLLISION_FORCE_INLINE GPInstance() { mVolumeType = GPInstance::UNUSED; };
#endif

    RW_COLLISION_FORCE_INLINE VolumeMethods *GetVolumeMethods(GPInstance::VolumeType volumeType);

#if defined(RW_COLLISION_GP_INSTANCE_PACKED)

    // generic accessors used by primitive pair query and for separation computation
    RW_COLLISION_FORCE_INLINE GPInstance::VolumeType Type() const                        { return static_cast<GPInstance::VolumeType>(mVolumeType); };
    RW_COLLISION_FORCE_INLINE uint32_t               Flags() const                       { return mFlags; };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3      Pos() const                         { return static_cast<const rwpmath::Vector3>(mPos); };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3      FaceNormal(uint32_t index)  const   { return reinterpret_cast<const rwpmath::Vector3*>(&mFaceNormal0)[index]; };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3      EdgeDirection(uint32_t index) const { return reinterpret_cast<const rwpmath::Vector3*>(&mEdgeDirection0)[index]; };
    RW_COLLISION_FORCE_INLINE const float         &Fatness() const                     { return mFatness; };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat     FatnessVec() const                  { return rwpmath::VecFloat(mFatness); };

#else // defined(RW_COLLISION_GP_INSTANCE_PACKED)

    // generic accessors used by primitive pair query and for separation computation
    RW_COLLISION_FORCE_INLINE GPInstance::VolumeType Type() const                        { return mVolumeType; };
    RW_COLLISION_FORCE_INLINE uint32_t               Flags() const                       { return mFlags; };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3      Pos() const                         { return mPos; };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3      FaceNormal(uint32_t index)  const   { return mFaceNormals[index]; };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3      EdgeDirection(uint32_t index) const { return mEdgeDirections[index]; };
    RW_COLLISION_FORCE_INLINE const float         &Fatness() const                     { return mFatness; };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat     FatnessVec() const                  { return rwpmath::VecFloat(mFatness); };

#endif // defined(RW_COLLISION_GP_INSTANCE_PACKED)

    // wrappers for volume specific method dispatch
    RW_COLLISION_FORCE_INLINE void GetBBox          (AABBox &bbox) const;

protected:

    // accessors for volume specific initialization & retrieval only
#if defined(RW_COLLISION_GP_INSTANCE_PACKED)
    RW_COLLISION_FORCE_INLINE void SetPos(rwpmath::Vector3::InParam pos)                                     { mPos = static_cast<const rw::math::fpu::Vector3U_32>(pos); };
    RW_COLLISION_FORCE_INLINE void SetFaceNormal(uint32_t index, rwpmath::Vector3::InParam normal)           { reinterpret_cast<rw::math::fpu::Vector3U_32&>(reinterpret_cast<rwpmath::Vector3*>(&mFaceNormal0)[index])    = static_cast<const rw::math::fpu::Vector3U_32>(normal); };
    RW_COLLISION_FORCE_INLINE void SetEdgeDirection(uint32_t index, rwpmath::Vector3::InParam edgeDirection) { reinterpret_cast<rw::math::fpu::Vector3U_32&>(reinterpret_cast<rwpmath::Vector3*>(&mEdgeDirection0)[index]) = static_cast<const rw::math::fpu::Vector3U_32>(edgeDirection); };
    RW_COLLISION_FORCE_INLINE void SetEdgeData(uint32_t index, float data)                                 { static_cast<float*>(&mEdgeData0)[index<<2] = data; };
    RW_COLLISION_FORCE_INLINE const float &EdgeData(uint32_t index) const                                  { return (&mEdgeData0)[index<<2]; };
#else // defined(RW_COLLISION_GP_INSTANCE_PACKED)
    RW_COLLISION_FORCE_INLINE void SetPos(rwpmath::Vector3::InParam pos)                                     { mPos = pos; };
    RW_COLLISION_FORCE_INLINE void SetFaceNormal(uint32_t index, rwpmath::Vector3::InParam normal)           { mFaceNormals[index] = normal; };
    RW_COLLISION_FORCE_INLINE void SetEdgeDirection(uint32_t index, rwpmath::Vector3::InParam edgeDirection) { mEdgeDirections[index] = edgeDirection; };
    RW_COLLISION_FORCE_INLINE void SetEdgeData(uint32_t index, float data)                                 { mEdgeData[index] = data; };
    RW_COLLISION_FORCE_INLINE const float &EdgeData(uint32_t index) const                                  { return mEdgeData[index]; };
#endif // defined(RW_COLLISION_GP_INSTANCE_PACKED)

};


RW_COLLISION_FORCE_INLINE GPInstance::VolumeMethods *
GPInstance::GetVolumeMethods(rw::collision::GPInstance::VolumeType volumeType)
{
    return &sVolumeMethods[volumeType];
}


RW_COLLISION_FORCE_INLINE void
GPInstance::GetBBox(AABBox &bbox) const
{
    EA_ASSERT(Type() > GPInstance::UNUSED);
    EA_ASSERT(Type() < GPInstance::NUMINTERNALTYPES);
    (this->*(rw_COLLISION_GP_INSTANCE_VOLUME_METHODS.mGetBBox))(bbox);
}


// type specific interfaces

// sphere

/**
\brief Legacy representation of a sphere as a GPInstance.
\deprecated GPSphere is no longer used internally and will be removed from a subsequent release.
\importlib rwccore
\internal
*/
struct GPSphere: public GPInstance
{
public:
    RW_COLLISION_FORCE_INLINE void Initialize(rwpmath::Vector3::InParam center, const float &radius, uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag);

    // sphere volume specific accessors
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3 Center() const { return Pos(); };
    RW_COLLISION_FORCE_INLINE const float  &Radius() const { return mFatness; };

    // GPInstance::VolumeMethods
    void GetBBox( AABBox &bbox ) const;
};


// Generalized Primitive Capsule Volume
/**
\brief Legacy representation of a capsule as a GPInstance.
\deprecated GPCapsule is no longer used internally and will be removed from a subsequent release.
\importlib rwccore
\internal
*/
struct GPCapsule : public GPInstance
{
public:
    RW_COLLISION_FORCE_INLINE void Initialize(rwpmath::Vector3::InParam center, const float &radius, rwpmath::Vector3::InParam axis, const float &halfHeight, uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag);

    // capsule volume specific accessors
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Center()        const { return Pos(); };
    RW_COLLISION_FORCE_INLINE const float   &Radius()        const { return mFatness; };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat RadiusVec()     const { return rwpmath::VecFloat(mFatness); };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Axis()          const { return EdgeDirection(0); };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat HalfHeight()    const { return mDimensions.GetX(); };

    // GPInstance::VolumeMethods
    void GetBBox( AABBox &bbox ) const;
};


// Generalized Primitive Box Volume
/**
\brief Legacy representation of a fat box as a GPInstance.
\deprecated GPBox is no longer used internally and will be removed from a subsequent release.
\importlib rwccore
\internal
*/
struct GPBox : public GPInstance
{
public:
    RW_COLLISION_FORCE_INLINE void Initialize(rwpmath::Vector3::InParam center, rwpmath::Vector3::InParam faceNormal0, rwpmath::Vector3::InParam faceNormal1, rwpmath::Vector3::InParam faceNormal2,
        rwpmath::Vector3::InParam dimensions, const float &fatness, uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag);

    // box volume specific accessors
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Center()                    const { return Pos(); };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat HalfSize(uint32_t index)    const { return mDimensions.GetComponent(static_cast<int32_t>(index)); };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  HalfSizeDimensionsVec() const { return mDimensions; };

    // GPInstance::VolumeMethods
    void GetBBox( AABBox &bbox ) const;

private:
    RW_COLLISION_FORCE_INLINE void SetDimensions(rwpmath::Vector3::InParam dimensions) { mDimensions = dimensions; };
};


// Generalized Primitive Triangle Volume
/**
\brief Legacy representation of a fat triangle as a GPInstance.
\deprecated GPTriangle is no longer used internally and will be removed from a subsequent release.
\importlib rwccore
\internal
*/
struct GPTriangle : public GPInstance
{
public:
    // TODO: REMOVE EdGECOS AS VECTOR CONSTRUCTOR
    RW_COLLISION_FORCE_INLINE void Initialize(rwpmath::Vector3::InParam p1, rwpmath::Vector3::InParam p2, rwpmath::Vector3::InParam p3, const float &fatness, uint32_t volumeFlags, rwpmath::Vector3::InParam edgeCosines, uintptr_t volumeTag, uint32_t userTag);
    RW_COLLISION_FORCE_INLINE void Initialize(rwpmath::Vector3::InParam p1, rwpmath::Vector3::InParam p2, rwpmath::Vector3::InParam p3, const float &fatness, uint32_t volumeFlags, rwpmath::Vector3::InParam edgeCosines, uintptr_t volumeTag, uint32_t userTag, rwpmath::Vector3::InParam normal);
    RW_COLLISION_FORCE_INLINE void Initialize(rwpmath::Vector3::InParam p1, rwpmath::Vector3::InParam p2, rwpmath::Vector3::InParam p3, const float &fatness, uint32_t volumeFlags, float edgeCosine0, float edgeCosine1, float edgeCosine2, uintptr_t volumeTag, uint32_t userTag, rwpmath::Vector3::InParam normal);

    // triangle volume specific accessors
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Normal()  const { return FaceNormal(0); };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Vertex0() const { return Pos(); };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Vertex1() const { return FaceNormal(1); };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Vertex2() const { return FaceNormal(2); };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat EdgeLength(uint32_t index)    const { return mDimensions.GetComponent(static_cast<int32_t>(index)); };

    RW_COLLISION_FORCE_INLINE void               GetEdgeLengths(rwpmath::VecFloat &edgeLength0, rwpmath::VecFloat &edgeLength1, rwpmath::VecFloat &edgeLength2) const
    {
        edgeLength0 = mDimensions.GetX();
        edgeLength1 = mDimensions.GetY();
        edgeLength2 = mDimensions.GetZ();
    };

    RW_COLLISION_FORCE_INLINE rwpmath::Vector3 EdgeCosines() const { return rwpmath::Vector3(EdgeData(0), EdgeData(1), EdgeData(2)); };

    // GPInstance::VolumeMethods
    void GetBBox( AABBox &bbox ) const;

private:
    RW_COLLISION_FORCE_INLINE void SetEdgeCosines(rwpmath::Vector3::InParam edgeCosines) { SetEdgeData(0, rwpmath::GetFloat(edgeCosines.GetX()));
                                                                                           SetEdgeData(1, rwpmath::GetFloat(edgeCosines.GetY()));
                                                                                           SetEdgeData(2, rwpmath::GetFloat(edgeCosines.GetZ())); };
    RW_COLLISION_FORCE_INLINE void SetEdgeCosines(float edgeCosine0, float edgeCosine1, float edgeCosine2)
                                                                                          { SetEdgeData(0, edgeCosine0);
                                                                                            SetEdgeData(1, edgeCosine1);
                                                                                            SetEdgeData(2, edgeCosine2); };
};


// Generalized Primitive Cylinder Volume
/**
\brief Legacy representation of a fat cylinder as a GPInstance.
\deprecated GPCylinder is no longer used internally and will be removed from a subsequent release.
\importlib rwccore
\internal
*/
struct GPCylinder : public GPInstance
{
public:
    RW_COLLISION_FORCE_INLINE void Initialize(rwpmath::Vector3::InParam center, const float &radius, rwpmath::Vector3::InParam axis, const float &halfHeight, const float &fatness,
         uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag, rwpmath::Vector3::InParam faceNormal0, rwpmath::Vector3::InParam faceNormal1 );

    // cylinder volume specific accessors
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Center()        const { return Pos(); };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat Radius()        const { return mDimensions.GetY(); };
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3  Axis()          const { return EdgeDirection(0); };
    RW_COLLISION_FORCE_INLINE rwpmath::VecFloat HalfHeight()    const { return mDimensions.GetX(); };

    // GPInstance::VolumeMethods
    void GetBBox( AABBox &bbox ) const;
};


RW_COLLISION_FORCE_INLINE void
GPSphere::Initialize( rwpmath::Vector3::InParam center, const float &radius, uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag )
{
    // configure as sphere
    mVolumeType        = GPInstance::SPHERE;
    mNumFaceNormals    = 0;
    mNumEdgeDirections = 0;

    // tags for identification
    mVolumeTag = volumeTag;
    mUserTag   = userTag;

    // set center, radius and flags
    SetPos(center);
    mFatness = radius;
    mFlags = volumeFlags;

#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    mVolumeMethods = *GetVolumeMethods(Type());
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

}


RW_COLLISION_FORCE_INLINE void
GPCapsule::Initialize( rwpmath::Vector3::InParam center, const float &radius, rwpmath::Vector3::InParam axis, const float &halfHeight, uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag )
{
    // configure as capsule
    mVolumeType        = GPInstance::CAPSULE;
    mNumFaceNormals    = 0;
    mNumEdgeDirections = 1;

    // tags for identification
    mVolumeTag = volumeTag;
    mUserTag   = userTag;

    // set center, radius, axis direction and half height
    SetPos(center);
    mFatness = radius;
    SetEdgeDirection(0, axis);
    mDimensions.SetX(halfHeight);
    mFlags = volumeFlags;

#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    mVolumeMethods = *GetVolumeMethods(Type());
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

}


RW_COLLISION_FORCE_INLINE void
GPBox::Initialize( rwpmath::Vector3::InParam center, rwpmath::Vector3::InParam faceNormal0, rwpmath::Vector3::InParam faceNormal1, rwpmath::Vector3::InParam faceNormal2,
                   rwpmath::Vector3::InParam dimensions, const float &fatness, uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag )
{
    // configure as box
    GPInstance::VolumeType type = GPInstance::BOX;
    mVolumeType        = type;
    mNumFaceNormals    = 3;
    mNumEdgeDirections = 3;

    // tags for identification
    mVolumeTag = volumeTag;
    mUserTag   = userTag;

    // set center, face normals, edge directions, dimensions and fatness
    SetPos(center);
    SetFaceNormal   (0, faceNormal0);
    SetEdgeDirection(0, faceNormal0);
    SetFaceNormal   (1, faceNormal1);
    SetEdgeDirection(1, faceNormal1);
    SetFaceNormal   (2, faceNormal2);
    SetEdgeDirection(2, faceNormal2);
    SetDimensions(dimensions);
    mFatness = fatness;
    mFlags = volumeFlags;

#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    mVolumeMethods = *GetVolumeMethods(type);
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

}


RW_COLLISION_FORCE_INLINE void
GPTriangle::Initialize( rwpmath::Vector3::InParam p1, rwpmath::Vector3::InParam p2, rwpmath::Vector3::InParam p3, const float &fatness, uint32_t volumeFlags, rwpmath::Vector3::InParam edgeCosines, uintptr_t volumeTag, uint32_t userTag )
{
    rwpmath::Vector3 normal = rwpmath::Cross(p2 - p1, p3 - p2);
    rwpmath::Normalize(normal, normal);
    Initialize(p1, p2, p3, fatness, volumeFlags, edgeCosines, volumeTag, userTag, normal);

}


RW_COLLISION_FORCE_INLINE void
GPTriangle::Initialize( rwpmath::Vector3::InParam p1, rwpmath::Vector3::InParam p2, rwpmath::Vector3::InParam p3, const float &fatness, uint32_t volumeFlags, rwpmath::Vector3::InParam edgeCosines, uintptr_t volumeTag, uint32_t userTag, rwpmath::Vector3::InParam normal )
{
    Initialize(p1, p2, p3, fatness, volumeFlags, rwpmath::GetFloat(edgeCosines.GetX()), rwpmath::GetFloat(edgeCosines.GetY()), rwpmath::GetFloat(edgeCosines.GetZ()), volumeTag, userTag, normal);

}

RW_COLLISION_FORCE_INLINE rwpmath::Vector3
NormalizeReturnMagnitude3Fast(rwpmath::Vector3 & v1, rwpmath::Vector3 & v2, rwpmath::Vector3 & v3)
{
    rwpmath::VecFloat magSquared1 = MagnitudeSquared(v1);
    rwpmath::VecFloat magSquared2 = MagnitudeSquared(v2);
    rwpmath::VecFloat magSquared3 = MagnitudeSquared(v3);

    rwpmath::Vector3 magnitudesInv(rwpmath::InvSqrtFast(magSquared1), rwpmath::InvSqrtFast(magSquared2), rwpmath::InvSqrtFast(magSquared3));

    v1 *= magnitudesInv.GetX();
    v2 *= magnitudesInv.GetY();
    v3 *= magnitudesInv.GetZ();

    return rwpmath::Vector3(rwpmath::GetVecFloat_One() / magnitudesInv.GetX(),
                             rwpmath::GetVecFloat_One() / magnitudesInv.GetY(),
                             rwpmath::GetVecFloat_One() / magnitudesInv.GetZ());
}


RW_COLLISION_FORCE_INLINE void
GPTriangle::Initialize( rwpmath::Vector3::InParam p1, rwpmath::Vector3::InParam p2, rwpmath::Vector3::InParam p3, const float &fatness, uint32_t volumeFlags,
                       float edgeCosine0, float edgeCosine1, float edgeCosine2, uintptr_t volumeTag, uint32_t userTag, rwpmath::Vector3::InParam normal )
{
    // configure as triangle
    GPInstance::VolumeType type = GPInstance::TRIANGLE;
    mVolumeType        = type;
    mNumFaceNormals    = 1;
    mNumEdgeDirections = 3;

    // tags for identification
    mVolumeTag = volumeTag;
    mUserTag   = userTag;

    // set vertices and fatness
    SetPos          (p1);
    SetFaceNormal(1, p2);
    SetFaceNormal(2, p3);
    mFatness       = fatness;

    // set normal and edge & vertex connectivity info
    SetFaceNormal(0, normal);
    SetEdgeCosines(edgeCosine0, edgeCosine1, edgeCosine2);
    mFlags = volumeFlags;

    // calculate edge directions
    rwpmath::Vector3 edgeDirection0 = p3 - p1;
    rwpmath::Vector3 edgeDirection1 = p2 - p3;
    rwpmath::Vector3 edgeDirection2 = p1 - p2;

    // calculate normalized edge directions & store edge lengths
    mDimensions = NormalizeReturnMagnitude3Fast(edgeDirection0, edgeDirection1, edgeDirection2);

    SetEdgeDirection(0, edgeDirection0);
    SetEdgeDirection(1, edgeDirection1);
    SetEdgeDirection(2, edgeDirection2);

#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    mVolumeMethods = *GetVolumeMethods(type);
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

}


RW_COLLISION_FORCE_INLINE void
GPCylinder::Initialize( rwpmath::Vector3::InParam center, const float &radius, rwpmath::Vector3::InParam axis, const float &halfHeight,
                       const float &fatness, uint32_t volumeFlags, uintptr_t volumeTag, uint32_t userTag, rwpmath::Vector3::InParam faceNormal0, rwpmath::Vector3::InParam faceNormal1 )
{
    // configure as cylinder
    mVolumeType        = GPInstance::CYLINDER;
    mNumFaceNormals    = 1;
    mNumEdgeDirections = 1;

    // tags for identification
    mVolumeTag = volumeTag;
    mUserTag   = userTag;

    // set center, axis, face normals, dimensions and fatness
    SetPos(center);
    SetEdgeDirection(0, axis);
    SetFaceNormal   (0, axis);
    SetFaceNormal   (1, faceNormal0);
    SetFaceNormal   (2, faceNormal1);
    mDimensions.SetX(halfHeight);
    mDimensions.SetY(radius);
    mFatness = fatness;
    mFlags = volumeFlags;

#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    mVolumeMethods = *GetVolumeMethods(Type());
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

}



} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DERECATED_GPINSTANCE_H
