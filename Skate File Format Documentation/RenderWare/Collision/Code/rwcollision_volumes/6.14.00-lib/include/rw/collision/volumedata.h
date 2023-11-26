// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_VOLUMEDATA_H
#define PUBLIC_RW_COLLISION_VOLUMEDATA_H

/*************************************************************************************************************

 File: rwcvolumedata.hpp

 Purpose: Declarations of primitive-specific data structures that are unionised in Volume class
 */


#include "rw/collision/common.h"

namespace rw
{
namespace collision
{

class Aggregate;

/**
\brief
This is the current list of collision arena objects. 

\note
To correctly load an arena containing these object types you will need to register 
the appropriate arena read callbacks:

\code
rw::collision::Volume::RegisterArenaReadCallbacks();
rw::collision::SimpleMappedArray::RegisterArenaReadCallbacks();
rw::collision::KDTreeMappedArray::RegisterArenaReadCallbacks();
rw::collision::TriangleKDTreeProcedural::RegisterArenaReadCallbacks();
rw::collision::AABBox::RegisterArenaReadCallbacks();
rw::collision::ClusteredMesh::RegisterArenaReadCallbacks();

\endcode

\hideinitializer
*/
#ifndef RWCOLLISION_VOLUMES_COMPONENTID
#define RWCOLLISION_VOLUMES_COMPONENTID 0x08
#endif

#ifndef RWCOLLISION_VOLUMES_MAKEOBJECTTYPE
#define RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(_comp, _obj) ((((_comp) & 0xff) << 16) | ((_obj) & 0xff))
#endif

enum ObjectType
{
    RWCOBJECTTYPE_NA                       = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x00), ///< Invalid collision object

    RWCOBJECTTYPE_VOLUME                   = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x01), ///< Collision Volume  rw::collision::Volume
    RWCOBJECTTYPE_SIMPLEMAPPEDARRAY        = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x02), ///< Simple Mapped Array Aggregate  rw::collision::SimpleMappedArrray
    RWCOBJECTTYPE_TRIANGLEKDTREEPROCEDURAL = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x03), ///< Triangle KDTree Procedural Aggregate  rw::collision::TriangleKDTreeProcedural
    RWCOBJECTTYPE_KDTREEMAPPEDARRAY        = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x04), ///< KDTree Mapped Array Aggregate  rw::collision::KDTreeMappedArray
    RWCOBJECTTYPE_BBOX                     = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x05), ///< Axis Aligned Bounding Box  rw::collision::AABBox
    RWCOBJECTTYPE_CLUSTEREDMESH            = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x06), ///< Clustered Mesh Procedural Aggregate \ref rw::collision::ClusteredMesh
    RWCOBJECTTYPE_MESHOPAGGREGATE          = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x07), ///< Data builders' MeshOp Procedural Aggregate
    RWCOBJECTTYPE_OCTREE                   = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x08), ///< Octree Spatial Map \ref rw::collision::Octree
    RWCOBJECTTYPE_HEIGHTFIELD              = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x09), ///< Heightfield  EA::Collision::HeightField

    RWCOBJECTTYPE_FPU_VOLUME               = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x0A), ///< FPU Collision Volume  rw::collision::Volume
    RWCOBJECTTYPE_FPU_SIMPLEMAPPEDARRAY    = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x0B), ///< FPU Simple Mapped Array Aggregate  rw::collision::SimpleMappedArrray
    RWCOBJECTTYPE_FPU_KDTREEMAPPEDARRAY    = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x0C), ///< FPU KDTree Mapped Array Aggregate  rw::collision::KDTreeMappedArray
    RWCOBJECTTYPE_FPU_CLUSTEREDMESH        = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x0D), ///< FPU Clustered Mesh Procedural Aggregate \ref rw::collision::ClusteredMesh

    RWCOBJECTTYPE_HALFFACEMESH_PROXY       = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x0E), ///< Data builders' Clay/SoftBody mesh, just a proxy to reserve the id defined in MeshOp

    RWCOBJECTTYPE_TRIANGLECLUSTERPROCEDURAL= RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x0F), ///< Clustered Mesh Cluster Procedural Aggregate

    RWCOBJECTTYPE_SCALEDCLUSTEREDMESH      = RWCOLLISION_VOLUMES_MAKEOBJECTTYPE(RWCOLLISION_VOLUMES_COMPONENTID, 0x10), ///< Scaled Clustered Mesh Procedural Aggregate \ref rw::collision::ScaledClusteredMesh

    // note: When adding a new type here, remember to update rw::collision::GetCollisionTypeName
    //       in rwcgraphcollisionobject.hpp

    RWCOBJECTTYPE_FORCEENUMSIZEINT  = EAPHYSICS_FORCEENUMSIZEINT
};
typedef enum ObjectType   ObjectType;

/**
\internal

\brief Sphere specific data struct description

There is no specific data for the sphere, because the only value that is needed to describe a sphere is
the radius and the radius is in the common volume data.

\importlib rwccore
*/
struct SphereSpecificData
{
    uint32_t nothing; ///< Placeholder
};

/**
\internal
\brief Capsule specific data struct description

The capsule axis is aligned to the Z axis.
The actual length of the capsule is two times the half height plus two times the radius.

\importlib rwccore
*/
struct CapsuleSpecificData
{
    float hh; ///< Capsule HalfHeight.
};

/**
\internal
\brief Triangle specific data struct description

The data stored for a triangle are the edgeCos values for each edge.  The vertices for the triangle
are stored in the volume relative transform (x,y,z rows) and the triangle normal is stored in the
transform (w row).  The edgeCos is enabled when the flag VOLUMEFLAG_TRIANGLEUSEEDGECOS is set.

\see VolumeFlag, TriangleVolume::SetEdgeCos

\importlib rwccore
*/
struct TriangleSpecificData
{
    float edgeCos0; ///< the edgecos of edge0
    float edgeCos1; ///< the edgecos of edge1
    float edgeCos2; ///< the edgecos of edge2
};

/**
\internal
\brief Box specific data struct description

The dimensions of the box are stored here.  The actually box size in each direction is two times the 
dimension plus two times the radius of the box.

\importlib rwccore
*/
struct BoxSpecificData
{
    float hx;
    float hy;
    float hz;
};

/**
\internal
\brief Cylinder specific data struct description

The radius and height of the cylinder are stored here.  
The cylinder axis is aligned to the Z axis.
The cylinder has two radii, the inner and outer.  The total radius is the sum of the inner and outer 
radii.  The inner radius is the radius of the flat face at the end of the cylinder.  The outer radius
is added to the cylinder to make it have rounded rims.  A cylinder with zero inner radius is the same as
a capsule (but less efficient).  A cylinder with zero outer radius has a sharp corner at the rim.
The actually cylinder length is two times the half height plus two times the outer radius.

\importlib rwccore
*/
struct CylinderSpecificData
{
    float hh;          ///< Half the cylinder height, not counting the outer radius.
    float innerRadius; ///< the radius of the cylinder end face.
};

/**
\internal
\brief Aggregate specific data struct description.

The aggregate pointer is the implementation object for the aggregate volume.  Many volumes can use the 
same aggregate object so that's why it is const.

\importlib rwccore
*/
struct AggregateSpecificData
{
    // todo: this should be const
    Aggregate * agg; ///< Pointer to the aggregate object.
};

/**
\internal
\brief Custom specific data struct description.

The customData pointer is used to refer to volume data that is requred that does not fit into the Volume
class, for example a height field or convex hull mesh.

\importlib rwccore
*/
struct CustomSpecificData
{
    void *   data; ///< Pointer to custom data.
    uint32_t type; ///< Type.
};

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_VOLUMEDATA_H
