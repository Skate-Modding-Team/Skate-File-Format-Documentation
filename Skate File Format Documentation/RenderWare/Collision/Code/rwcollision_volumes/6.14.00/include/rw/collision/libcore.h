// (c) Electronic Arts. All Rights Reserved.
#ifndef rw_collision_libcore_h
#define rw_collision_libcore_h

/**
\defgroup rwccore rwccore

\brief Core collision volume library.

Defines classes of objects for storing arbitrary, extensible collision volumes and performing queries on them, 
including
\li rw::collision::Volume, a base class for all collision volumes, including a standard set of primitives.
\li rw::collision::Aggregate, a base class for collision volumes that can be decomposed into primitive Volumes, 
    including rw::collision::ClusteredMesh.
\li Spatial map-related classes, such as rw::collision::AABBox, rw::collision::KDTree and rw::collision::Octree.
\li Line queries trough the rw::collision::VolumeLineQuery class
\li Bounding box queries through the rw::collision::VolumeBBoxQuery class
\li Collision queries through the rw::collision::VolumeVolumeQuery class
*/

#include "rw/collision/common.h"

#if !defined(EA_PLATFORM_PS3_SPU)
#include "rw/collision/aabbox.h"
#include "rw/collision/aalineclipper.h"
#include "rw/collision/kdtree.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"
#include "rw/collision/plane.h"
#include "rw/collision/octree.h"
#include "rw/collision/frustum.h"
#include "rw/collision/aggregate.h"
#include "rw/collision/sphere.h"
#include "rw/collision/capsule.h"
#include "rw/collision/computecontacts.h"
#include "rw/collision/cylinder.h"
#include "rw/collision/cylinderquery.h"
#include "rw/collision/volumelinequery.h"
#include "rw/collision/mappedarray.h"
#include "rw/collision/volumebboxquery.h"
#include "rw/collision/primitivepairquery.h"
#include "rw/collision/kdtreemappedarray.h"
#include "rw/collision/metrics.h"
#include "rw/collision/simplemappedarray.h"
#include "rw/collision/volumevolumequery.h"
#include "rw/collision/triangle.h"
#include "rw/collision/box.h"
#include "rw/collision/aggregatevolume.h"
#include "rw/collision/procedural.h"
#include "rw/collision/triangleclusterprocedural.h"
#include "rw/collision/trianglekdtreeprocedural.h"
#include "rw/collision/clusteredmesh.h"
#include "rw/collision/scaledclusteredmesh.h"
#include "rw/collision/trianglequery.h"
#include "rw/collision/initialize.h"

#else // defined(EA_PLATFORM_PS3_SPU)

#include "rw/collision/aabbox.h"
#include "rw/collision/aalineclipper.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"
#include "rw/collision/sphere.h"
#include "rw/collision/capsule.h"
#include "rw/collision/triangle.h"
#include "rw/collision/box.h"
#include "rw/collision/cylinder.h"
#include "rw/collision/cylinderquery.h"
#include "rw/collision/primitivepairquery.h"

#endif // defined(EA_PLATFORM_PS3_SPU)


// define the namespaces for documentation purposes
namespace rw
{
/// The rw::collision namespace contains all of the collision related code.
namespace collision
{
/// The rw::collision::detail namespace contains internal objects that should not be used externally and 
/// are subject to change without notice.
namespace detail
{
/// The rw::collision::detail::fpu namespace contains versions of some serializable objects that store all data
/// using rw::math::fpu objects in order to serialize objects for fpu platforms.
namespace fpu
{
} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif
