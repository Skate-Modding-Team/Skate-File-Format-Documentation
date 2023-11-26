// (c) Electronic Arts. All Rights Reserved.


#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_COMMON_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_COMMON_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU


#include <rw/math/fpu.h>
#include <rw/collision/aabbox.h>


/**
\brief The extended edge cosine value applied to unmatched edges.

Unmatched edges are those that are completely unique. The value of
minus one effectively treats the unmatched edge as fully convex, so that it
is enabled for collision in any direction.
*/
#define CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE (-1.0f)

/**
\brief The extended edge cosine value applied to unshared edges.

Unshared edges are those that share vertices with at least two other edges,
but have not been shared as they do not belong to any 'best pair of edges'.
The value of one effectively treats the unshared edge as a planar edge, so
that it is enabled for collision in the half-space of the triangle it belongs
to. The value of one was determined through use by the Skate3 Team. The desired
behavior of this edge, and therefore its edge cosine, is currently unknown.
*/
#define CLUSTEREDMESHBUILDER_EDGECOS_OF_UNSHARED_EDGE (1.0f)

/**
\brief Triangle neighbor index value used to indicated unmatched triangle edges.

Special triangle index value used to indicate that a triangle has no matched
neighbor on a specific edge.
*/
#define CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH 0xFFFFFFFF


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
\typedef The 3D floating point vector type used internally throughout the build process.
*/
typedef rw::math::fpu::Vector3U_32 VectorType;


/**
\typedef An axis-aligned bounding box with floating-point coordinates.
*/
typedef rw::collision::AABBoxU AABBoxType;


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_COMMON_H
