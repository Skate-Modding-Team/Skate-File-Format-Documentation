// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CONTAINERS_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CONTAINERS_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#if defined __SNC__
#pragma diag_suppress=1236          // no such warning number
#pragma diag_suppress=403,481,1785
#endif
#include <EASTL/map.h>
#if defined __SNC__
#pragma diag_default=403,481,1785
#pragma diag_default=1236
#endif

#include <rw/collision/kdtreebuilder.h>

#include <rw/collision/meshbuilder/common.h>

#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/detail/vector.h>
#include <rw/collision/meshbuilder/detail/eastlblockallocator.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
The leaf map is used to store the branchId of the leaf in the xKDTree for each unit that is the first unit
added to the leaf. We need this because initially the xKDTree is constructed from bboxes and the units
are reference in the KDTree simply by sequential numbers 0...numUnit-1. However, it is not efficient to
find a unit by it's number at runtime, so we need the leafMap in order to change the "start index" of
each leaf to be a clusterId + byte offset.
*/
typedef eastl::map<uint32_t, rw::collision::KDTreeBuilder::BuildNode*, eastl::less<uint32_t>, detail::EASTLBlockAllocator> LeafMap;


/**
List of IDs as uint32_t
*/
typedef detail::Vector<uint32_t> IDList;


/**
List of VectorType instances representing vertices.
*/
typedef detail::Vector<VectorType> VertexList;


/**
List of Units
*/
typedef detail::Vector<detail::Unit> UnitList;


/**
List of Triangle
*/
typedef detail::Vector<detail::Triangle> TriangleList;


/**
List of TriangleEdgeCodes
*/
typedef detail::Vector<detail::TriangleEdgeCodes> TriangleEdgeCodesList;


/**
List of TriangleSurfaceID
*/
typedef detail::Vector<detail::TriangleSurfaceID> TriangleSurfaceIDList;


/**
List of TriangleGroupID
*/
typedef detail::Vector<detail::TriangleGroupID> TriangleGroupIDList;


/**
List of TriangleEdgeCosines
*/
typedef detail::Vector<detail::TriangleEdgeCosines> TriangleEdgeCosinesList;


/**
List of TriangleNeighbors
*/
typedef detail::Vector<detail::TriangleNeighbors> TriangleNeighborsList;


/**
List of TriangleDataFlag
*/
typedef detail::Vector<detail::TriangleFlags> TriangleFlagsList;


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CONTAINERS_H
