// (c) Electronic Arts. All Rights Reserved.
#ifndef RW_COLLISION_DETAIL_SHAREDCLUSTERMETHODS_H
#define RW_COLLISION_DETAIL_SHAREDCLUSTERMETHODS_H

/*************************************************************************************************************

File: sharedclustermethods.hpp

Purpose: Contain shared detail functionality between ClusteredMeshCluster technology.
*/

#include "rw/collision/common.h"
#include "rw/collision/triangle.h"
#include "rw/collision/clustertriangleiterator.h"

namespace rw
{
namespace collision
{

/**
\internal Sets the edge cosine, ID and Flag details of a triangle volume addressed by an iterator.

This method is templated on an iterator type to avoid having to include the iterator header in
the class header.

\param triangleVolume the triangle volume.
\param triangleIterator the iterator addressing the source triangle.
*/
static void
InitializeTriangleVolumeDetails(
    rw::collision::TriangleVolume &triangleVolume,
    const ClusterTriangleIterator<> &triangleIterator)
{
    triangleVolume.SetGroup(triangleIterator.GetGroupID());
    triangleVolume.SetSurface(triangleIterator.GetSurfaceID());

    // Set Flags and Edge Cosines
    rwpmath::Vector3 edgeCosines(rwpmath::GetVector3_Zero());
    const uint32_t flags = triangleIterator.GetEdgeCosinesAndFlags(edgeCosines);
    triangleVolume.SetFlags(VOLUMEFLAG_TRIANGLENORMALISDIRTY | flags);
    triangleVolume.SetEdgeCos(edgeCosines.X(), edgeCosines.Y(), edgeCosines.Z());
}

} // namespace collision
} // namespace rw

#endif // RW_COLLISION_DETAIL_SHAREDCLUSTERMETHODS_H
