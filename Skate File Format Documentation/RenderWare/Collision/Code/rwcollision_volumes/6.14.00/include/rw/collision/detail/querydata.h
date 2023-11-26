// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_QUERYDATA_H
#define PUBLIC_RW_COLLISION_QUERYDATA_H

/*************************************************************************************************************

 File: querydata.h

 Purpose: Declarations of primitive-specific query data structures that are unionised in VolumeLineQuery and VolumeBBoxQuery
 */

#include <EABase/eabase.h>


namespace rw
{
namespace collision
{


/**
\internal
For use by BBoxOverlapQuery and LineIntersectionQuery methods to store state for restarting when the result buffer is full.
*/
struct ClusteredMeshQueryRestartData
{
    uint32_t entry;
    uint32_t unitCount;
    uint32_t numTrisLeftInUnit;
};


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_QUERYDATA_H
