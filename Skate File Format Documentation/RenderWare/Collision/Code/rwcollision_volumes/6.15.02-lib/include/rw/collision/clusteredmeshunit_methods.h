// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESHUNIT_METHODS_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESHUNIT_METHODS_H

/*************************************************************************************************************/

#include "rw/collision/common.h"

#if !defined(EA_PLATFORM_PS3_SPU)

#include "clusteredmeshunit.h"
#include "clusteredmesh.h"

namespace rw
{
namespace collision
{


//-------------------------------------------------------------------------------
//  Inline methods for ClusteredMeshUnit

/**
Constructor.
This returns a unit proxy that is pointing to the first unit in the cluster.  
\param cm a clustered mesh
\param clusterId the index of the cluster that you want to iterate over.
\return a unit proxy suitable for editing the first unit in the specified cluster.
*/
inline
ClusteredMeshUnit::ClusteredMeshUnit(ClusteredMesh *cm, uint32_t clusterId)
{
    mClusteredMesh = cm;
    mCluster = &cm->GetCluster(clusterId);
    mUnitData = mCluster->UnitData();
}

/**
Test the unit proxy is at the end of the cluster (it is after the last unit).  
\return true if the unit is at the end of the cluster, false otherwise.
*/
inline bool
ClusteredMeshUnit::AtEnd() const
{
    return mUnitData >= mCluster->UnitData() + mCluster->unitDataSize;
}

/**
Advance the unit proxy to the next unit in the cluster.  
\return true if the unit is valid, false if the unit is past the end of the cluster.
*/
inline void
ClusteredMeshUnit::Next()
{
    mUnitData += GetMemberOffset(0);
}

/**
Test if the unit proxy is valid.
Note this method is implemented for debug build only.
\return true if valid.  Note, an iterator that is returned by cluster->End() will not return a valid unit.
*/
inline bool
ClusteredMeshUnit::IsValid() const
{
    bool ok = mClusteredMesh != NULL;
    ok = ok && mCluster >= &mClusteredMesh->GetCluster(0);
    ok = ok && mClusteredMesh->GetNumCluster() > 0;
    ok = ok && mCluster <= &mClusteredMesh->GetCluster(mClusteredMesh->GetNumCluster() - 1);
    ok = ok && mUnitData >= mCluster->UnitData();
    ok = ok && mUnitData < mCluster->UnitData() + mCluster->unitDataSize;
    return ok;
}

/**
\internal
Get the offset to the member data within the unit.

This is the main low level method for parsing the guts of the unit.
Note, This does NO error checking, so if you call this with flag=UNITFLAG_GROUPID, for example, the offset to
where the groupid should be is returned, but it does not check that the group id actually exists in the unit.

\param flag one of 0, 1, UNITFLAG_NORMAL, UNITFLAG_EDGEANGLE, UNITFLAG_GROUPID, UNITFLAG_SURFACEID.
\return the offset to the specified member.  Note, flag=0 gets the total size of the unit. And flag=1 gets
the offset to the vertices.
*/
inline uint32_t 
ClusteredMeshUnit::GetMemberOffset(uint32_t flag) const
{
    uint32_t i = 1;
    uint32_t triangleCount = 1;

    if (GetType() == UNITTYPE_QUAD)
    {
        triangleCount = 2;
    }
    else if (GetType() == UNITTYPE_TRILIST)
    {
        triangleCount = mUnitData[i++];
    }
    if (flag == 1)                            // offset to vertices
    {
        return i;
    }
    i += triangleCount + 2;            // skip over vertex data
    if (flag == UNITFLAG_NORMAL)
    {
        return i;
    }
    if (mUnitData[0] & UNITFLAG_NORMAL)
    {
        ++i;
    }
    if (flag == UNITFLAG_EDGEANGLE)
    {
        return i;
    }
    if (mUnitData[0] & UNITFLAG_EDGEANGLE)
    {
        i += triangleCount + 2;
    }
    if (flag == UNITFLAG_GROUPID)
    {
        return i;
    }
    if (mUnitData[0] & UNITFLAG_GROUPID)
    {
        i += mClusteredMesh->GetGroupIdSize();
    }
    if (flag == UNITFLAG_SURFACEID)
    {
        return i;
    }
    if (mUnitData[0] & UNITFLAG_SURFACEID)
    {
        i += mClusteredMesh->GetSurfaceIdSize();
    }
    return i;
}

/**
Return the number of bytes to move to the next unit in the stream
*/
inline uint32_t
ClusteredMeshUnit::GetUnitSize() const
{
    uint8_t *data = mUnitData;
    uint32_t i = 1;
    uint32_t triangleCount = 1;

    switch (data[0] & UNITTYPE_MASK)
    {
    case UNITTYPE_QUAD:
        triangleCount = 2;
        break;
    case UNITTYPE_TRILIST:
        triangleCount = data[i++];
        break;
    }
    i += triangleCount + 2;            // skip over vertex data
    if (data[0] & UNITFLAG_EDGEANGLE)
    {
        i += triangleCount + 2;
    }
    if (data[0] & UNITFLAG_GROUPID)
    {
        i += mClusteredMesh->GetGroupIdSize();
    }
    if (data[0] & UNITFLAG_SURFACEID)
    {
        i += mClusteredMesh->GetSurfaceIdSize();
    }
    return i;
}

/**
Get the type of the unit.
\return the type and flags of the unit.
*/
inline uint32_t 
ClusteredMeshUnit::GetType() const
{
    return uint32_t(GetTypeAndFlags() & UNITTYPE_MASK);
}

/**
Get the type and flags of the unit.
\see enum UnitTypeAndFlags.  You can use the UNITTYPE_MASK to get the type and the rest of the bit are
the flags.
\return the type and flags of the unit.
*/
inline uint8_t 
ClusteredMeshUnit::GetTypeAndFlags() const
{
    EA_ASSERT(IsValid());
    return mUnitData[0];
}

/**
Get the number of subunits (or triangles) contained in this unit.
\return a positive number.
*/
inline uint32_t 
ClusteredMeshUnit::GetTriangleCount() const
{
    EA_ASSERT(IsValid());
    switch (GetType())
    {
    case UNITTYPE_QUAD:       return 2;
    case UNITTYPE_TRILIST:    return mUnitData[1];
    }
    return 1;
}

/**
Get the number of vertices contained in this unit.
The number of vertices is 2 + the number of triangles.  The vertices (0,1,2) are the first triangle
and in that order.  Then (3,2,1) is the second triangle, and so on.  
\return a positive number.
*/
inline uint32_t 
ClusteredMeshUnit::GetVertexCount() const
{
    return GetTriangleCount() + 2;
}

/**
Get the number of edges contained in this unit.
The number of edges is 2 + the number of triangles.  The indices (0,1,2) are the edges of the first triangle.
And indices (3,2,1) are the edges of the second triangle, and so on.  
\return a positive number.
*/
inline uint32_t 
ClusteredMeshUnit::GetEdgeCount() const
{
    return GetTriangleCount() + 2;
}

/**
Get the surface id of the unit.
Result is not defined if the unit does not have a surfaceid.
*/
inline uint32_t 
ClusteredMeshUnit::GetSurfaceId() const
{
    EA_ASSERT(GetTypeAndFlags() & UNITFLAG_SURFACEID);
    uint32_t i = GetMemberOffset(UNITFLAG_SURFACEID);
    uint32_t id = mUnitData[i];
    if (mClusteredMesh->GetSurfaceIdSize() == 2)
    {
        id += 256 * mUnitData[i + 1];
    }
    return id;
}

/**
Set the surface id of the unit.  If the unit does not already have a surface id then you cannot set it.
\param id the new id.
*/
inline void 
ClusteredMeshUnit::SetSurfaceId(uint32_t id) const
{
    EA_ASSERT(GetTypeAndFlags() & UNITFLAG_SURFACEID);
    uint32_t i = GetMemberOffset(UNITFLAG_SURFACEID);
    mUnitData[i] = static_cast<uint8_t>(id & 255);
    if (mClusteredMesh->GetSurfaceIdSize() == 2)
    {
        mUnitData[i + 1] = static_cast<uint8_t>((id >> 8) & 255);
    }
}

/**
Get the group id of the unit.
Result is not defined if the unit does not have a groupid.
*/
inline uint32_t 
ClusteredMeshUnit::GetGroupId() const
{
    EA_ASSERT(GetTypeAndFlags() & UNITFLAG_GROUPID);
    uint32_t i = GetMemberOffset(UNITFLAG_GROUPID);
    uint32_t id = mUnitData[i];
    if (mClusteredMesh->GetGroupIdSize() == 2)
    {
        id += 256 * mUnitData[i + 1];
    }
    return id;
}

/**
Set the group id of the unit.  If the unit does not already have a group id then you cannot set it.
\param id the new id.
*/
inline void 
ClusteredMeshUnit::SetGroupId(uint32_t id) const
{
    EA_ASSERT(GetTypeAndFlags() & UNITFLAG_GROUPID);
    uint32_t i = GetMemberOffset(UNITFLAG_GROUPID);
    mUnitData[i] = static_cast<uint8_t>(id & 255);
    if (mClusteredMesh->GetGroupIdSize() == 2)
    {
        mUnitData[i + 1] = static_cast<uint8_t>((id >> 8) & 255);
    }
}

/**
Get a vertex id of a unit.
\param i vertex index in the unit.  The valid range is up to NumTriangle + 2.
\return the vertex id.
*/
inline uint8_t 
ClusteredMeshUnit::GetVertexId(uint32_t i) const
{
    EA_ASSERT(i < GetVertexCount());
    i += GetMemberOffset(1);
    return mUnitData[i];
}

/**
Get a vertex id of a unit.
\param i vertex index in the unit.  The valid range is up to NumTriangle + 2.
\return the vertex coordinates.
*/
inline rwpmath::Vector3 
ClusteredMeshUnit::GetVertex(uint32_t i) const
{
    EA_ASSERT(i < GetVertexCount());
    i += GetMemberOffset(1);
    return mCluster->GetVertex(mUnitData[i], mClusteredMesh->GetVertexCompressionGranularity());
}

/**
This changes the vertices that are used by the unit.  
Warning!  It is highly unlikely that you should call this function.  Make sure you really know what
you are doing before using it.
The number of vertices is 2 + the number of triangles.  The vertices (0,1,2) are the first triangle
and in that order.  Then (3,2,1) is the second triangle, and so on.  
\param i vertex index in the unit.  The valid range is up to NumTriangle + 2.
\param newId the vertex id. 
*/
inline void 
ClusteredMeshUnit::SetVertexId(uint32_t i, uint8_t newId) const
{
    EA_ASSERT(i < GetVertexCount());
    i += GetMemberOffset(1);
    mUnitData[i] = newId;
}

/**
Get the edge cosine encoded byte of the specified edge of the unit.
The decoded cosine value is 1 - PI^2 / 2^(B+3).  So B=0 means cos = -0.2337.
The lower 5 bits are the B.  The upper three bits are the flags EDGEFLAG_EDGECONVEX, 
EDGEFLAG_VERTEXDISABLE, and EDGEFLAG_EDGEUNMATCHED.

\param i the edge index in the unit
\return edge cos encoded value from 0 to 26 plus UnitEdgeFlags.

Note to determine if the edge is convex or reflex, you need to look at the edge flags.
*/
inline uint8_t 
ClusteredMeshUnit::GetEdgeData(uint32_t i) const
{
    EA_ASSERT(GetTypeAndFlags() & UNITFLAG_EDGEANGLE);
    EA_ASSERT(i < GetEdgeCount());
    i += GetMemberOffset(UNITFLAG_EDGEANGLE);
    return mUnitData[i];
}

/**
Set the edge cosine encoded byte of the specified edge of the unit.
The cosine value is 1 - PI^2 / 2^(B+3) where B is the byte from 0 to 26.
The lower 5 bits are the B.  The upper three bits are the flags EDGEFLAG_EDGECONVEX, 
EDGEFLAG_VERTEXDISABLE, and EDGEFLAG_EDGEUNMATCHED.

\param i the edge index in the unit
\param newval edge cos encoded value from 0 to 26 plus UnitEdgeFlags.

*/
inline void
ClusteredMeshUnit::SetEdgeData(uint32_t i, uint8_t newval) const
{
    EA_ASSERT(GetTypeAndFlags() & UNITFLAG_EDGEANGLE);
    EA_ASSERT(i < GetEdgeCount());
    i += GetMemberOffset(UNITFLAG_EDGEANGLE);
    mUnitData[i] = newval;
}
}
}
#endif  // !defined(EA_PLATFORM_PS3_SPU)

#endif
