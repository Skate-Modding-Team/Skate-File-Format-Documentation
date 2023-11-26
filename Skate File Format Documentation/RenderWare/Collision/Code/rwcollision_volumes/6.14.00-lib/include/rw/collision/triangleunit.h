// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

File: triangleunit.h

Purpose: Specialized implementations of Unit interface for ClusteredMeshClusters containing only triangles.

*/

#ifndef PUBLIC_RW_COLLISION_TRIANGLE_CLUSTER_UNIT_H
#define PUBLIC_RW_COLLISION_TRIANGLE_CLUSTER_UNIT_H

#include "rw/collision/common.h"                          // for rwpmath::*
#include "rw/collision/clusteredmesh.h"                 // for ClusteredMeshCluster
#include "rw/collision/clusterunitbase.h"               // for rw::collision::ClusterUnitBase<>

namespace rw
{
namespace collision
{

/// Specialized unit for triangles with edge cosines and optional IDs.
/// The UnitIterator and TriangleIterator classes can be instantiated with this unit class for optimized
/// access to known format meshes.
/// See http://docs.ea.com/RWPhysicsDev:ClusteredMesh/Iterators for further information.
///
/// This class assumes unit byte stream layout is 1 byte for type and triangle flags, 3 bytes for vertex indices,
/// 3 bytes for edge cosines and edge flags then 0 or GROUP_ID_BYTES of group ID and 0 or SURFACE_ID_BYTES of surface ID.
/// Note that to support *any* IDs, they are assumed to be optional on a particular unit, because the
/// existing clustered mesh creation code will not write either ID if it is the same as the default ID (zero).
/// The result is code that will not be as fast as code written when IDs are always there.
template<uint8_t COMPRESSION = ClusteredMeshCluster::COMPRESSION_DYNAMIC, 
         uint8_t GROUP_ID_BYTES = 0u, 
         uint8_t SURFACE_ID_BYTES = 0u>
class TriangleUnitWithEdgeCosinesAndIDs : public ClusterUnitBase
{

public:     // Standard API for accessing Unit assumed by iterators.

    /// Allow users to access the compression mode assumed by this unit
    static const uint8_t CompressionMode = COMPRESSION;

    /// Construct to access a unit at given offset (default 0) within a cluster.
    RW_COLLISION_FORCE_INLINE TriangleUnitWithEdgeCosinesAndIDs(
        const ClusteredMeshCluster & cluster, 
        const ClusterParams & clusterParams, 
        uint32_t offset = 0) 
        : ClusterUnitBase(cluster)
    {
        SetClusterParams(clusterParams);
        Reset(offset);
    }

    /// Advance accessor to next unit
    RW_COLLISION_FORCE_INLINE void Advance()
    {
        Initialize(mData + GetSize());
    }

    /// Reset to access unit at given offset in cluster
    RW_COLLISION_FORCE_INLINE void Reset(uint32_t offset = 0u)
    {
        Initialize(ClusterUnitBase::GetUnitData(offset));
    }

    /// Gets the offset of the current unit
    RW_COLLISION_FORCE_INLINE uint32_t GetOffset() const
    {
        return (static_cast<uint32_t>(mData - ClusterUnitBase::GetUnitData(0u)));
    }

    /// Return number of triangles in the unit (always 1).
    RW_COLLISION_FORCE_INLINE uint32_t GetTriCount() const
    {
        EA_ASSERT(IsValid());
        return 1u;
    }

    /// Return number of vertices (always 3).
    RW_COLLISION_FORCE_INLINE uint32_t GetVertexCount() const
    {
        EA_ASSERT(IsValid());
        return 3u;
    }

    /// Check we're pointing at what looks like a valid unit and we can parse it.
    inline bool IsValid() const
    {
        if (mData)
        {
            const uint8_t unitFlags = mData[0];
            const uint32_t unitType = (uint32_t)(unitFlags & UNITTYPE_MASK);
            return 
                (unitType == UNITTYPE_TRIANGLE) &&
                (unitFlags & UNITFLAG_EDGEANGLE) &&
                (!(unitFlags & UNITFLAG_GROUPID) || (GROUP_ID_BYTES > 0)) &&
                (!(unitFlags & UNITFLAG_SURFACEID) || (SURFACE_ID_BYTES > 0));
        }
        return false;
    }

    /// Get size in bytes of this unit.
    RW_COLLISION_FORCE_INLINE uint32_t GetSize() const
    {
        EA_ASSERT(IsValid());
        // Hopefully, this will optimize away if GROUP_ID_BYTES == SURFACE_ID_BYTES == 0.
        // If necessary, we could specialize this method for this case.
        const uint8_t unitFlags = mData[0];
        const uint32_t groupIdMask    = GetFlagMask(unitFlags, UNITFLAG_GROUPID);
        const uint32_t surfaceIdMask  = GetFlagMask(unitFlags, UNITFLAG_SURFACEID);
        return BASIC_TRIANGLE_SIZE + (GROUP_ID_BYTES & groupIdMask) + (SURFACE_ID_BYTES & surfaceIdMask);
    }

    /// Get single vertex
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3 GetVertex(uint32_t i)
    {
        EA_ASSERT(i < GetVertexCount());
        return ClusterUnitBase::GetCluster(). template GetVertexBase<COMPRESSION>(mData[1+i], mVertexCompressionGranularity);
    }
    /// Get coordinates of three vertices of triangle.
    RW_COLLISION_FORCE_INLINE void GetTriVertices(
        rwpmath::Vector3 & vertex0, 
        rwpmath::Vector3 & vertex1,
        rwpmath::Vector3 & vertex2, 
        uint32_t EAPHYSICS_ASSERTARGUMENT(tri) = 0) const
    {
        EA_ASSERT(tri == 0);
        ClusterUnitBase::GetTriVertices<COMPRESSION>(vertex0, vertex1, vertex2, mData+1, mVertexCompressionGranularity);
    }

    /// Get indices of three vertices of triangle.
    RW_COLLISION_FORCE_INLINE void GetTriVertexIndices(
        uint8_t & vertex0,
        uint8_t & vertex1,
        uint8_t & vertex2,
        uint32_t EAPHYSICS_ASSERTARGUMENT(tri) = 0) const
    {
        EA_ASSERT(tri == 0);
        vertex0 = mData[1];
        vertex1 = mData[2];
        vertex2 = mData[3];
    }

    /// Get edge cosine data and triangle flags
    RW_COLLISION_FORCE_INLINE uint32_t GetEdgeCosinesAndFlags(
        rwpmath::Vector3 & edgeCosines, 
        uint32_t EAPHYSICS_ASSERTARGUMENT(tri) = 0) const
    {
        EA_ASSERT(tri == 0);
        EA_ASSERT(IsValid());
        return ClusterUnitBase::ExtractTriEdgeData(edgeCosines, mData+4, mDefaultFlags);
    }

    /// Get group and surface ID combined into a single word.
    RW_COLLISION_FORCE_INLINE uint32_t GetID() const
    {
        EA_ASSERT(IsValid());
        const uint8_t unitFlags = mData[0];
        // Hopefully the compiler can optimize this call for compile-time known ID sizes.
        return ClusterUnitBase::LoadID(mData + BASIC_TRIANGLE_SIZE, unitFlags, GROUP_ID_BYTES, SURFACE_ID_BYTES);
    }

    /// Get group ID.
    RW_COLLISION_FORCE_INLINE uint32_t GetGroupID() const
    {
        EA_ASSERT(IsValid());
        const uint8_t unitFlags = mData[0];
        // Hopefully the compiler can optimize this call for compile-time known ID sizes.
        return ClusterUnitBase::LoadID(mData + BASIC_TRIANGLE_SIZE, unitFlags, UNITFLAG_GROUPID, GROUP_ID_BYTES, SURFACE_ID_BYTES);
    }

    /// Get surface ID.
    RW_COLLISION_FORCE_INLINE uint32_t GetSurfaceID() const
    {
        EA_ASSERT(IsValid());
        const uint8_t unitFlags = mData[0];
        // Hopefully the compiler can optimize this call for compile-time known ID sizes.
        return ClusterUnitBase::LoadID(mData + BASIC_TRIANGLE_SIZE, unitFlags, UNITFLAG_SURFACEID, GROUP_ID_BYTES, SURFACE_ID_BYTES);
    }

private:    // Internal implementation

    /// Size of triangle unit without optional ID bytes. Flags byte + 3 vertices + 3 edge cosines.
    static const uint32_t BASIC_TRIANGLE_SIZE = 7u;

    /// Initialize to point at given unit.
    RW_COLLISION_FORCE_INLINE void Initialize(const uint8_t * dataStream)
    {
        mData = dataStream;
    }

    /// Return 0xffffffff if (flags & bit) set otherwise 0x00000000
    static RW_COLLISION_FORCE_INLINE uint32_t GetFlagMask(uint32_t flags, uint32_t bit)
    {
        return (uint32_t)((-(int32_t)(flags & bit)) >> 31);
    }

    /// Store the parameters we need for cluster access
    RW_COLLISION_FORCE_INLINE void SetClusterParams(const ClusterParams & clusterParams)
    {
        mDefaultFlags = clusterParams.mFlags;
        // Strictly only needed if vertices are or maybe compressed
        mVertexCompressionGranularity = clusterParams.mVertexCompressionGranularity;
    }

private:    // Internal data

    /// Pointer to the unit data
    const uint8_t * mData;
    /// Default CM flags (mainly used for CM_ONESIDED)
    uint16_t mDefaultFlags;
    /// Vertex compression granularity for compressed vertices
    float mVertexCompressionGranularity;
};

/// A simpler unit class for the case where there are known to be no IDs.
/// At some point, we might want to specialize some of the ID-related methods to make them faster.
template <uint8_t COMPRESSION>
class TriangleUnitWithEdgeCosines : public rw::collision::TriangleUnitWithEdgeCosinesAndIDs<COMPRESSION, 0, 0>
{
public:
    TriangleUnitWithEdgeCosines(const rw::collision::ClusteredMeshCluster & cluster, 
        const rw::collision::ClusterParams & clusterParams, uint32_t offset = 0) : 
    rw::collision::TriangleUnitWithEdgeCosinesAndIDs<COMPRESSION,0,0>(cluster, clusterParams, offset)
    {}
};

}   // namespace collision
}   // namespace rw

#endif  // !PUBLIC_RW_COLLISION_TRIANGLE_CLUSTER_UNIT_H

