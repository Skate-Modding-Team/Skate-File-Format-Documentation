// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

File: genericclusterunit.h

Purpose: Implementation of Unit interface for generic ClusteredMeshCluster data.

*/

#ifndef PUBLIC_RW_COLLISION_GENERIC_CLUSTER_UNIT_H
#define PUBLIC_RW_COLLISION_GENERIC_CLUSTER_UNIT_H

#include "rw/collision/common.h"                          // for rwpmath::*
#include "rw/collision/clusteredmesh.h"                 // for ClusteredMeshCluster
#include "rw/collision/clusterunitbase.h"               // for rw::collision::ClusterUnitBase<>

namespace rw
{
    namespace collision
    {

        /// Class to provide access to a single, generic, unit in a ClusteredMeshCluster.
        /// The UnitIterator and TriangleIterator classes can be instantiated with this unit class for optimized
        /// access to known format meshes.
        /// See http://docs.ea.com/RWPhysicsDev:ClusteredMesh/Iterators for further information.
        ///
        /// When accessing quads, data is returned for two triangles between v0,v1,v2 and v3,v2,v1.
        ///
        /// Doesn't support deprecated OLDTRIANGLE units or unimplemented unit types
        /// (types other than UNITTYPE_TRIANGLE or UNITTYPE_QUAD).
        template <uint8_t COMPRESSION = ClusteredMeshCluster::COMPRESSION_DYNAMIC>
        class GenericClusterUnit : public ClusterUnitBase
        {
        public:     // Public Unit API expected by templated unit iterators

            /// Allow users to access the compression mode assumed by this unit
            static const uint8_t CompressionMode = COMPRESSION;

            /// Construct to give access to given unit within a cluster.
            RW_COLLISION_FORCE_INLINE GenericClusterUnit(const ClusteredMeshCluster & cluster, 
                const ClusterParams & clusterParams, uint32_t offset = 0)
                : ClusterUnitBase(cluster), mClusterParams(clusterParams)
            {
#if defined(EA_COMPILER_GNUC)
                // GCC411 and later can produce warnings about potentially uninitialized variables in opt builds.
                // The only work around we know is to initialize these explicitly here.
                mVertices[3] = rwpmath::GetVector3_Zero(); 
                mEdgeCosines[0] = rwpmath::GetVector3_Zero();
                mEdgeCosines[1] = rwpmath::GetVector3_Zero();
                mFlags[0] = 0;
                mFlags[1] = 0;
#endif  // defined (EA_COMPILER_GNUC)
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

            /// Check we're pointing at what looks like a valid unit and we can parse it.
            inline bool IsValid() const
            {
                const rw::collision::ClusteredMeshCluster & cluster = ClusterUnitBase::GetCluster();
                if (mData && 
                    (mData >= cluster.UnitData()) && 
                    (mData < (cluster.UnitData() + cluster.unitDataSize)))
                {
                    const uint8_t unitFlags = mData[0];
                    const uint32_t unitType = (uint32_t)(unitFlags & UNITTYPE_MASK);
                    // Tri lists or old triangles not supported
                    return (unitType == UNITTYPE_TRIANGLE) ||
                        (unitType == UNITTYPE_QUAD);
                }
                return false;
            }

            /// Return number of bytes used by this unit
            RW_COLLISION_FORCE_INLINE uint32_t GetSize() const
            {
                return mSize;
            }
            /// Return number of triangles in this unit
            RW_COLLISION_FORCE_INLINE uint32_t GetTriCount() const
            {
                return mTriCount;
            }
            /// Return number of vertices in this unit
            RW_COLLISION_FORCE_INLINE uint32_t GetVertexCount() const
            {
                return mTriCount + 2u;
            }

            /// Return one vertex.
            RW_COLLISION_FORCE_INLINE rwpmath::Vector3 GetVertex(uint32_t i)
            {
                EA_ASSERT(i < GetVertexCount());
                EA_ASSERT(i < 4);
                return mVertices[i];
            }
            /// Get all three vertices for a given triangle
            RW_COLLISION_FORCE_INLINE void GetTriVertices(
                rwpmath::Vector3 & vertex0, 
                rwpmath::Vector3 & vertex1,
                rwpmath::Vector3 & vertex2, 
                uint32_t tri = 0) const
            {
                EA_ASSERT(tri < mTriCount);
                if (tri == 0)
                {
                    vertex0 = mVertices[0];
                    vertex1 = mVertices[1];
                    vertex2 = mVertices[2];
                }
                else
                {
                    vertex0 = mVertices[3];
                    vertex1 = mVertices[2];
                    vertex2 = mVertices[1];
                }
            }
            /// Get all three vertex indices for a given triangle
            RW_COLLISION_FORCE_INLINE void GetTriVertexIndices(
                uint8_t & vertex0,
                uint8_t & vertex1,
                uint8_t & vertex2,
                uint32_t tri = 0) const
            {
                EA_ASSERT(tri < mTriCount);
                // If this is of type trilist then the vertex data would be off by one.
                // However since we no longer support trilist units this should not happen.
                EA_ASSERT(!GetValueMask((uint32_t)(mData[0] & UNITTYPE_MASK), UNITTYPE_TRILIST));

                if (tri == 0)
                {
                    vertex0 = mData[1];
                    vertex1 = mData[2];
                    vertex2 = mData[3];
                }
                else
                {
                    vertex0 = mData[4];
                    vertex1 = mData[3];
                    vertex2 = mData[2];
                }
            }
            /// Return edge cosines and flags for the given triangle.
            RW_COLLISION_FORCE_INLINE uint32_t GetEdgeCosinesAndFlags(rwpmath::Vector3 & edgeCosines, uint32_t tri = 0) const
            {
                // Could remove the conditional by pointing at static zero vector
                EA_ASSERT(tri < mTriCount);
                if (mEdgeData)
                {
                    if (mTriCount == 1)
                    {
                        // Don't bother to cache the results in case there's a performance hit - assume they'd only be asked for once.
                        return ClusterUnitBase::ExtractTriEdgeData(edgeCosines, mEdgeData, mClusterParams.mFlags);
                    }
                    else
                    {
                        /// Cache results from both triangles since they share most of the work but we don't
                        /// want to take the hit of calling this code if not needed.
                        ClusterUnitBase::ExtractQuadEdgeData(mEdgeCosines[0], mFlags[0], mEdgeCosines[1], mFlags[1],
                            mVertices[0], mVertices[1], mVertices[2], mVertices[3], mEdgeData, mClusterParams.mFlags);
                        mEdgeData = 0;  // cache the answers and don't compute them again
                    }
                }
                edgeCosines = mEdgeCosines[tri];
                return mFlags[tri];
            }

            /// Return the group and surface ID in a single word, with bytes set to zero if not defined by this unit.
            RW_COLLISION_FORCE_INLINE uint32_t GetID() const
            {
                // Could cache mData[0] flags
                return ClusterUnitBase::LoadID(mIDData, mData[0], mClusterParams.mGroupIdSize, mClusterParams.mSurfaceIdSize);
            }

            /// Return the group ID in a single word, with bytes set to zero if not defined by this unit.
            RW_COLLISION_FORCE_INLINE uint32_t GetGroupID() const
            {
                // Could cache mData[0] flags
                return ClusterUnitBase::LoadID(mIDData, mData[0], UNITFLAG_GROUPID, mClusterParams.mGroupIdSize, mClusterParams.mSurfaceIdSize);
            }

            /// Return the surface ID in a single word, with bytes set to zero if not defined by this unit.
            RW_COLLISION_FORCE_INLINE uint32_t GetSurfaceID() const
            {
                // Could cache mData[0] flags
                return ClusterUnitBase::LoadID(mIDData, mData[0], UNITFLAG_SURFACEID, mClusterParams.mGroupIdSize, mClusterParams.mSurfaceIdSize);
            }

        private:        // Internal implementation

            /// Initialize to read from a unit at a given location.
            /// Loads vertex data into members (hopefully registers).
            /// Stores information to make it fast to load edge cosines and IDs only if needed.
            RW_COLLISION_FORCE_INLINE void Initialize(const uint8_t * dataStream)
            {
                mData = dataStream;
                EA_ASSERT(IsValid());
                const uint8_t unitFlags = dataStream[0];
                const uint32_t unitType = (uint32_t)(unitFlags & UNITTYPE_MASK);

                // Compute masks so we can avoid branches in the rest of this function
                const uint32_t triangleMask   = GetValueMask(unitType, UNITTYPE_TRIANGLE);
                const uint32_t quadMask       = GetValueMask(unitType, UNITTYPE_QUAD);
                const uint32_t triListMask    = GetValueMask(unitType, UNITTYPE_TRILIST);
                const uint32_t edgeDataMask   = GetFlagMask(unitFlags, UNITFLAG_EDGEANGLE);
                const uint32_t groupIdMask    = GetFlagMask(unitFlags, UNITFLAG_GROUPID);
                const uint32_t surfaceIdMask  = GetFlagMask(unitFlags, UNITFLAG_SURFACEID);

                // Branch free triangle count computation
                const uint32_t triCount = (1 & triangleMask) + (2 & quadMask) + (dataStream[1] & triListMask);
                const uint32_t vertexCount = triCount + 2;
                const uint32_t edgeCount = triCount + 2;

                // The vertex indices follow the type and optional count
                const uint8_t *vertexData = (dataStream + 1) + (triListMask & 1);

                // The optional edge angles follow the vertex indices
                const uint8_t *edgeData = vertexData + vertexCount;

                // ID data follows the optional edge angles
                const uint8_t *groupIDData = edgeData + (edgeDataMask & edgeCount);
                const uint8_t *surfaceIDData = groupIDData + (groupIdMask & mClusterParams.mGroupIdSize);
                const uint8_t *nextData = surfaceIDData + (surfaceIdMask & mClusterParams.mSurfaceIdSize);

                // Now we can compute the total size of the unit
                const uint32_t size = (uint32_t)(nextData - dataStream);

                // Read the vertex data, and store information for reading the rest.
                mTriCount = (uint8_t) triCount;
                mSize = (uint8_t) size;
                if (triCount == 1)
                {
                    ClusterUnitBase::GetTriVertices<COMPRESSION>(mVertices[0], mVertices[1], mVertices[2],
                        vertexData, mClusterParams.mVertexCompressionGranularity);
                    if (edgeDataMask)
                    {
                        mEdgeData = edgeData;   // Compute lazily
                    }
                    else
                    {
                        mEdgeCosines[0] = rwpmath::GetVector3_Zero();
                        mFlags[0] = uint32_t(mClusterParams.mFlags & CMFLAG_ONESIDED);
                        mEdgeData = 0;  // No need to compute
                    }
                }
                else
                {
                    EA_ASSERT_MSG(triCount == 2, ("GenericClusterUnit only supports triangles and quads"));
                    ClusterUnitBase::GetQuadVertices<COMPRESSION>(mVertices[0], mVertices[1], mVertices[2], mVertices[3],
                        vertexData, mClusterParams.mVertexCompressionGranularity);
                    if (edgeDataMask)
                    {
                        mEdgeData = edgeData;   // Load lazily only if needed
                    }
                    else
                    {
                        mEdgeCosines[0] = rwpmath::GetVector3_Zero();
                        mEdgeCosines[1] = rwpmath::GetVector3_Zero();
                        mFlags[0] = mFlags[1] = uint32_t(mClusterParams.mFlags & CMFLAG_ONESIDED);
                        mEdgeData = 0;  // No need to compute
                    }
                }
                // Load ID data lazily only if needed only
                mIDData = groupIDData;
            }

            /// Return the type of the unit being pointed at.
            RW_COLLISION_FORCE_INLINE UnitTypeAndFlags GetUnitType() const
            {
                EA_ASSERT(mData);
                return (UnitTypeAndFlags)(mData[0] & UNITTYPE_MASK);
            }

            /// Return 0xffffffff if type == value otherwise 0x00000000
            static RW_COLLISION_FORCE_INLINE uint32_t GetValueMask(uint32_t type, uint32_t value)
            {
                return ~(uint32_t)((-(int32_t)(value ^ type)) >> 31);
            }
            /// Return 0xffffffff if (flags & bit) set otherwise 0x00000000
            static RW_COLLISION_FORCE_INLINE uint32_t GetFlagMask(uint32_t flags, uint32_t bit)
            {
                return (uint32_t)((-(int32_t)(flags & bit)) >> 31);
            }

        private:        // Internal data

            /// Pointer to the data for this unit.
            const uint8_t * mData;
            /// Pointer to (optional) ID data for this unit.
            const uint8_t * mIDData;
            /// Pointer to edge cosine data (or NULL if already cached in mEdgeCosines and mFlags).
            mutable const uint8_t * mEdgeData;  // mutable since updated when data is cached

            /// The (3 or 4) vertices for the unit.
            rwpmath::Vector3 mVertices[4];

            /// Edge cosiness for the two triangles.
            mutable rwpmath::Vector3 mEdgeCosines[2];  // mutable since updated when data is cached
            /// Flags for the two triangles.
            mutable uint32_t mFlags[2];  // mutable since updated when data is cached

            /// The number of trinagles in the unit.
            uint8_t mTriCount;
            /// The size (in bytes) of the unit.
            uint8_t mSize;

            /// Parameters for access to the cluster
            rw::collision::ClusterParams mClusterParams;
        };

    }   // namespace collision
}   // namespace rw

#endif  // !PUBLIC_RW_COLLISION_GENERIC_CLUSTER_UNIT_H

