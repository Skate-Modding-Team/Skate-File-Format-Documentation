// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTERTRIANGLEITERATOR_H
#define PUBLIC_RW_COLLISION_CLUSTERTRIANGLEITERATOR_H

/*************************************************************************************************************

File: clustertrinagleiterator.h

Purpose: Iterator for accessing triangles from clustered mesh clusters.

*/

#include "rw/collision/common.h"                          // for rwpmath::*
#include "rw/collision/clusteredmesh.h"                 // for ClusteredMeshCluster
#include "rw/collision/genericclusterunit.h"            // for GenericClusterUnit
#include "rw/collision/clusterunitwalker.h"             // for ClusterUnitWalker<>

namespace rw
{
namespace collision
{

/**
This is an iterator class that you can use to iterate over triangles in a clustered mesh.

Note since this is a proxy (it points to the data, but doesn't contain the data) all the methods
are "const".

Sample usage (to compute bounding box of first 10 units in a cluster):
\begincode
const ClusteredMesh *cm;
const ClusterParams clusterParams = cm->GetClusterParams();
for (i = 0; i < cm->GetClusterCount(); ++i)
{
    const Cluster & cluster = cm->GetCluster(i);
    EA_ASSERT(cluster.NumUnitInCluster() >= 10);
    // Iterate triangles from the first 10 units in the cluster
    for (ClusterTriangleIterator it(cluster, clusterParams, 0, 10); !it.AtEnd(); it.Next())
    {
        rwpmath::Vector3 v0, v1, v2;
        it.GetVertices(v0, v1, v2);

        rwpmath::Vector3 triBBoxMin = rwpmath::Min(v0, rwpmath::Min(v1, v2));
        rwpmath::Vector3 triBBoxMax = rwpmath::Max(v0, rwpmath::Max(v1, v2));
        if (Overlaps(bboxMin, bboxMax, triBBoxMin, triBBoxMax).GetBool())
        {
            rwpmath::Vector3 edgeCosines;
            uint32_t flags = it.GetEdgeCosinesAndFlags(edgeCosines);
            uint32_t id = it.GetID();
            StoreTriangle(v0, v1, v2, id, flags, edgeCosines);
        }
    }
}
\endcode

This class uses a ClusterUnitWalker internally, so the UnitType must satisfy the requirements of this class.

See http://docs.ea.com/RWPhysicsDev:ClusteredMesh/Iterators for further details.

*/
template <class UNIT_TYPE = GenericClusterUnit<ClusteredMeshCluster::COMPRESSION_DYNAMIC> >
class ClusterTriangleIterator
{
public:

    /// Allow user code to refer to the underlying unit type without needing to remember the template parameter.
    typedef UNIT_TYPE UnitType;

    /// Iterate over all triangles in a cluster
    RW_COLLISION_FORCE_INLINE ClusterTriangleIterator(const ClusteredMeshCluster & cluster, 
        const ClusterParams & clusterParams)
        : mUnit(cluster, clusterParams), mUnitWalker(mUnit)
    {
        Initialize();
    }
    /// Iterate over all triangles from given number of units starting at given (byte) offset
    /// If the unit count is zero then the iterator will immediately be AtEnd.
    /// If numTrianglesLeftInFirstUnit is zero (the default), all triangles will be iterated,
    /// otherwise only the last numTrianglesLeftInFirstUnit will be iterated.
    RW_COLLISION_FORCE_INLINE ClusterTriangleIterator(const ClusteredMeshCluster & cluster,
        const ClusterParams & clusterParams,
        uint32_t unitOffset,
        uint32_t unitCount,
        uint32_t numTrianglesLeftInFirstUnit = 0u)
        : mUnit(cluster, clusterParams, unitOffset), mUnitWalker(mUnit, unitCount)
    {
        // If the unit count is zero the unit construction will unfortunately instance the unit.
        Initialize(numTrianglesLeftInFirstUnit);
    }
    /// Reset to the given offset and unit count
    /// If the unit count is zero then the iterator will immediately be AtEnd.
    RW_COLLISION_FORCE_INLINE void Reset(uint32_t offset, uint32_t unitCount, uint32_t numTrianglesLeftInFirstUnit = 0u)
    {
        // If the unit count is zero the ClusterUnitWalker Reset will unfortunately instance the unit.
        mUnitWalker.Reset(offset, unitCount);
        Initialize(numTrianglesLeftInFirstUnit);
    }
    /// Check whether there are more triangles to move onto.
    RW_COLLISION_FORCE_INLINE bool AtEnd() const
    {
        const bool result = !mNumTrisLeft && mUnitWalker.AtEnd();
        return result;
    }
    /// Move onto the next triangle when not at the end.
    RW_COLLISION_FORCE_INLINE void Next()
    {
        EA_ASSERT(!AtEnd());

        if (!--mNumTrisLeft)
        {
            mUnitWalker.Next();

            if (!mUnitWalker.AtEnd())
            {
                mNumTrisLeft = mUnit.GetTriCount();
            }
            else
            {
                mNumTrisLeft = 0;
            }

            EA_ASSERT(AtEnd() || (mNumTrisLeft > 0));
        }
    }
    /// For debugging purposes, expose method to check that the iterator is in a state in which it can return a triangle.
    RW_COLLISION_FORCE_INLINE bool IsValid() const
    {
        return !AtEnd() && mUnitWalker.IsValid();
    }
    /// Return the three vertices of the triangle.
    RW_COLLISION_FORCE_INLINE void GetVertices(
        rwpmath::Vector3 & vertex0,
        rwpmath::Vector3 & vertex1,
        rwpmath::Vector3 & vertex2) const
    {
        EA_ASSERT(IsValid());
        mUnit.GetTriVertices(vertex0, vertex1, vertex2, mNumTrisLeft-1);
    }
    /// Return the three vertex indices of the triangle.
    RW_COLLISION_FORCE_INLINE void GetVertexIndices(
        uint8_t & vertex0,
        uint8_t & vertex1,
        uint8_t & vertex2) const
    {
        EA_ASSERT(IsValid());
        mUnit.GetTriVertexIndices(vertex0, vertex1, vertex2, mNumTrisLeft-1);
    }

    /// Return the three edge cosines of the current triangle.
    /// If the unit doesn't contain edge cosines, the default values from the underlying UnitType are returned,
    /// except for a shared edge in a quad which has values computed using UnitType.ComputeCentralEdgeCos().
    RW_COLLISION_FORCE_INLINE uint32_t GetEdgeCosinesAndFlags(rwpmath::Vector3 & edgeCosines) const
    {
        EA_ASSERT(IsValid());
        return mUnit.GetEdgeCosinesAndFlags(edgeCosines, mNumTrisLeft-1);
    }
    /// Return the ID associated with the current triangle (same for all triangles in current unit).
    /// Bottom 16 bits are the GroupID, top 16 bits are the SurfaceID.
    /// If the unit doesn't contain an ID, the default values from the underlying UnitType are returned.
    RW_COLLISION_FORCE_INLINE uint32_t GetID() const
    {
        return mUnit.GetID();
    }
    /// Return the group ID associated with the current triangle (same for all triangles in current unit).
    /// If the unit doesn't contain an ID, the default values from the underlying UnitType are returned.
    RW_COLLISION_FORCE_INLINE uint32_t GetGroupID() const
    {
        return mUnit.GetGroupID();
    }
    /// Return the surface ID associated with the current triangle (same for all triangles in current unit).
    /// If the unit doesn't contain an ID, the default values from the underlying UnitType are returned.
    RW_COLLISION_FORCE_INLINE uint32_t GetSurfaceID() const
    {
        return mUnit.GetSurfaceID();
    }
    /// Return the offset of the current unit
    RW_COLLISION_FORCE_INLINE uint32_t GetOffset() const
    {
        uint32_t offset = mUnit.GetOffset();
        return offset;
    }
    /// Gets the number of remaining units left to iterate
    uint32_t GetRemainingUnits()
    {
        uint32_t remainingUnits = mUnitWalker.GetRemainingUnits();
        return remainingUnits;
    }
    /// Return all information about the current triangle.
    /// May be marginally faster than getting values separately if you need them all.
    RW_COLLISION_FORCE_INLINE void GetTriangle(
        rwpmath::Vector3 & vertex0,
        rwpmath::Vector3 & vertex1,
        rwpmath::Vector3 & vertex2,
        rwpmath::Vector3 & edgeCosines,
        uint32_t & flags,
        uint32_t & id) const
    {
        EA_ASSERT(IsValid());
        const uint32_t tri = mNumTrisLeft-1;
        mUnit.GetTriVertices(vertex0, vertex1, vertex2, tri);
        flags = mUnit.GetEdgeCosinesAndFlags(edgeCosines, tri);
        id = mUnit.GetID();
    }

    /// Return all information about the current triangle with expanded masks instead of flags.
    /// May be marginally faster than getting values separately if you need them all.
    RW_COLLISION_FORCE_INLINE void GetTriangle(
        rwpmath::Vector3 & vertex0,
        rwpmath::Vector3 & vertex1,
        rwpmath::Vector3 & vertex2,
        rwpmath::Vector3 & edgeCosines,
        rwpmath::MaskScalar & oneSided,
        rwpmath::Mask3 & edgeIsConvex,
        rwpmath::Mask3 & disableVertices,
        uint32_t & id) const
    {
        uint32_t flags;
        GetTriangle(vertex0, vertex1, vertex2, edgeCosines, flags, id);
        ClusterUnitBase::ComputeTriangleMasks(edgeIsConvex, disableVertices, oneSided, flags);
    }
    /// Expose the underlying unit, in case it has additional API the user wishes to invoke.
    /// This is a reference to the unit accessor that is walked over the units in the cluster.
    RW_COLLISION_FORCE_INLINE const UnitType & GetUnit() const
    {
        return mUnit;
    }

    /// Expose the number of trangles left to process in the current underlying unit.
    RW_COLLISION_FORCE_INLINE uint32_t GetNumTrianglesLeftInCurrentUnit() const
    {
        return mNumTrisLeft;
    }
private:

    /// Update internal state when we initialize the iterator to allow for starting part way through or at
    /// the end of a a unit
    RW_COLLISION_FORCE_INLINE void Initialize(uint32_t numTrianglesLeftInFirstUnit = 0u)
    {
        if (!mUnitWalker.AtEnd())
        {
            // Units left to process so set tri count accordingly.
            if (numTrianglesLeftInFirstUnit > 0)
            {
                EA_ASSERT(numTrianglesLeftInFirstUnit <= mUnit.GetTriCount());
                mNumTrisLeft = numTrianglesLeftInFirstUnit;
            }
            else
            {
                mNumTrisLeft = mUnit.GetTriCount();
            }
        }
        else
        {
            // No units left to process so tri count must be zero.
            // This code path will be taken it the iterator is constructed or reset with a unit count of zero.
            mNumTrisLeft = 0;
        }
    }

private:    // Non-copyable

    ClusterTriangleIterator(const ClusterTriangleIterator & other);
    ClusterTriangleIterator & operator = (const ClusterTriangleIterator & other);

private:    // Internal data

    /// The unit accessor we'll walk over the cluster.
    UnitType mUnit;
    /// The walker used to advance the unit over the cluster.
    ClusterUnitWalker<UNIT_TYPE> mUnitWalker;
    /// The number of triangles not yet returned from the current unit.
    uint32_t mNumTrisLeft;
};
}
}
#endif  // !PUBLIC_RW_COLLISION_CLUSTERTRIANGLEITERATOR_H
