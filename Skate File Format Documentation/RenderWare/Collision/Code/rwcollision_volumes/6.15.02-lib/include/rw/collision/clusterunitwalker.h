// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTERUNITWALKER_H
#define PUBLIC_RW_COLLISION_CLUSTERUNITWALKER_H

/*************************************************************************************************************

File: rwcclusterunitwalker.h

Purpose: Class for walking over units in rw::collision::ClusteredMeshCluster.

*/

#include "rw/collision/common.h"                          // for rwpmath::*
#include "rw/collision/clusteredmesh.h"                 // for ClusteredMeshCluster
#include "rw/collision/genericclusterunit.h"            // for GenericClusterUnit

namespace rw
{
namespace collision
{

/**
This class can be used to walk a unit accessor class sequentially over units in a clustered mesh cluster.

Sample usage (to compute bounding box of first 10 units in a cluster):
\begincode
ClusteredMesh *cm;
const Cluster & cluster = cm->GetCluster(i);
EA_ASSERT(cluster.NumUnitInCluster() >= 10);
// Iterate first 10 units in the cluster
ClusterUnitWalker::UnitType unit(cluster, 0);
for (ClusterUnitWalker w(unit, 10); !w.AtEnd(); w.Next())
{
    for (v = 0; v < unit.GetNumVertices(); ++v)
    {
        rwpmath::Vector3 vertex = unit.GetVertex(i);
        min = rwpmath::Min(min, vertex);
        max = rwpmath::Max(max, vertex);
    }
}
\endcode

The walker is templated on the type of accessor class for unit data that is passed to the constructor, so it
can be used with different unit classes if you know what type of unit data is stored in the Cluster.
This UNIT_TYPE class must implement:
* Constructor(const ClusteredMeshCluster & cluster, const ClusterParams & params);
* Constructor(const ClusteredMeshCluster & cluster, const ClusterParams & params, uint32_t offset);
* void Advance();
* void Reset(uint32_t offset);
* uint32_t GetSize() const;
* const ClusteredMeshCluster & GetCluster() const;
* bool IsValid() const;

See http://docs.ea.com/RWPhysicsDev:ClusteredMesh/Iterators for further details.

*/
template <class UNIT_TYPE = GenericClusterUnit<ClusteredMeshCluster::COMPRESSION_DYNAMIC> >
class ClusterUnitWalker
{
public:

    /// Allow user code to refer to the underlying unit type without needing to remember the template parameter.
    typedef UNIT_TYPE UnitType;

    /// Walk the given unit accessor over all units in a cluster
    RW_COLLISION_FORCE_INLINE ClusterUnitWalker(UnitType & unit)
        : mUnit(unit), mRemainingUnits(mUnit.GetCluster().unitCount)
    {
    }
    /// Walk the given unit over a number of units
    RW_COLLISION_FORCE_INLINE ClusterUnitWalker(UnitType & unit, uint32_t unitCount)
        : mUnit(unit), mRemainingUnits(unitCount)
    {
    }
    /// Reset to the given offset and unit count
    RW_COLLISION_FORCE_INLINE void Reset(uint32_t offset, uint32_t unitCount)
    {
        mUnit.Reset(offset);
        mRemainingUnits = unitCount;
    }
    /// Check whether there are more units to move onto.
    RW_COLLISION_FORCE_INLINE bool AtEnd() const
    {
        return mRemainingUnits == 0;
    }
    /// Move onto the next unit when not at the end.
    RW_COLLISION_FORCE_INLINE void Next()
    {
        EA_ASSERT(!AtEnd());
        if (--mRemainingUnits)
        {
            // Advance unit accessor onto next unit
            mUnit.Advance();
        }
    }
    /// For debugging purposes, expose method to check that the iterator is in a state in which it can return a unit.
    RW_COLLISION_FORCE_INLINE bool IsValid() const
    {
        return !AtEnd() && mUnit.IsValid();
    }
    /// Provide access to the underlying cluster since we store it.
    RW_COLLISION_FORCE_INLINE const ClusteredMeshCluster & GetCluster() const
    {
        return mUnit.GetCluster();
    }
    /// Provide access to the current unit.
    RW_COLLISION_FORCE_INLINE const UnitType & GetUnit() const
    {
        return mUnit;
    }
    /// Provide access to the remaining unit count.
    uint32_t GetRemainingUnits()
    {
        return mRemainingUnits;
    }

private:    // Non-copyable

    ClusterUnitWalker(const ClusterUnitWalker & other);
    ClusterUnitWalker& operator = (const ClusterUnitWalker & other);

private:    // Internal data

    /// Reference to the unit we'll use to access the data - unit is owned by the user.
    UnitType & mUnit;
    /// Number of remaining units to iterate over.
    uint32_t mRemainingUnits;
};

}   // namespace collision
}   // namespace rw

#endif  // !PUBLIC_RW_COLLISION_CLUSTERUNITWALKER_H
