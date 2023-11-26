// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESHUNIT_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESHUNIT_H

/*************************************************************************************************************

File: rwcclusteredmeshunit.h

Purpose: Compressed aggregate of triangles and quads with KDTree spatial map.

*/


#include "rw/collision/common.h"

namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Forward Declarations

class ClusteredMeshCluster;
class ClusteredMesh;


/************************************************************************/
/*                               Class Definition                      */
/************************************************************************/


/**
This is a proxy class so that you can query and modify the data stored in the clustered mesh.

Note since this is a proxy (it points to the data, but doesn't contain the data) all the methods
are "const", even the ones that modify the data.

Also recognize that the assignment operator changes the proxy, not the unit pointed to by the proxy.

Sample usage:
\begincode
ClusteredMesh *cm;
for (i = 0; i < cm->GetClusterCount(); ++i)
{
for (ClusteredMeshUnit unit(cm, i); !unit.AtEnd(); unit.Next())
{
for (e = 0; e < unit.GetEdgeCount(); ++e)
{
if (unit.GetEdgeData(e) & EDGEFLAG_EDGEUNMATCHED)
{
// disable collision with unmatched edges
unit.SetEdgeData(e, EDGEFLAG_ANGLEZERO);
}
}
}
}
\endcode
*/
class ClusteredMeshUnit
{
public:
    inline ClusteredMeshUnit(ClusteredMesh *cm, uint32_t clusterId);
    inline bool AtEnd() const;
    inline void Next();
    inline bool IsValid() const;

    inline uint32_t GetType() const;
    inline uint8_t GetTypeAndFlags() const;
    inline uint32_t GetTriangleCount() const;
    /// @note GetSize() is intentionally not implemented for backwards compatibililty. Use GetUnitSize() instead.
    inline uint32_t GetSize() const;
    inline uint32_t GetUnitSize() const;

    inline uint32_t GetSurfaceId() const;
    inline void SetSurfaceId(uint32_t id) const;

    inline uint32_t GetGroupId() const;
    inline void SetGroupId(uint32_t id) const;

    inline uint32_t GetVertexCount() const;
    inline uint8_t GetVertexId(uint32_t i) const;
    inline void SetVertexId(uint32_t i, uint8_t newId) const;
    inline rwpmath::Vector3 GetVertex(uint32_t i) const;

    inline uint32_t GetEdgeCount() const;
    inline uint8_t GetEdgeData(uint32_t i) const;
    inline void SetEdgeData(uint32_t i, uint8_t newec) const;

protected:

    inline uint32_t GetMemberOffset(uint32_t flag) const;

    ClusteredMesh *mClusteredMesh;
    ClusteredMeshCluster *mCluster;
    uint8_t *mUnitData;
private:
};
}
}
#endif
