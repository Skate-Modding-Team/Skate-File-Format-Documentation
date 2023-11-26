// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESHBASE_METHODS_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESHBASE_METHODS_H

/*************************************************************************************************************/

#include "clusteredmeshbase.h"
#include "clusteredmeshcluster_methods.h"


namespace rw
{
namespace collision
{


/**
\internal
Gets the unit type.
*/
inline uint32_t
    ClusteredMesh::GetUnitType(uint32_t index, uint32_t offset) const
{
    return GetCluster(index).GetUnitType(offset);
}


/**
\internal
Get the specified vertex from a cluster.
*/
inline rwpmath::Vector3
ClusteredMesh::GetVertex(uint32_t index, uint8_t vertid) const
{
    EA_ASSERT_FORMATTED(vertid < GetCluster(index).vertexCount,
        ("ClusteredMesh::GetVertex vertex index overflow, index %d [%d]", vertid, GetCluster(index).vertexCount ));
    return  GetCluster(index).GetVertex(vertid, mClusterParams.mVertexCompressionGranularity) ;
}


/**
\internal
Report the number of volumes in this unit (1 for triangles,
2 for quads that encode 2 adjacent triangles,
or possibly a larger number in the case of trilist/strip units).
\param index cluster index
\param offset byte offset of unit within cluster
\return number of volumes contained in this unit
*/
inline uint32_t
ClusteredMesh::NumVolumesInUnit(uint32_t index, uint32_t offset) const
{
    uint8_t *data = &GetCluster(index).UnitData()[offset];

    EA_ASSERT((data[0] & UNITTYPE_MASK) <= UNITTYPE_TRILIST);  

    switch (data[0] & UNITTYPE_MASK)
    {
    case UNITTYPE_QUAD:    return 2;
    case UNITTYPE_TRILIST: return data[1];
    }
    return 1;
}


/**
\internal
Get the bbox of the whole unit.  This just looks at the vertices.
\param index cluster index
\param offset byte offset of unit within cluster
\return bbox of the unit
*/
inline AABBox
ClusteredMesh::GetUnitBBox(uint32_t index, uint32_t offset) const
{
    uint8_t *data = &GetCluster(index).UnitData()[offset];
    uint8_t *vert = data + 1;

    rwpmath::Vector3 v[4];
    AABBox bb;       // Result

    EA_ASSERT((data[0] & UNITTYPE_MASK) <= UNITTYPE_QUAD);  

    if ((data[0] & UNITTYPE_MASK) == UNITTYPE_QUAD)
    {
        GetCluster(index).Get4Vertices(v, vert[0], vert[1], vert[2], vert[3], mClusterParams.mVertexCompressionGranularity);
        bb.Set(Min(Min(v[0], v[1]), Min(v[2], v[3])), Max(Max(v[0], v[1]), Max(v[2], v[3])));
    }
    else   // UNITTYPE_TRIANGLE and UNITTYPE_OLDTRIANGLE
    {
        GetCluster(index).Get3Vertices(v, vert[0], vert[1], vert[2], mClusterParams.mVertexCompressionGranularity);
        bb.Set(Min(Min(v[0], v[1]), v[2]), Max(Max(v[0], v[1]), v[2]));
    }

    return bb;
}


/**
\internal
Get the size of the whole unit in bytes.
\param index cluster index
\param offset byte offset of unit within cluster
\return size of the unit in bytes
*/
inline uint32_t
ClusteredMesh::GetUnitSize(uint32_t index, uint32_t offset) const
{
    return GetCluster(index).GetUnitSize(offset, mClusterParams);
}


/**
\internal
Initialize a volume from the given unit index and and subindex.  
Note the subindex is to get complex units as separate triangles, such as a quad as two triangles.
Note it is much more efficient to get all the volumes at once, so we should not call this function.

\param index cluster index
\param offset byte offset of unit within cluster
\param subindex triangle within the unit
\param vol OUTPUT volume
\return size of the unit in bytes.
*/
inline uint32_t
ClusteredMesh::GetUnitVolume(uint32_t index, uint32_t offset, uint32_t subindex, Volume *vol) const
{
    uint32_t triCount, size;

    if (GetUnitType(index, offset) <= UNITTYPE_TRIANGLE)     // only one triangle
    {
        EA_ASSERT(subindex == 0);
        size = GetUnitVolumes(index, offset, vol, triCount);
    }
    else 
    {
        Volume triList[2];
        size = GetUnitVolumes(index, offset, triList, triCount);
        EA_ASSERT(subindex < triCount);
        *vol = triList[subindex];
    }
    return size;
}


inline uint32_t
ClusteredMesh::UnitGetOverlappingGPInstances(uint32_t index, uint32_t offset, const AABBox &bbox, const rwpmath::Matrix44Affine *transform,
                                             GPTriangle *instances, uint32_t &numPrimitivesInUnit) const
{
    return GetCluster(index).UnitGetOverlappingGPInstances(offset, bbox, transform, instances, numPrimitivesInUnit, mClusterParams);
}


/**
\internal Calculates the child index of a triangle referred to by a unit offset, triangle index and
cluster index.

The triangle index parameter is used to specify a triangle within the indicated unit. The range of
values are as follows:
Unit is a single triangle - 0.
Unit is a triangle pair - 0 refers to first triangle / 1 refers to the second triangle.

\param unitOffset offset of the unit.
\param unitTriangleIndex index of the triangle in the unit.
\param clusterIndex index of the cluster.

\return the ChildIndex of the triangle
*/
inline uint32_t
ClusteredMesh::GetChildIndex(
    const uint32_t unitOffset,
    const uint32_t unitTriangleIndex,
    const uint32_t clusterIndex) const
{
    // Calculate the unit tag
    const uint32_t numUnitTagBits = GetNumUnitTagBits();
    const uint32_t unitTag = (unitTriangleIndex << numUnitTagBits) + unitOffset;
    // Calculate the complete tag by adding the cluster tag (cluster index)
    return (unitTag << mNumClusterTagBits) + clusterIndex;
}


/**
\internal Gets the number of bits required to store the cluster tag.

\return number of bits required to store the cluster tag.
*/
inline uint32_t
ClusteredMesh::GetNumClusterTagBits() const
{
    return mNumClusterTagBits;
}


/**
\internal Gets the number of bits required to store the unit tag.

\return number of bits required to store the cluster tag.
*/
inline uint32_t
ClusteredMesh::GetNumUnitTagBits() const
{
    return m_numTagBits - mNumClusterTagBits - 1;
}


/**
\brief Gets the Cluster index from a child index.

\param childIndex the child index.
\return the cluster index from the child index.
*/
inline uint32_t
ClusteredMesh::GetClusterIndexFromChildIndex(uint32_t childIndex) const
{
    return (~(0xffffffff << mNumClusterTagBits)) & childIndex;
}

/**
\brief Gets the unit offset from a child index.

\param childIndex the child index.
\return the unit offset from the child index.
*/
inline uint32_t
ClusteredMesh::GetUnitOffsetFromChildIndex(uint32_t childIndex) const
{
    const uint32_t unitTag = childIndex >> mNumClusterTagBits;
    return (~(0xffffffff << GetNumUnitTagBits()) & unitTag);
}

/**
\brief Gets the unit triangle index from a child index.

\param childIndex the child index.
\return the unit triangle index.
*/
inline uint32_t
ClusteredMesh::GetTriangleIndexWithinUnitFromChildIndex(uint32_t childIndex) const
{
    return childIndex >> (m_numTagBits - 1);
}

}
}
#endif
