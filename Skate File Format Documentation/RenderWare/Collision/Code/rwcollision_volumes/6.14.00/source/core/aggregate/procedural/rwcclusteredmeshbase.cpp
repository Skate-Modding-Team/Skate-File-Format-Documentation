// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcClusteredMesh.cpp

 Purpose: Procedural aggregate of triangles with KDTree spatial map.

 */

// ***********************************************************************************************************
// Includes

#include <EAAssert/eaassert.h>

#include "rw/collision/aggregate.h"
#include "rw/collision/volumelinequery.h"
#include "rw/collision/volumebboxquery.h"
#include "rw/collision/triangle.h"

#include "rw/collision/procedural.h"
#include "rw/collision/clusteredmeshbase.h"
#include "rw/collision/clusteredmeshcluster.h"
#include "rw/collision/clusteredmeshbase_methods.h"
#include "rw/collision/clusteredmeshcluster_methods.h"
#include "rw/collision/clustertriangleiterator.h"

#include "rw/collision/trianglequery.h"

#include "rw/collision/kdtree.h"
#include "rw/collision/kdsubtree.h"

#include "sharedclustermethods.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{


// *******************************************************************************************************
// Forward Declarations

// ***********************************************************************************************************
// Typedefs

/**
\internal
 */
class AllocationHelper
{
public:

    /**
    \internal
    */
    explicit
    AllocationHelper(EA::Physics::MemoryPtr res)
    {
        mem = reinterpret_cast<uintptr_t>(res.GetMemory());
    }

    /**
    \internal
    */
    template <typename T>
    void SubAlloc(T *&result, uint32_t size, uint32_t alignment)
    {
        mem = EA::Physics::SizeAlign<uintptr_t>(mem, alignment);
        result = reinterpret_cast<T*>(mem);
        mem += size;
    }

    /**
    \internal
    */
    void *SubAlloc(uint32_t size, uint32_t alignment = 1u)
    {
        void *p;
        SubAlloc(p, size, alignment);
        return p;
    }

    uintptr_t mem;
};

// ***********************************************************************************************************
// Defines + Enums + Consts

#define DEFAULT_FLAGS CMFLAG_ONESIDED
#define DEFAULT_EDGEANGLE 128
#define DEFAULT_GROUPIDSIZE 1
#define DEFAULT_GROUPID 0
#define DEFAULT_SURFACEIDSIZE 1
#define DEFAULT_SURFACEID 0

#define COPLANAR_EDGEANGLE 128

#define MAXTRISPERLEAF 16    // todo: this should be a data value in the CM or KDT object

// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions

/*
\classinternal LinearResourceAllocator
This is the default vtable used by objects of type ClusteredMesh.
*/
rw::collision::Procedural::VTable ClusteredMesh::sm_vTable =
{
    RWCOBJECTTYPE_CLUSTEREDMESH,
    static_cast<rw::collision::Aggregate::GetSizeFn>              (&ClusteredMesh::GetSizeThis),
    rwcCLUSTEREDMESH_ALIGNMENT,
    TRUE,
    static_cast<rw::collision::Aggregate::UpdateFn>               (&ClusteredMesh::UpdateThis),
    static_cast<rw::collision::Aggregate::LineIntersectionQueryFn>(&ClusteredMesh::LineIntersectionQueryThis),
    static_cast<rw::collision::Aggregate::BBoxOverlapQueryFn>     (&ClusteredMesh::BBoxOverlapQueryThis),
    0, // rw::collision::Aggregate::GetNextVolumeFn
    0, // rw::collision::Aggregate::ClearAllProcessedFlags
    0  // rw::collision::Aggregate::ApplyUniformScale
};

// ***********************************************************************************************************
// Structs + Unions + Classes

// ***********************************************************************************************************
// Static Functions

/**
Gets the resource requirements of a ClusteredMesh object.

\note Instead of calling this API directly, you should consider using the Creator<ClusteredMesh> template.

\param  maxClusters maximum number of clusters
\param  clusterDataSize total size of all the cluster data (not including the array of pointers)
\param  numBranchNodes number of branch nodes in the kdtree.
\param  maxUnits maximum number of units.  This parameter is ignored.
\param  bbox region of space containing the mesh.  This parameter is ignored.
\param  classSize size of clustered mesh.

\return a resource descriptor for the memory requirements of the proposed ClusteredMesh object.
*/
EA::Physics::SizeAndAlignment
ClusteredMesh::GetResourceDescriptor(uint32_t maxClusters,
                                     uint32_t clusterDataSize,
                                     uint32_t numBranchNodes,
                                     uint32_t EAPHYSICS_ASSERTARGUMENT(maxUnits),
                                     const rw::collision::AABBox & /*(bbox*/,
                                     float /*vertexCompressionGranularity*/, // this doesn't affect the size
                                     uint32_t classSize,
                                     RwpBool includeKDSubTrees)
{
    EA_ASSERT_FORMATTED(maxUnits >= maxClusters, ("The max number of units %d must not be more than the max number of"
                " clusters %d.", maxUnits, maxClusters));
    EA_ASSERT_FORMATTED(maxUnits < 1000*maxClusters, ("Given that the maximum number of vertices per cluster is 256,"
                " it is unlikely that you could fit units %d into %d clusters.", maxUnits, maxClusters));
    EA_ASSERT(classSize >= sizeof(ClusteredMesh));

    uint32_t numSubTrees = includeKDSubTrees ? maxClusters : 0u;
    KDTreeWithSubTrees::ObjectDescriptor kdtreeParams(numBranchNodes, numSubTrees);
    EA::Physics::SizeAndAlignment kdtree = KDTreeWithSubTrees::GetResourceDescriptor(kdtreeParams);
    EA_ASSERT_MSG(kdtree.GetAlignment() <= rwcCLUSTEREDMESH_ALIGNMENT,
                        ("KDtree alignment is not expected to be more strict that clustered mesh."));

    // This is the order of things in the memory chunk.
    uint32_t size = EA::Physics::SizeAlign<uint32_t>(classSize, rwcCLUSTEREDMESH_ALIGNMENT);
    size += EA::Physics::SizeAlign<uint32_t>(kdtree.GetSize(), rwcCLUSTEREDMESH_ALIGNMENT);
    size += EA::Physics::SizeAlign<uint32_t>(maxClusters*sizeof(ClusteredMeshCluster*), rwcCLUSTEREDMESH_ALIGNMENT);
    size += clusterDataSize;

    return EA::Physics::SizeAndAlignment(size, rwcCLUSTEREDMESH_ALIGNMENT);
}

#undef RWUNUSED_EXCEPT_ENFORCE

/**
Creates a new clustered mesh object using the specified resource.

\note The parameters you pass to this API must match the ones passed to the GetResourceDescriptor from
which the resource was allocated.

\param  maxClusters maximum number of clusters
\param  clusterDataSize total size of all the cluster data (not including the array of pointers)
\param  numBranchNodes number of branch nodes in the kdtree
\param  maxUnits maximum number of units (triangle, quad, trilist, etc) this clustered mesh can hold.
\param  bbox region of space containing the mesh.  This is used to initialize the spatial map.
\param  classSize size of clustered mesh

\return The new ClusteredMesh.
*/
ClusteredMesh *
ClusteredMesh::Initialize(const EA::Physics::MemoryPtr& resource,
                          uint32_t maxClusters,
                          uint32_t clusterDataSize,
                          uint32_t numBranchNodes,
                          uint32_t maxUnits,
                          const rw::collision::AABBox &bbox,
                          float vertexCompressionGranularity,
                          uint32_t classSize,
                          RwpBool includeKDSubTrees)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcCLUSTEREDMESH_ALIGNMENT);
    EA_ASSERT(classSize >= sizeof(ClusteredMesh));

    AllocationHelper heap(resource);

    // allocate mesh
    Aggregate *agg = new (heap.SubAlloc(classSize)) Aggregate(maxUnits, &sm_vTable);
    ClusteredMesh *mesh = static_cast<ClusteredMesh *>(agg);

    // allocate kdtree
    uint32_t numSubTrees = includeKDSubTrees ? maxClusters : 0u;
    KDTreeWithSubTrees::ObjectDescriptor kdtreeParams(numBranchNodes, numSubTrees);
    EA::Physics::SizeAndAlignment rd = KDTreeWithSubTrees::GetResourceDescriptor(kdtreeParams);
    mesh->mKDTree = KDTreeWithSubTrees::Initialize(
        EA::Physics::MemoryPtr(heap.SubAlloc(rd.GetSize(), rd.GetAlignment())), kdtreeParams);
    rwcASSERTALIGN(mesh->mKDTree, rwcKDTREE_ALIGNMENT);
    mesh->mKDTree->m_numBranchNodes = numBranchNodes;
    mesh->mKDTree->m_numEntries = maxUnits;
    mesh->mKDTree->m_bbox = bbox;

    // round up the heap pointer to correct alignment
    heap.SubAlloc(0, rwcCLUSTEREDMESH_ALIGNMENT);

    // allocate cluster pointers
    heap.SubAlloc(mesh->mCluster, maxClusters*sizeof(uint32_t), rwcCLUSTEREDMESH_ALIGNMENT);
    rwcASSERTALIGN(mesh->mCluster, rwcCLUSTEREDMESH_ALIGNMENT);

    // round up the heap pointer to correct alignment
    heap.SubAlloc(0, rwcCLUSTEREDMESH_ALIGNMENT);

    // set offset to first cluster from mCluster array (prior to version 3 this was relative to "this")
    uintptr_t offset = heap.mem - reinterpret_cast<uintptr_t>(mesh->mCluster);
    EA_ASSERT(offset < UINT32_MAX);
    mesh->mCluster[0] = static_cast<uint32_t>(offset);

    // reserve space for all clusters
    heap.SubAlloc(clusterDataSize, rwcCLUSTEREDMESH_ALIGNMENT);

    // set size of this for the GetSizeThis method
    mesh->mSizeOfThis = ClusteredMesh::GetResourceDescriptor(maxClusters, clusterDataSize, numBranchNodes,
                                                             maxUnits, bbox,
                                                             vertexCompressionGranularity,
                                                             classSize, includeKDSubTrees).GetSize();

    // assert that the memory allocated from the heap is not greater than the size allowed.
    EA_ASSERT(mesh->mSizeOfThis >= heap.mem - reinterpret_cast<uintptr_t>(mesh));

    // set default mesh data
    mesh->m_AABB = bbox;
    mesh->mNumClusters = 0;
    mesh->mMaxClusters = maxClusters;
    mesh->mNumUnits = 0;
    mesh->mMaxUnits = maxUnits;
    mesh->mClusterParams.mVertexCompressionGranularity = vertexCompressionGranularity;
    mesh->mClusterParams.mFlags = DEFAULT_FLAGS;
    mesh->mDefaultGroupId = DEFAULT_GROUPID;
    mesh->mDefaultSurfaceId = DEFAULT_SURFACEID;
    mesh->mDefaultEdgeAngle = DEFAULT_EDGEANGLE;
    mesh->mClusterParams.mGroupIdSize = DEFAULT_GROUPIDSIZE;
    mesh->mClusterParams.mSurfaceIdSize = DEFAULT_SURFACEIDSIZE;

    if (maxClusters > (1u << 16))
    {
        mesh->mClusterParams.mFlags = static_cast<uint16_t>(mesh->mClusterParams.mFlags | CMFLAG_20BITCLUSTERINDEX);
    }

    // Set Tag members
    mesh->mNumClusterTagBits = 0;

    return mesh;
}

// *********************************************************************************************************

struct ClusteredMeshValidityCheckNodeData
{
    uint32_t parent;
    AABBox   bbox;
};

/**
Tests the ClusteredMesh for internal consistency. This function is only available in the debug library.
\return true if and only if the ClusteredMesh is fully initialized and populated and ready to query.
*/
RwpBool
ClusteredMesh::IsValid() const
{
    RwpBool ok = static_cast<RwpBool>(mKDTree != NULL);

    ok = static_cast<RwpBool>(ok && m_AABB.IsValid());
    ok = static_cast<RwpBool>(ok && mKDTree->IsValid());
    ok = static_cast<RwpBool>(ok && mCluster != NULL);
    ok = static_cast<RwpBool>(ok && mNumClusters > 0 && mMaxClusters >= mNumClusters);
    ok = static_cast<RwpBool>(ok && mNumUnits > 0 && mMaxUnits >= mNumUnits);
    ok = static_cast<RwpBool>(ok && mClusterParams.mGroupIdSize <=2);
    ok = static_cast<RwpBool>(ok && mClusterParams.mSurfaceIdSize <=2);

    for (uint32_t i = 0; ok && i < mNumClusters; ++i)
    {
        ClusteredMeshCluster &cluster = GetCluster(i);

        if ( cluster.compressionMode == ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED )
        {
            uint32_t bytes = 3*sizeof(int32_t) + sizeof(ClusteredMeshCluster::Vertex16) * cluster.vertexCount;
            bytes = EA::Physics::SizeAlign<uint32_t>( bytes, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
            ok = static_cast<RwpBool>(ok && cluster.normalStart == static_cast<uint16_t>( bytes / rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT ));
            ok = static_cast<RwpBool>(ok && cluster.unitDataStart == cluster.normalStart + cluster.normalCount);
        }
        else
        if ( cluster.compressionMode == ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED )
        {
            uint32_t bytes = sizeof(ClusteredMeshCluster::Vertex32) * cluster.vertexCount;
            bytes = EA::Physics::SizeAlign<uint32_t>( bytes, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
            ok = static_cast<RwpBool>(ok && cluster.normalStart == static_cast<uint16_t>( bytes / rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT ));
            ok = static_cast<RwpBool>(ok && cluster.unitDataStart == cluster.normalStart + cluster.normalCount);
        }
        else
        {
            ok = static_cast<RwpBool>(ok && cluster.normalStart == cluster.vertexCount);
            ok = static_cast<RwpBool>(ok && cluster.unitDataStart == cluster.vertexCount + cluster.normalCount);
        }
    }

    // Now check units
    uint32_t clusterIndex=0;
    ClusteredMeshCluster cluster = GetCluster(0);
    uint32_t offsetIntoCluster=0;
    uint32_t unitCount=0, totalUnitCount=0;

    ClusteredMeshValidityCheckNodeData curData;
    curData.bbox = mKDTree->GetBBox();
    curData.parent = 0;

    KDTree::Traversal<ClusteredMeshValidityCheckNodeData> traversal(mKDTree, curData);

    while (traversal.PopNode(curData))
    {
        if (traversal.CurrentNodeIsBranch())
        {
            uint32_t index = traversal.GetBranchIndex();
            KDTree::BranchNode &branch = mKDTree->m_branchNodes[index];
            ClusteredMeshValidityCheckNodeData childData;

            childData.parent = traversal.GetBranchIndex();

            // Push right
            childData.bbox = curData.bbox;
            childData.bbox.m_min.SetComponent((uint16_t)branch.m_axis, branch.m_extents[1]);
            traversal.PushChildNode(1, childData);

            // Push left
            childData.bbox = curData.bbox;
            childData.bbox.m_max.SetComponent((uint16_t)branch.m_axis, branch.m_extents[0]);
            traversal.PushChildNode(0, childData);
        }
        else
        {
            uint32_t first, count;
            Volume vol;
            traversal.GetLeafNodeEntries(first, count);

            // For all units
            // 'first' is split 16/16 (or 20/12) int
            uint32_t shift = (uint32_t)16 + (mClusterParams.mFlags & CMFLAG_20BITCLUSTERINDEX);
            uint32_t mask = (uint32_t)((1 << shift) - 1);
            uint32_t offset = first & mask;
            uint32_t index  = first >> shift;

            //Check values of index & count from KDTree leaf match manually incremented values
            if(count > 0)
            {
                if(offset != offsetIntoCluster)
                {
                    EAPHYSICS_MESSAGE("Cluster Offset error in KDTree leaf node!");
                    ok = FALSE;
                }

                if(index != clusterIndex)
                {
                    EAPHYSICS_MESSAGE("Cluster Index error in KDTree leaf node!");
                    ok = FALSE;
                }
            }

            VecFloat granularityImprecision = GetVecFloat_Two() * VecFloat(mClusterParams.mVertexCompressionGranularity );
            AABBox bboxGranular = curData.bbox;
            // this is to work around the fact that vertex compression makes vertex coordinates
            // move around within the compression granularity
            // The granularity imprecision factor is increased by 100% to account for floating point
            // precision errors.
            bboxGranular.m_min -= granularityImprecision;
            bboxGranular.m_max += granularityImprecision;

            //Go through all leaf elements
            for (uint32_t i = 0; i < count; i++)
            {
                uint32_t type = GetUnitType(clusterIndex, offsetIntoCluster);
                if(type > UNITTYPE_QUAD)
                {
                    EAPHYSICS_MESSAGE("Clustered Mesh Unit type %d is not a Triangle or Quad! [%d]",
                               type, GetCluster(clusterIndex).UnitData()[offsetIntoCluster]);
                    ok = FALSE;
                    break; // It will crash on the next call
                }

                offsetIntoCluster += GetUnitVolume(clusterIndex, offsetIntoCluster, 0, &vol);
                unitCount++; // Increment the count of units

                //Increment clusterIndex when we get to numUnits
                if(unitCount == cluster.unitCount)
                {
                    // Check if the current leaf node spans more than a single cluster
                    // i.e if there are still leaf node entries to deal with after having
                    // reached the end of the current cluster.
                    if (i < (count - 1))
                    {
                        EAPHYSICS_MESSAGE("A KDTree LeafNode spans more than one Cluster.");
                        EAPHYSICS_MESSAGE("NOTE: Future changes in behavior will regarded this as an Invalid ClusteredMesh. Avoid this issue by regenerating this mesh.");
                    }

                    //When we move to the next cluster, the offset should have moved beyond the end of the data
                    if(offsetIntoCluster < cluster.unitDataSize)
                    {
                        EAPHYSICS_MESSAGE("Mismatch between total unit data size and number of units in Cluster");
                        ok = FALSE;
                    }
                    offsetIntoCluster = 0;
                    totalUnitCount += unitCount;
                    unitCount = 0;
                    clusterIndex++;

                    if(clusterIndex < mNumClusters)
                    {
                        cluster = GetCluster(clusterIndex);
                    }
                    else if(totalUnitCount < mNumUnits)
                    {
                        //Fail if we've got to the end of the CLusters but still haven't accounted for all units
                        EAPHYSICS_MESSAGE("Some Units not in Clusters of Clustered Mesh");
                        ok = FALSE;
                    }
                }


                AABBox bb;
                vol.GetBBox(0,0,bb);

                // Check that leaf node bbox contains volume
                if (!bboxGranular.Contains(bb))
                {
                    EAPHYSICS_MESSAGE("Triangle %d outside of leaf bounding box (internal node %d).",
                               i, curData.parent);
                    ok = FALSE;
                }
            }
        }
    }

    return ok;
}



/**
\internal
Set the pointer for the next cluster to a vacant chunk of memory.  alignment = rwcCLUSTEREDMESHCLUSTER_ALIGNMENT
\param datalen size of cluster data in bytes.
\return pointer to the next cluster memory block
*/
ClusteredMeshCluster *
ClusteredMesh::AllocateNextCluster(uint32_t datalen, uint32_t numUnits)
{
    EA_ASSERT_MSG(mNumClusters < mMaxClusters, ("Too many clusters added."));
    uint32_t id = mNumClusters++;

    mNumUnits += numUnits;
    EA_ASSERT_MSG(mNumUnits <= mMaxUnits, ("Too many units added."));

    EA_ASSERT_FORMATTED((mCluster[id] + datalen + reinterpret_cast<uintptr_t>(mCluster) - reinterpret_cast<uintptr_t>(this)) <= mSizeOfThis,
        ("ClusteredMeshCluster data cannot fit, id=%d, startoffset=%d, len=%d, maxoffset=%d.", 
        id, mCluster[id], datalen, mSizeOfThis));

    // set the offset to the next cluster
    if (mNumClusters < mMaxClusters)
    {
        mCluster[mNumClusters] = mCluster[id] + EA::Physics::SizeAlign<uint32_t>(datalen, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    }
    rwcASSERTALIGN(&GetCluster(id), rwcCLUSTEREDMESH_ALIGNMENT);

    return &GetCluster(id);
}

ClusteredMeshCluster *
ClusteredMesh::AllocateNextCluster(const ClusterConstructionParameters & parameters)
{
    EA_ASSERT_MSG(mNumClusters < mMaxClusters, ("Too many clusters added."));
    uint32_t id = mNumClusters++;

    mNumUnits += parameters.mTriangleUnitCount + parameters.mQuadUnitCount;
    EA_ASSERT_MSG(mNumUnits <= mMaxUnits, ("Too many units added."));

    uint32_t size = ClusteredMeshCluster::GetSize(parameters);

    EA_ASSERT_FORMATTED((mCluster[id] + size + reinterpret_cast<uintptr_t>(mCluster) - reinterpret_cast<uintptr_t>(this)) <= mSizeOfThis,
        ("ClusteredMeshCluster data cannot fit, id=%d, startoffset=%d, len=%d, maxoffset=%d.", 
        id, mCluster[id], size, mSizeOfThis));

    // set the offset to the next cluster
    if (mNumClusters < mMaxClusters)
    {
        mCluster[mNumClusters] = mCluster[id] + EA::Physics::SizeAlign<uint32_t>(size, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    }
    rwcASSERTALIGN(&GetCluster(id), rwcCLUSTEREDMESH_ALIGNMENT);

    return ClusteredMeshCluster::Initialize(reinterpret_cast<void*>(&GetCluster(id)), parameters);
}

/**
\brief Fills out a triangle volume with the triangle details referred to by a child index

\param triangleVolume the triangle volume.
\param childIndex the child index referring to the source triangle.
*/
void
ClusteredMesh::GetVolumeFromChildIndex(
    rw::collision::TriangleVolume & triangleVolume,
    const uint32_t childIndex) const
{
    // Extract the indices/offsets from the child index
    const uint32_t clusterIndex = GetClusterIndexFromChildIndex(childIndex);
    const uint32_t unitOffset = GetUnitOffsetFromChildIndex(childIndex);
    const uint32_t triangleIndex = GetTriangleIndexWithinUnitFromChildIndex(childIndex);

    // Get the volume from the cluster
    GetCluster(clusterIndex).GetTriangleVolume(
        triangleVolume,
        unitOffset,
        triangleIndex,
        mClusterParams);
}

void
ClusteredMesh::UpdateNumTagBits()
{
    // Set the aggregate tag details
    mNumClusterTagBits = 1u + static_cast<uint32_t>(rwpmath::Log(static_cast<float>(mNumClusters)) / rwpmath::Log(2.0f));

    // Determine the maximum unit stream length
    uint32_t maxUnitStreamLength = 0u;
    for (uint32_t clusterIndex = 0 ; clusterIndex < mNumClusters ; ++clusterIndex)
    {
        ClusteredMeshCluster &cluster = GetCluster(clusterIndex);
        maxUnitStreamLength = (maxUnitStreamLength < cluster.unitDataSize ? cluster.unitDataSize : maxUnitStreamLength);
    }

    const uint32_t numUnitTagBits = 1u + static_cast<uint32_t>(rwpmath::Log(static_cast<float>(maxUnitStreamLength)) / rwpmath::Log(2.0f));

    // The complete number of tag bits is the sum of the cluster tag bits,
    // unit tag bits and one byte to indicate the unit triangle index
    m_numTagBits = mNumClusterTagBits + numUnitTagBits + 1;
}

// *****************************************************************************************************
//   Virtual functions required by the Aggregate interface


/**
\see rw::collision::Procedural::GetSize.
 */

uint32_t
ClusteredMesh::GetSizeThis()
{
    return mSizeOfThis;
}


/**
\internal
Copies the AABBox from the kdtree to the aggregate.
*/
void
ClusteredMesh::UpdateThis()
{
    // Set the aabbox
    m_AABB = mKDTree->GetBBox();

    // Set the tag members
    UpdateNumTagBits();
}




/**
Initialize a list of volumes from the given unit.

\param index cluster index
\param offset byte offset of unit within cluster
\param triList OUTPUT array of volumes
\param triCount OUTPUT number of volumes created in triList
\return size of the unit in bytes.

Note the caller MUST ensure that the output array is large enough.
*/
uint32_t
ClusteredMesh::GetUnitVolumes(uint32_t index, uint32_t offset, Volume *triList, uint32_t &triCount) const
{
    uint8_t *data = &GetCluster(index).UnitData()[offset];
    uint32_t unitType = (uint32_t) data[0] & UNITTYPE_MASK;
    EA_ASSERT(unitType <= UNITTYPE_TRILIST);
    uint8_t *vert = data + 1;     // The vertices come right after the type and count

    triCount = 1;

    if (unitType == UNITTYPE_QUAD)
    {
        triCount = 2;
    }
    else if (unitType == UNITTYPE_TRILIST)
    {
        triCount = *(vert++);
    }

    uint8_t *edge = vert + triCount + 2;    // The edges come right after the vertices
    uint8_t *misc = edge + ((data[0] & UNITFLAG_EDGEANGLE) ? triCount + 2 : 0);

    // The default ids are zero, see volume.hpp
    uint32_t groupId = 0;
    uint32_t surfaceId = 0;

    //  Parse the misc data

    if (data[0] & UNITFLAG_GROUPID)
    {
        groupId = *(misc++);
        groupId += (mClusterParams.mGroupIdSize==2) ? *(misc++) * 256 : 0;
    }
    if (data[0] & UNITFLAG_SURFACEID)
    {
        surfaceId = *(misc++);
        surfaceId += (mClusterParams.mSurfaceIdSize==2) ? *(misc++) * 256 : 0;
    }
    uint32_t size = (uint32_t) (misc - data);
    Vector3 v[4];

    if (triCount == 1)          // Single triangle
    {
        TriangleVolume *tri;

        // Decompress the vertices
        GetCluster(index).Get3Vertices(v, vert[0], vert[1], vert[2], mClusterParams.mVertexCompressionGranularity);

        tri = TriangleVolume::Initialize(EA::Physics::MemoryPtr(&triList[0]), v[0], v[1], v[2]);
        tri->SetGroup(groupId);
        tri->SetSurface(surfaceId);

        if (unitType == UNITTYPE_OLDTRIANGLE)
        {
            // For "old triangles" just copy the upper nibble of the unitflags onto the triangle flags
            // and turn off the "edgecos" flag.   TODO: deprecate and remove OLDTRIANGLE.

            tri->SetFlags((tri->GetFlags() & ~0x1F0) | (data[0] & 0xF0));
        }
        else if (data[0] & UNITFLAG_EDGEANGLE)
        {
            tri->SetEdgeCos(DecodeEdgeCos((uint32_t)edge[0] & EDGEFLAG_ANGLEMASK),
                            DecodeEdgeCos((uint32_t)edge[1] & EDGEFLAG_ANGLEMASK),
                            DecodeEdgeCos((uint32_t)edge[2] & EDGEFLAG_ANGLEMASK));
            tri->SetFlags(VOLUMEFLAG_TRIANGLENORMALISDIRTY | ComputeTriangleFlags(edge[0], edge[1], edge[2], mClusterParams.mFlags));
        }
    }
    else if (triCount == 2)        // Quad
    {
        TriangleVolume *tri0, *tri1;

        float innerEdgeCos = 0.0f;
        int8_t innerFlags;

        // Decompress the vertices
        GetCluster(index).Get4Vertices(v, vert[0], vert[1], vert[2], vert[3], mClusterParams.mVertexCompressionGranularity);

        // FIRST TRIANGLE (0,1,2)
        tri0 = TriangleVolume::Initialize(EA::Physics::MemoryPtr(&triList[0]), v[0], v[1], v[2]);
        tri0->SetGroup(groupId);
        tri0->SetSurface(surfaceId);

        // SECOND TRIANGLE (3,2,1)
        tri1 = TriangleVolume::Initialize(EA::Physics::MemoryPtr(&triList[1]), v[3], v[2], v[1]);
        tri1->SetGroup(groupId);
        tri1->SetSurface(surfaceId);

        if (data[0] & UNITFLAG_EDGEANGLE)
        {
            // Compute the edgecos and flags for the interior edge.
            innerEdgeCos = ComputeEdgeCos(innerFlags, v[0], v[1], v[2], v[3]);

            // Set flags and edgecos of FIRST TRIANGLE
            tri0->SetEdgeCos(DecodeEdgeCos((uint32_t)edge[0] & EDGEFLAG_ANGLEMASK),
                             innerEdgeCos,
                             DecodeEdgeCos((uint32_t)edge[2] & EDGEFLAG_ANGLEMASK));
            tri0->SetFlags(VOLUMEFLAG_TRIANGLENORMALISDIRTY | ComputeTriangleFlags(edge[0], static_cast<uint8_t>
                    ((edge[1] & EDGEFLAG_VERTEXDISABLE) | innerFlags), edge[2], mClusterParams.mFlags));

            // Set flags and edgecos of SECOND TRIANGLE
            tri1->SetEdgeCos(DecodeEdgeCos((uint32_t)edge[3] & EDGEFLAG_ANGLEMASK),
                             innerEdgeCos,
                             DecodeEdgeCos((uint32_t)edge[1] & EDGEFLAG_ANGLEMASK));
            tri1->SetFlags(VOLUMEFLAG_TRIANGLENORMALISDIRTY | ComputeTriangleFlags(edge[3], static_cast<uint8_t>
                    ((edge[2] & EDGEFLAG_VERTEXDISABLE) | innerFlags), edge[1], mClusterParams.mFlags));
        }
    }
    else
    {
        EA_FAIL_MSG(("Trilist size > 2 not implemented yet."));
    }

    return size;
}


/**
Gets the number of units that are in a specified cluster of the clustered mesh.
\return the numUnits in cluster
*/
uint32_t
ClusteredMesh::GetNumUnitInCluster(uint32_t clusterId) const
{
    return GetCluster(clusterId).unitCount;
}


// Disable "assignment within conditional" warning, as it is used intentionally below.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4706)
#endif


/**
\internal
This tests the input line query against the collision data in the clustered mesh.
The input line query structure contains the results buffer and this function
can be called multiple times to retrieve all intersections in the case of the
buffer becoming full.

\param lineQuery Initialized line query structure.
\param tm The transform of the aggregate in the query frame.

\return TRUE if the query finished, FALSE if the results buffer overflowed and
        LineIntersectionQuery needs to be called again.
\see rw::collision::Aggregate::LineIntersectionQuery.
*/
RwpBool
ClusteredMesh::LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                                         const rwpmath::Matrix44Affine *tm)
{
    uint32_t entry = 0;
    uint32_t clusterIndex = rwcKDTREE_INVALID_INDEX;
    uint32_t unitOffset = 0;
    uint32_t unitCount = 0;
    uint32_t numTrisLeftInUnit = 0;

    // Map line into spatial map space
    const Matrix44Affine invTm(InverseOfMatrixWithOrthonormal3x3(*tm));
    const Vector3 localLineStart = TransformPoint(lineQuery->m_pt1, invTm);
    const Vector3 localLineEnd   = TransformPoint(lineQuery->m_pt2, invTm);
    const Vector3 localLineDelta = localLineEnd - localLineStart;

    // See whether to start a new query
    KDTree::LineQuery *mapQuery = static_cast<KDTree::LineQuery *>(lineQuery->m_curSpatialMapQuery);

    if (!mapQuery)
    {
        const VecFloat granularityImprecision = GetVecFloat_Two() * VecFloat(mClusterParams.mVertexCompressionGranularity);

        mapQuery = new (lineQuery->m_spatialMapQueryMem)KDTree::LineQuery(
            GetKDTreeBase(), localLineStart, localLineEnd, granularityImprecision + lineQuery->m_fatness);
        lineQuery->m_curSpatialMapQuery = reinterpret_cast<void*>(mapQuery);

        // Get the first leaf node
        if (!mapQuery->GetNext(entry, unitCount))
        {
            // No more results
            return TRUE;
        }
    }
    else
    {
        // Resume from the last saved point
        entry = lineQuery->m_clusteredMeshRestartData.entry;
        unitCount = lineQuery->m_clusteredMeshRestartData.unitCount;
        numTrisLeftInUnit = lineQuery->m_clusteredMeshRestartData.numTrisLeftInUnit;
    }

    // Far clip val might have been set by another volume hit
    if (lineQuery->m_resultsSet != VolumeLineQuery::ALLLINEINTERSECTIONS)
    {
        mapQuery->ClipEnd(lineQuery->m_endClipVal);
    }

    const uint32_t shift = (uint32_t)(16 + (mClusterParams.mFlags & CMFLAG_20BITCLUSTERINDEX));
    const uint32_t mask = (uint32_t)((1 << shift) - 1);

    //Keep going as long as:
    //1) there is more space in the line query results buffer
    //2) there is more space in the instance volume buffer
    //3) there are more results from the kd tree query
    do
    {
        clusterIndex = entry >> shift;
        unitOffset = entry & mask;

nextCluster:
        ClusterTriangleIterator<> cti(GetCluster(clusterIndex), mClusterParams, unitOffset, unitCount, numTrisLeftInUnit);
        EA_ASSERT(cti.IsValid());

        for (; !cti.AtEnd(); cti.Next())
        {
            Vector3 v0, v1, v2;
            cti.GetVertices(v0, v1, v2);

            RwpBool hit = FALSE;
            VolumeLineSegIntersectResult tmpRes;

            if (IsOneSided())
            {
                hit = TriangleLineSegIntersect(tmpRes, localLineStart, localLineDelta, v0, v1, v2, lineQuery->m_fatness);
            }
            else
            {
                hit = TriangleLineSegIntersectTwoSided(tmpRes, localLineStart, localLineDelta, v0, v1, v2, lineQuery->m_fatness);
            }

            if (hit)
            {
                if (lineQuery->m_resCount == lineQuery->m_resMax ||
                    lineQuery->m_instVolCount == lineQuery->m_instVolMax)
                {
                    // Cache current position in the query so we can restart from this exact point.
                    lineQuery->m_clusteredMeshRestartData.entry = (clusterIndex << shift) | cti.GetOffset();
                    lineQuery->m_clusteredMeshRestartData.unitCount = cti.GetRemainingUnits();
                    lineQuery->m_clusteredMeshRestartData.numTrisLeftInUnit = cti.GetNumTrianglesLeftInCurrentUnit();

                    return FALSE;
                }

                VolumeLineSegIntersectResult *res = &lineQuery->m_resBuffer[lineQuery->m_resCount];

                // Instance triangle volume
                Volume *vol = &lineQuery->m_instVolPool[lineQuery->m_instVolCount];
                TriangleVolume *tri = TriangleVolume::Initialize(EA::Physics::MemoryPtr(vol), v0, v1, v2);

                // Set Group and Surface ID
                InitializeTriangleVolumeDetails(
                    *tri,
                    cti);

                res->inputIndex = lineQuery->m_currInput-1;
                res->v = lineQuery->m_inputVols[res->inputIndex];

                // Map intersect result back into query space
                res->position = TransformPoint(tmpRes.position, *tm);
                res->normal = TransformVector(tmpRes.normal, *tm);
                res->volParam = tmpRes.volParam;
                res->lineParam = tmpRes.lineParam;

                // In future the vref should be in a freelist
                res->vRef.volume = vol;
                res->vRef.tmContents = *tm;
                res->vRef.tm = &res->vRef.tmContents;

                // Setup tag to this triangle
                uint32_t tag = lineQuery->m_tag;
                uint32_t numTagBits = lineQuery->m_numTagBits;

                // Get the child index of the triangle
                uint32_t childIndex = GetChildIndex(
                                          cti.GetOffset(),
                                          cti.GetNumTrianglesLeftInCurrentUnit() - 1u,
                                          clusterIndex);

                UpdateTagWithChildIndex(tag, numTagBits, childIndex);

                res->vRef.tag = tag;
                res->vRef.numTagBits = static_cast<uint8_t>(numTagBits);

                // We have a hit
                lineQuery->m_resCount++;
                lineQuery->m_instVolCount++;

                // Clip the line to min distance
                if (lineQuery->m_resultsSet != VolumeLineQuery::ALLLINEINTERSECTIONS)
                {
                    if(res->lineParam < lineQuery->m_endClipVal )
                    {
                        lineQuery->m_endClipVal = res->lineParam;
                        mapQuery->ClipEnd(lineQuery->m_endClipVal);
                    }
                }
            }
            // Temporary workaround in case KDTree leaf nodes span across cluster boundaries
            if ((cti.GetNumTrianglesLeftInCurrentUnit() <= 1) && 
                (cti.GetRemainingUnits() > 1) &&
                (cti.GetOffset() + cti.GetUnit().GetSize() >= GetCluster(clusterIndex).unitDataSize))
            {
                clusterIndex++;
                numTrisLeftInUnit = 0;
                unitOffset = 0;
                unitCount = cti.GetRemainingUnits()-1;
                goto nextCluster;
            }
        }

        numTrisLeftInUnit = 0;
    }
    while (mapQuery->GetNext(entry, unitCount));

    return TRUE;
}

/**
\internal
Tests the input bounding box query against the collision data in the clustered mesh.
The input bbox query structure contains the VolRef results buffer and this function
can be called multiple times to retrieve all overlaps in the case of the
buffer becoming full.

\param bboxQuery Initialized bounding box query structure.
\param tm The transform of the aggregate in the query frame.

\return TRUE if the query finished, FALSE if the results buffer could not hold
all the results and BBoxOverlapQuery needs to be called again.
*/
RwpBool
ClusteredMesh::BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                                    const rwpmath::Matrix44Affine *tm)
{
    uint32_t entry = 0;
    uint32_t clusterIndex = rwcKDTREE_INVALID_INDEX;
    uint32_t unitOffset = 0;
    uint32_t unitCount = 0;
    uint32_t numTrisLeftInUnit = 0;

    KDTree::BBoxQuery *mapQuery = reinterpret_cast<KDTree::BBoxQuery*>(bboxQuery->m_curSpatialMapQuery);

    //See whether to start a new query
    if (!mapQuery)
    {
        AABBox *localBBox;
        AABBox tempbb;

        //map bb into spatial map space
        if (tm)
        {
            const Matrix44Affine invTm(InverseOfMatrixWithOrthonormal3x3(*tm));
            tempbb = bboxQuery->m_aabb.Transform(&invTm);
            localBBox = &tempbb;
        }
        else
        {
            localBBox = &bboxQuery->m_aabb;
        }

        // grow the query bbox by vertex compression granularity
        VecFloat granularityImprecision = GetVecFloat_Two() * VecFloat(mClusterParams.mVertexCompressionGranularity );
        AABBox bboxGranular = *localBBox;
        // this is to work around the fact that vertex compression makes vertex coordinates
        // move around within the compression granularity
        bboxGranular.m_min -= granularityImprecision;
        bboxGranular.m_max += granularityImprecision;

        // Initialize query
        mapQuery = new (bboxQuery->m_spatialMapQueryMem) KDTree::BBoxQuery(GetKDTreeBase(), bboxGranular);
        bboxQuery->m_curSpatialMapQuery = reinterpret_cast<void*>(mapQuery);

        // Get the first leaf node
        if (!mapQuery->GetNext(entry, unitCount))
        {
            // No more results
            return TRUE;
        }
    }
    else
    {
        // Resume from last saved point
        entry = bboxQuery->m_clusteredMeshRestartData.entry;
        unitCount = bboxQuery->m_clusteredMeshRestartData.unitCount;
        numTrisLeftInUnit = bboxQuery->m_clusteredMeshRestartData.numTrisLeftInUnit;
    }

    const uint32_t shift = (uint32_t)(16 + (mClusterParams.mFlags & CMFLAG_20BITCLUSTERINDEX));
    const uint32_t mask = (uint32_t)((1 << shift) - 1);

    //Keep going as long as:
    //1) there is more space in the volume buffer
    //2) there is more space in the primitive  buffer
    //3) there are more results from the kd tree query
    do
    {
        clusterIndex = entry >> shift;
        unitOffset = entry & mask;

nextCluster:

        ClusterTriangleIterator<> cti(GetCluster(clusterIndex), mClusterParams, unitOffset, unitCount, numTrisLeftInUnit);
        EA_ASSERT(cti.IsValid());

        for (; !cti.AtEnd(); cti.Next())
        {
            Vector3 v0, v1, v2;
            cti.GetVertices(v0, v1, v2);

            // Calculate the triangles aabbox
            const Vector3 bboxMin(Min(Min(v0, v1), v2));
            const Vector3 bboxMax(Max(Max(v0, v1), v2));
            const AABBox triaabbox(bboxMin, bboxMax);

            // Test the bbox of the triangle against the query bbox.
            if (mapQuery->GetBBox().Overlaps(triaabbox))
            {
                // Make sure there is enough room for all the volumes of this unit to fit.
                if (bboxQuery->m_primNext == bboxQuery->m_primBufferSize ||
                    bboxQuery->m_instVolCount == bboxQuery->m_instVolMax)
                {
                    if (bboxQuery->m_primNext == bboxQuery->m_primBufferSize)
                    {
                        bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE);
                    }

                    if (bboxQuery->m_instVolCount == bboxQuery->m_instVolMax)
                    {
                        bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFINSTANCEBUFFERSPACE);
                    }

                    // Cache current position in the query so we can restart from this exact point.
                    bboxQuery->m_clusteredMeshRestartData.entry = (clusterIndex << shift) | cti.GetOffset();
                    bboxQuery->m_clusteredMeshRestartData.unitCount = cti.GetRemainingUnits();
                    bboxQuery->m_clusteredMeshRestartData.numTrisLeftInUnit = cti.GetNumTrianglesLeftInCurrentUnit();

                    return FALSE;
                }

                // Instance triangle volume
                Volume *vol = &bboxQuery->m_instVolPool[bboxQuery->m_instVolCount++];
                TriangleVolume *tri = TriangleVolume::Initialize(EA::Physics::MemoryPtr(vol), v0, v1, v2);

                InitializeTriangleVolumeDetails(
                    *tri,
                    cti);

                // Setup tag to this triangle
                uint32_t tag = bboxQuery->m_tag;
                uint32_t numTagBits = bboxQuery->m_numTagBits;

                // Get the child index of this triangle
                const uint32_t childIndex = GetChildIndex(
                                                cti.GetOffset(),
                                                cti.GetNumTrianglesLeftInCurrentUnit() - 1u,
                                                clusterIndex);

                UpdateTagWithChildIndex(tag, numTagBits, childIndex);

                if (tm)
                {
                    const Vector3 v0_t(TransformPoint(v0, *tm));
                    const Vector3 v1_t(TransformPoint(v1, *tm));
                    const Vector3 v2_t(TransformPoint(v2, *tm));

                    const Vector3 tribboxMin(Min(Min(v0_t, v1_t), v2_t));
                    const Vector3 tribboxMax(Max(Max(v0_t, v1_t), v2_t));
                    const AABBox triaabbox_t(tribboxMin, tribboxMax);

                    bboxQuery->AddPrimitiveRef(vol, tm, triaabbox_t, tag, static_cast<uint8_t>(numTagBits));
                }
                else
                {
                    bboxQuery->AddPrimitiveRef(vol, tm, triaabbox, tag, static_cast<uint8_t>(numTagBits));
                }
            }
            // Temporary workaround in case KDTree leaf nodes span across cluster boundaries
            if ((cti.GetNumTrianglesLeftInCurrentUnit() <= 1) && 
                (cti.GetRemainingUnits() > 1) &&
                (cti.GetOffset() + cti.GetUnit().GetSize() >= GetCluster(clusterIndex).unitDataSize))
            {
                clusterIndex++;
                numTrisLeftInUnit = 0;
                unitOffset = 0;
                unitCount = cti.GetRemainingUnits()-1;
                goto nextCluster;
            }
        }

        numTrisLeftInUnit = 0;
    }
    while (mapQuery->GetNext(entry, unitCount));

    return TRUE;
}


// *********************************************************************************************************

// *********************************************************************************************************

struct NodeData
{
    uint32_t nodeIndex;
    uint32_t clusterNo;
    uint32_t numEntries;
    uint32_t lastNode;
};

/**
\brief
Returns a EA::Physics::SizeAndAlignment for the 'Workspace' needed to generate the KDSubTree array. 
This is the worst case requirement - (an unbalanced tree of containing only one cluster) 

In most cases, a smaller EA::Physics::MemoryPtr could be used instead - large enough to contain nodedata 
for the number of clusters and the maximum number of leafbranchnodes in a cluster.

\param clusteredMesh The ClusteredMesh for which the array is to be generated

\return EA::Physics::SizeAndAlignment for KDSubTree array
*/

EA::Physics::SizeAndAlignment
GetKDSubTreeWorkSpaceResourceDescriptor(rw::collision::ClusteredMesh &clusteredMesh)
{
    uint32_t size;
    uint32_t numBranchnodes; 

    //Space for Stack - Space for NodeData - sufficient for number of clusters+maxnumber of nodes containing leaves in a cluster... 
    //                - However, this would require looping through the tree, so for now just using the number of branchnodes in original tree
    numBranchnodes = clusteredMesh.GetKDTreeBase()->GetNumBranchNodes();
    size = (numBranchnodes+1)*sizeof(NodeData);

    //Return EA::Physics::SizeAndAlignment
    return EA::Physics::SizeAndAlignment(size,16);
}

/**
\brief
Creates an array of KDSubTrees from a ClusteredMesh

\param kdSubTreeArrayRes This must be big enough to hold the array of KDSubTrees.
\param workspaceRes This must be big enough to (potentially) hold every leaf node in the KDTree. It can be freed on completion.
\param clusteredMesh ClusteredMesh from which the KDSubTree array is to be generated

\return Pointer to KDSubTree array.
*/
void
CreateKDSubTreeArray(rw::collision::KDSubTree *kdSubTreeArray, const EA::Physics::MemoryPtr &workspaceRes, rw::collision::ClusteredMesh &clusteredMesh)
{           
    //Get number of clusters
    uint32_t numClusters = clusteredMesh.GetNumCluster();

    //Get KDTree
    rw::collision::KDTreeBase *meshKDTree = clusteredMesh.GetKDTreeBase(); 
    EA_ASSERT(meshKDTree);
    if (numClusters == 1)
    {
        // special case for one cluster - just a copy of the whole KD tree
        kdSubTreeArray[0].Initialize(meshKDTree,
            0,
            meshKDTree->m_numBranchNodes,
            meshKDTree->m_numEntries,
            0,
            meshKDTree->m_bbox);
        return;
    }

    //Generate Array of Cluster RootBranchnodes
    NodeData *branchNodeStack = (NodeData*)workspaceRes.GetMemory();
    uint32_t branchNodeStackTop=0;
    rw::collision::KDTree::BranchNode *branchNodes=meshKDTree->m_branchNodes;
    //Loop through branchnodes - 
    for(uint32_t branchNo=0;branchNo<meshKDTree->m_numBranchNodes;branchNo++)
    {
        //if branchnode contains a leafnode 
        bool left, right;
        left = branchNodes[branchNo].m_childRefs[0].m_content!=rwcKDTREE_BRANCH_NODE;
        right = branchNodes[branchNo].m_childRefs[1].m_content!=rwcKDTREE_BRANCH_NODE;
        if(left||right)
        {
            uint32_t unit=0;
            //add to stack.
            if(left&&right)
            {
                clusteredMesh.GetClusterIndexAndUnitFromNode(branchNodes[branchNo].m_childRefs[0].m_index,branchNodeStack[branchNodeStackTop].clusterNo,unit);

                branchNodeStack[branchNodeStackTop].numEntries= branchNodes[branchNo].m_childRefs[0].m_content+
                    branchNodes[branchNo].m_childRefs[1].m_content;
            }  
            else
            {
                if(left)
                {
                    clusteredMesh.GetClusterIndexAndUnitFromNode(branchNodes[branchNo].m_childRefs[0].m_index,branchNodeStack[branchNodeStackTop].clusterNo,unit);
                    branchNodeStack[branchNodeStackTop].numEntries= branchNodes[branchNo].m_childRefs[0].m_content;
                }
                else //right
                {
                    clusteredMesh.GetClusterIndexAndUnitFromNode(branchNodes[branchNo].m_childRefs[1].m_index,branchNodeStack[branchNodeStackTop].clusterNo,unit);
                    branchNodeStack[branchNodeStackTop].numEntries= branchNodes[branchNo].m_childRefs[1].m_content;
                }
            }
            branchNodeStack[branchNodeStackTop].nodeIndex=branchNo;
            branchNodeStack[branchNodeStackTop].lastNode=branchNo;

            //Simplify the stack if a double leaf node added
            bool simplify=branchNodeStackTop>0 && left&&right;

            while(simplify)
            {
                EA_ASSERT(branchNodeStackTop > 0);

                //Look up parent
                uint32_t parentIndex=branchNodes[branchNodeStack[branchNodeStackTop].nodeIndex].m_parent;
                //is  left Child of the parent a branch
                bool parentLeftChildABranch = (branchNodes[parentIndex].m_childRefs[0].m_content==rwcKDTREE_BRANCH_NODE);
                //is left Child of parent the second node on the stack?
                bool parentLeftChildNextOnStack = (branchNodes[parentIndex].m_childRefs[0].m_index==branchNodeStack[branchNodeStackTop-1].nodeIndex);
                //is parent of Child the second node on the stack?
                bool parentNextOnStack = (parentIndex==branchNodeStack[branchNodeStackTop-1].nodeIndex);
                //does second node on the stack belong to the same cluster (or is cluster 0 with zero entries)
                bool secondNodeSameClusterOrZero = (branchNodeStack[branchNodeStackTop-1].clusterNo==branchNodeStack[branchNodeStackTop].clusterNo)||(branchNodeStack[branchNodeStackTop-1].numEntries==0);

                // Two cases when simplification can occur 
                // Case one: If the parents left child is the next item on the stack - and it belongs to the same cluster
                bool caseone = parentLeftChildABranch && parentLeftChildNextOnStack && secondNodeSameClusterOrZero;

                // Case two: If the parent is the next item on the stack, and it belongs to the same cluster.
                bool casetwo = parentNextOnStack && secondNodeSameClusterOrZero;

                if(caseone)
                {
                    branchNodeStack[branchNodeStackTop-1].numEntries+=branchNodeStack[branchNodeStackTop].numEntries;
                    branchNodeStack[branchNodeStackTop-1].nodeIndex=parentIndex;
                    branchNodeStack[branchNodeStackTop-1].lastNode=branchNodeStack[branchNodeStackTop].lastNode;
                    branchNodeStackTop--;
                }

                if(casetwo)
                {
                    branchNodeStack[branchNodeStackTop-1].numEntries+=branchNodeStack[branchNodeStackTop].numEntries;
                    branchNodeStack[branchNodeStackTop-1].lastNode=branchNodeStack[branchNodeStackTop].lastNode;
                    branchNodeStack[branchNodeStackTop-1].clusterNo=branchNodeStack[branchNodeStackTop].clusterNo;
                    branchNodeStackTop--;
                }

                // Further simplifications may be possible...
                simplify = (caseone||casetwo) && (branchNodeStackTop > 0);
            }
            branchNodeStackTop++;

        }
    }

    uint32_t newTop=0;

    if(branchNodeStackTop!=numClusters)
    {
        //Remove Zero Entries
        for(uint32_t i = 0; i<branchNodeStackTop; i++)
        {
            branchNodeStack[newTop]=branchNodeStack[i];
            if(branchNodeStack[newTop].numEntries>0)
            {
                newTop++;
            }
        }
        branchNodeStackTop=newTop;

        EA_ASSERT(branchNodeStackTop==numClusters);
    }
  
    //Create Array of KDSubTrees
    rwpmath::VecFloat compressionGranularity = clusteredMesh.GetVertexCompressionGranularity();
    rw::collision::AABBox clusterBBox;

    for(uint32_t clusterNo=0;clusterNo<numClusters;clusterNo++)
    {
        NodeData &currentNodeData = branchNodeStack[clusterNo];
        
        //GenerateBBox for cluster
        rw::collision::ClusteredMeshCluster &currentCluster = clusteredMesh.GetCluster(currentNodeData.clusterNo);
        clusterBBox.m_min=rwpmath::Vector3(rwpmath::MAX_FLOAT,rwpmath::MAX_FLOAT,rwpmath::MAX_FLOAT);
        clusterBBox.m_max=rwpmath::Vector3(-rwpmath::MAX_FLOAT,-rwpmath::MAX_FLOAT,-rwpmath::MAX_FLOAT);

        for(uint8_t vertexNo=0;vertexNo<currentCluster.vertexCount;vertexNo++)
        {
            clusterBBox.Union(currentCluster.GetVertex(vertexNo,compressionGranularity));
        }
        clusterBBox.m_min-=compressionGranularity;
        clusterBBox.m_max+=compressionGranularity;

        //Initialize KDSubTree
        //Check for oneleafed tree
        if(currentNodeData.lastNode==currentNodeData.nodeIndex)
        {
            uint32_t defaultEntry;
            //Get Default Entry
            if(branchNodes[currentNodeData.nodeIndex].m_childRefs[0].m_content!=rwcKDTREE_BRANCH_NODE)
            {
                //left leaf
                defaultEntry = branchNodes[currentNodeData.nodeIndex].m_childRefs[0].m_index;
            }
            else
            {
                //right leaf
                defaultEntry = branchNodes[currentNodeData.nodeIndex].m_childRefs[1].m_index;
            }
            kdSubTreeArray[currentNodeData.clusterNo].Initialize(meshKDTree,
                    currentNodeData.nodeIndex,
                    0,
                    currentNodeData.numEntries,
                    defaultEntry,
                    clusterBBox);
        }
        else
        {
            
            kdSubTreeArray[currentNodeData.clusterNo].Initialize(meshKDTree,
                currentNodeData.nodeIndex,
                currentNodeData.lastNode-currentNodeData.nodeIndex+1,
                currentNodeData.numEntries,
                0,
                clusterBBox);
        }
    }
}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif
} // namespace collision
} // namespace rw
