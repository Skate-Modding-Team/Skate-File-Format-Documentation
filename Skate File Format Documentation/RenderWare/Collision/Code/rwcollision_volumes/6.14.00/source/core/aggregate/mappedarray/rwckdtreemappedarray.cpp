// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwckdtreeaggregate.cpp

 Purpose: Array of volumes with indexed kdtree spatial map.
 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/kdtree.h"
#include "rw/collision/aggregate.h"
#include "rw/collision/mappedarray.h"
#include "rw/collision/volumelinequery.h"
#include "rw/collision/volumebboxquery.h"

#include "rw/collision/kdtreemappedarray.h"
#include "rw/collision/aalineclipper.h"

#include <rw/collision/detail/fpu/kdtreemappedarray.h>

using namespace rwpmath;

// ***********************************************************************************************************
// Defines

/**
\internal
*/
#define rwcKDTREEMAPPEDARRAYALIGNMENT rwcVOLUMEALIGNMENT


namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Enums + Consts


// ***********************************************************************************************************
// Structs + Unions + Classes

/**
\internal
\brief
 */
struct DTreeValidityCheckNodeData
{
    uint32_t    parent;
    AABBox      bb;
};

// ***********************************************************************************************************
// Static Variables


// ***********************************************************************************************************
// Static Functions


// ***********************************************************************************************************
// External Functions

// ***********************************************************************************************************
//                                            KDTreeMappedArray Definition
// ***********************************************************************************************************


// Static Data Member Definitions

/**
\internal
\brief The initialisation of the static member variable that holds the functions pointers.
*/
rw::collision::Aggregate::VTable KDTreeMappedArray::sm_vTable =
{
    RWCOBJECTTYPE_KDTREEMAPPEDARRAY,
    (GetSizeFn)(&KDTreeMappedArray::GetSizeThis),
    rwcKDTREEMAPPEDARRAYALIGNMENT,
    FALSE,
    (UpdateFn)(&KDTreeMappedArray::UpdateThis),
    (LineIntersectionQueryFn)(&KDTreeMappedArray::LineIntersectionQueryThis),
    (BBoxOverlapQueryFn)(&KDTreeMappedArray::BBoxOverlapQueryThis),
    (GetNextVolumeFn)(&MappedArray::GetNextVolumeThis),
    (ClearAllProcessedFlagsFn)(&MappedArray::ClearAllProcessedFlags),
    (ApplyUniformScaleFn)(&KDTreeMappedArray::ApplyUniformScale)
};

// Static Methods

// Construction & Destruction

/**
\internal
\brief Constructor for an rw::collision::KDTreeMappedArray.

\param numVols The number of volumes in this KDTreeMappedArray.
\param vTable Pointer to the function table.
\param classSize Size of the actual class (might be derived). Data buffers are placed
after this in memory.
*/
KDTreeMappedArray::KDTreeMappedArray(uint32_t numVols,
                                 VTable *vTable,
                                 uint32_t classSize)
                                 :  MappedArray(numVols, vTable)
{
    uintptr_t addr = (uintptr_t)(this);

    //Class structure
    addr += classSize;

    //Set the pointer to the data for the volumes
    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcVOLUMEALIGNMENT);
    m_volumes = (Volume *)addr;
    addr += numVols*sizeof(Volume);

    //set the ptr to the spatial map 
    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcKDTREE_ALIGNMENT);
    m_map = (KDTree *)addr;

}

/**
\brief Get the resource requirements of the KDTreeMappedArray.

\param numVols The number of volumes in this KDTreeMappedArray.
\param numNodes The number of nodes in the indexed KDTree map.
\param bbox Bounding box specifying the spatial extent of the volumes in the aggregate.
\param vTable
\param classSize
\return The new KDTreeMappedArray.
*/
EA::Physics::SizeAndAlignment
KDTreeMappedArray::GetResourceDescriptor(uint32_t numVols, uint32_t numNodes,
                                         const rw::collision::AABBox & /*bbox*/,
                                         const VTable * /*vTable*/,
                                         uint32_t /*classSize*/)
{
    uint32_t size = 0;

    // Class data
    size = sizeof(KDTreeMappedArray);

    // Volume array
    size = EA::Physics::SizeAlign<uint32_t>(size, rwcVOLUMEALIGNMENT);
    size += numVols*sizeof(Volume);

    // Spatialmap
    EA::Physics::SizeAndAlignment kdTreeResDesc = KDTree::GetResourceDescriptor(numNodes, 0, rw::collision::AABBox());
    size = EA::Physics::SizeAlign(size, kdTreeResDesc.GetAlignment());
    size += kdTreeResDesc.GetSize();

    return EA::Physics::SizeAndAlignment(size, rwcKDTREEMAPPEDARRAYALIGNMENT);
}

/**
\brief Initializes a KDTreeMappedArray from a EA::Physics::MemoryPtr.

\param resource The EA::Physics::MemoryPtr the object is initialized into.
\param numVols The number of volumes in this KDTreeMappedArray.
\param numNodes The number of nodes in the indexed KDTree map.
\param bbox Bounding box specifying the spatial extent of the volumes in the aggregate.
\param vTable Pointer to the function table.
\param classSize
\return The new KDTreeMappedArray.
*/
KDTreeMappedArray *
KDTreeMappedArray::Initialize(const EA::Physics::MemoryPtr& resource,
                              uint32_t numVols,
                              uint32_t numNodes,
                              const AABBox  &bbox,
                              VTable *vTable,
                              uint32_t classSize)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcKDTREEMAPPEDARRAYALIGNMENT);

    KDTreeMappedArray *agg = new (resource.GetMemory()) KDTreeMappedArray(numVols, vTable, classSize);
    KDTree::Initialize(agg->m_map, numNodes, numVols, bbox);

    return agg;
}

/**
\deprecated This is deprecated functionality
\brief Initializes a block of memory to be a KDTreeMappedArray.
\param ptr Pointer to the block of memory that this KDTreeMappedArray will occupy.
\param numVols The number of volumes in this KDTreeMappedArray.
\param numNodes The number of nodes in the indexed KDTree map.
\param bbox Bounding box specifying the spatial extent of the volumes in the aggregate.
\param vTable Pointer to the function table.
\return The new KDTreeMappedArray.
*/
KDTreeMappedArray *
KDTreeMappedArray::Initialize(void *ptr,
                           uint32_t numVols,
                           uint32_t numNodes,
                           const AABBox  &bbox,
                           VTable *vTable,
                           uint32_t classSize)
{
    rwcDEPRECATED("KDTreeMappedArray::Initialize(void*, ...) is deprecated. KDTreeMappedArray::Initialize(EA::Physics::MemoryPtr, ...) should be used instead.");
    return Initialize(EA::Physics::MemoryPtr(ptr), numVols, numNodes, bbox, vTable, classSize);
}


/**
\brief Releases a block of memory that was being used for a KDTreeMappedArray.
*/
void
KDTreeMappedArray::Release()
{
    m_map->Release();

}


// Interface Implementations

// Methods

// Virtual Functions

/**
\internal
\brief Internal vTable function.
*/
void 
KDTreeMappedArray::UpdateThis(void)
{
    //update the overall bounding box
    for(uint32_t i = 0; i < m_numVolumes; i++)
    {
        if(i == 0)
        {
            m_volumes[i].GetBBox(NULL,0,m_AABB);
        }
        else
        {
            AABBox bbox;
            m_volumes[i].GetBBox(NULL,0,bbox);
            m_AABB = Union(m_AABB, bbox);
        }
    }

}

// Disable "assignment within conditional" warning, as it is used intentionally below.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4706)
#endif

/**
\internal
\brief Internal vTable function. See  rw::collision::Aggregate::LineIntersectionQuery.
\param lineQuery
\param tm
\return
*/
RwpBool
KDTreeMappedArray::LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                                           const Matrix44Affine *tm)
{
    KDTree::LineQuery *mapQuery = (KDTree::LineQuery *)lineQuery->m_curSpatialMapQuery;

    // See whether to start a new query
    if (!mapQuery)
    {
        // Map line into spatial map space
        Matrix44Affine invTm(*tm);
        invTm= InverseOfMatrixWithOrthonormal3x3(invTm);
        Vector3 localLineStart = TransformPoint(lineQuery->m_pt1, invTm);
        Vector3 localLineEnd   = TransformPoint(lineQuery->m_pt2, invTm);

        mapQuery = new (lineQuery->m_spatialMapQueryMem) 
            KDTree::LineQuery(m_map, localLineStart, localLineEnd, lineQuery->m_fatness);

        lineQuery->m_curSpatialMapQuery = (void*)mapQuery;
    }

    // Far clip val might have been set by another volume hit
    if (lineQuery->m_resultsSet != VolumeLineQuery::ALLLINEINTERSECTIONS)
    {
        mapQuery->ClipEnd(lineQuery->m_endClipVal);
    }

    // As long as there's buffer space, find next potential volume instersections. 
    uint32_t index = rwcKDTREE_INVALID_INDEX;
    while ( (lineQuery->m_resCount < lineQuery->m_resMax) && mapQuery->GetNext(index))
    {
        const Volume& vol = m_volumes[index];
        AABBox bbox;

        // Get the BBox of the Volume in KD-tree space
        vol.GetBBox(0, FALSE, bbox);
        float pa = 0.0f;
        float pb = lineQuery->m_endClipVal;

        // Check that the line intersects the Volume's BBox
        if (mapQuery->m_lineClipper.ClipToAABBox(pa, pb, bbox))
        {
            // Get the new tag for the child at this level
            uint32_t tag = lineQuery->m_tag;
            uint32_t numTagBits = lineQuery->m_numTagBits;
            UpdateTagWithChildIndex(tag, numTagBits, index);

            // Add volume to the stack
            if (!lineQuery->AddVolumeRef(&vol, tm, tag, static_cast<uint8_t>(numTagBits)))
            {
                return FALSE; // Either primitive or Stack buffer runs out of space
            }
        }
    }

    // Return false if we failed to complete query due to lack of buffer space (will resume later).
    return static_cast<RwpBool>(lineQuery->m_resCount < lineQuery->m_resMax);
}

/**
\internal
\brief
See  rw::collision::Aggregate::BBoxOverlapQuery.
*/
RwpBool
KDTreeMappedArray::BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                                      const Matrix44Affine *tm)
{
    KDTree::BBoxQuery *mapQuery = (KDTree::BBoxQuery *)bboxQuery->m_curSpatialMapQuery;

    //See whether to start a new query
    if (!mapQuery)
    {
        AABBox tempbb, *localBBox;

        //map bb into spatial map space
        if (tm)
        {
            Matrix44Affine invTm(*tm);
            invTm= InverseOfMatrixWithOrthonormal3x3(invTm);
            tempbb = bboxQuery->m_aabb.Transform(&invTm);
            localBBox = &tempbb;
        }
        else
        {
            localBBox = &bboxQuery->m_aabb;
        }

        // Initialize query
        mapQuery = new (bboxQuery->m_spatialMapQueryMem) KDTree::BBoxQuery(m_map, *localBBox);
        bboxQuery->m_curSpatialMapQuery = (void *)mapQuery;
    }

    uint32_t index = rwcKDTREE_INVALID_INDEX;
    while (bboxQuery->m_primNext < bboxQuery->m_primBufferSize &&
           bboxQuery->m_stackNext < bboxQuery->m_stackMax)
    {
        if (FALSE == mapQuery->GetNext(index))
        {
            // No more entries in the kdtree to process so return we're complete
            return TRUE;
        }

        Volume *volume = &m_volumes[index];

        // See if the child volume is enabled
        if (volume->IsEnabled())
        {
            //Get the child volume bounding box
            AABBox bb;
            volume->GetBBox(tm, 0, bb);

            //If input bb overlaps the childvol bb then process
            if (bboxQuery->m_aabb.Overlaps(bb))
            {
                //Get the new tag for the child at this level
                uint32_t tag = bboxQuery->m_tag;
                uint32_t numTagBits = bboxQuery->m_numTagBits;
                UpdateTagWithChildIndex(tag, numTagBits, index);

                //Add volume to the stack
                if (!bboxQuery->AddVolumeRef(volume, tm, bb, tag, static_cast<uint8_t>(numTagBits)))
                {
                    return FALSE; // Either primitive or Stack buffer runs out of space
                }
            }
        }
    }

    // Flag the bboxQuery to say why we're returning before completion. Note that because we're unable to
    // detect what the next entry in the kdtree is (primitive or aggregate) and we can't rewind a kdtree query
    // we have to early out as soon as we run out of space in either of the buffers
    RwpBool outOfPrimitiveSpace = static_cast<RwpBool>(bboxQuery->m_primNext >= bboxQuery->m_primBufferSize);
    RwpBool outOfStackSpace = static_cast<RwpBool>(bboxQuery->m_stackNext >= bboxQuery->m_stackMax);
    if (outOfPrimitiveSpace)
    {
        bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE);
    }
    if (outOfStackSpace)
    {
        bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE);
    }

    // Return false to say we failed to complete query due to lack of buffer space (will resume later).
    return FALSE;
}



/**
\brief Applies uniform scaling to all volumes in aggregate and KDtree
\param scale uniform scale value to apply to volume
\param useProcessedFlags default false ignores volume processed flag
*/
void
KDTreeMappedArray::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    EA_ASSERT(scale > 0.0f);

    if (!useProcessedFlags || !(m_flags & AGGREGATEFLAG_ISPROCESSED))
    {
        // Scale child volumes
        MappedArray::ApplyUniformScale(scale, useProcessedFlags);

        // Scale KDTree
        for(uint32_t i=0; i< m_map->GetNumBranchNodes(); i++)
        {
            m_map->m_branchNodes[i].m_extents[0] *= scale;
            m_map->m_branchNodes[i].m_extents[1] *= scale;
        }

        m_map->m_bbox.m_min *= scale;
        m_map->m_bbox.m_max *= scale;
        Update();

        if (useProcessedFlags)
        {
            SetProcessedFlag();
        }
    }
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif

/**
\internal
\brief
Internal vTable function. See  rw::collision::Aggregate::GetSize
*/
uint32_t
KDTreeMappedArray::GetSizeThis() const
{
    return KDTreeMappedArray::GetResourceDescriptor(m_numVolumes,
                                                    m_map->GetNumBranchNodes(),
                                                    m_AABB).GetSize();
}

/**
\brief Check validity of KDTreeMappedArray. Only available in debug library.
\return TRUE if object is internally consistent.
 */
RwpBool
KDTreeMappedArray::IsValid() const
{
    RwpBool isValid = TRUE;

    // Check KDTree
    if (!m_map->IsValid())
    {
        return FALSE;
    }

    // Now check volumes
    DTreeValidityCheckNodeData curData;
    curData.bb = m_map->GetBBox();
    curData.parent = 0;

    KDTree::Traversal<DTreeValidityCheckNodeData> traversal(m_map, curData);
    while (traversal.PopNode(curData))
    {
        if (traversal.CurrentNodeIsBranch())
        {
            uint32_t branchIndex = traversal.GetBranchIndex();
            KDTree::BranchNode &branch = m_map->m_branchNodes[branchIndex];

            DTreeValidityCheckNodeData childData;
            childData.parent = branchIndex;

            // Push right
            childData.bb = curData.bb;
            childData.bb.m_min.SetComponent((uint16_t)branch.m_axis, branch.m_extents[1]);
            traversal.PushChildNode(1, childData);

            // Push left
            childData.bb = curData.bb;
            childData.bb.m_max.SetComponent((uint16_t)branch.m_axis, branch.m_extents[0]);
            traversal.PushChildNode(0, childData);
        }
        else
        {
            uint32_t first, count;
            traversal.GetLeafNodeEntries(first, count);

            // For all volumes in leaf
            for (uint32_t i = first; i < (first + count); i++)
            {
                AABBox bb;
                GetVolumeArray()[i].GetBBox(NULL, FALSE, bb);

                // Check that leaf node bbox contains volume
                if (!curData.bb.Contains(bb))
                {
                    EAPHYSICS_MESSAGE("Volume %d outside of leaf bounding box (internal node %d).",
                        i, curData.parent);
                    isValid = FALSE;
                }
            }
        }
    }

    return isValid;
}

// Helper Member Functions


rw::collision::Aggregate::VTable rw::collision::detail::fpu::KDTreeMappedArray::sm_vTable;


} // namespace collision
} // namespace rw



