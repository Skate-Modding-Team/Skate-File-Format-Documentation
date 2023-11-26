// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcsimplemappedarray.cpp

 Purpose: Simple array of volumes (no spatial map).
 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/aggregate.h"
#include "rw/collision/volumelinequery.h"
#include "rw/collision/volumebboxquery.h"
#include "rw/collision/mappedarray.h"
#include "rw/collision/simplemappedarray.h"
#include "rw/collision/aalineclipper.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{


// *******************************************************************************************************
// Forward Declarations

// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts


// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions

/**
\internal

The initialisation of the static member variable that holds the functions pointers.
 */
rw::collision::Aggregate::VTable SimpleMappedArray::sm_vTable =
{
    RWCOBJECTTYPE_SIMPLEMAPPEDARRAY,
    (GetSizeFn)(&SimpleMappedArray::GetSizeThis),
    rwcSIMPLEMAPPEDARRAYALIGNMENT,
    FALSE,
    (UpdateFn)(&SimpleMappedArray::UpdateThis),
    (LineIntersectionQueryFn)(&SimpleMappedArray::LineIntersectionQueryThis),
    (BBoxOverlapQueryFn)(&SimpleMappedArray::BBoxOverlapQueryThis),
    (GetNextVolumeFn)(&MappedArray::GetNextVolumeThis),
    (ClearAllProcessedFlagsFn)(&MappedArray::ClearAllProcessedFlags),
    (ApplyUniformScaleFn)(&MappedArray::ApplyUniformScale)
};

// ***********************************************************************************************************
// Structs + Unions + Classes

// ***********************************************************************************************************
// Static Functions

// ***********************************************************************************************************
// External Functions

// ***********************************************************************************************************
// Class Member Functions

/**
\internal

\brief Constructor for an rw::collision::SimpleMappedArray. User code should use SimpleMappedArray::Initialize.
\param numVols The number of volumes in this SimpleMappedArray.
\param vTable Pointer to the function table.
\param classSize Size of the actual class (might be derived). Data buffers are placed
after this in memory.
 */
SimpleMappedArray::SimpleMappedArray(uint32_t numVols,
                                 VTable *vTable,
                                 uint32_t classSize)
                                 :   MappedArray(numVols, vTable)
{
    uintptr_t addr = (uintptr_t)this;

    // Class structure
    addr += classSize;

    // Set the pointer to the data for the vertices
    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcVOLUMEALIGNMENT);
    m_volumes = (Volume *)addr;

}

/**
\brief Get the resource requirements for the SimpleMappedArray.
\param numVols The number of volumes in this SimpleMappedArray.
\return The EA::Physics::SizeAndAlignment
 */
EA::Physics::SizeAndAlignment
SimpleMappedArray::GetResourceDescriptor(uint32_t numVols,
                                         const VTable * /*vTable*/,
                                         uint32_t /*classSize*/)
{
    uint32_t size = 0;
    size = EA::Physics::SizeAlign<uint32_t>(sizeof(SimpleMappedArray), rwcSIMPLEMAPPEDARRAYALIGNMENT);
    size += numVols*sizeof(Volume);
    return EA::Physics::SizeAndAlignment(size, rwcSIMPLEMAPPEDARRAYALIGNMENT);
}

/**
\brief Initializes a block of memory to be a SimpleMappedArray.
\param resource The memory resource that this SimpleMappedArray will occupy.
\param numVols The number of volumes in this SimpleMappedArray.
\param vTable (Optional) Pointer to the function table. Use in derived classes.
\param classSize (Optional) Size of the actual derived class. Data buffers are placed
after this in memory.
\return The new SimpleMappedArray.
 */
SimpleMappedArray *
SimpleMappedArray::Initialize(const EA::Physics::MemoryPtr& resource,
                              uint32_t numVols,
                              VTable *vTable,
                              uint32_t classSize)
{
    SimpleMappedArray *agg = new (resource.GetMemory()) SimpleMappedArray(numVols, vTable, classSize);

    return agg;
}

/**
\brief Releases a block of memory that was being used for a SimpleMappedArray.
 */
void
SimpleMappedArray::Release()
{
}


/**
\internal

\brief Internal vTable function.
*/
void
SimpleMappedArray::UpdateThis(void)
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


/**
\internal

\brief Internal vTable function.
\see rw::collision::Aggregate::LineIntersectionQuery.
 */
RwpBool
SimpleMappedArray::LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                                           const Matrix44Affine *tm)
{
    Volume *childVol;
    uint32_t tag;
    uint32_t numTagBits;

    AALineClipper *lineClipper = (AALineClipper *)lineQuery->m_curSpatialMapQuery;

    // See whether to start a new query
    if (!lineClipper)
    {
        // Map line into spatial map space
        Matrix44Affine invTm(*tm);
        invTm = InverseOfMatrixWithOrthonormal3x3(invTm);
        Vector3 localLineStart = TransformPoint(lineQuery->m_pt1, invTm);
        Vector3 localLineEnd   = TransformPoint(lineQuery->m_pt2, invTm);
        float fatness = lineQuery->m_fatness;

        lineClipper = new (lineQuery->m_spatialMapQueryMem) 
            AALineClipper(localLineStart, localLineEnd, rwpmath::Vector3(fatness, fatness, fatness), GetBBox());

        lineQuery->m_curSpatialMapQuery = (void*)lineClipper;
    }

    //Add elements to primitive array
    uint32_t aggIndex = lineQuery->m_aggIndex;
    while(aggIndex < GetVolumeCount())
    {
        //Add aggregate components to stack
        childVol = GetVolume(static_cast<uint16_t>(aggIndex));

        AABBox bbox;

        // Get the BBox of the Volume in KD-tree space
        childVol->GetBBox(0, FALSE, bbox);
        float pa = 0.0f;
        float pb = lineQuery->m_endClipVal;

        // Check that the line intersects the Volume's BBox
        if (lineClipper->ClipToAABBox(pa, pb, bbox))
        {
            //Get the new tag for the child at this level
            tag = lineQuery->m_tag;
            numTagBits = lineQuery->m_numTagBits;
            UpdateTagWithChildIndex(tag, numTagBits, aggIndex);

            //If the aggregate element is another container then add it
            //to the stack, otherwise process like individual vols
            if(childVol->GetType() == rw::collision::VOLUMETYPEAGGREGATE)
            {
                if (!lineQuery->AddVolumeRef(childVol, tm, tag, static_cast<uint8_t>(numTagBits)))
                {
                    lineQuery->m_aggIndex = aggIndex; // remember where we got to
                    return FALSE; // Either primitive or Stack buffer runs out of space
                }
            }
            else if(!lineQuery->AddPrimitiveRef(childVol, tm, tag, static_cast<uint8_t>(numTagBits)))
            {
                lineQuery->m_aggIndex = aggIndex; // remember where we got to
                //we couldn't add this vol try processing some primitives
                return FALSE;
            }
        }
        aggIndex++;
    }

    //We are finished
    return TRUE;
}


/**
\internal

\brief
\see rw::collision::Aggregate::BBoxOverlapQuery.
 */
RwpBool
SimpleMappedArray::BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                                      const Matrix44Affine *tm)
{
    //Add elements to primitive array
    while (bboxQuery->m_aggIndex < GetVolumeCount())
    {
        //Add aggregate components to stack
        Volume *childVol = GetVolume(static_cast<uint16_t>(bboxQuery->m_aggIndex));

        // See if the child volume is enabled
        if (childVol->IsEnabled())
        {
            //Get the child volume bounding box
            AABBox bb;
            childVol->GetBBox(tm, 0, bb);

            //If input bb overlaps the childvol bb then process
            if (bboxQuery->m_aabb.Overlaps(bb))
            {
                //Get the new tag for the child at this level
                uint32_t tag = bboxQuery->m_tag;
                uint32_t numTagBits = bboxQuery->m_numTagBits;
                UpdateTagWithChildIndex(tag, numTagBits, bboxQuery->m_aggIndex);

                //If the aggregate element is another container then add it
                //to the stack, otherwise process like individual vols
                if (childVol->GetType() == rw::collision::VOLUMETYPEAGGREGATE)
                {
                    if(!bboxQuery->AddVolumeRef(childVol, tm, bb, tag, static_cast<uint8_t>(numTagBits)))
                    {
                        //Can't fit aggregate on stack or Primitive buffer. Allow for the potential to resume later or
                        //assert on out of stack space.
                        return FALSE;
                    }
                }
                else if(!bboxQuery->AddPrimitiveRef(childVol, tm, bb, tag, static_cast<uint8_t>(numTagBits)))
                {
                    //Can't fit primitive in buffer. Resume later.
                    return FALSE;
                }
            }
        }

        bboxQuery->m_aggIndex++;
    }

    //We are finished
    return TRUE;
}


/**
\internal

\brief Internal vTable function.
\see rw::collision::Aggregate::GetSize.
 */
uint32_t
SimpleMappedArray::GetSizeThis() const
{
    return SimpleMappedArray::GetResourceDescriptor(m_numVolumes).GetSize();
}


} // namespace collision
} // namespace rw
