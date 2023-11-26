// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcvolumelinequery.cpp

 Purpose:
 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/aggregate.h"
#include "rw/collision/volumebboxquery.h"

#include "rw/collision/primitivepairquery.h"
#include "rw/collision/metrics.h"
#include "rw/collision/volumevolumequery.h"

#include "rw/collision/aggregatevolume.h"
#include "rw/collision/procedural.h"

using namespace rwpmath;


namespace rw
{
namespace collision
{

#define rwcVOLUMEVOLUMEALIGNMENT RWMATH_VECTOR3_ALIGNMENT

/**
\brief In place constructor.

\param stackSize The max number of entries on the internal bbox query stack. The stack needs to be large
       enough to handle the processing of aggregate volumes . It
       never needs to be larger than the total number of aggregate volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some intersections
       may be lost.

\param resBufferSize The max size of the results arrays. There are 2 internal buffers
       for the bbox overlaps and the primitive intersections.

\note  VolumeVolumeQuery objects should be created using  VolumeVolumeQuery::Initialize and
       not constructed directly
 */
VolumeVolumeQuery::VolumeVolumeQuery(uint32_t stackSize,
                                     uint32_t resBufferSize)
{
    uint32_t size;

    m_volRefPairBufferSize = resBufferSize;

    // container volume stack
    size = EA::Physics::SizeAlign<uint32_t>(sizeof(VolumeVolumeQuery), rwcVOLUMEVOLUMEALIGNMENT);
    m_bBoxQueryAtoB = (VolumeBBoxQuery *)((uintptr_t)(this) + size);

    // instanced volume buffer
    size = VolumeBBoxQuery::GetResourceDescriptor(stackSize, resBufferSize).GetSize();
    m_bBoxQueryBtoA = (VolumeBBoxQuery *)((uintptr_t)(m_bBoxQueryAtoB) + size);

    // overlaps buffer
    m_volRefPairBuffer = (VolRefPair *)(((uintptr_t)(m_bBoxQueryBtoA) + size));

    // intersections buffer
    size = sizeof(VolRefPair) * m_volRefPairBufferSize;
    m_intersectionBuffer = (PrimitivePairIntersectResult *)(((uintptr_t)(m_volRefPairBuffer) + size));
    m_intersectionBufferMaxSize = (int32_t)m_volRefPairBufferSize;

    size = sizeof(PrimitivePairIntersectResult) * resBufferSize;
    m_instancingSPR = (GPInstance*)( (uintptr_t)m_intersectionBuffer + size );

    m_cullTable = 0;

    m_edgeCosBendNormalThreshold = -1.0f;
    m_convexityEpsilon = 0.0f;

}

/**
\brief Get the EA::Physics::MemoryPtr requirements for initialising this volume intersection query.

\param stackSize The max number of entries on the internal bbox query stack. The stack needs to be large
       enough to handle the processing of aggregate volumes. It
       never needs to be larger than the total number of aggregate volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some intersections
       may be lost.

\param resBufferSize The max size of the results arrays. There are 2 internal buffers
       for the bbox overlaps and the primitive intersections.

\return The EA::Physics::SizeAndAlignment
 */
EA::Physics::SizeAndAlignment
VolumeVolumeQuery::GetResourceDescriptor(uint32_t stackSize, uint32_t resBufferSize)
{
    uint32_t size = 0;

    size += EA::Physics::SizeAlign<uint32_t>(sizeof(VolumeVolumeQuery), rwcVOLUMEVOLUMEALIGNMENT);

    //2 Internal bbox queries
    size += 2 * VolumeBBoxQuery::GetResourceDescriptor(stackSize, resBufferSize).GetSize();

    size += sizeof(VolRefPair) * resBufferSize;

    size += sizeof(PrimitivePairIntersectResult) * resBufferSize;

    size += sizeof(GPInstance) * (resBufferSize + 1); //Additional element to account for first entry being special
                                                      //See PrimitiveBatchIntersect(..)

    return EA::Physics::SizeAndAlignment(size, rwcVOLUMEVOLUMEALIGNMENT);
}


/**
\brief Initialize a EA::Physics::MemoryPtr as a VolumeVolumeQuery.

\param resource The EA::Physics::MemoryPtr the VolumeVolumeQuery is initialized into.

\param stackSize The max number of entries on the internal bbox query stack. The stack needs to be large
       enough to handle the processing of aggregate volumes. It
       never needs to be larger than the total number of aggregate volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some intersections
       may be lost.

\param resBufferSize The max size of the results arrays. There are 2 internal buffers
       for the bbox overlaps and the primitive intersections.

\return The initialized VolumeVolumeQuery.
 */
VolumeVolumeQuery *
VolumeVolumeQuery::Initialize(const EA::Physics::MemoryPtr& resource,
                              uint32_t stackSize,
                              uint32_t resBufferSize)
{
    VolumeVolumeQuery *pVVQ = new (resource.GetMemory()) VolumeVolumeQuery(stackSize, resBufferSize);

    //Initialize 2 internal bbox queries
    //TODO - bbox queries may be able to share some buffers like the stack etc.
    VolumeBBoxQuery::Initialize(EA::Physics::MemoryPtr(pVVQ->m_bBoxQueryAtoB), stackSize, resBufferSize);
    VolumeBBoxQuery::Initialize(EA::Physics::MemoryPtr(pVVQ->m_bBoxQueryBtoA), stackSize, resBufferSize);

    return pVVQ;
}


// ***********************************************************************************************************
// External Functions


// ***********************************************************************************************************
// Class Member Functions

/**
\brief Gets number of primitive pair bounding box overlaps.

Queries the stored volume against the input volumes and returns the set of primitive
pairs which have overlapping bounding boxes. Use  VolumeVolumeQuery::GetOverlapResultsBuffer()
to get the array of rw::collision::VolRefPairs. GetPrimitiveBBoxOverlaps() will exit if all the
overlaps have been added to the results buffer or
the results buffer overflows. Overlaps may be missed if any of the buffers in the internal
bounding box queries overflow \see rw::collision::VolumeBBoxQuery.

\see VolumeVolumeQuery::GetNumOverlaps().

\return The number of primitive pair bounding box overlaps.
 */
uint32_t
VolumeVolumeQuery::GetPrimitiveBBoxOverlaps()
{
    AABBox queryVolBBox,inputVolBBox, *smallBBox, bigBBox;
    uint32_t i,j, numResSmallToBig, numResBigToSmall;
    VolRef *resSmallToBig, *resBigToSmall;
    const Volume *bigVolPtr, *smallVolPtr;
    const Matrix44Affine *bigVolMtxPtr, *smallVolMtxPtr;
    float queryVolVolume, inputVolVolume;

    Vector3 paddingVector( m_padding, m_padding, m_padding );

    //Get the bounding box of the query Volume
    m_queryVol->GetBBox(m_queryMtx, 0, queryVolBBox);
    queryVolVolume = (static_cast<float>(queryVolBBox.m_max.GetX()) - static_cast<float>(queryVolBBox.m_min.GetX())) *
                     (static_cast<float>(queryVolBBox.m_max.GetY()) - static_cast<float>(queryVolBBox.m_min.GetY())) *
                     (static_cast<float>(queryVolBBox.m_max.GetZ()) - static_cast<float>(queryVolBBox.m_min.GetZ()));

    queryVolBBox.m_min -= paddingVector;
    queryVolBBox.m_max += paddingVector;

    //Get the result buffers
    resSmallToBig = m_bBoxQueryAtoB->GetOverlapResultsBuffer();
    resBigToSmall = m_bBoxQueryBtoA->GetOverlapResultsBuffer();

    //Initialize the first bbox query
    size_t bufferBytesAvailable = m_volRefPairBufferSize * sizeof(VolRefPair);

    m_volRefPairCount = 0;
    m_volRef1xNCount = 0;

    // This cast is bad however because this code is approaching the end of its life it isn't going to be fixed
    // Avoid ghs compiler warning by casting via uintptr_t
    VolRef1xN *curRef1xN = (VolRef1xN*)(uintptr_t)m_volRefPairBuffer;

    while(bufferBytesAvailable >= sizeof(VolRef1xN) && m_currInput < m_numInputs)
    {
        //Start by assuming the query volume will be larger
        bigVolPtr       = m_queryVol;
        bigVolMtxPtr    = m_queryMtx;
        smallVolPtr     = (const Volume *)m_inputVols[m_currInput];
        smallBBox       = &inputVolBBox;

        //use input m_at array if we've been give one
        if(m_inputMats)
        {
            smallVolMtxPtr = m_inputMats[m_currInput];
        }
        else
        {
            smallVolMtxPtr = NULL;
        }

        smallVolPtr->GetBBox(smallVolMtxPtr, 0, inputVolBBox);
        inputVolVolume = (static_cast<float>(inputVolBBox.m_max.GetX()) - static_cast<float>(inputVolBBox.m_min.GetX())) *
                         (static_cast<float>(inputVolBBox.m_max.GetY()) - static_cast<float>(inputVolBBox.m_min.GetY())) *
                         (static_cast<float>(inputVolBBox.m_max.GetZ()) - static_cast<float>(inputVolBBox.m_min.GetZ()));

        inputVolBBox.m_min -= paddingVector;
        inputVolBBox.m_max += paddingVector;

        RwpBool volumesGotSwapped = FALSE;

        //Always query the smaller BBox first so swap if query is not the larger
        if(queryVolVolume < inputVolVolume)
        {
            bigVolPtr         = smallVolPtr;
            bigVolMtxPtr      = smallVolMtxPtr;
            smallVolPtr       = m_queryVol;
            smallVolMtxPtr    = m_queryMtx;
            smallBBox         = &queryVolBBox;
            volumesGotSwapped = TRUE;
        }

        m_bBoxQueryAtoB->InitQuery(&bigVolPtr, bigVolMtxPtr==NULL ? NULL : &bigVolMtxPtr, 1, *smallBBox);

        //Get all the overlaps
        numResSmallToBig = m_bBoxQueryAtoB->GetOverlaps();

#ifdef EA_DEBUG
        //Issue warning if query didn't finish  - Currently we don't try to re-enter
        if(!m_bBoxQueryAtoB->Finished())
        {
            EAPHYSICS_MESSAGE("In VolumeVolumeQuery, BBox Overlaps didn't finish due to buffer size being too small.");
        }
#endif

        //Continue if there are any overlaps
        if(numResSmallToBig)
        {
            //Build the bbox of the results
            bigBBox = resSmallToBig[0].bBox;
            for(i=1; i<numResSmallToBig; i++)
            {
                bigBBox = Union(bigBBox, resSmallToBig[i].bBox);
            }

            bigBBox.m_min -= paddingVector;
            bigBBox.m_max += paddingVector;

            //initialize the second bbox test
            m_bBoxQueryBtoA->InitQuery(&smallVolPtr,
                                       &smallVolMtxPtr,
                                       1,
                                       bigBBox);

            //Get Second set of overlaps
            numResBigToSmall = m_bBoxQueryBtoA->GetOverlaps();

#ifdef EA_DEBUG
            //Issue warning if query didn't finish  - Currently we don't try to re-enter
            if(!m_bBoxQueryAtoB->Finished())
            {
                EAPHYSICS_MESSAGE("In VolumeVolumeQuery, BBox Overlaps didn't finish due to buffer size being too small.");
            }
#endif

            j=0;

            while(j<numResBigToSmall && bufferBytesAvailable >= sizeof(VolRef1xN))
            {
                AABBox bb1 = resBigToSmall[j].bBox;

                bb1.m_min -= paddingVector;
                bb1.m_max += paddingVector;

                // An entry without and refs is never added to the list, so it's ok to to assume some things here...
                curRef1xN->vRefsNCount = 0;
                curRef1xN->volumesSwapped = volumesGotSwapped;
                curRef1xN->vRef1 = &resBigToSmall[j];

                // The base size for this entry. I know that this isn't the size of the struct!
                bufferBytesAvailable -= (sizeof(VolRef1xN) - sizeof(VolRef*));

                i=0;
                while(i<numResSmallToBig && bufferBytesAvailable >= sizeof(VolRef*))
                {
                    if(!m_cullTable ||
                    !m_cullTable->GetBit(resBigToSmall[j].volume->GetGroup(),
                                            resSmallToBig[i].volume->GetGroup()))
                    {
                        if(resSmallToBig[i].bBox.Overlaps(bb1))
                        {
                            rwcMETRICS(m_metrics.m_gpProbes++);

                            curRef1xN->vRefsN[curRef1xN->vRefsNCount] = &resSmallToBig[i];
                            curRef1xN->vRefsNCount++;
                            m_volRefPairCount++;
                            bufferBytesAvailable -= sizeof(VolRef*);
                        }
                    }
                    i++;
                }

                if ( curRef1xN->vRefsNCount > 0 )
                {
                    curRef1xN = (VolRef1xN*)( (uintptr_t)curRef1xN +
                                                sizeof(VolRef1xN) +
                                                sizeof(VolRef*) * (curRef1xN->vRefsNCount-1) );
                    m_volRef1xNCount++;
                }
                else
                {
                    // Didn't work out, put the size back on!
                    bufferBytesAvailable += (sizeof(VolRef1xN) - sizeof(VolRef*));
                }

                j++;
            }
        }
        m_currInput++;
    }

    return m_volRefPairCount;
}

/**
\brief Gets the number of primitive pair intersections.

Queries the stored volume against the input volumes and returns the individual primitive
intersections in the results buffer \see VolumeVolumeQuery::GetIntersectionResultsBuffer().
GetPrimitiveIntersections() will exit if all the intersections have been added to the results buffer or
the results buffer overflows. The function first calls
 VolumeVolumeQuery::GetPrimitiveBBoxOverlaps() and the intermediate results from this are available
using  VolumeVolumeQuery::GetOverlapResultsBuffer() and
 VolumeVolumeQuery::GetNumOverlaps().

\return The number of primitive pair intersections.
 */
uint32_t
VolumeVolumeQuery::GetPrimitiveIntersections()
{
    GetPrimitiveBBoxOverlaps();

    rwcMETRICS(m_metrics.m_gpTime.Start());

    int32_t intersectionCount = detail::PrimitiveBatchIntersect( m_intersectionBuffer,
                                                         (int32_t) m_intersectionBufferMaxSize,
                                                         m_instancingSPR,
                                                         m_volRefPairBuffer,
                                                         (int32_t)m_volRef1xNCount,
                                                         m_padding,
                                                         m_edgeCosBendNormalThreshold,
                                                         m_convexityEpsilon );
    rwcMETRICS(m_metrics.m_gpTime.Stop());

    return (uint32_t) intersectionCount;
}

} // namespace collision
} // namespace rw
