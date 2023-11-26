// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcvolumelinequery.cpp

 Purpose:

 */

// ***********************************************************************************************************
// Includes


#include <new>

#include "rw/collision/kdtree.h"
#include "rw/collision/octree.h"

#include "rw/collision/aggregate.h"
#include "rw/collision/volumelinequery.h"
#include "rw/collision/aggregatevolume.h"
#include "rw/collision/procedural.h"


using namespace rwpmath;

namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts

#define rwcVOLUMELINEQUERYALIGNMENT RWMATH_VECTOR3_ALIGNMENT

// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions

// ***********************************************************************************************************
// Structs + Unions + Classes


// ***********************************************************************************************************
// Static Functions


// ***********************************************************************************************************
// External Functions


// ***********************************************************************************************************
// Class Member Functions
/**
\brief In place constructor

\note  VolumeLineQuery objects should be created using  VolumeLineQuery::Initialize and
       not constructed directly.

\param stackMax The max number of entries on the internal stack. The stack needs to be large
       enough to handle the processing of container volumes (Aggregates & Procedurals). It
       never needs to be larger than the total number of container volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some intersections
       may be lost.

\param primsBufferSize The size of the internal prims array. If this buffer overflows while
       processing the stack, the query will perform a batched line intersect to
       empty it before continuing with processing the stack. This defaults to resBufferSize.

\param resBufferSize The max number of results held in the output array. If this
       number is reached during  VolumeLineQuery::GetAllIntersections then the query
       will exit.


\see VolumeLineQuery::GetResourceDescriptor
 */
VolumeLineQuery::VolumeLineQuery(uint32_t stackMax,
                                 uint32_t primsBufferSize,
                                 uint32_t resBufferSize)
{
    uint32_t size;

    m_stackMax = stackMax;
    m_primBufferSize = primsBufferSize;
    m_resBufferSize = resBufferSize;
    m_instVolMax = resBufferSize; //All the results might be instanced

    //container volume stack
    size = EA::Physics::SizeAlign<uint32_t>(sizeof(VolumeLineQuery), rwcVOLUMELINEQUERYALIGNMENT);
    m_stackVRefBuffer = (VolRef *)((uintptr_t)(this) + size);

    //Primitive buffer
    size += stackMax * sizeof(VolRef);
    m_primVRefBuffer = (VolRef *)((uintptr_t)(this) + size);

    //Instanced volume buffer
    size += primsBufferSize * sizeof(VolRef);
    m_instVolPool = (Volume *)((uintptr_t)(this) + size);

    //Results buffer
    size += resBufferSize * sizeof(Volume);
    m_resBuffer = (VolumeLineSegIntersectResult *)((uintptr_t)(this) + size);

    //Spatial map query gets the rest - Iterator get initialized when query gets created
    size += resBufferSize * sizeof(VolumeLineSegIntersectResult);

    // Ensure the buffer meets the LineQuery alignment requirement.
    const uint32_t queryAlignment = static_cast<uint32_t>(rwpmath::Max(EA_ALIGN_OF(KDTree::LineQuery), EA_ALIGN_OF(Octree::LineQuery)));
    size = EA::Physics::SizeAlign<uint32_t>(size, queryAlignment);
    m_spatialMapQueryMem = (void*)((uintptr_t)(this) + size);

}


/**
\brief Get the resource requirements of this object

\param stackMax The max number of entries on the internal stack. The stack needs to be large
       enough to handle the processing of aggregate volumes. It
       never needs to be larger than the total number of aggregate volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some intersections
       may be lost.

\param resBufferSize The max number of results held in the output array.  The internal
       primitive buffer is also resBufferSize big. If this
       number is reached during  VolumeLineQuery::GetAllIntersections then the query
       will exit.

\see VolumeLineQuery::GetResourceDescriptor
\see VolumeLineQuery::Initialize

\return The EA::Physics::SizeAndAlignment
 */
EA::Physics::SizeAndAlignment
VolumeLineQuery::GetResourceDescriptor(uint32_t stackMax,
                                       uint32_t resBufferSize)
{
    uint32_t size = 0;

    size += EA::Physics::SizeAlign<uint32_t>(sizeof(VolumeLineQuery), rwcVOLUMELINEQUERYALIGNMENT);

    //container volume stack
    size += sizeof(VolRef) * stackMax;
    //intermediate primitive buffer
    size += sizeof(VolRef) * resBufferSize;
    //Instanced volume buffer
    size += sizeof(Volume) * resBufferSize;
    //results buffer
    size += sizeof(VolumeLineSegIntersectResult) * resBufferSize;

    // Ensure we meet the alignment requirement of the LineQuery, which may be greater than that of of the
    // previous buffers.
    const uint32_t queryAlignment = static_cast<uint32_t>(rwpmath::Max(EA_ALIGN_OF(KDTree::LineQuery), EA_ALIGN_OF(Octree::LineQuery)));
    size = EA::Physics::SizeAlign<uint32_t>(size, queryAlignment);

    //Find the largest spatial map query size
    const uint32_t querySize = rwpmath::Max(static_cast<uint32_t>(sizeof(KDTree::LineQuery)), static_cast<uint32_t>(sizeof(Octree::LineQuery)));
    size += querySize;

    return EA::Physics::SizeAndAlignment(size, rwcVOLUMELINEQUERYALIGNMENT);
}


/**
\brief Initialize a EA::Physics::MemoryPtr as a VolumeLineQuery.

\param resource The EA::Physics::MemoryPtr the VolumeLineQuery is initialized into.

\param stackMax The max number of entries on the internal stack. The stack needs to be large
       enough to handle the processing of container volumes. It never needs to be
       larger than the total number of container volumes below the top level aggregate.

       \note The query will issue a message if the stack overflows and some intersections
       may be lost.

\param resBufferSize The max number of results held in the output array.  The internal
       primitive buffer is also resBufferSize big. If this
       number is reached during  VolumeLineQuery::GetAllIntersections then the query
       will exit.

\return
*/
VolumeLineQuery *
VolumeLineQuery::Initialize(const EA::Physics::MemoryPtr& resource,
                            uint32_t stackMax,
                            uint32_t resBufferSize)
{
    VolumeLineQuery *pVLQ = new (resource.GetMemory()) VolumeLineQuery(stackMax, resBufferSize, resBufferSize);

    return pVLQ;
}


/**
\internal

\brief Called from GetAllIntersections, GetAnyIntersection & GetNearestIntersection().

GetIntersections performs the mechanics of flattening the aggregate hierarcies and filling the
primitive or results buffers. It then calls the primitive intersection functions
while there is still space in the results buffer.

\return The number of line intersections added to the results buffer
 */
uint32_t
VolumeLineQuery::GetIntersections()
{
    Aggregate *agg;
    RwpBool overflow;
    m_resCount = 0; //Reset results buffer
    m_instVolCount = 0;
    m_tag = 0;
    m_numTagBits = 0;

    //keep going while there are volumes or primitives to process
    //and still room for results
    while((m_currInput < m_numInputs ||
           m_currVRef.volume ||
           (m_stackNext+m_primNext > 0))
          && (m_resCount < m_resMax))
    {
        overflow = FALSE;

        //more to do on the stack and space in primitive buffer if no oveflow and primitive buffer is empty
        while((m_currInput < m_numInputs || m_currVRef.volume || (m_stackNext > 0))
              && (!overflow) && (m_primNext == 0))
        {
            //move next input vol to stack or prim buffer if we're not processing one
            if(!m_currVRef.volume &&
                m_stackNext == 0 &&
                m_currInput < m_numInputs)
            {
                const Matrix44Affine *mtx;
                const Volume *vol = m_inputVols[m_currInput];

                //Skip this input volume if it is not enabled
                if ( !vol->IsEnabled() )
                {
                    m_currInput++;
                    continue;
                }

                //use input m_at array if we've been give one
                if(m_inputMats)
                {
                    mtx = m_inputMats[m_currInput];
                }
                else
                {
                    mtx = NULL;
                }

                //If primitive then will be added directly to prim buffer
                if(AddVolumeRef(vol, mtx, 0, 0))
                {
                    m_currInput++;
                }
                else
                {
                    overflow = TRUE; // Primitive or Stack buffer runs out of space
                }
            }

            //If we're processing a volume or more on the stack
            if(m_currVRef.volume || m_stackNext)
            {
                //Get the next volume off the stack
                if(!m_currVRef.volume)
                {
                    m_currVRef = m_stackVRefBuffer[--m_stackNext];
                }

                m_tag        = m_currVRef.tag;
                m_numTagBits = m_currVRef.numTagBits;

                //Process containers
                switch(m_currVRef.volume->GetType())
                {
                case rw::collision::VOLUMETYPEAGGREGATE:
                    {
                        Matrix44Affine mtx(m_currVRef.volume->GetLocalTransform());
                        const Matrix44Affine *mtxPtr = &mtx;

                        if( m_currVRef.tm )
                        {
                            mtx *= (*m_currVRef.tm);
                        }

                        agg = ((const AggregateVolume *)m_currVRef.volume)->GetAggregate();
                        //If we've reached the end of this aggregate then on to next vref
                        if(agg->LineIntersectionQuery(this, mtxPtr))
                        {
                            m_curSpatialMapQuery = 0;
                            m_aggIndex = 0; //reset for next volume on stack
                            m_currVRef.volume = 0; //flag to get another vref off the stack
                        }
                        else
                        {
                            overflow = TRUE; // Either primitive or Stack buffer runs out of space
                        }
                        break;
                    }
                default: //Add any primitives to primitive buffer
                    if(AddPrimitiveRef(m_currVRef.volume,
                                       m_currVRef.tm,
                                       m_currVRef.tag,
                                       m_currVRef.numTagBits))
                    {
                        m_currVRef.volume = 0; //flag to get another vref off the stack
                    }
                    else
                    {
                        overflow = TRUE; // Primitive buffer full
                    }
                    break;
                }
            }
        }


        // The loop below is running for every input only in order to keep track of correct input index in the results
        // Either we've processed all inputs or the prim buffer has been written with prims from the current input
        // so start processing primitive buffer from end
        while(m_primNext > 0 &&
               (m_resCount < m_resMax))
        {
            uint32_t idx = --m_primNext;
            const Volume *vol = m_primVRefBuffer[idx].volume;
            Matrix44Affine *tm = m_primVRefBuffer[idx].tm;
            VolumeLineSegIntersectResult *res = &m_resBuffer[m_resCount];
            if(vol->LineSegIntersect(m_pt1,
                                     m_pt2,
                                     tm,
                                     *res,
                                     m_fatness))
            {
                //Set far clip for any future kdtree decents
                if(m_resultsSet != ALLLINEINTERSECTIONS)
                {
                    if(res->lineParam < m_endClipVal )
                    {
                        m_endClipVal = res->lineParam;
                    }
                }
                res->inputIndex = m_currInput-1;
                res->v = m_inputVols[res->inputIndex];
                //In future the vref should be in a freelist
                res->vRef.volume = vol;
                if ( tm )
                {
                    res->vRef.tmContents = *tm;
                    res->vRef.tm = &res->vRef.tmContents;
                }
                else
                {
                    res->vRef.tm = NULL;
                }
                res->vRef.tag = m_primVRefBuffer[idx].tag;

                //These were primitives so will only have added one result
                m_resCount++;
            }
        }
        //if we failed to complete the query due to the stack being full then we need to ignore the top item in the stack and print a message
        if (overflow && (m_primNext == 0) && (m_stackNext >= m_stackMax))
        {
            //we found an overflow on the stack so the stack needs to be bigger
            EAPHYSICS_MESSAGE(("VRef stack not large enough to cope with volume hierarchy."));
            //to avoid any infinite loops we pop the last result off the stack before continuing
            m_stackNext--;
        }
    }

    return m_resCount;
}

/**
\brief
Queries the stored line against the input volumes and attempts to return all
all the intersections in the results buffer. GetAllIntersections() will exit if all
the intersections have been added to the results buffer or the results buffer
is full. If the results buffer is full then GetAllIntersections() can be
restarted and will continue from where it left off but overwriting the first
set of results. Use in conjunction with the  VolumeLineQuery::Finished function.

\code
while(!lineQuery.Finished())
{
    //This performs the line intersection query so don't wrap in the if() statement
    numRes = lineQuery.GetAllIntersections();

    results = lineQuery.GetIntersectionResultsBuffer();

    for(i=0; i<numRes; i++)
    {
        //Do something with intersection results
        ApplicationProcess(results[i]);
    }
}
\endcode

\par
To restart with a new query, call  VolumeLineQuery::InitQuery. To retrieve a pointer
to the results buffer, use  VolumeLineQuery::GetIntersectionResultsBuffer().

\note The results buffer is set internally during  VolumeLineQuery::Initialize.

\return The number of line intersections added to the results buffer.
 */
uint32_t
VolumeLineQuery::GetAllIntersections()
{
    m_resultsSet = ALLLINEINTERSECTIONS;

    //Utilise the whole output buffer Size
    m_resMax = m_resBufferSize;

    return GetIntersections();
}

/**
\brief
Queries the stored line against the input volumes and will return as soon as one
result is found.

\note To restart with a new query, call  VolumeLineQuery::InitQuery.

\return Ptr to an intersection result or NULL if none found.
 */
VolumeLineSegIntersectResult *
VolumeLineQuery::GetAnyIntersection()
{
    m_resultsSet = ANYLINEINTERSECTION;

    //Only need 1 result
    m_resMax = 1;

    if(GetIntersections())
    {
        return m_resBuffer;
    }
    else
    {
        return 0;
    }
}

/**
\brief
Queries the stored line against the input volumes and will return the result
closest to the start of the line segment.

\note To restart with a new query, call  VolumeLineQuery::InitQuery.

\return Ptr to the nearest intersection result or NULL if none found.
 */
VolumeLineSegIntersectResult *
VolumeLineQuery::GetNearestIntersection()
{
    VolumeLineSegIntersectResult result;

    //This may get used to clip the spatial map decents
    m_resultsSet = NEARESTLINEINTERSECTION;

    //Set the size of the results buffer 
    m_resMax = m_resBufferSize;

    //Get all the intersections and store the nearest one
    result.lineParam = MAX_FLOAT;
    while(!Finished())
    {
        //Store the nearest result
        uint32_t numRes = GetIntersections();
        if (numRes > 0)
        {
            VolumeLineSegIntersectResult *nearest = &result;
            for(uint32_t i=0; i<numRes; i++)
            {
                if(m_resBuffer[i].lineParam < nearest->lineParam)
                {
                    nearest = &m_resBuffer[i];
                }
            }

            // Store the nearest result
            result = *nearest;

            //Set clipping val for any future kdtree decents
            m_endClipVal = result.lineParam;
        }
    }

    //Return nearest result in the results buffer
    if(result.lineParam < MAX_FLOAT)
    {
        //copy the result over to [0]
        m_resBuffer[0] = result;
        return m_resBuffer;
    }
    else
    {
        return 0;
    }
}

} // namespace collision
} // namespace rw
