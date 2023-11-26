// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcVolumeBBoxQuery.cpp

 Purpose:
 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/kdtree.h"
#include "rw/collision/octree.h"
#include "rw/collision/aggregate.h"
#include "rw/collision/aggregatevolume.h"
#include "rw/collision/procedural.h"
#include "rw/collision/trianglekdtreeprocedural.h"
#include "rw/collision/volumebboxquery.h"

using namespace rwpmath;

namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts

#define rwcVOLUMEBBOXALIGNMENT RWMATH_VECTOR3_ALIGNMENT

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
\brief In place constructor.

\note This should not be called directly. VolumeBBoxQueries should be created using
      the VolumeBBoxQuery::Initialize() function with a preallocated memory block.

\param stackMax The max number of entries on the stack. The stack needs to be large
       enough to handle the processing of aggregate volumes. It
       never needs to be larger than the total number of aggregate volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some overlaps
       may be lost.

\param instVolBufferSize The max number of volumes that can be instanced from a
       procedural container. This defaults to primsBufferSize. If this
       number is reached during  GetOverlaps then the query will exit.

\param primsBufferSize The size of the prims array. If this buffer overflows while
       processing the stack, the query will exit.
*/
VolumeBBoxQuery::VolumeBBoxQuery(uint32_t stackMax,
                                 uint32_t instVolBufferSize,
                                 uint32_t primsBufferSize)
{
    uint32_t size;

    m_stackMax = stackMax;
    m_primBufferSize = primsBufferSize;
    m_instVolMax     = instVolBufferSize;

    //container volume stack
    size = EA::Physics::SizeAlign<uint32_t>(sizeof(VolumeBBoxQuery), rwcVOLUMEBBOXALIGNMENT);
    m_stackVRefBuffer = (VolRef *)((uintptr_t)(this) + size);

    //instanced volume buffer
    size += stackMax * sizeof(VolRef);
    m_instVolPool = (Volume *)((uintptr_t)(this) + size);

    //Results buffer
    size += instVolBufferSize * sizeof(Volume);
    m_primVRefBuffer = (VolRef *)((uintptr_t)(this) + size);

    //Spatial map query gets the rest - Iterator get initialized when query gets created
    size += primsBufferSize * sizeof(VolRef);
    m_spatialMapQueryMem = (void*)((uintptr_t)(this) + size);

    //Reset the flags
    m_flags = 0;

}


/**
\brief Get the resource requirements of this object.

\param stackMax The max number of entries on the stack. The stack needs to be large
       enough to handle the processing of aggregate volumes. It
       never needs to be larger than the total number of aggregate volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some overlaps
       may be lost.

\param resBufferSize The size of the output VolRef results array. The internal instanced
       volume buffer is also resBufferSize big since, at most, all the VolRefs
       will have been instanced. \note If the results buffer overflows while
       processing the stack, the query will exit.

\return The EA::Physics::SizeAndAlignment
 */
EA::Physics::SizeAndAlignment
VolumeBBoxQuery::GetResourceDescriptor(uint32_t stackMax,
                                       uint32_t resBufferSize)
{
    uint32_t size = 0;

    size += EA::Physics::SizeAlign<uint32_t>(sizeof(VolumeBBoxQuery), rwcVOLUMEBBOXALIGNMENT);

    size += sizeof(VolRef) * stackMax;

    size += sizeof(VolRef) * resBufferSize;

    size += sizeof(Volume) * resBufferSize;

    // Add size of largest spatial map query
    // TODO: need a way of registering spatial maps with their query size
    size += rwpmath::Max(static_cast<uint32_t>(sizeof(KDTree::BBoxQuery)), static_cast<uint32_t>(sizeof(Octree::BBoxQuery)));

    return EA::Physics::SizeAndAlignment(size, rwcVOLUMEBBOXALIGNMENT);
}


/**
\brief Initialize a EA::Physics::MemoryPtr as a VolumeBBoxQuery.

\param resource The EA::Physics::MemoryPtr the object is initialized into.
\param stackMax The max number of entries on the stack. The stack needs to be large
       enough to handle the processing of aggregate volumes. It
       never needs to be larger than the total number of aggregate volumes below the top level
       aggregate.

       \note The query will issue a message if the stack overflows and some overlaps
       may be lost.

\param resBufferSize The size of the output VolRef results array. The internal instanced
       volume buffer is also resBufferSize big since, at most, all the VolRefs
       will have been instanced. \note If the results buffer overflows while
       processing the stack, the query will exit.
 */
VolumeBBoxQuery *
VolumeBBoxQuery::Initialize(const EA::Physics::MemoryPtr& resource,
                            uint32_t stackMax,
                            uint32_t resBufferSize)
{
    VolumeBBoxQuery *pVBBQ =
        new (resource.GetMemory()) VolumeBBoxQuery(stackMax, resBufferSize, resBufferSize);

    return pVBBQ;
}



/**
\brief Determine if a matrix is identity, using a very strict condition.

\note Note this assumes orthonormality.
*/
static inline RwpBool
IsIdentity(const Matrix44Affine &m)
{
    return static_cast<RwpBool>(
        Abs(float(m.GetX().GetX()) - 1.0f) < EPSILON &&
        Abs(float(m.GetY().GetY()) - 1.0f) < EPSILON &&
        static_cast<float>(MagnitudeSquared(m.GetW())) < MINIMUM_RECIPROCAL);
}



/**
\brief Queries the stored bbox against the input volumes and attempts to return all
all the overlaps in the results buffer. 

GetOverlaps() will exit if all
the overlaps have been added to the results buffer or the results buffer
overflows. If the results buffer overflowed then GetOverlaps() can be
restarted and will continue from where it left off but overwriting the first
set of results. Use in conjunction with the  VolumeBBoxQuery::Finished() function.

\code
while(!bboxQuery.Finished())
{
    numRes = bboxQuery.GetOverlaps();
    results = bboxQuery.GetOverlapResultsBuffer();

    for(i=0; i<numRes; i++)
    {
        //Do something with the results
        ApplicationProcess(results[i]);
    }
}
\endcode

\par
To restart with a new query, call  VolumeBBoxQuery::InitQuery. To retrieve a pointer
to the results buffer, use  VolumeBBoxQuery::GetOvelapResultsBuffer().

\note The results buffer size is set using  VolumeBBoxQuery::Initialize

\return The number of bbox overlaps added to the results buffer.
 */
uint32_t
VolumeBBoxQuery::GetOverlaps()
{
    Aggregate *agg;
    RwpBool overflow;
    m_primNext = 0; //Reset results buffer
    m_instVolCount = 0;
    m_numTagBits = 0;
    //Reset the flags relating to running out buffer space
    m_flags &= ~(VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE |
                 VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE |
                 VOLUMEBBOXQUERY_RANOUTOFINSTANCEBUFFERSPACE);

    overflow = FALSE;

    //more to do on the stack and space in primitive buffer
    while((m_currInput < m_numInputs || m_currVRef.volume || (m_stackNext > 0))
          && (!overflow))
    {
        //move next input vol to stack or prim buffer if we're not processing one
        if(!m_currVRef.volume &&
            m_stackNext == 0 &&
            m_currInput < m_numInputs)
        {
            const Matrix44Affine *mtx;
            AABBox bb;
            const Volume *vol = m_inputVols[m_currInput];

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

            vol->GetBBox(mtx,0,bb);

            //If overlapping then add to volume stack
            if(m_aabb.Overlaps(bb))
            {
                //If primitive then will be added directly to prim buffer
                if(AddVolumeRef(vol, mtx, bb, 0, 0))
                {
                    m_currInput++;
                }
                else
                {
                    overflow = TRUE; // Primitive or Stack buffer runs out of space
                }
            }
            else
            {
                 m_currInput++;
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

            m_tag = m_currVRef.tag;
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
                    else
                    {
                        // For efficiency, convert identity matrix to NULL pointer
                        if (IsIdentity(mtx))
                        {
                            mtxPtr = NULL;
                        }
                    }

                    agg = ((const AggregateVolume *)m_currVRef.volume)->GetAggregate();

                    //If we've reached the end of this aggregate then on to next vref
                    if(agg->BBoxOverlapQuery(this, mtxPtr))
                    {
                        m_curSpatialMapQuery = 0;
                        m_aggIndex = 0; //reset for next volume on stack
                        m_currVRef.volume = 0; //flag to get another vref off the stack
                    }
                    else
                    {
                        overflow = TRUE; // Primitive or Stack buffer runs out of space
                    }
                }
                break;
            default: //Add any primitives to primitive buffer
                if(AddPrimitiveRef(m_currVRef.volume,
                                    m_currVRef.tm,
                                    m_currVRef.bBox,
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

    //if we failed to complete the query due to the stack being full then we need to ignore the top item in the stack and print a message
    if (overflow && (m_flags & VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE))
    {
        EA_ASSERT(m_stackNext == m_stackMax);
        //we found an overflow on the stack so the stack needs to be bigger
        EAPHYSICS_MESSAGE(("VRef stack not large enough to cope with volume hierarchy."));
        //to avoid any infinite loops we pop the last result off the stack before continuing
        m_stackNext--;
    }

    return m_primNext; //return the number of primitives
}


// ***********************************************************************************************************
//                                                Rw... CLASS
// ***********************************************************************************************************
} // namespace collision
} // namespace rw
