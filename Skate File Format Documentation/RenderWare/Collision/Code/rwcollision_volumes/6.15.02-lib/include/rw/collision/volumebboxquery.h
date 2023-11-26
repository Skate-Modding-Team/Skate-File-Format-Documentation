// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_VOLUMEBBOXQUERY_H
#define PUBLIC_RW_COLLISION_VOLUMEBBOXQUERY_H

/*************************************************************************************************************

 File: rwcVolumeBBoxQuery.hpp

 Purpose:
 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/detail/querydata.h"

namespace rw
{
namespace collision
{

typedef class VolumeBBoxQuery VolumeBBoxQuery;


/**
\brief
Class for collision volume bbox query

\importlib rwccore
*/
class VolumeBBoxQuery
{
public:
    enum VolumeBBoxQueryFlags
    {
        VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE    = 0x1,  ///< Results buffer wasn't big enough for query
        VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE           = 0x2,  ///< Stack buffer wasn't big enough for query
        VOLUMEBBOXQUERY_RANOUTOFINSTANCEBUFFERSPACE  = 0x4,  ///< Instance buffer wasn't big enough for query
        VOLUMEBBOXQUERY_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
    };

// See .cpp file for docs
    VolumeBBoxQuery(uint32_t stackMax,
                    uint32_t instVolBufferSize,
                    uint32_t primsBufferSize);

    /**
    \brief
    Add a primitive volume ref to the query primitive buffer.

    \param vol A ptr to a primitive volume (i.e. Not an aggregate volume)
    \param tm The transform of this volume in the query reference frame.
    \param bb
    \param tag
    \param numTagBits

    \return TRUE if the volume was added successfully, FALSE otherwise
    */
    RwpBool
    AddPrimitiveRef(const Volume *vol,
                    const rwpmath::Matrix44Affine *tm,
                    const AABBox &bb,
                    const uint32_t tag,
                    const uint8_t numTagBits)
    {
        EA_ASSERT(vol->GetType() != rw::collision::VOLUMETYPEAGGREGATE);

        if (m_primNext >= m_primBufferSize)
        {
            m_flags |= VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE;
            return FALSE;
        }

        m_primVRefBuffer[m_primNext].volume = vol;
        if ( tm )
        {
            m_primVRefBuffer[m_primNext].tmContents = *tm;
            m_primVRefBuffer[m_primNext].tm = &m_primVRefBuffer[m_primNext].tmContents;
        }
        else
        {
            m_primVRefBuffer[m_primNext].tm = NULL;
        }
        m_primVRefBuffer[m_primNext].bBox = bb;
        m_primVRefBuffer[m_primNext].tag = tag;
        m_primVRefBuffer[m_primNext].numTagBits = numTagBits;
        m_primNext++;

        return TRUE;
    }

    /**
    \brief
    Add a volume ref. If the volume is not a container volume
    (i.e. Not an aggregate volume) , it will be added directly to
    the query primitive buffer otherwise it will be added to the stack.

    \param vol A ptr to a volume
    \param tm The transform of this volume in the query reference frame.
    \param bb
    \param tag
    \param numTagBits

    \return TRUE if the volume was added successfully, FALSE otherwise.
    */
    RwpBool
    AddVolumeRef(const Volume *vol,
                 const rwpmath::Matrix44Affine *tm,
                 const AABBox &bb,
                 const uint32_t tag,
                 const uint8_t numTagBits)
    {
        if (vol->GetType() != rw::collision::VOLUMETYPEAGGREGATE)
        {
            return AddPrimitiveRef(vol, tm, bb, tag, numTagBits);
        }

        if (m_stackNext >= m_stackMax)
        {
            m_flags |= VOLUMEBBOXQUERY_RANOUTOFSTACKSPACE;
            return FALSE;
        }

        m_stackVRefBuffer[m_stackNext].volume = vol;
        if ( tm )
        {
            m_stackVRefBuffer[m_stackNext].tmContents = *tm;
            m_stackVRefBuffer[m_stackNext].tm = &m_stackVRefBuffer[m_stackNext].tmContents;
        }
        else
        {
            m_stackVRefBuffer[m_stackNext].tm = NULL;
        }
        m_stackVRefBuffer[m_stackNext].bBox   = bb;
        m_stackVRefBuffer[m_stackNext].tag    = tag;
        m_stackVRefBuffer[m_stackNext].numTagBits = numTagBits;
        m_stackNext++;

        return TRUE;
    }

    uint32_t
    GetOverlaps();

    /**
    \brief Get the bbox overlap results buffer
    \return A ptr to the internally asigned results buffer
    */
    VolRef *
    GetOverlapResultsBuffer() const
    {
        return m_primVRefBuffer;
    }

    /**
    \brief Get the bbox overlap results buffer count
    \return the number of volumes in the results buffer
    */
    uint32_t
    GetOverlapResultsBufferCount() const
    {
        return m_primNext;
    }

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t stackMax,
                          uint32_t resBufferSize);

    static VolumeBBoxQuery *
    Initialize(const EA::Physics::MemoryPtr& resource,
               uint32_t stackMax,
               uint32_t resBufferSize);

    /**
    \brief
    Releases a VolumeBBoxQuery object. The memory block that this object was initialized
    with is not freed by this function.
    \param query
    */
    static void
    Release(VolumeBBoxQuery * /*query*/)
    {
    };

    /**
    \brief
    Initializes a new bounding box query with the input volumes to test and the query bbox.
    This also initializes all the internal query state so that a subsequent call to
     VolumeBBoxQuery::GetOverlaps will start from the beginning.

    \param inputVols Array of pointers to volumes to test.
    \param inputMats Array of pointers to parent transforms for each input volume.
    If this is NULL then just the volumes internal transforms will be used.
    \param numInputs Number of volumes in the input array.
    \param aabb Axis Aligned Bounding Box to query against input volume array.
    */
    void
    InitQuery(const rw::collision::Volume **inputVols,
              const rwpmath::Matrix44Affine **inputMats,
              const uint32_t numInputs,
              const AABBox &aabb)
    {
        // Initialize application input
        m_inputVols = inputVols;
        m_inputMats = inputMats;
        m_numInputs = numInputs;
        m_currInput = 0;

        //initialize internal buffer states
        m_stackNext = 0;
        m_primNext = 0;
        m_currVRef.volume = 0;
        m_aggIndex = 0;
        m_curSpatialMapQuery = 0;
        m_instVolCount = 0;

        //initialize bbox data
        m_aabb = aabb;

        //reset tagging
        m_tag = 0;
        m_numTagBits = 0;

        //reset status
        m_flags = 0;

    }

    /**
    \brief
    Examine whether the current query has returned all possible intersections or
    whether it still has more input volumes to test. Generally, this will be used
    when all the bbox overlap results are required and GetOverlaps() might
    have returned due to an internal buffer overflow.

    \return TRUE if the current query is finished, FALSE otherwise
    */
    RwpBool
    Finished()
    {
        if(m_currInput == m_numInputs &&
           m_currVRef.volume == 0 &&
           m_stackNext == 0)
        {
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }

    /**
    \brief Sets the flags on the VolumeBBoxQuery

    \param flags Flags to set.
    */
    inline void
    SetFlags(uint32_t flags)
    {
        m_flags = flags;
    }

    /**
    \brief Gets the flags on the VolumeBBoxQuery

    \return the VolumeBBoxQuery flags
    */
    inline uint32_t
    GetFlags()
    {
        return m_flags;
    }

    //Input buffer
    const rw::collision::Volume **m_inputVols;
    const rwpmath::Matrix44Affine **m_inputMats;
    uint32_t m_numInputs;
    uint32_t m_currInput;

    //Query bbox parameters
    AABBox m_aabb;

    //Input volume stack
    VolRef      m_currVRef;  //this is the one we're working on
    VolRef      *m_stackVRefBuffer;
    uint32_t    m_stackNext; //where we stick the next entry
    uint32_t    m_stackMax;  //max entries

    //Primitive buffer
    VolRef      *m_primVRefBuffer;
    uint32_t    m_primNext;
    uint32_t    m_primBufferSize;

    //Instanced volume buffer
    Volume      *m_instVolPool;
    uint32_t    m_instVolCount;
    uint32_t    m_instVolMax;

    //Query state variables
    uint32_t    m_aggIndex;
    void*       m_spatialMapQueryMem;
    void*       m_curSpatialMapQuery;

    //Keep track of tags
    uint32_t    m_tag;
    uint8_t     m_numTagBits;

    //Flags used to track things like stack and result buffer overflow
    uint32_t    m_flags;

    // Space for storing state to allow restarting when the result buffer is full.
    union
    {
        ClusteredMeshQueryRestartData m_clusteredMeshRestartData;
    };
};

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_VOLUMEBBOXQUERY_H
