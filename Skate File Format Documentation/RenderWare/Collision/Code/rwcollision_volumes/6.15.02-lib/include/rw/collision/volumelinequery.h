// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_VOLUMELINEQUERY_H
#define PUBLIC_RW_COLLISION_VOLUMELINEQUERY_H

/*************************************************************************************************************

 File: rwcvolumelinequery.hpp

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

typedef class VolumeLineQuery VolumeLineQuery;


/**
\brief Volume line query interface class.

\par
A typical usage of a volume line query would be:
\code
//Initialize the volume line query object
VolumeLineQuery *volLineQuery = VolumeLineQuery::Initialize(resource,    
                                                            STACKSIZE,   
                                                            RESBUFSIZE); 
//Get a ptr to the results buffer
VolumeLineSegIntersectResult *results = volLineQuery->GetIntersectionResultsBuffer();

//Initialize the specific query parameters
volLineQuery->InitQuery(volumeArray,
                        volumeMtxPtrArray,
                        numVols,
                        lineStart,
                        lineEnd);

//Continue while there are still volumes left to query
while(!volLineQuery->Finished())
{
    //Get as many results as possible
    //This runs the intersection query so don't wrap in the if() statement
    uint32_t numRes = volLineQuery->GetAllIntersections();

    //Process the results
    for(uint32_t i=0; i<numRes; i++)
    {
        //Do something with results[i]
    }
}
\endcode

\importlib rwccore
*/
class VolumeLineQuery
{
public:
    /**
    \internal

    \brief Query results set enum.
    */
    typedef enum {
        /** \brief Keep going until all the line intersections have been added to the results buffer.
          */
        ALLLINEINTERSECTIONS,
        /** \brief Stop after the first line intersection has been found.
          */
        ANYLINEINTERSECTION,
        /** \brief Find the nearest intersection to the start of the line and add it to results buffer.
            For aggregates or procedurals with spatial maps, the line is progressively clipped as leaf
            nodes are hit.
          */
        NEARESTLINEINTERSECTION
    } QueryResultsSet;

    //See .cpp file for documentation
    VolumeLineQuery(uint32_t stackMax,
                    uint32_t primsBufferSize,
                    uint32_t resBufferSize);

    /**
    \internal

    \brief
    Add a primitive volume ref to the query primitive buffer.

    \param vol A ptr to a volume 
    \param tm The transform of this volume in the query reference frame.
    \param tag The tag for this volume
    \param numTagBits The number of bits to reserve for this tag. Any children of this ref will
        be given a tag constructed from this tag and the child index shifted left by numTagBits.

    \return TRUE if the volume was added successfully, FALSE otherwise

    \importlib rwccore
    */
    RwpBool
    AddPrimitiveRef(const Volume *vol, 
                    const rwpmath::Matrix44Affine *tm,
                    const uint32_t tag,
                    const uint8_t numTagBits)
    {
        EA_ASSERT(vol->GetType() != rw::collision::VOLUMETYPEAGGREGATE);

        if (m_primNext >= m_primBufferSize)
        {
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
        m_primVRefBuffer[m_primNext].tag = tag;
        m_primVRefBuffer[m_primNext].numTagBits = numTagBits;
        m_primNext++;
        return TRUE;
    }

    /**
    \internal

    \brief
    Add a volume ref. If the volume is not a container volume 
    (i.e. Not an aggregate volume) , it will be added directly to 
    the query primitive buffer otherwise it will be added to the stack.

    \param vol A ptr to a volume 
    \param tm The transform of this volume in the query reference frame.
    \param tag The tag for this volume
    \param numTagBits The number of bits to reserve for this tag. Any children of this ref will
        be given a tag constructed from this tag and the child index shifted left by numTagBits.

    \return TRUE if the volume was added successfully, FALSE otherwise
    */
    RwpBool
    AddVolumeRef(const Volume *vol, 
                 const rwpmath::Matrix44Affine *tm,
                 const uint32_t tag,
                 const uint8_t numTagBits)
    {
        if (vol->GetType() != rw::collision::VOLUMETYPEAGGREGATE)
        {
            return AddPrimitiveRef(vol, tm, tag, numTagBits);
        }

        if (m_stackNext >= m_stackMax)
        {
            //we couldn't add this vol which means the stack needs to be bigger
            EAPHYSICS_MESSAGE("VRef stack not large enough to cope with volume hierarchy.");
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
        m_stackVRefBuffer[m_stackNext].tag = tag;
        m_stackVRefBuffer[m_stackNext].numTagBits = numTagBits;
        m_stackNext++;
        return TRUE;
    }

protected:
    //See .cpp file for documentation
    uint32_t
    GetIntersections();

public:
    //See .cpp file for documentation
    uint32_t
    GetAllIntersections();

    //See .cpp file for documentation
    VolumeLineSegIntersectResult *
    GetAnyIntersection();

    //See .cpp file for documentation
    VolumeLineSegIntersectResult *
    GetNearestIntersection();

    /**
    \brief Get intersection result buffer.

    Get the line intersection results buffer. The results buffer is assigned internally from the
    allocated memory block during the  VolumeLineQuery::Initialize call.

    \see VolumeLineQuery::Initialize

    \return A ptr to the internally asigned results buffer.
    */
    VolumeLineSegIntersectResult *
    GetIntersectionResultsBuffer() const
    {
        return m_resBuffer;
    }

    //See .cpp file for documentation
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t stackMax,
                          uint32_t resBufferSize);

    //See .cpp file for documentation
    static VolumeLineQuery *
    Initialize(const EA::Physics::MemoryPtr& memoryResource,
               uint32_t stackMax,
               uint32_t resBufferSize);

    /**
    \brief
    Releases a VolumeLineQuery object. The memory block that this object was initialized
    with is not freed by this function.

    \param query Pointer to a VolumeLineQuery object.
    */
    static void
    Release(VolumeLineQuery * /*query*/)
    {
    };


    /**
    \brief Releases a VolumeLineQuery object. 
    The memory block that this object was initialized with is not freed by this function.
    */
    void
    Release()
    {
    };


    /**
    \brief Initialize a line segment query.

    Initializes a new line segment query with the input volumes and line endpoints. 
    This also initializes all the internal query state so that a subsequent call to 
     VolumeLineQuery::GetAllIntersections will start from the beginning.

    \param inputVols    Array of pointers to volumes to test.
    \param inputMats    Array of pointers to transforms for each volume. If this is NULL then the 
                        volumes internal transforms will be used.
    \param numInputs    Number of volumes in the input array.
    \param pt1            Start of line segment
    \param pt2            End of line segment
    */
    void
    InitQuery(const rw::collision::Volume **inputVols,
              const rwpmath::Matrix44Affine **inputMats,
              const uint32_t numInputs,
              const rwpmath::Vector3 &pt1,
              const rwpmath::Vector3 &pt2,
              const float fatness =0.0)
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
        m_resCount = 0;
        m_instVolCount = 0;

        //initialize line segment
        m_pt1 = pt1;
        m_pt2 = pt2;
        m_endClipVal = 1.0f;
        m_fatness = fatness;

        //Default Query results set
        m_resultsSet = ALLLINEINTERSECTIONS;
        m_resMax = m_resBufferSize;

        //reset tagging
        m_tag = 0;
        m_numTagBits = 0;

    }


    /**
    \brief Check if current query is finished.

    Examine whether the current query has returned all possible intersections or 
    whether it still has more input volumes to test. Generally, this will be used
    when all the line intersection results are required and GetAllIntersections() might
    have returned due to an internal buffer overflow.

    For example:
    \code
    while(!lineQuery.Finished())
    {
        numRes = lineQuery.GetAllIntersections();

        resBuffer = lineQuery.GetResultsBuffer();

        for(i=0; i<numRes; i++)
        {
            //Do something with resBuffer[i]
        }
    }
    \endcode

    \return TRUE if the current query is finished, FALSE otherwise
    */
    RwpBool
    Finished()
    {
        if(m_currInput >= m_numInputs &&
           m_currVRef.volume == 0 &&
           m_stackNext == 0 && 
           m_primNext == 0)
        {
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }

    //Input buffer
    const rw::collision::Volume **m_inputVols;
    const rwpmath::Matrix44Affine **m_inputMats;
    uint32_t m_numInputs;
    uint32_t m_currInput;


    //intersection results buffer
    VolumeLineSegIntersectResult *m_resBuffer;
    uint32_t    m_resCount;
    uint32_t    m_resMax; //Max results we want output
    uint32_t    m_resBufferSize;//Size of results Buffer

    //Line parameters
    rwpmath::Vector3 m_pt1;
    rwpmath::Vector3 m_pt2;
    float          m_fatness;

    //Input volume stack
    VolRef      *m_stackVRefBuffer;
    VolRef      m_currVRef;  //this is the one we're working on
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
    float     m_endClipVal;

    //Results set
    QueryResultsSet m_resultsSet;

    //Keep track of the tags for results
    uint32_t    m_tag;
    uint8_t    m_numTagBits;

    // Space for storing state to allow restarting when the result buffer is full.
    union
    {
        ClusteredMeshQueryRestartData m_clusteredMeshRestartData;
    };
};


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_VOLUMELINEQUERY_H
