// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_VOLUMEVOLUMEQUERY_H
#define PUBLIC_RW_COLLISION_VOLUMEVOLUMEQUERY_H

/*************************************************************************************************************

 File: rwcvolumevolumequery.hpp

 Purpose:
 */


#include "rw/collision/common.h"
#include "rw/collision/bittable.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"
#include "rw/collision/aabbox.h"

namespace rw
{
namespace collision
{

struct PrimitivePairIntersectResult;

typedef class VolumeVolumeQuery VolumeVolumeQuery;


/**
\brief Class for collision volume volume query

\importlib rwccore
*/
class VolumeVolumeQuery
{
public:

    VolumeVolumeQuery(uint32_t stackSize,
                      uint32_t resBufferSize);

    uint32_t
    GetPrimitiveBBoxOverlaps();

    uint32_t
    GetPrimitiveIntersections();

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t stackSize, uint32_t resBufferSize);

    static VolumeVolumeQuery *
    Initialize(const EA::Physics::MemoryPtr& resource, uint32_t stackSize, uint32_t resBufferSize);

    /**
    \brief
    Releases a VolumeVolumeQuery object. The memory block that this object was initialized
    with is not freed by this function.




    \param query
    */
    static void
    Release(VolumeVolumeQuery * /*query*/)
    {
    };

    /**
    \brief Get the volume / volume bbox overlap results buffer.
    \return A ptr to the internally assigned VolRefPair buffer.
    */
    VolRefPair *
    GetOverlapResultsBuffer() const
    {
        return m_volRefPairBuffer;
    }


    /**
    \brief
    Get the number of volume / volume bbox overlaps from the last call to
     VolumeVolumeQuery::GetOverlaps(). Used after VolumeVolumeQuery::GetIntersections()

    \return A ptr to the internally assigned VolRefPair buffer
    */
    uint32_t
    GetNumOverlaps() const
    {
        return m_volRef1xNCount;
    }


    /**
    \brief Get the volume / volume intersection results buffer.

    \return A ptr to the internally assigned PrimitivePairIntersectResult buffer.
    */
    PrimitivePairIntersectResult*
    GetIntersectionResultsBuffer() const
    {
        return m_intersectionBuffer;
    }


    /**
    \brief Initializes a specific volume-volume query.

    Initializes a specific volume-volume query. In general, this may be one volume queried against many.
    The  VolumeVolumeQuery::Initialize function must have been called prior to this to initialize the
    query buffer memory layout, but this need only be done once for many queries.

    \param inputVols    Array of pointers to volumes to test.
    \param inputMats    Array of pointers to parent transforms for each input volume.
                        If this is NULL, then no transforms are applied to any of the volumes.
                        If a particular transform pointer is NULL, then no transform is applied to
                        the corresponding volume.
    \param numInputs    Number of volumes in the input array.
    \param queryVol     Single volume to query against the input volume set.
    \param queryMtx     Transformation matrix of the query volume (NULL means no transform).
    \param cullTable    Optional culling table specifying volume groups that do not collide.
    \param padding      Optional padding value (collision is detected between two surfaces separated by this distance).
    \param edgeCosBendNormalThreshold Do bent-normal processing if edgecos is below this threshold.
    \param convexityEpsilon Accept the contact if the dot is within this tolerance of the edgeCos.

    The default value of edgeCosBendNormalThreshold is -1, which disables bent-normal processing.
    The default value of convexityEpsilon is zero, which uses plain edgeCos testing.

    For an explanation if bent-normal processing please see
    http://easites.ea.com/EATech/Physics/Documents/Design_Docs/Collision/Triangle_Filtering_Overview.docx 
    */
    void
    InitQuery(const rw::collision::Volume **inputVols,
              const rwpmath::Matrix44Affine **inputMats,
              const uint32_t numInputs,
              const rw::collision::Volume *queryVol,
              const rwpmath::Matrix44Affine *queryMtx,
              const rw::collision::BitTable *cullTable = NULL,
              float padding = 0.0f,
              float edgeCosBendNormalThreshold = -1.0f,
              float convexityEpsilon = 0.0f)
    {
        // Initialize application input
        m_inputVols = inputVols;
        m_inputMats = inputMats;
        m_numInputs = numInputs;
        m_currInput = 0;

        // initialize internal buffer states
        m_volRefPairCount = 0;
        m_queryVol = queryVol;
        m_queryMtx = queryMtx;

        m_padding = padding;
        m_edgeCosBendNormalThreshold = edgeCosBendNormalThreshold;
        m_convexityEpsilon = convexityEpsilon;

        m_cullTable = cullTable;

    }

    //Input buffer
    const rw::collision::Volume **m_inputVols;
    const rwpmath::Matrix44Affine **m_inputMats;
    uint32_t m_numInputs;
    uint32_t m_currInput;

    // Culling Table
    const rw::collision::BitTable *m_cullTable;

    // SafeTime padding
    float m_padding;

    // Normal Bending & convexity values for primitive queries
    float m_edgeCosBendNormalThreshold;
    float m_convexityEpsilon;

    // overlap results buffer
    VolRefPair *m_volRefPairBuffer;
    uint32_t    m_volRefPairCount;
    uint32_t    m_volRefPairBufferSize;//Size of results Buffer
    VolRef1xN  *m_volRef1xNBuffer;
    uint32_t    m_volRef1xNCount;

    // instancing scratch pad
    GPInstance *m_instancingSPR;

    // intersection results buffer
    PrimitivePairIntersectResult   *m_intersectionBuffer;
    int32_t                         m_intersectionBufferMaxSize;

    // Query Volume parameters
    const rw::collision::Volume *m_queryVol;
    const rwpmath::Matrix44Affine *m_queryMtx;
    VolumeBBoxQuery *m_bBoxQueryAtoB;
    VolumeBBoxQuery *m_bBoxQueryBtoA;


#ifdef RWMETRICS
public:
    /**
    \internal

    \importlib rwccore
     */
    struct Metrics
    {
        rw::collision::Timer  m_gpTime;
        uint32_t    m_gpProbes;

        void Reset()
        {
            m_gpTime.Reset();
            m_gpProbes = 0;
        }
    };

    Metrics         m_metrics;
#endif
};


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_VOLUMEVOLUMEQUERY_H
