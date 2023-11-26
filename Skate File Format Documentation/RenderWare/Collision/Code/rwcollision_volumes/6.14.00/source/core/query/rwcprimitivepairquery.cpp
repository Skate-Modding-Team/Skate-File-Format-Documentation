// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcprimitivepairquery.cpp

 Purpose: Implementation of the system for querying intersections of primitive pairs.

 */

// ***********************************************************************************************************
// Includes
#include <EAAssert/eaassert.h>

#include "rw/collision/primitivepairquery.h"
#include "rw/collision/computecontacts.h"


/* Use platform specific maths */
using namespace rwpmath;

namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Forward Declarations

// ***********************************************************************************************************
// Global variables

VecFloat gDefaultMinimumSeparatingDistance =                COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance;
VecFloat gDefaultTriangleFaceNormalTolerance =              COMPUTECONTACTS_DEFAULT_TriangleFaceNormalTolerance;
VecFloat gDefaultFeatureSimplificationThreshold =           COMPUTECONTACTS_DEFAULT_FeatureSimplificationThreshold;
VecFloat gDefaultCosSquaredMaximumAngleConsideredParallel = COMPUTECONTACTS_DEFAULT_CosSquaredMaximumAngleConsideredParallel;
VecFloat gDefaultValidDirectionMinimumLengthSquared =       COMPUTECONTACTS_DEFAULT_ValidDirectionMinimumLengthSquared;
VecFloat gDefaultClippingLengthTolerance =                  COMPUTECONTACTS_DEFAULT_ClippingLengthTolerance;

// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions


#if defined(EA_ASSERT_ENABLED) && !defined(EA_PLATFORM_PS3_SPU)
static bool IsVolumeTypeValid(VolumeType type)
{
    return ((VOLUMETYPESPHERE <= type) && (VOLUMETYPEAGGREGATE >= type)) ? true : false;
}
#endif

/**
\deprecated
This parameter is not used anymore because it is not necessary in eacollision_primitives.
*/
float
SetTriangleEdgeCullingTolerance(float)
{
    rwcDEPRECATED("The TriangleEdgeCullingTolerance is no longer used.");
    return 0.f;
}

/**
This sets the tolerance for the face contact of a triangle.  The contact is considered a face contact only
if the absolute value of the dot product of contact normal and the triangle face normal is above this
tolerance.  If you set this number too high, then objects may drop through the mesh, particularly if
all the edges are disabled (edgecos == 1).  If you set this tolerance too low, then edge contacts will not
be removed, causing objects to hop or snag as they slide over the mesh.

The default value is 0.99985 which is cosine of 1 degree.
\return the old tolerance value.
*/
float
SetTriangleFaceNormalTolerance(float newTolerance)
{
    rwcDEPRECATED("The SetTriangleFaceNormalTolerance is no longer used.");
    float oldValue = gDefaultTriangleFaceNormalTolerance;
    gDefaultTriangleFaceNormalTolerance = newTolerance;
    return oldValue;
}


// the following are not required for PS3 SPU
#if       !defined(EA_PLATFORM_PS3_SPU)
namespace detail {
/**
\internal

\brief
Given a single primitive and a batch of "other" primitives, test them 1-versus-N
and put the results into the buffer provided by the caller.

\param resBuf array of intersection results
\param resBufMaxSize maximum number of intersection results
\param inst1 single primitive
\param insts2 array of primitives
\param num number of primitives in the insts2 array
\param minimumSeparatingDistance minimum separating distance between any two primitives for which contacts will NOT be generated.
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

\return the total number of intersection results returned in resBuf. (not total number of points)
*/
int32_t
GPInstanceBatchIntersect1xN( PrimitivePairIntersectResult *resBuf,
                             int32_t resBufMaxSize,
                             const GPInstance &inst1,
                             const GPInstance *insts2, int32_t num,
                             float minimumSeparatingDistance,
                             float edgeCosBendNormalThreshold,
                             float convexityEpsilon)
{
    int32_t numIntersections = 0;

    if (num > resBufMaxSize)
    {
        EAPHYSICS_MESSAGE("GPInstanceBatchIntersect1xN: Only enough room for %d results in result buffer, incoming instances is %d...clamping...intersections will be lost.",
            resBufMaxSize, num);
        num = resBufMaxSize;
    }
    const VecFloat minimumSeparatingDistanceVec = minimumSeparatingDistance;

    for (int32_t i = 0; i < num; ++i)
    {
        uint32_t hit = ComputeContacts(inst1, insts2[i], resBuf[numIntersections], minimumSeparatingDistanceVec,
                                        edgeCosBendNormalThreshold,
                                        convexityEpsilon,
                                        gDefaultTriangleFaceNormalTolerance,
                                        gDefaultFeatureSimplificationThreshold,
                                        gDefaultCosSquaredMaximumAngleConsideredParallel,
                                        gDefaultValidDirectionMinimumLengthSquared,
                                        gDefaultClippingLengthTolerance);

        resBuf[numIntersections].vNindex = i;
        numIntersections += (int32_t) hit;
    }

    return  numIntersections ;
}

/**
\internal

\brief
Given a single primitive and a batch of "other" primitives, test them 1-versus-N
and put the results into the buffer provided by the caller.

\param resBuf array of intersection results
\param resBufMaxSize maximum number of intersection results
\param insts1 array of primitives
\param num number of primitives in the insts1 array
\param inst2 single primitive
\param minimumSeparatingDistance minimum separating distance between any two primitives for which contacts will NOT be generated.
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

\return the total number of intersection results returned in resBuf. (not total number of points)
*/
int32_t
GPInstanceBatchIntersectNx1( PrimitivePairIntersectResult *resBuf,
                             int32_t resBufMaxSize,
                             const GPInstance *insts1, int32_t num,
                             const GPInstance &inst2,
                             float minimumSeparatingDistance,
                             float edgeCosBendNormalThreshold,
                             float convexityEpsilon)
{
    int32_t numIntersections = 0;

    if (num > resBufMaxSize)
    {
        EAPHYSICS_MESSAGE("GPInstanceBatchIntersectNx1: Only enough room for %d results in result buffer, incoming instances is %d...clamping...intersections will be lost.",
            resBufMaxSize, num);
        num = resBufMaxSize;
    }
    const VecFloat minimumSeparatingDistanceVec = minimumSeparatingDistance;

    for (int32_t i = 0; i < num; ++i)
    {
        uint32_t hit = ComputeContacts(insts1[i], inst2, resBuf[numIntersections], minimumSeparatingDistanceVec,
                                        edgeCosBendNormalThreshold,
                                        convexityEpsilon,
                                        gDefaultTriangleFaceNormalTolerance,
                                        gDefaultFeatureSimplificationThreshold,
                                        gDefaultCosSquaredMaximumAngleConsideredParallel,
                                        gDefaultValidDirectionMinimumLengthSquared,
                                        gDefaultClippingLengthTolerance);

        resBuf[numIntersections].vNindex = i;
        numIntersections += (int32_t) hit;
    }

    return  numIntersections ;
}


/**
\brief
Given two primitive volumes and their additional transformation matrices,
perform an intersection test between them.
Returns TRUE if the primitives intersect, FALSE otherwise.
Result structure contains the contact normal, the penetration distance, a pair of points
that represent the "center" of the intersection, and the full intersection result:
a set of pairs of points, accompanied by separations; this set of points is
what we call "feature intersection prism".

\param res results of the intersection
\param v1 first primitive collision volume
\param tm1 part to world transform.  The v1 relative transform is composed with this.
\param v2 second primitive collision volume
\param tm2 part to world transform.  The v2 relative transform is composed with this.
\param minimumSeparatingDistance the minimum separating distance for which the result is generated
\param sepDir (legacy) this parameter is ignored.
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

\return true if contact points are found.  False if the distance between the two volumes is greater than minimumSeparatingDistance
*/
RwpBool
PrimitivePairIntersect( PrimitivePairIntersectResult &res,
                        const Volume *v1, const Matrix44Affine *tm1,
                        const Volume *v2, const Matrix44Affine *tm2,
                        float minimumSeparatingDistance, const Vector4 * /* sepDir */,
                        float edgeCosBendNormalThreshold,
                        float convexityEpsilon )
{
    if ( !v1->IsEnabled()  ||  !v2->IsEnabled() )
    {
        return FALSE;
    }

    GPInstance inst1, inst2;

    EA_ASSERT(IsVolumeTypeValid(v1->GetType()));
    EA_ASSERT(IsVolumeTypeValid(v2->GetType()));

    v1->CreateGPInstance( inst1, tm1 );
    v2->CreateGPInstance( inst2, tm2 );

    return ComputeContacts(inst1, inst2, res, minimumSeparatingDistance,
                            edgeCosBendNormalThreshold,
                            convexityEpsilon,
                            gDefaultTriangleFaceNormalTolerance,
                            gDefaultFeatureSimplificationThreshold,
                            gDefaultCosSquaredMaximumAngleConsideredParallel,
                            gDefaultValidDirectionMinimumLengthSquared,
                            gDefaultClippingLengthTolerance);
}

/**
\internal

\brief
Given an array of primitives, instantiates an array of GPInstance structures into a given buffer.

\param resBuf The output buffer for GPInstance structures to be instantiated into
\param vN     Array of source primitive volumes
\param tmN    Array of pointers to transforms for the source primitive volumes
\param num    The number of source volumes submitted

*/
void
PrimitiveBatchInstance( GPInstance *resBuf,
                        const Volume *vN, const Matrix44Affine *tmN, int32_t num )
{
    rwcDEPRECATED("This internal api will be removed next release.");
    int32_t vi;

    for ( vi = 0;  vi < num;  vi++ )
    {
        EA_ASSERT(IsVolumeTypeValid(vN[vi].GetType()));
        vN[vi].CreateGPInstance( resBuf[vi], &tmN[vi] );
    }
}

/**
\internal

\brief

\param resBuf
\param vN
\param tmN
\param num

*/
void
PrimitiveBatchInstance( GPInstance *resBuf,
                        const Volume *vN, const Matrix44Affine **tmN, int32_t num )
{
    rwcDEPRECATED("This internal api will be removed next release.");
    int32_t vi;

    for ( vi = 0;  vi < num;  vi++ )
    {
        EA_ASSERT(IsVolumeTypeValid(vN[vi].GetType()));
        vN[vi].CreateGPInstance( resBuf[vi], tmN[vi] );
    }
}

/**
\internal

\brief
Given a single volume and a batch of "other" volumes, test them 1-versus-N
and put the results into the buffer provided by the caller.
This version assumes that the "other" volumes and their transformation matrices
are both stored in contiguous arrays in memory.

\param resBuf
\param resBufMaxSize
\param instancingSPR
\param v1
\param tm1
\param vN
\param tmN
\param num
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

\return The total number of intersections found.
*/
int32_t
PrimitiveBatchIntersect1xN( PrimitivePairIntersectResult *resBuf,
                            int32_t resBufMaxSize,
                            GPInstance *instancingSPR,
                            const Volume *v1, const Matrix44Affine *tm1,
                            const Volume *vN, const Matrix44Affine *tmN, int32_t num,
                            float edgeCosBendNormalThreshold,
                            float convexityEpsilon )
{
    rwcDEPRECATED("This internal api will be removed next release.");
    int32_t vi;

    GPInstance *instN = instancingSPR;
    GPInstance &inst1 = instancingSPR[num];

    EA_ASSERT(IsVolumeTypeValid(v1->GetType()));
    v1->CreateGPInstance( inst1, tm1 );

    for ( vi = 0;  vi < num;  vi++ )
    {
        EA_ASSERT(IsVolumeTypeValid(vN[vi].GetType()));
        vN[vi].CreateGPInstance( instN[vi], &tmN[vi] );
    }

    return  GPInstanceBatchIntersect1xN( resBuf, resBufMaxSize, inst1, instN, num, 
        gDefaultMinimumSeparatingDistance, edgeCosBendNormalThreshold, convexityEpsilon ) ;
}

/**
\internal

\brief
Given a single volume and a batch of "other" volumes, test them 1-versus-N
and put the results into the buffer provided by the caller.
This version accepts arrays of pointers for the "other" volumes and their TMs,
so there is no assumption made about the arrangement of the volumes or their TMs in memory.

\param resBuf
\param resBufMaxSize
\param instancingSPR
\param v1
\param tm1
\param vN
\param tmN
\param num
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

\return The total number of intersections found.
*/
int32_t
PrimitiveBatchIntersect1xN( PrimitivePairIntersectResult *resBuf,
                            int32_t resBufMaxSize,
                            GPInstance *instancingSPR,
                            const Volume *v1, const Matrix44Affine *tm1,
                            const Volume **vN, const Matrix44Affine **tmN, int32_t num,
                            float edgeCosBendNormalThreshold,
                            float convexityEpsilon )
{
    rwcDEPRECATED("This internal api will be removed next release.");
    int32_t vi;

    GPInstance *instN = instancingSPR;
    GPInstance &inst1 = instancingSPR[num];

    EA_ASSERT(IsVolumeTypeValid(v1->GetType()));
    v1->CreateGPInstance( inst1, tm1 );

    for ( vi = 0;  vi < num;  vi++ )
    {
        EA_ASSERT(IsVolumeTypeValid(vN[vi]->GetType()));
        vN[vi]->CreateGPInstance( instN[vi], tmN[vi] );
    }

    return  GPInstanceBatchIntersect1xN( resBuf, resBufMaxSize, inst1, instN, num, 
        gDefaultMinimumSeparatingDistance, edgeCosBendNormalThreshold, convexityEpsilon ) ;
}

/**
\brief
Given two batches of volumes, test them N-versus-M
and put the results into the buffer provided by the caller.
This version accepts arrays of pointers for the volumes and their TMs,
so there is no assumption made about the arrangement of the volumes or their TMs in memory.

\param resBuf
\param resBufMaxSize
\param instancingSPR - temp area for converting Volumes to GPInstances, NOTE BELOW
\param vN
\param tmN
\param numN
\param vM
\param tmM
\param numM
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos
\return The total number of intersections found.

PLEASE NOTE: the instancingSPR is a memory area used by this function to convert volumes into GPInstance,
which is an internal representation.  On this release you must provide memory for at least (numN + numM)
GPInstances.  However, next release we are going to eliminate GPInstance and therefore you will
not need to pass this parameter.

*/
int32_t
PrimitiveBatchIntersectNxM( PrimitivePairIntersectResult *resBuf,
                            int32_t resBufMaxSize,
                            GPInstance *instancingSPR,    ///< SEE NOTES ABOVE
                            const Volume **vN, const Matrix44Affine **tmN, int32_t numN,
                            const Volume **vM, const Matrix44Affine **tmM, int32_t numM,
                            float edgeCosBendNormalThreshold,
                            float convexityEpsilon )
{
    int32_t vni, vmi;

    GPInstance *instN = instancingSPR;
    GPInstance *instM = instancingSPR+numN;

    for ( vni = 0;  vni < numN;  vni++ )
    {
        EA_ASSERT(IsVolumeTypeValid(vN[vni]->GetType()));
        vN[vni]->CreateGPInstance( instN[vni], tmN[vni] );
    }

    int32_t numIntersections = 0;
    int32_t bufPos = 0;
    for ( vmi = 0;  vmi < numM;  vmi++ )
    {
        EA_ASSERT(IsVolumeTypeValid(vM[vmi]->GetType()));
        vM[vmi]->CreateGPInstance( instM[vmi], tmM[vmi] );

        // Count all intersections even if we overrun buffer.
        numIntersections += GPInstanceBatchIntersect1xN( resBuf+bufPos,
                                                         resBufMaxSize-bufPos,
                                                         instM[vmi], instN, numN,
                                                         gDefaultMinimumSeparatingDistance,
                                                         edgeCosBendNormalThreshold,
                                                         convexityEpsilon );
        bufPos = Min(numIntersections, resBufMaxSize);
    }

    return numIntersections;
}

/**
\internal

\brief
Given an array of VolRefPairs and a result buffer resBuf, test all the pairs for
intersections and put the positive results into resBuf. This function accepts the maximum
number of results it's allowed to put into resBuf.

\param resBuf array of intersection results
\param resBufMaxSize maximum number of intersection results
\param instancingSPR
\param pairs list of volume reference pairs
\param numPairs number of pairs
\param minimumSeparatingDistance minimum separating distance between any two primitives for which contacts will NOT be generated.
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

\Return The number of intersection results put into resBuf.
*/
int32_t
PrimitiveBatchIntersect( PrimitivePairIntersectResult *resBuf,
                         int32_t resBufMaxSize,
                         GPInstance *instancingSPR,
                         VolRefPair *pairs,
                         int32_t numPairs,
                         float minimumSeparatingDistance,
                         float edgeCosBendNormalThreshold,
                         float convexityEpsilon )
{
    int32_t numIntersections = 0, numSpans = numPairs;

    // This cast is bad however because this code is approaching the end of its life it isn't going to be fixed
    // Avoid ghs compiler warning by casting via uintptr_t
    VolRef1xN *curSpan = (VolRef1xN*)(uintptr_t)pairs;

    for ( int32_t i = 0;  i < numSpans;  i++ )
    {
        if ( numIntersections < resBufMaxSize )
        {
            EA_ASSERT(IsVolumeTypeValid(curSpan->vRef1->volume->GetType()));
            curSpan->vRef1->volume->CreateGPInstance( instancingSPR[0], curSpan->vRef1->tm );
            instancingSPR[0].mUserTag = curSpan->vRef1->tag;
            for ( uint32_t overlap = 0;  overlap < curSpan->vRefsNCount;  overlap++ )
            {
                EA_ASSERT(IsVolumeTypeValid(curSpan->vRefsN[overlap]->volume->GetType()));
                curSpan->vRefsN[overlap]->volume->CreateGPInstance( instancingSPR[1+overlap],
                                                                    curSpan->vRefsN[overlap]->tm );
                instancingSPR[1+overlap].mUserTag = curSpan->vRefsN[overlap]->tag;
            }

            if ( curSpan->volumesSwapped )
            {
                numIntersections += GPInstanceBatchIntersectNx1( resBuf + numIntersections,
                                                                 (int32_t)(resBufMaxSize - numIntersections),
                                                                 instancingSPR + 1, (int32_t)curSpan->vRefsNCount,
                                                                 instancingSPR[0],
                                                                 minimumSeparatingDistance,
                                                                 edgeCosBendNormalThreshold,
                                                                 convexityEpsilon );
            }
            else
            {
                numIntersections += GPInstanceBatchIntersect1xN( resBuf + numIntersections,
                                                                 (int32_t)(resBufMaxSize - numIntersections),
                                                                 instancingSPR[0],
                                                                 instancingSPR + 1, (int32_t)curSpan->vRefsNCount,
                                                                 minimumSeparatingDistance,
                                                                 edgeCosBendNormalThreshold,
                                                                 convexityEpsilon );
            }
        }

        curSpan = (VolRef1xN*)( (uintptr_t)curSpan +
                                sizeof(VolRef1xN) +
                                sizeof(VolRef*) * (curSpan->vRefsNCount-1) );
    }

    return  numIntersections ;
}


/**
\internal

Computes the contact points between a single GPInstance and another GPInstance. This would be used to query a primitive against another primitive.

\param gp1 the first GPInstance
\param gp2 the second GPInstance
\param minimumSeparatingDistance contacts with distance greater than this will not be returned.
\param result OUTPUT a GPInstance::ContactPoints object that stores the contact points
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

The default value of edgeCosBendNormalThreshold is -1, which disables bent-normal processing.
The default value of convexityEpsilon is zero, which uses plain edgeCos testing.

\return zero or number of contact points
*/
uint32_t
ComputeContactPoints(const GPInstance &gp1, const GPInstance &gp2,
                     const float &minimumSeparatingDistance, GPInstance::ContactPoints &result,
                     float edgeCosBendNormalThreshold, float convexityEpsilon)
{
    uint32_t hit = ComputeContacts(gp1, gp2, result, minimumSeparatingDistance, edgeCosBendNormalThreshold, convexityEpsilon,
                                    gDefaultTriangleFaceNormalTolerance,
                                    gDefaultFeatureSimplificationThreshold,
                                    gDefaultCosSquaredMaximumAngleConsideredParallel,
                                    gDefaultValidDirectionMinimumLengthSquared,
                                    gDefaultClippingLengthTolerance);

    return hit ? result.numPoints : 0;
}

/**
\internal

Computes the contact points between a single GPInstance and an array of GPInstances.

An example of using this function would be querying a primitive against a triangle cache.

\param gp the single GPInstance
\param gps pointer to array of GPInstances
\param count size of the array
\param minimumSeparatingDistance contacts with distance greater than this will not be returned.
\param results OUTPUT buffer to store results in. Must provide enough storage to contain the results
\param intersectionCount OUTPUT total number of intersections produced across all the GP Queries
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

The default value of edgeCosBendNormalThreshold is -1, which disables bent-normal processing.
The default value of convexityEpsilon is zero, which uses plain edgeCos testing.

\return number of contact points found across all the GP Queries
*/
uint32_t
ComputeContactPoints(const GPInstance &gp, const GPInstance *gps, uint32_t count, const float &minimumSeparatingDistance,
                     GPInstance::ContactPoints *results, uint32_t &intersectionCount,
                     float edgeCosBendNormalThreshold,
                     float convexityEpsilon)
{
    uint32_t numIntersections;
    uint32_t numPoints;
    uint32_t i;

    numIntersections = 0;
    numPoints = 0;
    const VecFloat minimumSeparatingDistanceVec = minimumSeparatingDistance;

    for(i = 0; i < count; i++)
    {
        // get contacts for a single intersection
        uint32_t hit = ComputeContacts(gp, gps[i], results[numIntersections], minimumSeparatingDistanceVec,
                                        edgeCosBendNormalThreshold,
                                        convexityEpsilon,
                                        gDefaultTriangleFaceNormalTolerance,
                                        gDefaultFeatureSimplificationThreshold,
                                        gDefaultCosSquaredMaximumAngleConsideredParallel,
                                        gDefaultValidDirectionMinimumLengthSquared,
                                        gDefaultClippingLengthTolerance);

        if (hit)
        {
            numPoints += results[numIntersections++].numPoints;
        }
    }

    intersectionCount = numIntersections;
    return numPoints;
}
} // namespace detail


/**
\brief
Given two primitive volumes and their additional transformation matrices,
perform an intersection test between them.
Returns TRUE if the primitives intersect, FALSE otherwise.
Result structure contains the contact normal, the penetration distance, a pair of points
that represent the "center" of the intersection, and the full intersection result:
a set of pairs of points, accompanied by separations; this set of points is
what we call "feature intersection prism".

\param res results of the intersection
\param v1 first primitive collision volume
\param tm1 part to world transform.  The v1 relative transform is composed with this.
\param v2 second primitive collision volume
\param tm2 part to world transform.  The v2 relative transform is composed with this.
\param minimumSeparatingDistance the minimum separating distance for which the result is generated
\param sepDir (legacy) this parameter is ignored.
\param edgeCosBendNormalThreshold  do bent-normal processing if edgecos is below this threshold
\param convexityEpsilon  Accept the contact if the dot is within this tolerance of the edgeCos

\return true if contact points are found.  False if the distance between the two volumes is greater than minimumSeparatingDistance
*/
RwpBool
    PrimitivePairIntersect( PrimitivePairIntersectResult &res,
    const Volume *v1, const Matrix44Affine *tm1,
    const Volume *v2, const Matrix44Affine *tm2,
    float minimumSeparatingDistance, const Vector4 *sepDir,
    float edgeCosBendNormalThreshold,
    float convexityEpsilon )
{
    rwcDEPRECATED("This api will be removed next release.");
    return detail::PrimitivePairIntersect(res, v1, tm1, v2, tm2, minimumSeparatingDistance, sepDir, edgeCosBendNormalThreshold, convexityEpsilon);
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

} // namespace collision
} // namespace rw
