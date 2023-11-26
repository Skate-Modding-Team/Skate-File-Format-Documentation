// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_PRIMITIVEPAIRQUERY_H
#define PUBLIC_RW_COLLISION_PRIMITIVEPAIRQUERY_H

/*************************************************************************************************************

 File: rwcprimitivepairquery.hpp

 Purpose: Definitions for the system for querying intersections of primitive pairs.
 */

#include "rw/collision/common.h"
#include "rw/collision/volume.h"


namespace rw
{
namespace collision
{

struct PrimitivePairIntersectResult;
struct GPInstance;

// Default values for all the tolerance parameters

#define COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance 0.f
#define COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold -1.f
#define COMPUTECONTACTS_DEFAULT_ConvexityEpsilon 0.f
#define COMPUTECONTACTS_DEFAULT_TriangleFaceNormalTolerance 0.99985f
#define COMPUTECONTACTS_DEFAULT_FeatureSimplificationThreshold 0.05f
#define COMPUTECONTACTS_DEFAULT_CosSquaredMaximumAngleConsideredParallel 0.9975f
#define COMPUTECONTACTS_DEFAULT_ValidDirectionMinimumLengthSquared 1e-5f
#define COMPUTECONTACTS_DEFAULT_ClippingLengthTolerance 1e-5f


/**
These are the default parameters that are used for calling the other low level primitive collision API:
ComputeContactPoints, PrimitivePairIntersect, GPInstanceBatchIntersectNx1, and 1xN.
You can change these if you want. The default value of these parameters is above.
*/
extern rwpmath::VecFloat gDefaultMinimumSeparatingDistance;
extern rwpmath::VecFloat gDefaultTriangleFaceNormalTolerance;
extern rwpmath::VecFloat gDefaultFeatureSimplificationThreshold;
extern rwpmath::VecFloat gDefaultCosSquaredMaximumAngleConsideredParallel;
extern rwpmath::VecFloat gDefaultValidDirectionMinimumLengthSquared;
extern rwpmath::VecFloat gDefaultClippingLengthTolerance;

//---------------------------------------------------------------------

/**
\brief
Result of an intersection test between two collision primitives.
Please note that this structure's layout isn't yet finalised and is subject to change in the future releases.

\importlib rwccore
 */
struct PrimitivePairIntersectResult
{
    /**
    */
    enum
    {
        MAXPOINTCOUNT = 16    ///< the maximum number of points to describe the intersection polygon.
    };

    const Volume            *v1;          ///< First intersecting volume
    uint32_t                tag1;         ///< First volume's tag
    const Volume            *v2;          ///< Second intersecting volume
    uint32_t                tag2;         ///< Second volume's tag
    int32_t                 vNindex;      ///< Index to the volume array for batched 1xN queries

    rwpmath::Vector3       normal;       ///< Separating normal
    rwpmath::Vector3       pointOn1;     ///< DEPRECATED - this will be removed next release
    rwpmath::Vector3       pointOn2;     ///< DEPRECATED - this will be removed next release
    float                distance;     ///< DEPRECATED - this will be removed next release

    rwpmath::Vector3       pointsOn1[MAXPOINTCOUNT]; ///< Points on v1 that correspond to points on v2 and distances
                                                      /// between the points along the direction defined by normal
    rwpmath::Vector3       pointsOn2[MAXPOINTCOUNT]; ///< Points on v2 that correspond to points on v1 and distances
                                                      ///  between the points along the direction defined by normal
    float                distances[MAXPOINTCOUNT]; ///< DEPRECATED - this will be removed next release
    uint32_t               numPoints;    ///< The total number of intersection points produced

    /**
    \internal
    */
    inline PrimitivePairIntersectResult()
    {
    }
};

namespace detail {
#if       !defined(EA_PLATFORM_PS3_SPU)
/**
\internal
Given a single primitive and a batch of "other" primitives, test them 1-versus-N
and put the results into the buffer provided by the caller.
Returns the total number of intersections found.
*/
int32_t
GPInstanceBatchIntersect1xN( PrimitivePairIntersectResult *res,
                             int32_t resBufMaxSize,
                             const GPInstance &inst1, const GPInstance *inst2, int32_t num,
                             float padding = COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance,
                             float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                             float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon );

int32_t
GPInstanceBatchIntersectNx1( PrimitivePairIntersectResult *resBuf,
                            int32_t resBufMaxSize,
                            const GPInstance *insts1, int32_t num, const GPInstance &inst2,
                            float padding  = COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance,
                            float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                            float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon );

/**
\internal
*/
void
PrimitiveBatchInstance( GPInstance *resBuf,
                        const Volume *vN, const rwpmath::Matrix44Affine *tmN, int32_t num );

/**
\internal
*/
void
PrimitiveBatchInstance( GPInstance *resBuf,
                        const Volume *vN, const rwpmath::Matrix44Affine **tmN, int32_t num );

/**
\internal
*/
RwpBool PrimitivePairIntersect( PrimitivePairIntersectResult &res,
                               const Volume *v1, const rwpmath::Matrix44Affine *tm1,
                               const Volume *v2, const rwpmath::Matrix44Affine *tm2,
                               float padding = COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance, 
                               const rwpmath::Vector4 *sepDir = NULL,
                               float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                               float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon);

/**
\internal
*/
int32_t
PrimitiveBatchIntersect1xN( PrimitivePairIntersectResult *res,
                            int32_t resBufMaxSize,
                            GPInstance *instancingSPR,
                            const Volume *v1, const rwpmath::Matrix44Affine *tm1,
                            const Volume *vN, const rwpmath::Matrix44Affine *tmN, int32_t num,
                            float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                            float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon);

/**
\internal
Given a single volume and a batch of "other" volumes, test them 1-versus-N
and put the results into the buffer provided by the caller.
This version accepts arrays of pointers for the "other" volumes and their TMs,
so there is no assumption made about the arrangement of the volumes or their TMs in memory.
Returns the total number of intersections found.
*/
int32_t
PrimitiveBatchIntersect1xN( PrimitivePairIntersectResult *res,
                            int32_t resBufMaxSize,
                            GPInstance *instancingSPR,
                            const Volume *v1, const rwpmath::Matrix44Affine *tm1,
                            const Volume **vN, const rwpmath::Matrix44Affine **tmN, int32_t num,
                            float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                            float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon );

/**
\internal
Given two batches of volumes, test them N-versus-M
and put the results into the buffer provided by the caller.
This version accepts arrays of pointers for the volumes and their TMs,
so there is no assumption made about the arrangement of the volumes or their TMs in memory.
Returns the total number of intersections found.
*/
int32_t
PrimitiveBatchIntersectNxM( PrimitivePairIntersectResult *res,
                            int32_t resBufMaxSize,
                            GPInstance *instancingSPR,
                            const Volume **vN, const rwpmath::Matrix44Affine **tmN, int32_t numN,
                            const Volume **vM, const rwpmath::Matrix44Affine **tmM, int32_t numM,
                            float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                            float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon );

/**
\internal
*/
int32_t
PrimitiveBatchIntersect( PrimitivePairIntersectResult *resBuf,
                         int32_t resBufMaxSize,
                         GPInstance *instancingSPR,
                         VolRefPair *pairs,
                         int32_t numPairs,
                         float padding = COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance,
                         float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                         float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon );


#endif // !defined(EA_PLATFORM_PS3_SPU)



// intersection query API suitable for generating contact constraints
uint32_t ComputeContactPoints(const GPInstance &gp1, const GPInstance &gp2,                 
                              const float &padding, GPInstance::ContactPoints &result, 
                              float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold, 
                              float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon);


uint32_t ComputeContactPoints(const GPInstance &gp,  const GPInstance *gps, uint32_t count, const float &padding, 
                              GPInstance::ContactPoints *results, uint32_t &numIntersections,
                              float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold, 
                              float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon);

}

#if !defined(EA_PLATFORM_PS3_SPU)
/**
\internal
*/
RwpBool PrimitivePairIntersect( PrimitivePairIntersectResult &res,
                               const Volume *v1, const rwpmath::Matrix44Affine *tm1,
                               const Volume *v2, const rwpmath::Matrix44Affine *tm2,
                               float padding = COMPUTECONTACTS_DEFAULT_MinimumSeparatingDistance, 
                               const rwpmath::Vector4 *sepDir = NULL,
                               float edgeCosBendNormalThreshold = COMPUTECONTACTS_DEFAULT_EdgeCosBendNormalThreshold,
                               float convexityEpsilon = COMPUTECONTACTS_DEFAULT_ConvexityEpsilon);
#endif // !defined(EA_PLATFORM_PS3_SPU)


// The following API are deprecated. 
float SetTriangleEdgeCullingTolerance(float);
float SetTriangleFaceNormalTolerance(float);

} // namespace collision
} // namespace rw
#endif // PUBLIC_RW_COLLISION_PRIMITIVEPAIRQUERY_H
