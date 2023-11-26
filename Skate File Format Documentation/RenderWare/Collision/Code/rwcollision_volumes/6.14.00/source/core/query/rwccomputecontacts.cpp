// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwccomputecontacts.cpp

 Purpose:

 */

// ***********************************************************************************************************
// Includes

#include "rw/collision/volume.h"
#include "rw/collision/primitivepairquery.h"
#include "rw/collision/computecontacts.h"

#include "genericcontacthandler.h"
#include "wrapcomputecontacts.h"

using namespace rwpmath;
using namespace EA::Collision;

namespace rw
{
namespace collision
{


/*
The following macros make the calls to ComputeContactPoints a little easier to read.
Notice that all unit vectors are renormalized, because otherwise the eacollision_primitives api may not work.
*/
#define SPHERE_DATA(A) gpInstance##A.Pos(), gpInstance##A.FatnessVec()
#define CAPSULE_DATA(A) gpInstance##A.Pos(), \
    NormalizeFast(static_cast<const GPCapsule&>(gpInstance##A).Axis()), \
    static_cast<const GPCapsule&>(gpInstance##A).HalfHeight(), gpInstance##A.FatnessVec()
#define TRIANGLE_DATA(A) \
    static_cast<const GPTriangle&>(gpInstance##A).Vertex0(), \
    static_cast<const GPTriangle&>(gpInstance##A).Vertex1(), \
    static_cast<const GPTriangle&>(gpInstance##A).Vertex2(), gpInstance##A.FatnessVec()
#define BOX_DATA(A) gpInstance##A.Pos(), xFace##A, yFace##A, zFace##A, \
    dim##A.X(), dim##A.Y(), dim##A.Z(), gpInstance##A.FatnessVec()
#define CYLINDER_DATA(A) gpInstance##A.Pos(), \
    NormalizeFast(static_cast<const GPCylinder&>(gpInstance##A).Axis()), \
    static_cast<const GPCylinder&>(gpInstance##A).HalfHeight(), \
    static_cast<const GPCylinder&>(gpInstance##A).Radius(), gpInstance##A.FatnessVec()
#define DECLARE_BOX_DATA(A) \
    Vector3 dim##A = static_cast<const GPBox&>(gpInstance##A).HalfSizeDimensionsVec(); \
    Vector3 xFace##A = NormalizeFast(gpInstance##A.FaceNormal(0)); \
    Vector3 yFace##A = NormalizeFast(Cross(gpInstance##A.FaceNormal(2), gpInstance##A.FaceNormal(0))); \
    Vector3 zFace##A = Cross(xFace##A, yFace##A)

/**
Compute contact points from a single pair.  typeA and typeB must both be spheres.
\return 0 on failure, non-zero on success
*/
static uint32_t
ComputeContacts_SortedGPInstancePair_Sphere(GenericContactHandler& handler,
                                            GPInstance::VolumeType typeA,
                                            GPInstance::VolumeType typeB,
                                            const GPInstance& gpInstanceA,
                                            const GPInstance& gpInstanceB,
                                            VecFloatInParam minimumSeparatingDistance,
                                            VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                                            VecFloatInParam validDirectionMinimumLengthSquared,
                                            VecFloatInParam clippingLengthTolerance)
{
    EA_ASSERT(typeA == GPInstance::SPHERE);
    EA_ASSERT(typeA == typeB);

    return ComputeContactPointsSphereSphere_Generic(handler, SPHERE_DATA(A), SPHERE_DATA(B),
        minimumSeparatingDistance, validDirectionMinimumLengthSquared);
}

/**
Compute contact points from a single pair.  typeA must be a capsule, and not less than typeB.
\return 0 on failure, non-zero on success
*/
static uint32_t
ComputeContacts_SortedGPInstancePair_Capsule(GenericContactHandler& handler,
                                             GPInstance::VolumeType typeA,
                                             GPInstance::VolumeType typeB,
                                             const GPInstance& gpInstanceA,
                                             const GPInstance& gpInstanceB,
                                             VecFloatInParam minimumSeparatingDistance,
                                             VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                                             VecFloatInParam validDirectionMinimumLengthSquared,
                                             VecFloatInParam clippingLengthTolerance)
{
    EA_ASSERT(typeA == GPInstance::CAPSULE);
    uint32_t ok = 0;

    handler.SetCapsuleA(static_cast<const GPCapsule*>(&gpInstanceA));
    if (typeB == GPInstance::SPHERE)
    {
        ok = ComputeContactPointsCapsuleSphere_Generic(handler, CAPSULE_DATA(A), SPHERE_DATA(B),
            minimumSeparatingDistance, validDirectionMinimumLengthSquared);
    }
    else
    {
        EA_ASSERT(typeA == typeB);
        handler.SetCapsuleB(static_cast<const GPCapsule*>(&gpInstanceB));
        ok = ComputeContactPointsCapsuleCapsule_Generic(handler, CAPSULE_DATA(A), CAPSULE_DATA(B),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared);
    }
    return ok;
}

/**
Compute contact points from a single pair.  typeA must be a triangle, and not less than typeB.
\return 0 on failure, non-zero on success
*/
static uint32_t
ComputeContacts_SortedGPInstancePair_Triangle(GenericContactHandler& handler,
                                              GPInstance::VolumeType typeA,
                                              GPInstance::VolumeType typeB,
                                              const GPInstance& gpInstanceA,
                                              const GPInstance& gpInstanceB,
                                              VecFloatInParam minimumSeparatingDistance,
                                              VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                                              VecFloatInParam validDirectionMinimumLengthSquared,
                                              VecFloatInParam clippingLengthTolerance)
{
    EA_ASSERT(typeA == GPInstance::TRIANGLE);
    uint32_t ok = 0;

    handler.SetTriangleA(static_cast<const GPTriangle*>(&gpInstanceA), gpInstanceB.Fatness());
    switch (typeB)
    {
    case GPInstance::SPHERE:
        ok = ComputeContactPointsTriangleSphere_Generic(handler, TRIANGLE_DATA(A), SPHERE_DATA(B),
            minimumSeparatingDistance, validDirectionMinimumLengthSquared);
        break;
    case GPInstance::CAPSULE:
        handler.SetCapsuleB(static_cast<const GPCapsule*>(&gpInstanceB));
        ok = ComputeContactPointsTriangleCapsule_Generic(handler, TRIANGLE_DATA(A), CAPSULE_DATA(B),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared);
        break;
    default:
        EA_ASSERT(typeA == typeB);
        handler.SetTriangleB(static_cast<const GPTriangle*>(&gpInstanceB), gpInstanceA.Fatness());
        ok = ComputeContactPointsTriangleTriangle_Generic(handler, TRIANGLE_DATA(A), TRIANGLE_DATA(B),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
    }

    return ok;
}

/**
Compute contact points from a single pair.  typeA must be a box, and not less than typeB.
\return 0 on failure, non-zero on success
*/
static uint32_t
ComputeContacts_SortedGPInstancePair_Box(GenericContactHandler& handler,
                                         GPInstance::VolumeType typeA,
                                         GPInstance::VolumeType typeB,
                                         const GPInstance& gpInstanceA,
                                         const GPInstance& gpInstanceB,
                                         VecFloatInParam minimumSeparatingDistance,
                                         VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                                         VecFloatInParam validDirectionMinimumLengthSquared,
                                         VecFloatInParam clippingLengthTolerance)
{
    EA_ASSERT(typeA == GPInstance::BOX);
    uint32_t ok = 0;

    DECLARE_BOX_DATA(A);
    switch (typeB)
    {
    case GPInstance::SPHERE:
        ok = ComputeContactPointsBoxSphere_Generic(handler, BOX_DATA(A), SPHERE_DATA(B),
            minimumSeparatingDistance, validDirectionMinimumLengthSquared);
        break;
    case GPInstance::CAPSULE:
        handler.SetCapsuleB(static_cast<const GPCapsule*>(&gpInstanceB));
        ok = ComputeContactPointsBoxCapsule_Generic(handler, BOX_DATA(A), CAPSULE_DATA(B),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared);
        break;
    case GPInstance::TRIANGLE:
        handler.SetTriangleB(static_cast<const GPTriangle*>(&gpInstanceB), gpInstanceA.Fatness());
        handler.SwapAB();
        ok = ComputeContactPointsTriangleBox_Generic(handler, TRIANGLE_DATA(B), BOX_DATA(A),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        break;
    default:
        EA_ASSERT(typeA == typeB);
        {
            DECLARE_BOX_DATA(B);
            ok = ComputeContactPointsBoxBox_Generic(handler, BOX_DATA(A), BOX_DATA(B),
                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        }
    }

    return ok;
}

/**
Compute contact points from a single pair.  typeA must be a cylinder, and not less than typeB.
\return 0 on failure, non-zero on success
*/
static uint32_t
ComputeContacts_SortedGPInstancePair_Cylinder(GenericContactHandler& handler,
                                              GPInstance::VolumeType typeA,
                                              GPInstance::VolumeType typeB,
                                              const GPInstance& gpInstanceA,
                                              const GPInstance& gpInstanceB,
                                              VecFloatInParam minimumSeparatingDistance,
                                              VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                                              VecFloatInParam validDirectionMinimumLengthSquared,
                                              VecFloatInParam clippingLengthTolerance)
{
    EA_ASSERT(typeA == GPInstance::CYLINDER);
    uint32_t ok = 0;

    switch (typeB)
    {
    case GPInstance::SPHERE:
        ok = ComputeContactPointsCylinderSphere_Generic(handler, CYLINDER_DATA(A), SPHERE_DATA(B),
            minimumSeparatingDistance, validDirectionMinimumLengthSquared);
        break;
    case GPInstance::CAPSULE:
        handler.SetCapsuleB(static_cast<const GPCapsule*>(&gpInstanceB));
        ok = ComputeContactPointsCylinderCapsule_Generic(handler, CYLINDER_DATA(A), CAPSULE_DATA(B),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared);
        break;
    case GPInstance::TRIANGLE:
        handler.SetTriangleB(static_cast<const GPTriangle*>(&gpInstanceB), gpInstanceA.Fatness());
        ok = ComputeContactPointsCylinderTriangle_Generic(handler, CYLINDER_DATA(A), TRIANGLE_DATA(B),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        break;
    case GPInstance::BOX:
        {
            DECLARE_BOX_DATA(B);
            ok = ComputeContactPointsCylinderBox_Generic(handler, CYLINDER_DATA(A), BOX_DATA(B),
                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        }
        break;
    default:
        EA_ASSERT(typeA == typeB);
        ok = ComputeContactPointsCylinderCylinder_Generic(handler, CYLINDER_DATA(A), CYLINDER_DATA(B),
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
    }

    return ok;
}

/**
Compute contact points from a single pair.  typeA must be not less than typeB.
\return 0 on failure, non-zero on success
*/
static uint32_t
ComputeContacts_SortedGPInstancePair(GenericContactHandler& handler,
                                     GPInstance::VolumeType typeA,
                                     GPInstance::VolumeType typeB,
                                     const GPInstance& gpInstanceA,
                                     const GPInstance& gpInstanceB,
                                     VecFloatInParam minimumSeparatingDistance,
                                     VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                                     VecFloatInParam validDirectionMinimumLengthSquared,
                                     VecFloatInParam clippingLengthTolerance)
{
    EA_ASSERT(typeA >= typeB);
    uint32_t ok = 0;

    switch (typeA)
    {
    case GPInstance::SPHERE:
        ok = ComputeContacts_SortedGPInstancePair_Sphere(handler, typeA, typeB, gpInstanceA, gpInstanceB,
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        break;
    case GPInstance::CAPSULE:
        ok = ComputeContacts_SortedGPInstancePair_Capsule(handler, typeA, typeB, gpInstanceA, gpInstanceB, 
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        break;
    case GPInstance::TRIANGLE:
        ok = ComputeContacts_SortedGPInstancePair_Triangle(handler, typeA, typeB, gpInstanceA, gpInstanceB,
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        break;
    case GPInstance::BOX:
        ok = ComputeContacts_SortedGPInstancePair_Box(handler, typeA, typeB, gpInstanceA, gpInstanceB,
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
        break;
    default:
        ok = ComputeContacts_SortedGPInstancePair_Cylinder(handler, typeA, typeB, gpInstanceA, gpInstanceB,
            minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel, validDirectionMinimumLengthSquared, clippingLengthTolerance);
    }

    return ok;
}


/* remove macros no longer needed - to make bulkbuild safer. */
#undef SPHERE_DATA
#undef CAPSULE_DATA
#undef TRIANGLE_DATA
#undef BOX_DATA
#undef CYLINDER_DATA
#undef DECLARE_BOX_DATA



/**
Compute contact points from a single pair.
\return 0 on failure, non-zero on success
*/
static uint32_t
ComputeContacts_UnsortedGPInstancePair(GenericContactHandler& handler,
                                       const GPInstance& gpInstanceA,
                                       const GPInstance& gpInstanceB,
                                       VecFloatInParam minimumSeparatingDistance,
                                       VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                                       VecFloatInParam validDirectionMinimumLengthSquared,
                                       VecFloatInParam clippingLengthTolerance)
{
    uint32_t ok;

    GPInstance::VolumeType typeA = gpInstanceA.Type();
    GPInstance::VolumeType typeB = gpInstanceB.Type();

    // Sort the pair by volume type.  This reduces the size of the switch statement.
    if (typeA < typeB)
    {
        handler.SwapAB();
        ok = ComputeContacts_SortedGPInstancePair(handler, typeB, typeA, gpInstanceB, gpInstanceA,
                                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                                validDirectionMinimumLengthSquared, clippingLengthTolerance);
    }
    else
    {
        ok = ComputeContacts_SortedGPInstancePair(handler, typeA, typeB, gpInstanceA, gpInstanceB,
                                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                                validDirectionMinimumLengthSquared, clippingLengthTolerance);
    }
    return ok;
}




/**
Compute contact points between two gp instance.
\return 1 if any contacts were generated, 0 if no contacts.
*/
uint32_t
ComputeContacts(const GPInstance& gpInstanceA, const GPInstance& gpInstanceB,
                GPInstance::ContactPoints& result,
                VecFloatInParam minimumSeparatingDistance,
                VecFloatInParam edgeCosBendNormalThreshold,
                VecFloatInParam convexityEpsilon,
                VecFloatInParam triangleFaceNormalTolerance,
                VecFloatInParam featureSimplificationThreshold,
                VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                VecFloatInParam validDirectionMinimumLengthSquared,
                VecFloatInParam clippingLengthTolerance)
{
    const uint32_t stride = 2;
    const uint32_t maxCount = sizeof(result.pointPairs) / sizeof(*result.pointPairs);

    // Assert to make sure the stride is correct.
    EA_ASSERT(&result.pointPairs[0].p1 + stride == &result.pointPairs[1].p1);

    GenericContactHandler handler(&result.normal, &result.pointPairs[0].p1, &result.pointPairs[0].p2, maxCount, stride);

    handler.SetFilterToleranceValues(edgeCosBendNormalThreshold, convexityEpsilon,
                                     triangleFaceNormalTolerance, featureSimplificationThreshold);

    uint32_t ok = ComputeContacts_UnsortedGPInstancePair(handler, gpInstanceA, gpInstanceB,
                                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                                validDirectionMinimumLengthSquared, clippingLengthTolerance);

    if (ok)
    {
        result.numPoints = handler.GetNumberOfPoints();
        result.userTag1 = gpInstanceA.mUserTag;
        result.userTag2 = gpInstanceB.mUserTag;
        result.volumeTag1 = gpInstanceA.mVolumeTag;
        result.volumeTag2 = gpInstanceB.mVolumeTag;
        return (uint32_t) (result.numPoints > 0);
    }
    return 0;
}


/**
Compute contact points between two gp instance.
\return 1 if any contacts were generated, 0 if no contacts.
*/
uint32_t
ComputeContacts(const GPInstance& gpInstanceA, const GPInstance& gpInstanceB,
                PrimitivePairIntersectResult& result,
                VecFloatInParam minimumSeparatingDistance,
                VecFloatInParam edgeCosBendNormalThreshold,
                VecFloatInParam convexityEpsilon,
                VecFloatInParam triangleFaceNormalTolerance,
                VecFloatInParam featureSimplificationThreshold,
                VecFloatInParam cosSquaredMaximumAngleConsideredParallel,
                VecFloatInParam validDirectionMinimumLengthSquared,
                VecFloatInParam clippingLengthTolerance)
{
    GenericContactHandler handler(&result.normal, result.pointsOn1, result.pointsOn2, result.MAXPOINTCOUNT);

    handler.SetFilterToleranceValues(edgeCosBendNormalThreshold, convexityEpsilon,
                                     triangleFaceNormalTolerance, featureSimplificationThreshold);

    uint32_t ok = ComputeContacts_UnsortedGPInstancePair(handler, gpInstanceA, gpInstanceB,
                                                minimumSeparatingDistance, cosSquaredMaximumAngleConsideredParallel,
                                                validDirectionMinimumLengthSquared, clippingLengthTolerance);

    if (ok)
    {
        // Fill out the rest of the data in the ppir (perhaps not all this is necessary?)

        Vector3 unitSeparatingDirectionBToA = result.normal;
        uint32_t count = handler.GetNumberOfPoints();

        result.normal = -unitSeparatingDirectionBToA;
        result.numPoints = count;

        for (uint32_t i = 0; i < count; ++i)
        {
            result.distances[i] = Dot(unitSeparatingDirectionBToA, result.pointsOn1[i] - result.pointsOn2[i]);
        }

        result.distance = result.distances[0];
        result.pointOn1 = result.pointsOn1[0];
        result.pointOn2 = result.pointsOn2[0];
        result.tag1 = gpInstanceA.mUserTag;
        result.tag2 = gpInstanceB.mUserTag;
        result.v1 = reinterpret_cast<Volume*>(gpInstanceA.mVolumeTag);
        result.v2 = reinterpret_cast<Volume*>(gpInstanceB.mVolumeTag);
        result.vNindex = 0;           // caller GPInstanceBatchIntersect will overwrite this value
        return (uint32_t) (count > 0);
    }
    return 0;
}




} // namespace collision
} // namespace rw

