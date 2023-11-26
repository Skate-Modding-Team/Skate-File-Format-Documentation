// (c) Electronic Arts. All Rights Reserved.
#ifndef RW_COLLISION_DETAIL_GENERICCONTACTHANDLER_H
#define RW_COLLISION_DETAIL_GENERICCONTACTHANDLER_H

/*************************************************************************************************************

 File: genericcontacthandler.h

 Purpose: Used by rwccomputecontacts.cpp to call the eacollision_primitives api
*/


#include "rw/collision/volume.h"

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(push)
#pragma warning(disable: 4714)  // "function marked as __forceinline not inlined" may occur from eacollision headers
#endif

#include "eacollision/features/contactfiltering/filtertrianglecontact_branching.h"
#include "eacollision/features/contactfiltering/filtertrianglecontact_branchless.h"

#include <eacollision_features/version.h>
#if EACOLLISION_FEATURES_VERSION >= EACOLLISION_FEATURES_CREATE_VERSION_NUMBER(1,7,00)
#include <eacollision/features/contactfiltering/filtercapsulecontact_branchless.h>
#endif

#if defined(EA_PLATFORM_WINDOWS)
#pragma warning(pop)
#endif

namespace rw
{
namespace collision
{

// Little utility to exchange two values of any type (that supports assignment)
template <typename T>
void Swap(T& a, T& b)
{
    T temp = a;
    a = b;
    b = temp;
}

/**
This is a contact handler that stores contacts into an array in memory.

By default pointsOnA and pointsOnB are stored separately (SOA).  However they may also be stored alternately (AOS) by
indicating an appropriate "stride" number.

This class also performs triangle filtering and normal bending.  To use it, you must call SetTriangleA() 
and/or SetTriangleB() before BeginContact(). Call SetFilterToleranceValues() to specify non-default filter tolerances.

It will also perform capsule vertex filtering. To enable this, you must call SetCapsuleA() and/or SetCapsuleB()
before BeginContact().

The class is meant for single use only, i.e. BeginContact() must never be called twice on the same object.
*/
class GenericContactHandler
{
public:
    /**
    Constructor

    \param normalBToA pointer to result storage for contact normal
    \param pointsOnA pointer to result storage for points on A
    \param pointsOnB pointer to result storage for points on B
    \param maxCount the max number of point pairs that can be stored.  USED FOR ASSERT ONLY
    \param strideMultiplier the amount of increment of the array pointers in increments of sizeof(Vector3).
        For example, if the pointsOnA are in a single array (SOA) then the strideMultiplier is 1.  But if the
        points are stored alternately A0,B0,A1,B1, etc,  then the strideMultiplier is 2.
    */
    GenericContactHandler(rwpmath::Vector3 *normalBToA,
                          rwpmath::Vector3 *pointsOnA,
                          rwpmath::Vector3 *pointsOnB,
                          uint32_t maxCount,
                          uint32_t strideMultiplier = 1)
        :   mSeparatingDirectionBToA(normalBToA),
            mPointsOnA(pointsOnA),
            mPointsOnB(pointsOnB),
            mMaxCount(maxCount),
            mStride(strideMultiplier)
    {
        EA_UNUSED(mMaxCount);
        mFlipNormal = rwpmath::GetVecFloat_One();
        mCount = 0;
        mTriangleA = mTriangleB = 0;
        mCapsuleA = mCapsuleB = 0;
        mNeedsNormalBendingA = mNeedsNormalBendingB = false;
    }

    /**
    Enable triangle filtering for A.  You must call this before BeginContact.
    Also you need to specify the radius of the OTHER shape.
    */
    void SetTriangleA(const GPTriangle *triangleA, rwpmath::VecFloatInParam radiusB)
    {
        mTriangleA = triangleA;
        mRadiusB = radiusB;
    }

    /**
    Enable triangle filtering for B.  You must call this before BeginContact.
    Also you need to specify the radius of the OTHER shape.
    */
    void SetTriangleB(const GPTriangle *triangleB, rwpmath::VecFloatInParam radiusA)
    {
        mTriangleB = triangleB;
        mRadiusA = radiusA;
    }

    /**
    Enable capsule vertex filtering for A.  You must call this before BeginContact.
    */
    void SetCapsuleA(const GPCapsule *capsuleA)
    {
        mCapsuleA = capsuleA;
    }

    /**
    Enable capsule vertex filtering for B.  You must call this before BeginContact.
    */
    void SetCapsuleB(const GPCapsule *capsuleB)
    {
        mCapsuleB = capsuleB;
    }

    /**
    Set tolerance values for triangle filtering.
    You must call this before BeginContact if either instance is a triangle.
    */
    void SetFilterToleranceValues(rwpmath::VecFloatInParam edgeCosBendNormalThreshold,
                                  rwpmath::VecFloatInParam convexityEpsilon,
                                  rwpmath::VecFloatInParam triangleFaceNormalTolerance = 0.99985f,
                                  rwpmath::VecFloatInParam featureSimplificationThreshold = 0.05f)
    {
        mEdgeCosBendNormalThreshold = edgeCosBendNormalThreshold;
        mConvexityEpsilon = convexityEpsilon;
        mTriangleFaceNormalTolerance = triangleFaceNormalTolerance;
        mFeatureSimplificationThreshold = featureSimplificationThreshold;
    }

    void GetTriangleFilteringMasksFromFlags(rwpmath::Mask3 &disableVertices,
                                            rwpmath::Mask3 &edgeIsConvex,
                                            rwpmath::MaskScalar &oneSided,
                                            uint32_t triangleFlags);

    bool RejectContactNormal(const GPTriangle& triangle,
                            rwpmath::Vector3::InParam contactNormalTowardsTriangle,
                            bool& needsNormalBending);

    bool RejectContactPoint(const GPCapsule& capsule,
                            rwpmath::Vector3::InParam point);

    //--------------------------------------------------------
    // methods called in ccp

    unsigned int BeginContact(rwpmath::Vector3::InParam unitContactDirectionBToA);

    void BeginContactQuick(rwpmath::Vector3::InParam unitContactDirectionBToA);

    void AddPoint(rwpmath::Vector3::InParam unitContactDirectionBToA,
                    rwpmath::Vector3::InParam contactPointOnA,
                    rwpmath::Vector3::InParam contactPointOnB);

    void AddPointConditional(rwpmath::Vector3::InParam unitContactDirectionBToA,
                            rwpmath::Vector3::InParam contactPointOnA,
                            rwpmath::Vector3::InParam contactPointOnB,
                            rwpmath::MaskScalar::InParam contactPointReturned);

    void AddPointQuick(rwpmath::Vector3::InParam contactPointOnA,
                       rwpmath::Vector3::InParam contactPointOnB,
                       rwpmath::MaskScalar::InParam contactPointReturned);

    /**
    Post process contacts. (nothing to do)
    */
    void EndContact() {}

    /** Get number of contact points added
    */
    uint32_t GetNumberOfPoints()
    {
        return mCount;
    }

    /**
    Swap all the A fields with all the B fields
    */
    void SwapAB()
    {
        Swap(mPointsOnA, mPointsOnB);
        Swap(mTriangleA, mTriangleB);
        Swap(mRadiusA, mRadiusB);
        mFlipNormal *= rwpmath::GetVecFloat_NegativeOne();
    }

private:
    // data
    rwpmath::VecFloat mFlipNormal;
    rwpmath::VecFloat mEdgeCosBendNormalThreshold;
    rwpmath::VecFloat mConvexityEpsilon;
    rwpmath::VecFloat mTriangleFaceNormalTolerance;
    rwpmath::VecFloat mFeatureSimplificationThreshold;
    rwpmath::VecFloat mRadiusA;
    rwpmath::VecFloat mRadiusB;
    rwpmath::Vector3* mSeparatingDirectionBToA;
    rwpmath::Vector3* mPointsOnA;
    rwpmath::Vector3* mPointsOnB;
    uint32_t mMaxCount;
    uint32_t mStride;
    uint32_t mCount;
    bool mNeedsNormalBendingA;
    bool mNeedsNormalBendingB;
    const GPTriangle *mTriangleA;
    const GPTriangle *mTriangleB;
    const GPCapsule *mCapsuleA;
    const GPCapsule *mCapsuleB;
};


/*******************************************************************************************************
    Inline methods
*******************************************************************************************************/

/**
Input triangleFlags, Output Mask values representing the flags.
*/
inline void
GenericContactHandler::GetTriangleFilteringMasksFromFlags(rwpmath::Mask3 &disableVertices,
                                                          rwpmath::Mask3 &edgeIsConvex,
                                                          rwpmath::MaskScalar &oneSided,
                                                          uint32_t triangleFlags)
{
    disableVertices = rwpmath::Mask3(
        (triangleFlags & GPInstance::FLAG_TRIANGLEVERT0DISABLE) != 0,
        (triangleFlags & GPInstance::FLAG_TRIANGLEVERT1DISABLE) != 0,
        (triangleFlags & GPInstance::FLAG_TRIANGLEVERT2DISABLE) != 0);
    edgeIsConvex = rwpmath::Mask3(
        (triangleFlags & GPInstance::FLAG_TRIANGLEEDGE0CONVEX) != 0,
        (triangleFlags & GPInstance::FLAG_TRIANGLEEDGE1CONVEX) != 0,
        (triangleFlags & GPInstance::FLAG_TRIANGLEEDGE2CONVEX) != 0);
    oneSided = rwpmath::MaskScalar(
        (triangleFlags & GPInstance::FLAG_TRIANGLEONESIDED) != 0u);
}



/**
Test whether the contactNormal should be rejected (filtered away) based on the flags of the given triangle.
This is called by BeginContact.  The triangle will be either A or B.
*/
inline bool
GenericContactHandler::RejectContactNormal(const GPTriangle& triangle,
                                            rwpmath::Vector3::InParam contactNormalTowardsTriangle,
                                            bool& needsNormalBending)
{
    rwpmath::Mask3 triangleDisableVertices;
    rwpmath::Mask3 triangleEdgeIsConvex;
    rwpmath::MaskScalar triangleOneSided;
    rwpmath::MaskScalar rejectContact;

    uint32_t triangleFlags = triangle.Flags();
    GetTriangleFilteringMasksFromFlags(triangleDisableVertices, triangleEdgeIsConvex, triangleOneSided, triangleFlags);
    rwpmath::Mask3 triangleFeature = EA::Collision::ContactFiltering::ComputeFeatureFromDirection_Branching(
                            contactNormalTowardsTriangle,
                            triangle.Vertex0(), triangle.Vertex1(), triangle.Vertex2(),
                            mTriangleFaceNormalTolerance, mFeatureSimplificationThreshold);

    if (triangleFlags & GPInstance::FLAG_TRIANGLEUSEEDGECOS)
    {
        rwpmath::MaskScalar needsNormalBendingMask;
        const rwpmath::Vector3 negOne(rwpmath::GetVecFloat_NegativeOne());
        const rwpmath::Vector3 negTwo(rwpmath::GetVecFloat_NegativeTwo());
        rwpmath::Vector3 edgeCosines = triangle.EdgeCosines();
        // Note: the following line should not be necessary. see hansoft bug#85
        edgeCosines = rwpmath::Select(rwpmath::CompEqual(edgeCosines, negOne), negTwo, edgeCosines);
        rejectContact = EA::Collision::ContactFiltering::FilterTriangleContactByEdgeCosines_Branching(
                            needsNormalBendingMask, contactNormalTowardsTriangle, triangleFeature,
                            triangle.Vertex0(), triangle.Vertex1(), triangle.Vertex2(), edgeCosines,
                            triangleEdgeIsConvex, triangleDisableVertices, triangleOneSided,
                            mEdgeCosBendNormalThreshold, mConvexityEpsilon);
        needsNormalBending = needsNormalBendingMask.GetBool();
    }
    else
    {
        rejectContact = EA::Collision::ContactFiltering::FilterTriangleContact_Branching(
                            contactNormalTowardsTriangle, triangleFeature,
                            triangle.Vertex0(), triangle.Vertex1(), triangle.Vertex2(),
                            triangleEdgeIsConvex, triangleDisableVertices, triangleOneSided);
    }

    return rejectContact.GetBool();
}

#if EACOLLISION_FEATURES_VERSION >= EACOLLISION_FEATURES_CREATE_VERSION_NUMBER(1,7,00)
inline bool GenericContactHandler::RejectContactPoint(const GPCapsule& capsule,
                                                      rwpmath::Vector3::InParam point)
{
    const rwpmath::Vector3 centre = capsule.Center();
    const rwpmath::Vector3 unitAxis = capsule.Axis();
    const rwpmath::VecFloat halfHeight = capsule.HalfHeight();
    const uint32_t flags = capsule.Flags();
    const rwpmath::Mask2 disableVertices((flags & GPInstance::FLAG_TRIANGLEVERT0DISABLE) != 0,(flags & GPInstance::FLAG_TRIANGLEVERT1DISABLE) != 0);
    return 
        EA::Collision::ContactFiltering::FilterCapsuleContact_Branchless(point,
            centre, unitAxis, halfHeight, disableVertices).GetBool();
}
#else
inline bool GenericContactHandler::RejectContactPoint(const GPCapsule& /* capsule */,
                                                      rwpmath::Vector3::InParam /* point */)
{
    return false;
}
#endif

/**
Store the contact direction.  Does NOT apply triangle filters.
*/
inline void
GenericContactHandler::BeginContactQuick(rwpmath::Vector3::InParam unitContactDirectionBToA)
{
    EA_ASSERT(mCount == 0);
    *mSeparatingDirectionBToA = unitContactDirectionBToA * mFlipNormal;
}

/**
Test if the contact direction is valid.  Apply triangle filters.
*/
inline unsigned int
GenericContactHandler::BeginContact(rwpmath::Vector3::InParam unitContactDirectionBToA)
{
    EA_ASSERT(mCount == 0);
    *mSeparatingDirectionBToA = unitContactDirectionBToA * mFlipNormal;

    if (mTriangleA && RejectContactNormal(*mTriangleA, unitContactDirectionBToA, mNeedsNormalBendingA))
    {
        return 0u;
    }

    if (mTriangleB)
    {
        if (RejectContactNormal(*mTriangleB, -unitContactDirectionBToA, mNeedsNormalBendingB))
        {
            return 0u;
        }
        if (mNeedsNormalBendingA && mNeedsNormalBendingB)
        {
            // If both triangles are trying to bend the contact normal, then reject the contact.
            return 0u;
        }
    }
    return 1u;
}

/**
Add a contact point to memory.  Increment pointers and counter.
Apply normal bending if necessary.
*/
inline void
GenericContactHandler::AddPoint(rwpmath::Vector3::InParam unitContactDirectionBToA,
                                rwpmath::Vector3::InParam contactPointOnA,
                                rwpmath::Vector3::InParam contactPointOnB)
{
    if (mCapsuleA && RejectContactPoint(*mCapsuleA, contactPointOnA))
    {
        return;
    }

    if (mCapsuleB && RejectContactPoint(*mCapsuleB, contactPointOnB))
    {
        return;
    }

    EA_ASSERT_MSG(mCount < mMaxCount, "Insufficient memory provided for contacts between primitive pair.");

    if (mNeedsNormalBendingA)
    {
        // the instanceA is a triangle, and we have to adjust the points to be collinear with its normal

        rwpmath::Vector3 faceNormal = mTriangleA->Normal();
        rwpmath::Vector3 nonfatPointOnB = contactPointOnB - unitContactDirectionBToA * mRadiusB;

        *mPointsOnA = nonfatPointOnB - faceNormal * rwpmath::Dot(faceNormal, nonfatPointOnB - contactPointOnA);
        *mPointsOnB = nonfatPointOnB - faceNormal * mRadiusB;
    }
    else if (mNeedsNormalBendingB)
    {
        // the instanceB is a triangle, and we have to adjust the points to be collinear with its normal

        rwpmath::Vector3 faceNormal = mTriangleB->Normal();
        rwpmath::Vector3 nonfatPointOnA = contactPointOnA + unitContactDirectionBToA * mRadiusA;

        *mPointsOnA = nonfatPointOnA - faceNormal * mRadiusA;
        *mPointsOnB = nonfatPointOnA - faceNormal * rwpmath::Dot(faceNormal, nonfatPointOnA - contactPointOnB);
    }
    else
    {
        *mPointsOnA = contactPointOnA;
        *mPointsOnB = contactPointOnB;
    }
    mPointsOnA += mStride;
    mPointsOnB += mStride;
    ++mCount;
}


/**
Add a contact point to memory location id if "contactPointReturned" == TRUE.  Increment id on success.
Apply normal bending if necessary.
*/
inline void
GenericContactHandler::AddPointConditional(rwpmath::Vector3::InParam unitContactDirectionBToA,
                                           rwpmath::Vector3::InParam contactPointOnA,
                                           rwpmath::Vector3::InParam contactPointOnB,
                                           rwpmath::MaskScalar::InParam contactPointReturned)
{
    if (contactPointReturned.GetBool())
    {
        AddPoint(unitContactDirectionBToA, contactPointOnA, contactPointOnB);
    }
}

/**
Add a contact point to memory location id if "contactPointReturned" == TRUE.  Increment id on success.
Do not test for normal bending.  Use with BeginContactQuick.  Does not require EndContact.
*/
inline void
GenericContactHandler::AddPointQuick(rwpmath::Vector3::InParam contactPointOnA,
                                     rwpmath::Vector3::InParam contactPointOnB,
                                     rwpmath::MaskScalar::InParam contactPointReturned)
{
    if (contactPointReturned.GetBool())
    {
        *mPointsOnA = contactPointOnA;
        *mPointsOnB = contactPointOnB;
        mPointsOnA += mStride;
        mPointsOnB += mStride;
        ++mCount;
    }
}

} // namespace collision
} // namespace rw

#endif // RW_COLLISION_DETAIL_GENERICCONTACTHANDLER_H
