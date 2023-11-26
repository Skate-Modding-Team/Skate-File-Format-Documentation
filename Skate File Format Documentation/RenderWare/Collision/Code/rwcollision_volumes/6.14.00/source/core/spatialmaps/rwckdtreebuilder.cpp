// (c) Electronic Arts. All Rights Reserved.

#include <rw/math/math.h>
#include <rw/collision/aabbox.h>
#include <rw/collision/kdtree.h>
#include <rw/collision/kdtreebuilder.h>

#include <coreallocator/icoreallocator.h>

#include <new>    // for placement new

#include <stdlib.h>
#include <cstdio>

using namespace rwpmath;
using namespace rw::collision;

namespace rw
{
namespace collision
{

// TODO: these thresholds need to be parameters

/**
Splits if split cost is less than this value (0-1)
*/
#define rwcKDTREEBUILD_SPLIT_COST_THRESHOLD     0.95f

/**
Splits off empty leaf if extent is greater than this fraction of parent
*/
#define rwcKDTREEBUILD_EMPTY_LEAF_THRESHOLD     0.6f

/**
\internal
\brief
Axis aligned KD tree splitting plane.
*/
struct KDTreeSplit
{
    KDTreeSplit()
        : m_axis(0)
        , m_value(0.0f)
        , m_numLeft(0)
        , m_numRight(0)
        , m_leftBBox(AABBox::Vector3Type(0.0f, 0.0f, 0.0f),AABBox::Vector3Type(0.0f, 0.0f, 0.0f))
        , m_rightBBox(AABBox::Vector3Type(0.0f, 0.0f, 0.0f), AABBox::Vector3Type(0.0f, 0.0f, 0.0f))
    {
    }

    uint32_t            m_axis;      ///< Split axis.
    rwpmath::VecFloat   m_value;     ///< Position of split along axis.
    uint32_t            m_numLeft;   ///< Number on left of split.
    uint32_t            m_numRight;  ///< Number on right of split.
    AABBox              m_leftBBox;  ///<
    AABBox              m_rightBBox; ///<
};

struct KDTreeMultiAxisSplit
{
    KDTreeMultiAxisSplit()
    {
        m_value = rwpmath::GetVector3_Zero();
        m_numLeft = rwpmath::GetVector3_Zero();
        m_numRight = rwpmath::GetVector3_Zero();
    };

    rwpmath::Vector3 m_value;           ///< Position of split along axis.
    rwpmath::Vector3 m_numLeft;         ///< Number on left of split.
    rwpmath::Vector3 m_numRight;        ///< Number on right of split.
    AABBox           m_leftBBox[3];     ///< AABBoxes on left side of split
    AABBox           m_rightBBox[3];    ///< AABBoxes on right side of split
};

struct Entry
{
public:
    Entry & operator=(const Entry &other)
    {
        this->entryIndex = other.entryIndex;
        this->entryBBoxSurfaceArea = other.entryBBoxSurfaceArea;

        return *this;
    }

    uint32_t entryIndex;            ///< Entry Index
    float    entryBBoxSurfaceArea;  ///< ENtry BBox SurfaceArea
};

// ***********************************************************************************************************
// Inlined Functions

/**
\internal
\brief
Swaps two entries in a given array and adjusts the associated indices accordingly.

If a swap takes place the rightIndex is decremented. If a swap doesn't take place
the leftIndex is incremented.

\param swap                 Flag indicating if a swap need to take place
\param entries              Entries in this node, sorted into left and right
groups on completion.
\param rightIndex           Right index of entries
\param rightIndex           Left index of entries
*/
EA_FORCE_INLINE static void
rwc_SwapEntriesAndAdjustIndices(const bool swap,
                                Entry       *entries,
                                int32_t    &rightIndex,
                                int32_t    &leftIndex)
{
    if (swap)
    {
        // Swap with right
        Entry tempEntry = entries[leftIndex];
        entries[leftIndex] = entries[rightIndex];
        entries[rightIndex] = tempEntry;
        rightIndex--;
    }
    else
    {
        leftIndex++;
    }
}


/**
\internal
\brief Return surface area of a bounding box.
\param bbox Bounding box
\return Surface area of bounding box
*/
EA_FORCE_INLINE static rwpmath::VecFloat
rwc_BBoxSurfaceArea(const AABBox &bbox)
{
    rwpmath::Vector3 diag = bbox.Max() - bbox.Min();
    rwpmath::VecFloat area = rwpmath::GetVecFloat_Two() * (diag.GetX()*diag.GetY() + diag.GetY()*diag.GetZ() + diag.GetZ()*diag.GetX());

    return area;
}


/**
\internal
\brief Return surface area of a bounding box.
\param bbox Bounding box
\return Surface area of bounding box
*/
EA_FORCE_INLINE static rw::math::fpu::VecFloat
rwc_BBoxSurfaceAreaU(const AABBoxU &bbox)
{
    rw::math::fpu::Vector3U_32 diag = bbox.Max() - bbox.Min();
    rw::math::fpu::VecFloat area = rw::math::fpu::GetVecFloat_Two() * (diag.GetX()*diag.GetY() + diag.GetY()*diag.GetZ() + diag.GetZ()*diag.GetX());

    return area;
}


/**
\internal
\brief
Get statistics for a given node split along the 3 principal axis.

\param axisComparison       Mask used to update statistics
\param bb                   Outer extent of node
\param rightCount           Count of right items
\param leftCount            Count of left items
\param leftBBoxX            Extents of x-axis left bounding box
\param rightBBoxX           Extents of x-axis right bounding box
\param leftBBoxY            Extents of y-axis left bounding box
\param rightBBoxY           Extents of y-axis right bounding box
\param leftBBoxZ            Extents of z-axis left bounding box
\param rightBBoxZ           Extents of z-axis right bounding box
*/
EA_FORCE_INLINE static void
rwc_UpdateSplitStats(rwpmath::Mask3::InParam       axisComparison,
                     rwpmath::Vector3::InParam     entryBBoxMin,
                     rwpmath::Vector3::InParam     entryBBoxMax,
                     rwpmath::Vector3::InOutParam  rightCount,
                     rwpmath::Vector3::InOutParam  leftCount,
                     rw::collision::AABBox         &leftBBoxX,
                     rw::collision::AABBox         &rightBBoxX,
                     rw::collision::AABBox         &leftBBoxY,
                     rw::collision::AABBox         &rightBBoxY,
                     rw::collision::AABBox         &leftBBoxZ,
                     rw::collision::AABBox         &rightBBoxZ)
{
    // Adjust each of the split left/right counts
    rightCount += rwpmath::Select(axisComparison, rwpmath::GetVector3_One(), rwpmath::GetVector3_Zero());
    leftCount += rwpmath::Select(axisComparison, rwpmath::GetVector3_Zero(), rwpmath::GetVector3_One());

    // Adjust each of the split AABBoxes

    // X-Axis split

    // Generate the new Min and Max AABBox values
    // Left AABBox
    rwpmath::Vector3 newLeftMin = Min(leftBBoxX.Min(), entryBBoxMin);
    rwpmath::Vector3 newLeftMax = Max(leftBBoxX.Max(), entryBBoxMax);
    // Right AABBox
    rwpmath::Vector3 newRightMin = Min(rightBBoxX.Min(), entryBBoxMin);
    rwpmath::Vector3 newRightMax = Max(rightBBoxX.Max(), entryBBoxMax);

    // Adjust the AABBoxes with the new Min and Max values, if they require updating
    // Left AABBox
    leftBBoxX.m_min = Select(axisComparison.GetX(), leftBBoxX.Min(), newLeftMin);
    leftBBoxX.m_max = Select(axisComparison.GetX(), leftBBoxX.Max(), newLeftMax);
    // Right AABBox
    rightBBoxX.m_min = Select(axisComparison.GetX(), newRightMin, rightBBoxX.Min());
    rightBBoxX.m_max = Select(axisComparison.GetX(), newRightMax, rightBBoxX.Max());

    // Y-Axis split

    // Generate the new Min and Max AABBox values
    // Left AABBox
    newLeftMin = Min(leftBBoxY.Min(), entryBBoxMin);
    newLeftMax = Max(leftBBoxY.Max(), entryBBoxMax);
    // Right AABBox
    newRightMin = Min(rightBBoxY.Min(), entryBBoxMin);
    newRightMax = Max(rightBBoxY.Max(), entryBBoxMax);

    // Adjust the AABBoxes with the new Min and Max values, if they require updating
    // Left AABBox
    leftBBoxY.m_min = Select(axisComparison.GetY(), leftBBoxY.Min(), newLeftMin);
    leftBBoxY.m_max = Select(axisComparison.GetY(), leftBBoxY.Max(), newLeftMax);
    // Right AABBox
    rightBBoxY.m_min = Select(axisComparison.GetY(), newRightMin, rightBBoxY.Min());
    rightBBoxY.m_max = Select(axisComparison.GetY(), newRightMax, rightBBoxY.Max());

    // Z-Axis split

    // Generate the new Min and Max AABBox values
    // Left AABBox
    newLeftMin = Min(leftBBoxZ.Min(), entryBBoxMin);
    newLeftMax = Max(leftBBoxZ.Max(), entryBBoxMax);
    // Right AABBox
    newRightMin = Min(rightBBoxZ.Min(), entryBBoxMin);
    newRightMax = Max(rightBBoxZ.Max(), entryBBoxMax);

    // Adjust the AABBoxes with the new Min and Max values, if they require updating
    // Left AABBox
    leftBBoxZ.m_min = Select(axisComparison.GetZ(), leftBBoxZ.Min(), newLeftMin);
    leftBBoxZ.m_max = Select(axisComparison.GetZ(), leftBBoxZ.Max(), newLeftMax);
    // Right AABBox
    rightBBoxZ.m_min = Select(axisComparison.GetZ(), newRightMin, rightBBoxZ.Min());
    rightBBoxZ.m_max = Select(axisComparison.GetZ(), newRightMax, rightBBoxZ.Max());
}

// ***********************************************************************************************************
// Static Functions

/**
\internal
\brief
Sorts the entries for a given node, along a specific split axis.

\param split                KDTree split
\param nodeBB               Outer extent of node
\param entryBBoxes          Bounding boxes of all entries
\param entries              Entries in this node, sorted into left and right
groups on completion.
\param numEntries           Number of objects in this node
*/
static void
rwc_SortSplitEntries(KDTreeSplit      &split,
                     const AABBoxU    *entryBBoxes,
                     Entry            *entries,
                     uint32_t          numEntries)
{
    // Sort entry indices into left and right groups, accumulating bboxes
    int32_t iLeft = 0;
    int32_t iRight = (int32_t) numEntries - 1;
    while (iLeft <= iRight)
    {
        const AABBoxU& bb = entryBBoxes[entries[iLeft].entryIndex];

        // Min and Max of current entry AABBox
#if RWPMATH_IS_VPU
        const rwpmath::Vector3 minExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_min, 0));
        const rwpmath::Vector3 maxExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_max, 0));
#else
        const rwpmath::Vector3 minExtent(bb.m_min);
        const rwpmath::Vector3 maxExtent(bb.m_max);
#endif

        // Center point of current entry AABBox
        const rwpmath::Vector3 center = (minExtent + maxExtent) * rwpmath::GetVecFloat_Half();

        const rwpmath::VecFloat centerAxis = center.GetComponent((int)split.m_axis);

        // Create a mask to select center components which are greater then the node AABBox components
        rwpmath::MaskScalar axisComparison = rwpmath::CompGreaterThan(centerAxis, split.m_value);

        bool swap = axisComparison.GetBool();

        rwc_SwapEntriesAndAdjustIndices(swap,
                                        entries,
                                        iRight,
                                        iLeft);
    }

    EA_ASSERT_MSG(split.m_numLeft == (uint32_t) iLeft, ("Count of entries on left of split does not match."));
    EA_ASSERT_MSG(split.m_numRight == numEntries - iLeft, ("Count of entries on right of split does not match"));
}


/**
\internal
\brief
Sorts the entries for a given node, along a specific split axis.

\param split                KDTree split
\param nodeBB               Outer extent of node
\param entryBBoxes          Bounding boxes of all entries
\param entries              Entries in this node, sorted into left and right
groups on completion.
\param numEntries           Number of objects in this node
\param largeItemThreshold   the threshold above which items are considered to be large
*/
static void
rwc_SortSplitEntriesLargeItems(KDTreeSplit                &split,
                               const AABBox               &nodeBB,
                               const AABBoxU              *entryBBoxes,
                               Entry                      *entries,
                               uint32_t                   numEntries,
                               const float                largeItemThreshold)
{
    // Sort entry indices into left and right groups, accumulating bboxes

    const rw::math::fpu::VecFloat nodeSize(nodeBB.Max().GetComponent((int)split.m_axis) - nodeBB.Min().GetComponent((int)split.m_axis));
    const rw::math::fpu::VecFloat thresholdSize = nodeSize * largeItemThreshold;

    int32_t iLeft = 0;
    int32_t iRight = (int32_t) numEntries - 1;
    while (iLeft <= iRight)
    {
        const AABBoxU& bb = entryBBoxes[entries[iLeft].entryIndex];
#if RWPMATH_IS_VPU
        const rwpmath::Vector3 minExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_min, 0));
        const rwpmath::Vector3 maxExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_max, 0));
#else
        const rwpmath::Vector3 minExtent(bb.m_min);
        const rwpmath::Vector3 maxExtent(bb.m_max);
#endif

        // Size of current entry AABBox
        const rwpmath::Vector3 boxSize(maxExtent - minExtent);

        const rwpmath::VecFloat boxSizeAxis = boxSize.GetComponent((int)split.m_axis);

        // Create a mask to select center components which are greater then the node AABBox components
        rwpmath::MaskScalar axisComparison = rwpmath::CompGreaterEqual(boxSizeAxis, thresholdSize);

        bool swap = axisComparison.GetBool();


        rwc_SwapEntriesAndAdjustIndices(swap,
                                        entries,
                                        iRight,
                                        iLeft);
    }

    EA_ASSERT_MSG(split.m_numLeft == (uint32_t) iLeft, ("Count of entries on left of split does not match."));
    EA_ASSERT_MSG(split.m_numRight == numEntries - iLeft, ("Count of entries on right of split does not match"));
}

/**
\internal
\brief
ENtry comparison method used by qsort.

\param arg1         First entry for comparison
\param arg2         Second entry for comparison
\return indicates whether a swap should take place.
*/
static int CompareEntries(const void * arg1,
                       const void * arg2)
{
    const Entry & entryA = *(reinterpret_cast<const Entry*>(arg1));
    const Entry & entryB = *(reinterpret_cast<const Entry*>(arg2));

    if (entryA.entryBBoxSurfaceArea > entryB.entryBBoxSurfaceArea)
    {
        return -1;
    }
    else if (entryA.entryBBoxSurfaceArea < entryB.entryBBoxSurfaceArea)
    {
        return 1;
    }
    return 0;
}


/**
\internal
\brief
Get statistics for a given node split along the 3 principal axis.

\param split                KDTree multiple axis split
\param nodeBB               Outer extent of node
\param entryBBoxes          Bounding boxes of all entries
\param entries              Entries in this node
\paran numEntries           Number of objects in this node
*/
static void
rwc_GetSplitStatsAllAxis(KDTreeMultiAxisSplit       &split,
                         AABBox                     &nodeBB,
                         const AABBoxU              *entryBBoxes,
                         const Entry            *entries,
                         uint32_t                   numEntries)
{

    // Initialize stats (start with inverted bbox)
    // Local copies are used here to avoid LHS on xenon
    AABBox leftBBoxX(nodeBB.Max(), nodeBB.Min());
    AABBox leftBBoxY(nodeBB.Max(), nodeBB.Min());
    AABBox leftBBoxZ(nodeBB.Max(), nodeBB.Min());
    AABBox rightBBoxX(nodeBB.Max(), nodeBB.Min());
    AABBox rightBBoxY(nodeBB.Max(), nodeBB.Min());
    AABBox rightBBoxZ(nodeBB.Max(), nodeBB.Min());

    // These counts are used to accumulate the left and right
    // item counts along each of the axis.
    rwpmath::Vector3 leftCount(rwpmath::GetVector3_Zero());
    rwpmath::Vector3 rightCount(rwpmath::GetVector3_Zero());

    // Loop through each of the entries and determine which group
    // (left or right) they fall into for each of the axis.
    for (uint32_t index = 0 ; index < numEntries ; ++index)
    {
        // Current entry AABBox
        const AABBoxU& bb = entryBBoxes[entries[index].entryIndex];

        // Min and Max of current entry AABBox
#if RWPMATH_IS_VPU
        const rwpmath::Vector3 minExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_min, 0));
        const rwpmath::Vector3 maxExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_max, 0));
#else
        const rwpmath::Vector3 minExtent(bb.m_min);
        const rwpmath::Vector3 maxExtent(bb.m_max);
#endif

        // Center point of current entry AABBox
        const rwpmath::Vector3 center = (minExtent + maxExtent) * rwpmath::GetVecFloat_Half();

        // Create a mask to select center components which are greater then the node AABBox components
        rwpmath::Mask3 axisComparison = rwpmath::CompGreaterThan(center, split.m_value);

        rwc_UpdateSplitStats(axisComparison,
                             minExtent,
                             maxExtent,
                             rightCount,
                             leftCount,
                             leftBBoxX,
                             rightBBoxX,
                             leftBBoxY,
                             rightBBoxY,
                             leftBBoxZ,
                             rightBBoxZ);
    }

    // Assign the local copy of the split details.
    split.m_numLeft = leftCount;
    split.m_numRight = rightCount;

    split.m_leftBBox[0] = leftBBoxX;
    split.m_leftBBox[1] = leftBBoxY;
    split.m_leftBBox[2] = leftBBoxZ;
    split.m_rightBBox[0] = rightBBoxX;
    split.m_rightBBox[1] = rightBBoxY;
    split.m_rightBBox[2] = rightBBoxZ;
}


/**
\internal
\brief
Get statistics for a given node split along the 3 principal axis.

\param split                KDTree multiple axis split
\param nodeBB               Outer extent of node
\param entryBBoxes          Bounding boxes of all entries
\param entries              Entries in this node
\param numEntries           Number of objects in this node
\param largeItemThreshold   The threshold above which items are considered to be large
*/
static void
rwc_GetSplitStatsAllAxisLargeItems(KDTreeMultiAxisSplit      &split,
                                   const AABBox              &nodeBB,
                                   const AABBoxU             *entryBBoxes,
                                   const Entry              *entries,
                                   uint32_t                  numEntries,
                                   const float               largeItemThreshold)
{
    // Initialize stats (start with inverted bbox)
    // Local copies are used here to avoid LHS on xenon
    AABBox leftBBoxX(nodeBB.Max(), nodeBB.Min());
    AABBox leftBBoxY(nodeBB.Max(), nodeBB.Min());
    AABBox leftBBoxZ(nodeBB.Max(), nodeBB.Min());
    AABBox rightBBoxX(nodeBB.Max(), nodeBB.Min());
    AABBox rightBBoxY(nodeBB.Max(), nodeBB.Min());
    AABBox rightBBoxZ(nodeBB.Max(), nodeBB.Min());

    // Determine the node size along each of the principal axis
    const rwpmath::Vector3 nodeSize = nodeBB.Max() - nodeBB.Min();
    const rwpmath::Vector3 thresholdSize = nodeSize * largeItemThreshold;

    // These counts are used to accumulate the left and right
    // item counts along each of the axis.
    rwpmath::Vector3 leftCount(rwpmath::GetVector3_Zero());
    rwpmath::Vector3 rightCount(rwpmath::GetVector3_Zero());

    // Loop through each of the entries and determine which group
    // (left or right) they fall into for each of the axis.
    for (uint32_t index = 0 ; index < numEntries ; ++index)
    {
        // Current entry AABBox
        const AABBoxU& bb = entryBBoxes[entries[index].entryIndex];
#if RWPMATH_IS_VPU
        const rwpmath::Vector3 minExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_min, 0));
        const rwpmath::Vector3 maxExtent(rw::math::vpl::VecLoadUnaligned(&bb.m_max, 0));
#else
        const rwpmath::Vector3 minExtent(bb.m_min);
        const rwpmath::Vector3 maxExtent(bb.m_max);
#endif

        // Size of current entry AABBox
        const rwpmath::Vector3 boxSize(maxExtent - minExtent);

        // Create a mask to select center components which are greater then the node AABBox components
        rwpmath::Mask3 axisComparison = rwpmath::CompGreaterEqual(boxSize, thresholdSize);

        rwc_UpdateSplitStats(axisComparison,
                             minExtent,
                             maxExtent,
                             rightCount,
                             leftCount,
                             leftBBoxX,
                             rightBBoxX,
                             leftBBoxY,
                             rightBBoxY,
                             leftBBoxZ,
                             rightBBoxZ);
    }

    // Assign the local copy of the split details.
    split.m_numLeft = leftCount;
    split.m_numRight = rightCount;

    split.m_leftBBox[0] = leftBBoxX;
    split.m_leftBBox[1] = leftBBoxY;
    split.m_leftBBox[2] = leftBBoxZ;
    split.m_rightBBox[0] = rightBBoxX;
    split.m_rightBBox[1] = rightBBoxY;
    split.m_rightBBox[2] = rightBBoxZ;
}

/**
\internal
\brief
Sets a KDTreeSplit given a KDTreeMultiAxisSplit and a vector of related costs.

\param result               Resulting KDTree split
\param multiSplit           Multiple axis KDTree split
\param costs                Costs related to each of the KDTree splits
*/
static rwpmath::VecFloat
rwc_SelectLowestCostSplit(KDTreeSplit               &result,
                          KDTreeMultiAxisSplit      &multiSplit,
                          rwpmath::Vector3::InParam costs)
{
    rwpmath::VecFloat lowestCost;
    if (costs.GetX() <= costs.GetY() && costs.GetX() <= costs.GetZ())
    {
        lowestCost = costs.GetX();
        result.m_axis = 0;
        result.m_value = multiSplit.m_value.GetX();
    }
    else if (costs.GetY() <= costs.GetZ())
    {
        lowestCost = costs.GetY();
        result.m_axis = 1;
        result.m_value = multiSplit.m_value.GetY();
    }
    else
    {
        lowestCost = costs.GetZ();
        result.m_axis = 2;
        result.m_value = multiSplit.m_value.GetZ();
    }

    result.m_leftBBox.m_min = multiSplit.m_leftBBox[result.m_axis].Min();
    result.m_leftBBox.m_max = multiSplit.m_leftBBox[result.m_axis].Max();

    result.m_rightBBox.m_min = multiSplit.m_rightBBox[result.m_axis].Min();
    result.m_rightBBox.m_max = multiSplit.m_rightBBox[result.m_axis].Max();

    result.m_numLeft = static_cast<uint32_t>(multiSplit.m_numLeft[static_cast<int32_t>(result.m_axis)]);
    result.m_numRight = static_cast<uint32_t>(multiSplit.m_numRight[static_cast<int32_t>(result.m_axis)]);

    return lowestCost;
}

/**
\internal

\brief
Gets the cost of a node split along 3 principal axis. Specialized for line queries by using
relative surface areas of children. Also reasonable for box queries, especially as it nicely
handles 2D cases(eg tessellated trilist floor).

Note that tight bboxes of the children are used even though these don't correspond to the
actual child node box (it's limited by the planar split). This is because we want
to bias towards choosing a configuration where the child entries can be more tightly bounded.
When it's worth doing, we can build up the tight bbox by splitting off empty leaves.

\param nodeBB        Outer extent of parent node
\param multiSplit    Multiple axis split statistics

\return Cost of each split. This numbers range from 0 in the case where each child is
very small compared to the parent node, to 1 where the children each
cover the same extent as the parent (ie the split is pretty useless).

*/
static rwpmath::Vector3
rwc_GetMultiSplitLowestCost(AABBox &nodeBB,
                            KDTreeMultiAxisSplit &multiSplit)
{
    /* If no objects on left or right, surface area can be negative since the BBox is in its
    initial inverted state, but it gets multiplied by zero anyway in this case.
    */
    // Weights of left split bboxes
    rwpmath::Vector3 leftWeight = multiSplit.m_numLeft * rwpmath::Vector3(rwc_BBoxSurfaceArea(multiSplit.m_leftBBox[0]),
        rwc_BBoxSurfaceArea(multiSplit.m_leftBBox[1]),
        rwc_BBoxSurfaceArea(multiSplit.m_leftBBox[2]));

    // Weights of right split bboxes
    rwpmath::Vector3 rightWeight = multiSplit.m_numRight * rwpmath::Vector3(rwc_BBoxSurfaceArea(multiSplit.m_rightBBox[0]),
        rwc_BBoxSurfaceArea(multiSplit.m_rightBBox[1]),
        rwc_BBoxSurfaceArea(multiSplit.m_rightBBox[2]));

    // Weight of Parent
    rwpmath::VecFloat nodeBBArea = (multiSplit.m_numLeft[0] + multiSplit.m_numRight[0]) *rwc_BBoxSurfaceArea(nodeBB);
    rwpmath::Vector3 parentWeight(nodeBBArea, nodeBBArea, nodeBBArea);

    // Costs of each split
    rwpmath::Vector3 costs = (leftWeight + rightWeight) / parentWeight;

    return costs;
}

/**
\internal

\brief
Splits the node along each principle axis for the boxes and finds the most efficient (smallest total
surface area) spatial split from the non-spatial mean surface area split

\param nonSpatialSplit pre-calculated split
\param nodeBBox bounding box of node
\param leftTightBBox tight bounding box of all the left child entries
\param rightTightBBox tight bounding box of all the right child entries
*/
static void
rwc_FindBestNonSpatialAxis(KDTreeSplit        &nonSpatialSplit,
                            const AABBox       &nodeBBox,
                            const AABBox       &leftTightBBox,
                            const AABBox       &rightTightBBox)
{
    rwpmath::VecFloat minChildSurfaceArea = rwpmath::GetVecFloat_MaxValue();

    // loop over all the principle axes
    for (uint32_t i=0; i<3; i++)
    {
        AABBox leftChildBBox;
        AABBox rightChildBBox;
        AABBox resultBBox;

        // Work out total surface area of both boxes to find the best split axis
        leftChildBBox = nodeBBox;
        rightChildBBox = nodeBBox;

        leftChildBBox.m_max.SetComponent(static_cast<int32_t>(i), leftTightBBox.m_max.GetComponent(static_cast<int32_t>(i)));
        rightChildBBox.m_min.SetComponent(static_cast<int32_t>(i), rightTightBBox.m_min.GetComponent(static_cast<int32_t>(i)));

        const rwpmath::VecFloat childSurfaceArea = rwc_BBoxSurfaceArea(leftChildBBox) + rwc_BBoxSurfaceArea(rightChildBBox);

        // If we have found a better split, save this info
        if (childSurfaceArea < minChildSurfaceArea)
        {
            minChildSurfaceArea = childSurfaceArea;
            nonSpatialSplit.m_axis = i;
        }
    }
}
/**
\internal

\brief
Create the split point to be the mean surface areas of all the entries.

Note: The entries must be ordered in descending order of bounding box surface area.

\param meanSplit split result to be filled in.
\param meanBBoxSurfaceArea mean surface area of entries in node
\param entries Entries in node
\param numEntries number of entries in node
*/
static void
rwc_GetSplitMeanSurfaceArea(KDTreeSplit             &meanSplit,
                            const rwpmath::VecFloat &meanBBoxSurfaceArea,
                            const Entry             *entries,
                            const uint32_t          numEntries)
{
    uint32_t entryIndex = 0;
    for (; entryIndex < numEntries; ++entryIndex)
    {
        // Determine the surface area of the current entry bounding box
        if (entries[entryIndex].entryBBoxSurfaceArea <= meanBBoxSurfaceArea)
        {
            // These are larger than the mean and so go in the left child node
            break;
        }
    }

    meanSplit.m_numLeft = entryIndex;
    // Remainder of the entries go in the right child node
    meanSplit.m_numRight = numEntries - meanSplit.m_numLeft;
}

/**
\internal

\brief
Find a non spatial split based on the mean surface area of all the boxes.

Note: The entries must be ordered in descending order of bounding box surface area.

\param geometricSplit split result to be filled in.
\param nodeBBox bounding box of node
\param meanBBoxSurfaceAreaRatio ratio of mean surface area of entries to node surface area
\param entryBBoxes Bounding boxes of entries in node
\param entries Entries in node
\param numEntries number of entries in node
\param minChildEntriesThreshold user defined minimum proportion of entries in a child node

\return TRUE if an acceptable split was found.
*/

static void
rwc_SplitNonSpatial(KDTreeSplit            &nonSpatialSplit,
                   const AABBox            &nodeBBox,
                   const rwpmath::VecFloat &meanBBoxSurfaceArea,
                   const AABBoxU           *entryBBoxes,
                   const Entry             *entries,
                   const uint32_t          numEntries,
                   const float             minChildEntriesThreshold)
{
    // Use the mean surface area to create a non spatial geometric split point
    rwc_GetSplitMeanSurfaceArea(nonSpatialSplit, meanBBoxSurfaceArea, entries, numEntries);

    //const float numEntriesFloat = static_cast<float>(numEntries);

    // With a small number of boxes we may end up with minEntries being less that 1 which is then truncated to 0.
    const uint32_t minEntries = (numEntries * minChildEntriesThreshold) > 1.0f ? static_cast<uint32_t>(numEntries * minChildEntriesThreshold) : 1;

    // If we have less than minimum of the entries in either of nodes then we just pad out the node with smallest number of entries 
    if ((static_cast<float>(nonSpatialSplit.m_numLeft) < minEntries) || (static_cast<float>(nonSpatialSplit.m_numRight) < minEntries))
    {
        // Change number of entries into each bin so that the min padding is satisfied
        // if we have more entries in the left node we need to increase in the right node
        if (nonSpatialSplit.m_numLeft > nonSpatialSplit.m_numRight)
        {
            nonSpatialSplit.m_numRight = minEntries;
            nonSpatialSplit.m_numLeft = numEntries - nonSpatialSplit.m_numRight;
        }
        // otherwise increase in the left node
        else
        {
            nonSpatialSplit.m_numLeft = minEntries;
            nonSpatialSplit.m_numRight = numEntries - nonSpatialSplit.m_numLeft;
        }
        
    }

    // Find the best axis for child boxes to be split spatially

    // Create a tight BBox around left child entries
    AABBoxU tightFPUBBox = entryBBoxes[entries[0].entryIndex];
    for (uint32_t i=1; i<nonSpatialSplit.m_numLeft; ++i)
    {
        const AABBoxU &currentBox = entryBBoxes[entries[i].entryIndex];

        tightFPUBBox.m_min = Min(tightFPUBBox.m_min, currentBox.m_min);
        tightFPUBBox.m_max = Max(tightFPUBBox.m_max, currentBox.m_max);

    }

    // Convert into a VPU AABBox
    AABBox tightLeftBBox(rwpmath::Vector3(tightFPUBBox.m_min), rwpmath::Vector3(tightFPUBBox.m_max));

    // Create a tight BBox around the right child entries
    tightFPUBBox = entryBBoxes[entries[nonSpatialSplit.m_numLeft].entryIndex];
    for (uint32_t i=(nonSpatialSplit.m_numLeft+1); i<numEntries; ++i)
    {
        const AABBoxU &currentBox = entryBBoxes[entries[i].entryIndex];

        tightFPUBBox.m_min = Min(tightFPUBBox.m_min, currentBox.m_min);
        tightFPUBBox.m_max = Max(tightFPUBBox.m_max, currentBox.m_max);

    }

    // Convert to a VPU BBox
    AABBox tightRightBBox(rwpmath::Vector3(tightFPUBBox.m_min), rwpmath::Vector3(tightFPUBBox.m_max));

    // Go over all the axis to see which configuration gives us the least total surface area
    rwc_FindBestNonSpatialAxis(nonSpatialSplit, nodeBBox, tightLeftBBox, tightRightBBox);

    // Assign the tight boxes created as the child nodes, the final split will be made later on based on the info in the split
    nonSpatialSplit.m_leftBBox = tightLeftBBox;
    nonSpatialSplit.m_rightBBox = tightRightBBox;

}


/**
\internal

\brief
Find the best split of a KDTree build node.

Finds a good split of a KDTree build node. If it proves effective,
an empty leaf is split off. Otherwise we try bisecting the tight bounding box
around the node entries on all three axes.

\param result                   Returns best split.
\param nodeBB                   Outer extent of this node.
\param entryBBoxes              BBox array (referenced by entryIndices).
\param entryIndices             Entries in this node, sorted on exit into
                                left and right groups.
\param numEntries               Number of entries in this node.
\param largeItemThreshold       Threshold at which objects larger than this size are split.
\param minChildEntriesThreshold Threshold for the minimum number of objects in a child node.
\param maxEntriesPerNode        Maximum allowed entries in a single node.
\param minSimilarAreaThreshold  Threshold at which are larger than this value are considered similar.

\return TRUE if an acceptable split was found.
*/
static RwpBool
rwc_FindBestSplit(KDTreeSplit                       *result,
                  AABBox                            &nodeBBox,
                  const AABBoxU                     *entryBBoxes,
                  Entry                             *entries,
                  uint32_t                          numEntries,
                  const float                       largeItemThreshold,
                  const float                       minChildEntriesThreshold,
                  const uint32_t                    maxEntriesPerNode,
                  const float                       minSimilarAreaThreshold)
{
    KDTreeSplit curSplit;
    uint32_t i;

    // Get tight bbox around entries
    AABBoxU tightFPUBBox = entryBBoxes[entries[0].entryIndex];

    // Entry BBox metrics
    float sumBBoxSurfaceArea(entries[0].entryBBoxSurfaceArea);
    float smallestBBoxSurfaceArea(entries[0].entryBBoxSurfaceArea);

    for (i=1; i<numEntries; i++)
    {
        const AABBoxU &currentBox = entryBBoxes[entries[i].entryIndex];

        tightFPUBBox.m_min = Min(tightFPUBBox.m_min, currentBox.m_min);
        tightFPUBBox.m_max = Max(tightFPUBBox.m_max, currentBox.m_max);

        const float currentSurfaceArea(entries[i].entryBBoxSurfaceArea);
        sumBBoxSurfaceArea += currentSurfaceArea;

        // Check to see if we have a smaller surface area
        smallestBBoxSurfaceArea = Min(currentSurfaceArea,smallestBBoxSurfaceArea);
    }

    AABBox tightBBox(rwpmath::Vector3(tightFPUBBox.m_min), rwpmath::Vector3(tightFPUBBox.m_max));

    // Compare the mean BBoxes to the node BBox to get a ratio
    rwpmath::VecFloat nodeSurfaceArea = rwc_BBoxSurfaceArea(nodeBBox);

    // Get the mean entry bbox surface area
    const float meanBBoxSurfaceArea = sumBBoxSurfaceArea / (numEntries);

    // See if it's worth making an empty leaf
    rwpmath::VecFloat minChildSurfaceArea = nodeSurfaceArea;
    for (i=0; i<3; i++)
    {
        rwpmath::VecFloat childSurfaceArea;
        AABBox childBBox;

        // Try keeping entries in left
        childBBox = nodeBBox;
        childBBox.m_max.SetComponent((int)i, tightBBox.m_max.GetComponent((int)i));
        childSurfaceArea = rwc_BBoxSurfaceArea(childBBox);

        if (childSurfaceArea < minChildSurfaceArea)
        {
            minChildSurfaceArea = childSurfaceArea;
            curSplit.m_axis = i;
            curSplit.m_value = tightBBox.m_max.GetComponent((int)i);
            curSplit.m_numLeft = numEntries;
            curSplit.m_numRight = 0;
            curSplit.m_leftBBox = tightBBox;
            curSplit.m_rightBBox = AABBox(nodeBBox.m_max, nodeBBox.m_min); // Inverted
        }

        // Try keeping entries in right
        childBBox = nodeBBox;
        childBBox.m_min.SetComponent((int)i, tightBBox.m_min.GetComponent((int)i));
        childSurfaceArea = rwc_BBoxSurfaceArea(childBBox);

        if (childSurfaceArea < minChildSurfaceArea)
        {
            minChildSurfaceArea = childSurfaceArea;
            curSplit.m_axis = i;
            curSplit.m_value = tightBBox.m_min.GetComponent((int)i);
            curSplit.m_numLeft = 0;
            curSplit.m_numRight = numEntries;
            curSplit.m_leftBBox = AABBox(nodeBBox.m_max, nodeBBox.m_min); // Inverted
            curSplit.m_rightBBox = tightBBox;
        }
    }

    if (minChildSurfaceArea < (rwcKDTREEBUILD_EMPTY_LEAF_THRESHOLD * nodeSurfaceArea))
    {
        *result = curSplit;
        return TRUE;
    }

    KDTreeMultiAxisSplit multiSplit;

    // Find best of X, Y, and Z axes

    // Initialize the split position/value along each principal axis
    multiSplit.m_value = (tightBBox.Min() + tightBBox.Max()) * rwpmath::VecFloat(0.5f);

    // Get the split stats for each principal axis
    rwc_GetSplitStatsAllAxis(multiSplit, nodeBBox, entryBBoxes, entries, numEntries);
    // Get the costs of each split
    rwpmath::Vector3 costs = rwc_GetMultiSplitLowestCost(nodeBBox, multiSplit);
    // Determine the lowest cost split
    rwpmath::VecFloat cost = rwc_SelectLowestCostSplit(*result, multiSplit, costs);

    // Check the validity of the cheapest split
    if (result->m_numLeft > 0 && result->m_numRight > 0 && cost < rwcKDTREEBUILD_SPLIT_COST_THRESHOLD)
    {
        // Sort the entires in the order corresponding to the cheapest split
        rwc_SortSplitEntries(*result, entryBBoxes, entries, numEntries);
        return TRUE;
    }

    if (largeItemThreshold < 1.0f)
    {
        // Try an alternative solution with the "big" items in one box and the rest in the other.
        // Get the split stats for each principal axis
        rwc_GetSplitStatsAllAxisLargeItems(multiSplit, nodeBBox, entryBBoxes, entries, numEntries, largeItemThreshold);
        // Get the costs of each split
        costs = rwc_GetMultiSplitLowestCost(nodeBBox, multiSplit);
        // Determine the lowest cost split
        cost = rwc_SelectLowestCostSplit(*result, multiSplit, costs);

        // Check the validity of the cheapest split
        if (result->m_numLeft > 0 && result->m_numRight > 0 && cost < rwcKDTREEBUILD_SPLIT_COST_THRESHOLD)
        {
            // Sort the entires in the order corresponding to the cheapest split
            rwc_SortSplitEntriesLargeItems(*result, nodeBBox, entryBBoxes, entries, numEntries, largeItemThreshold);
            return TRUE;
        }
    }

    // Well, if we are here then our default routines have failed. We introduce the safety net to stop various errors creeping in
    // like overflowing clusters where we have more vertices than can be contained. For more info see the KDTreeBuilder wiki page.

    // If the smallest bbox is smaller than the threshold
    // OR If the number of entries per leaf node is greater than or equal to the threshold
    if ((smallestBBoxSurfaceArea < (minSimilarAreaThreshold * nodeSurfaceArea)) || (numEntries >= maxEntriesPerNode))
    {
        // Sort the entries by descending size
        qsort(reinterpret_cast<void*>(entries), numEntries, sizeof(Entry), CompareEntries);

        KDTreeSplit nonSpatialSplit;

        // Now we use a non spatial split determined by the size of the objects.
        // Here we do our split over the mean surface area of the bounding boxes.
        rwc_SplitNonSpatial(nonSpatialSplit, nodeBBox, meanBBoxSurfaceArea, entryBBoxes, entries, numEntries, minChildEntriesThreshold);
        *result = nonSpatialSplit;
        return TRUE;
    }

    // Failed to split
    return FALSE;
}



/**
\internal

\brief
Recursively split KDTreeBuilder::BuildNode

\param entryBBoxes              Array of extents of entries
\param entryIndices             Array of entry indices which are sorted into the flattened tree
                                order on exit. These should be initialized indices[i] = i. This is the
                                complete set of entry indices. The node references a slice of entries
                                from this array.
\param splitThreshold           Threshold for number of entries above which nodes are split.
\param largeItemThreshold       Threshold at which objects larger than this size are split.
\param minChildEntriesThreshold Threshold for the minimum number of objects in a child node.
\param maxEntriesPerNode        Maximum allowed entries in a single node.
\param minSimilarAreaThreshold  Threshold at which are larger than this value are considered similar.

\return Number of nodes created (0 if no splits)
*/
uint32_t
KDTreeBuilder::BuildNode::SplitRecurse(EA::Allocator::ICoreAllocator & allocator,
                                       const AABBoxU * entryBBoxes,
                                       Entry *entries,
                                       uint32_t splitThreshold,
                                       uint32_t depth,
                                       const float largeItemThreshold,
                                       const float minChildEntriesThreshold,
                                       const uint32_t maxEntriesPerNode,
                                       const float minSimilarAreaThreshold)
{
    KDTreeSplit split;

    AABBox nodeBBox(rwpmath::Vector3(m_bbox.m_min), rwpmath::Vector3(m_bbox.m_max));

    // Can we find a split?
    if (m_numEntries <= splitThreshold
        || depth > rwcKDTREE_MAX_DEPTH
        || !rwc_FindBestSplit(&split,
                              nodeBBox,
                              entryBBoxes,
                              entries + m_firstEntry,
                              m_numEntries,
                              largeItemThreshold,
                              minChildEntriesThreshold,
                              maxEntriesPerNode,
                              minSimilarAreaThreshold))
    {
        // Not splittable
        return 0;
    }

    // Set the split axis
    m_splitAxis = split.m_axis;

    // Get actual child bboxes for planar split (note empty children can have inverted box)
    AABBoxU leftBBox = m_bbox;
    leftBBox.m_max.SetComponent((int)m_splitAxis, (AABBoxU::FloatType)(split.m_leftBBox.m_max.GetComponent((int)m_splitAxis)));
    AABBoxU rightBBox = m_bbox;
    rightBBox.m_min.SetComponent((int)m_splitAxis, (AABBoxU::FloatType)(split.m_rightBBox.m_min.GetComponent((int)m_splitAxis)));

    // Allocate child nodes
    void * mem = allocator.Alloc(sizeof(BuildNode) * 2, NULL, 0, 4);
    EAPHYSICS_WARNING(NULL != mem, "Allocation Failure: Failed to allocate BuildNodes.");
    if (NULL == mem)
    {
        return rwcKDTREEBUILDER_BUILDFAILED;
    }

    m_left = new (static_cast<BuildNode*>(mem)) KDTreeBuilder::BuildNode(this, leftBBox, m_firstEntry, split.m_numLeft);
    m_right = new ((static_cast<BuildNode*>(mem) + 1)) KDTreeBuilder::BuildNode(this, rightBBox, m_firstEntry + split.m_numLeft, split.m_numRight);

    //Increment depth
    depth += 1;
    if(depth > rwcKDTREE_MAX_DEPTH)
    {
        EAPHYSICS_MESSAGE("KDTree Leaf splitting will stop because tree depth has reached max allowable of %d.\nCheck geometry because performance may be sub-optimal.", rwcKDTREE_MAX_DEPTH);
    }

    // Set child indices and recurse
    m_left->m_index = m_index + 1;
    uint32_t numLeft = m_left->SplitRecurse(allocator, 
                                            entryBBoxes, 
                                            entries, 
                                            splitThreshold, 
                                            depth, 
                                            largeItemThreshold, 
                                            minChildEntriesThreshold, 
                                            maxEntriesPerNode,
                                            minSimilarAreaThreshold);

    if (rwcKDTREEBUILDER_BUILDFAILED == numLeft)
    {
        return rwcKDTREEBUILDER_BUILDFAILED;
    }

    m_right->m_index = m_left->m_index + (int32_t) numLeft + 1;
    uint32_t numRight = m_right->SplitRecurse(allocator, 
                                              entryBBoxes, 
                                              entries, 
                                              splitThreshold, 
                                              depth, 
                                              largeItemThreshold, 
                                              minChildEntriesThreshold, 
                                              maxEntriesPerNode,
                                              minSimilarAreaThreshold);

    if (rwcKDTREEBUILDER_BUILDFAILED == numRight)
    {
        return rwcKDTREEBUILDER_BUILDFAILED;
    }

    // Return total number of nodes created during splitting
    return numLeft + numRight + 2;
}


// ***********************************************************************************************************
// External Functions


// ***********************************************************************************************************
// Class Member Functions

/**
\brief KDTreeBuilder destructor.

Releases resources.
*/
KDTreeBuilder::~KDTreeBuilder()
{
    if (m_root)
    {
        DeleteSubTree(m_root);
        m_allocator.Free(m_root);
        m_root = NULL;
    }

    if (m_entryIndices)
    {
        m_allocator.Free(m_entryIndices);
        m_entryIndices = NULL;
    }
}

/**
\brief Recursive method with deals with resource deallocation.

Recursive method which deallocates the resources of the build nodes structure.

\param node   The build node to deallocate
*/
void
KDTreeBuilder::DeleteSubTree(BuildNode *node)
{
    if (node->m_right)
    {
        DeleteSubTree(node->m_right);
    }
    if (node->m_left)
    {
        DeleteSubTree(node->m_left);
    }

    if (node->m_left)
    {
        m_allocator.Free(node->m_left);
    }
}


/**
\brief Build a KDTree.

Due to a number of internal entry counters the limiting number of entries the builder
can handle is 2^24.

\param numEntries               Number of entries to be indexed by the KDTree.
\param entryBBoxes              Vector of bounding boxes that bound the entries.
\param splitThreshold           Threshold for number of entries above which nodes are split.
\param largeItemThreshold       Threshold at which objects larger than this size are split.
\param minChildEntriesThreshold Threshold for the minimum number of objects in a child node.
\param maxEntriesPerNode        Maximum allowed entries in a single node.
\param minSimilarAreaThreshold  Threshold at which are larger than this value are considered similar.
*/
uint32_t
KDTreeBuilder::BuildTree(uint32_t numEntries, 
                         const AABBoxU * entryBBoxes, 
                         uint32_t splitThreshold, 
                         const float largeItemThreshold,
                         const float minChildEntriesThreshold,
                         const uint32_t maxEntriesPerNode,
                         const float minSimilarAreaThreshold)
{
    EA_ASSERT(entryBBoxes);
    // Since floats are used to count node entries the maximum
    // number of entries each node can count is 2^24;
    EA_ASSERT(numEntries <= (1<<24));

    EA_ASSERT(minChildEntriesThreshold <= 1.0f);

    m_success = true;

    // Allocate and initialize entry array. This will be sorted by
    // the node splitting process.
    Entry * entries = reinterpret_cast<Entry *>(m_allocator.Alloc(numEntries * sizeof(Entry), NULL, 0, 4));
    EAPHYSICS_WARNING(NULL != entries, "Allocation Failure: Failed to allocate entry array.");

    if (NULL == entries)
    {
        m_success = false;
        return 0;
    }

    if(numEntries)
    {
        // Find the overall bounding box and initialize the entry array
        AABBoxU rootBBox(entryBBoxes[0].Min(), entryBBoxes[0].Max());
        entries[0].entryIndex = 0;
        entries[0].entryBBoxSurfaceArea = rwc_BBoxSurfaceAreaU(entryBBoxes[0]);

        for (uint32_t entryIndex = 1; entryIndex < numEntries;  ++entryIndex)
        {
            const AABBoxU & entryBBox = entryBBoxes[entryIndex];

            rootBBox.m_min = Min(rootBBox.m_min, entryBBox.m_min);
            rootBBox.m_max = Max(rootBBox.m_max, entryBBox.m_max);

            entries[entryIndex].entryIndex = entryIndex;
            entries[entryIndex].entryBBoxSurfaceArea = rwc_BBoxSurfaceAreaU(entryBBox);
        }

        // Start with single node containing all entries and recursively split it
        void * mem = m_allocator.Alloc(sizeof(BuildNode), NULL, 0, 4);
        EAPHYSICS_WARNING(NULL != mem, "Allocation Failure: Failed to allocate BuildNode.");

        if (NULL == mem)
        {
            if (NULL != entries)
            {
                m_allocator.Free(entries);
                entries = NULL;
            }
            m_success = false;
            return 0;
        }

        m_root = new (mem) BuildNode(0, rootBBox, 0, numEntries);
        m_numNodes = 1 + m_root->SplitRecurse(m_allocator, 
                                              entryBBoxes, 
                                              entries,
                                              splitThreshold, 
                                              1, 
                                              largeItemThreshold, 
                                              minChildEntriesThreshold, 
                                              maxEntriesPerNode, 
                                              minSimilarAreaThreshold);
    }
    else
    {
        m_root = NULL;
        m_numNodes = 0;
    }

    m_success = (m_numNodes != rwcKDTREEBUILDER_BUILDFAILED);

    if (m_success)
    {
        // Allocate and initialize entry index table. This will be sorted by
        // the node splitting process.
        m_entryIndices = (uint32_t *) m_allocator.Alloc(numEntries * sizeof(uint32_t), NULL, 0, 4);
        EAPHYSICS_WARNING(NULL != m_entryIndices, "Allocation Failure: Failed to allocate entryIndices array.");

        if (NULL == m_entryIndices)
        {
            if (NULL != entries)
            {
                m_allocator.Free(entries);
                entries = NULL;
            }
            m_success = false;
            return 0;
        }

        // Copy the sorted entry indices into the array
        for (uint32_t entryIndex = 0;  entryIndex < numEntries; ++entryIndex)
        {
            m_entryIndices[entryIndex] = entries[entryIndex].entryIndex;
        }
    }

    if (NULL != entries)
    {
        m_allocator.Free(entries);
        entries = NULL;
    }

    return m_numNodes;
}

/**
\brief Initialise a runtime KDTree from the build tree data
*/
void
KDTreeBuilder::InitializeRuntimeKDTree(rw::collision::KDTree *kdtree) const
{
    EA_ASSERT((1 + 2 * ((uint32_t) (kdtree->GetNumBranchNodes()))) == GetNumNodes());

    // Now fill in kdtree branch nodes
    if (kdtree->GetNumBranchNodes() > 0)
    {
        // The GraphKDTree stores branch and leaf nodes
        // in the array. The runtime version compresses the leaf nodes into the
        // branch nodes. This means there is not a one-to-one node mapping for the
        // flattened node arrays. We need to do a tree traversal.

        struct stackValue
        {
            uint32_t rtParent;
            uint32_t rtChild;
            KDTreeBuilder::BuildNode *node;
        };

        stackValue stack[rwcKDTREE_STACK_SIZE], cur;

        stack[0].node = m_root;
        stack[0].rtParent = 0;
        stack[0].rtChild = 0;
        uint32_t top = 1;
        uint32_t rtIndex;

        // Traverse tree
        for (rtIndex = 0; top > 0; rtIndex++)
        {
            cur = stack[--top];

            // Set reference to us in parent (unless we're the root node)
            if (rtIndex != 0)
            {
                kdtree->m_branchNodes[cur.rtParent].m_childRefs[cur.rtChild].m_index = rtIndex;
            }

            // Get current graph kdtree branch node, child nodes, and runtime node
            KDTree::BranchNode &rtNode = kdtree->m_branchNodes[rtIndex];
            BuildNode *childNodes[2];
            childNodes[0] = cur.node->m_left;
            childNodes[1] = cur.node->m_right;

            // Initialize runtime node
            rtNode.m_parent     = cur.rtParent;
            rtNode.m_axis       = cur.node->m_splitAxis;
            rtNode.m_extents[0] = cur.node->m_left->m_bbox.Max().GetComponent((int)rtNode.m_axis);
            rtNode.m_extents[1] = cur.node->m_right->m_bbox.Min().GetComponent((int)rtNode.m_axis);

            // Will traverse left first, so add any right child branch to stack first.
            for (int32_t i = 1; i >= 0; --i)
            {
                if (!childNodes[i]->m_left ) //if child node is a leaf store data in childref
                {
                    // Child is leaf node so store leaf content info
                    rtNode.m_childRefs[i].m_content = childNodes[i]->m_numEntries;
                    rtNode.m_childRefs[i].m_index = childNodes[i]->m_firstEntry;
                }
                else
                {
                    // Put child branch node on stack
                    stack[top].rtParent = rtIndex;
                    stack[top].rtChild = (uint32_t) i;
                    stack[top].node = childNodes[i];
                    top++;

                    // Will fill in reference to child branch later
                    rtNode.m_childRefs[i].m_content = rwcKDTREE_BRANCH_NODE;
                    rtNode.m_childRefs[i].m_index = rwcKDTREE_INVALID_INDEX;
                }
            }
        }

        EA_ASSERT_MSG(rtIndex == kdtree->GetNumBranchNodes(), ("Invalid number of nodes in the KDTree!"));
    }

    EA_ASSERT_MSG(kdtree->IsValid(), ("Failed to initialize a valid KDTree!"));
}

} // namespace collision
} // namespace rw
