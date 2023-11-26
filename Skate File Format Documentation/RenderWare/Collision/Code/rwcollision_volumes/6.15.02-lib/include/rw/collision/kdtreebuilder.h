// (c) Electronic Arts. All Rights Reserved.

#ifndef KDTREEBUILDER_HPP
#define KDTREEBUILDER_HPP

#include "rw/collision/common.h"                      // for basic types and rwpmath::
#include "rw/collision/aabbox.h"                    // for AABBoxU

#include "coreallocator/icoreallocator.h"

// Forwards Declarations
namespace EA
{
    namespace Allocator
    {
        class ICoreAllocator;
    }
}

namespace rw
{
    namespace collision
    {
        struct Entry;
    }
}


namespace rw
{
namespace collision
{

class KDTreeBase;

/**
Default value for the threshold factor used to determine whether items are "large" relative to the
bounding box of their containing KDTree node. An item is considered large if its extent in the
split dimension, as a factor of the extent of the containing box, is greater than or equal to the
threshold.
*/
#define rwcKDTREEBUILDER_DEFAULTLARGEITEMTHRESHOLD  0.8f

/**
Default value for the smallest size of an item relative to the node box in which it is in for it to be
considered similar size. The reasoning for this is that if the smallest object is bigger than this value,
then all the boxes are considered to be a similar size so then is no need to split if we have less than
the rwcKDTREEBUILER_DEFAULTMAXBBOXESPERNODE number of boxes.
*/
#define rwcKDTREEBUILDER_DEFAULTMINSIMILARSIZETHRESHOLD 0.8f

/**
Default value that determines the minimum number of entries in a child node for the non spatial split. Using
the mean surface area as a heuristic to split the entries can result in too few entries in one node. This 
property makes sure that the proportion of entries in the child node with the fewest entries is above this value
by means of a padding in the routine.
*/
#define rwcKDTREEBUILDER_DEFAULTMINPROPORTIONNODEENTRIES 0.3f

/**
Default value for the maximum number of entries per leaf node. Nodes with a higher count of entries that this value
will be split, unless the maximum depth of the tree is reached.

NOTE: The value of 63 is used here in relation to a ClusteredMesh technical issue and has no performance basis.
*/
#define rwcKDTREEBUILER_DEFAULTMAXENTRIESPERNODE 63

/**
Helper object to construct a rw::collision::KDTree from a list of axis aligned bounding boxes.

Call BuildTree() to create the tree internally, allocate a KDTree using the number of branch nodes
that is returned from GetNumBranchNodes() and then call InitializeRuntimeKDTree() to fill in the tree.
 
The following example code shows how to create a KDTreeMappedArray from a set of bounding boxes
@code
 rw::collision::KDTreeBuilder kdTreeBuilder;
 kdTreeBuilder.BuildTree(numVolumes, bboxList, splitThreshold);

 // Allocate a block of memory for our KDTreeMappedArray
 uint32_t numBranchNodes = kdTreeBuilder.GetNumBranchNodes();
 EA::Physics::SizeAndAlignment kdTreeAggResDesc =
     KDTreeMappedArray::GetResourceDescriptor(numVolumes, numBranchNodes, bbox);
 EA::Physics::MemoryPtr kdTreeAggResource = GetDefaultAllocator().Allocate(kdTreeAggResDesc);
 rw::collision::KDTreeMappedArray *kdTreeAgg =
     rw::collision::KDTreeMappedArray::Initialize(kdTreeAggResource, numVolumes, numBranchNodes, bbox);

 // Create the child volumes using order of objects defined by kd tree
 uint32_t * entryIndices = kdTreeBuilder.GetSortedEntryIndices();
 childVolumes = kdTreeAgg->GetVolumeArray();
 for (uint32_t vol = 0; vol < numVolumes; ++vol)
 {
     InitializeVolume(vol, &childVolumes[entryIndices[vol]]);
 }

 // Initialize the kdtree
 kdTreeBuilder.InitializeRuntimeKDTree(kdTreeAgg->GetKDTreeMap());

@endcode

@note This class currently performs a number of small allocations during BuildTree() and corresponding
de-allocations during its destructor that may make it unsuitable for runtime use. All allocations
are done through the allocator passed to the constructor. It is currently asserted that memory allocations
are successful.
 */
class KDTreeBuilder
{
private:

    /**
    Value used to indicate a failed build process due to failed memory allocation. Upon a failure this
    value will obscure the number of nodes successfully created by hijacking the m_numNodes member.
    */
    static const uint32_t rwcKDTREEBUILDER_BUILDFAILED = 0xFFFFFFFF;

public:

    KDTreeBuilder(EA::Allocator::ICoreAllocator & allocator)
    : m_allocator(allocator), m_root(0), m_numNodes(0), m_entryIndices(0), m_success(FALSE)
    {
    }

    ~KDTreeBuilder();

    /**
    \brief
    Temporary data structure used when building KDTree. This is not stored in the
    graph.
    */
    class BuildNode
    {
    public:

        BuildNode(BuildNode *parent,
                        AABBoxU &bbox,
                        uint32_t firstEntry,
                        uint32_t numEntries)
            : m_parent(parent),
            m_index(0),
            m_bbox(bbox),
            m_firstEntry(firstEntry),
            m_numEntries(numEntries),
            m_splitAxis(0),
            m_left(0),
            m_right(0)
        {
        }

        ~BuildNode()
        {
        }

        uint32_t
            SplitRecurse(EA::Allocator::ICoreAllocator & allocator,
            const AABBoxU * entryBBoxes,
            Entry *entries,
            uint32_t splitThreshold,
            uint32_t depth,
            const float largeItemThreshold = rwcKDTREEBUILDER_DEFAULTLARGEITEMTHRESHOLD,
            const float minChildEntriesThreshold = rwcKDTREEBUILDER_DEFAULTMINPROPORTIONNODEENTRIES,
            const uint32_t maxEntriesPerNode = rwcKDTREEBUILER_DEFAULTMAXENTRIESPERNODE,
            const float minSimilarAreaThreshold = rwcKDTREEBUILDER_DEFAULTMINSIMILARSIZETHRESHOLD);

        // Link to parent
        BuildNode           *m_parent;

        // Index of this node in flattened depth first order
        int32_t             m_index;

        // Node bbox
        AABBoxU             m_bbox;

        // Slice of entries contained within this node
        uint32_t            m_firstEntry;
        uint32_t            m_numEntries;

        // Children
        uint32_t            m_splitAxis;
        BuildNode           *m_left;
        BuildNode           *m_right;
    };


    uint32_t
    BuildTree(uint32_t numEntries,
              const AABBoxU * entryBBoxes,
              uint32_t splitThreshold,
              const float largeItemThreshold = rwcKDTREEBUILDER_DEFAULTLARGEITEMTHRESHOLD,
              const float minChildEntriesThreshold = rwcKDTREEBUILDER_DEFAULTMINPROPORTIONNODEENTRIES,
              const uint32_t maxEntriesPerNode = rwcKDTREEBUILER_DEFAULTMAXENTRIESPERNODE,
              const float minSimilarAreaThreshold = rwcKDTREEBUILDER_DEFAULTMINSIMILARSIZETHRESHOLD);

    uint32_t
    GetNumNodes() const
    {
        return(m_numNodes);
    }

    uint32_t
    GetNumBranchNodes() const
    {
        return((m_numNodes-1)/2);
    }

    AABBox
    GetRootBBox() const
    {
        EA_ASSERT(m_root);
        return AABBox(rwpmath::Vector3(m_root->m_bbox.Min()), rwpmath::Vector3(m_root->m_bbox.Max()));
    }

    BuildNode *
    GetRootNode() const
    {
        return(m_root);
    }

    const uint32_t *
    GetSortedEntryIndices() const
    {
        return(m_entryIndices);
    }

    void
    InitializeRuntimeKDTree(rw::collision::KDTreeBase *kdtree) const;

    /**
    \brief Returns a bool indicating whether or not a successful build
    \has taken place.
    */
    bool SuccessfulBuild() const
    {
        return m_success;
    }

protected:

    void DeleteSubTree(BuildNode *node);

    EA::Allocator::ICoreAllocator & m_allocator;
    BuildNode   *m_root;
    uint32_t    m_numNodes;

    uint32_t    *m_entryIndices;

private:

    // Non-copyable
    KDTreeBuilder(const KDTreeBuilder & other);
    KDTreeBuilder & operator = (const KDTreeBuilder & other);

    // A flag indicating a build failure
    bool m_success;
};

} // namespace collision
} // namespace rw

#endif //KDTREEBUILDER_HPP
