// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/kdtreebuilder.h>
#include <rw/collision/kdtree.h>
#include <rw/collision/initialize.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilder.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example demonstrates building a KDTree from a set of vertices representing bounding boxes
around some quads or triangles
*/


int EAMain(int /*argc*/, char ** /*argv*/)
{
    typedef rw::collision::meshbuilder::VectorType VectorType;

    // Initialize the input parameters of the KDTree

    // Minimum objects per node
    const uint32_t splitThreshold(4);

    // If Threshold is not 1.0 then objects larger than threshold are put into one node and smaller objects
    // are put into the other node
    const float largeItemThreshold(1.0f);

    // For nodes with objects that have mean surface areas lower than this threshold, objects larger than 
    // the mean are separated into one node and smaller in the other node
    //const float meanSurfaceAreaThreshold(0.5f);

    // If we have more BBoxes in a node that this value, we will always force a split
    //const uint32_t maxBBoxesPerNode(63);

    const uint32_t numVolumes = 11;
    const VectorType volumeExtents[numVolumes * 2] =
    {
        // big items
        VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),

        // small items
        VectorType(0.0f, 0.0f, 0.0f), VectorType(0.1f, 0.1f, 0.1f),
        VectorType(0.1f, 0.1f, 0.1f), VectorType(0.2f, 0.2f, 0.2f),
        VectorType(0.2f, 0.2f, 0.2f), VectorType(0.3f, 0.3f, 0.3f),
        VectorType(0.3f, 0.3f, 0.3f), VectorType(0.4f, 0.4f, 0.4f),
        VectorType(0.4f, 0.4f, 0.4f), VectorType(0.5f, 0.5f, 0.5f),
        VectorType(0.5f, 0.5f, 0.5f), VectorType(0.6f, 0.6f, 0.6f),
        VectorType(0.6f, 0.6f, 0.6f), VectorType(0.7f, 0.7f, 0.7f),
        VectorType(0.7f, 0.7f, 0.7f), VectorType(0.8f, 0.8f, 0.8f),
        VectorType(0.8f, 0.8f, 0.8f), VectorType(0.9f, 0.9f, 0.9f),
        VectorType(0.9f, 0.9f, 0.9f), VectorType(1.0f, 1.0f, 1.0f),
    };

    // Create the input volumes and accumulate the root bbox.
    rw::collision::AABBoxU * bboxList = new rw::collision::AABBoxU[numVolumes];
    for (uint32_t i = 0; i < numVolumes; i++)
    {
        bboxList[i] = rw::collision::AABBoxU(volumeExtents[i * 2], volumeExtents[i * 2 + 1]);
    }

    // Build the KD tree

    rw::collision::KDTreeBuilder builder(*EA::Allocator::ICoreAllocator::GetDefaultAllocator());
    builder.BuildTree(numVolumes, bboxList, splitThreshold, largeItemThreshold);

    // Get number of branch nodes in the tree and diplay this info
    uint32_t numBranchNodes = builder.GetNumBranchNodes();

    printf("Tree created with %i branch nodes \n", numBranchNodes);


    // Create and initialize a runtime KD Tree from the builder

    rw::collision::AABBox bbox = builder.GetRootBBox();

    //rw::collision::KDTree * kdtree = EA::Physics::UnitFramework::Creator<rw::collision::KDTree>().New(numBranchNodes, numVolumes, bbox);
    // Create an aggregate volume with the clusteredmesh as its aggregate.
    EA::Physics::SizeAndAlignment sal = rw::collision::KDTree::GetResourceDescriptor(numBranchNodes, numVolumes, bbox);
    void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
    rw::collision::KDTree * kdtree = rw::collision::KDTree::Initialize(mem, numBranchNodes, numVolumes, bbox);
    builder.InitializeRuntimeKDTree(kdtree);

    if (kdtree->IsValid())
    {
        printf("Runtime KD Tree successfully initialized");
    }


    // Release the tree
    if (kdtree != NULL)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(kdtree);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU

