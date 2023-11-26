// (c) Electronic Arts. All Rights Reserved.
#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/kdtree.h>
#include <rw/collision/kdsubtree.h>
#include <rw/collision/kdtreebboxquery.h>
#include <rw/collision/kdtreelinequery.h>

#include <serialization/serialization.h>
#include <serialization/binary_stream_oarchive.h>
#include <serialization/binary_stream_iarchive.h>

#include <unit/unit.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

#include "SimpleStream.hpp"
#include "clusteredmesh_test_helpers.hpp"

using namespace rw::collision;
using namespace rwpmath;

#define COURTYARD  "courtyard.dat"


// ***********************************************************************************************************
// Test suite

class TestKDSubTree : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestKDSubTree");
        EATEST_REGISTER("TestKDSubTreeArray", "TestKDSubTreeArray", TestKDSubTree, TestKDSubTreeArray);
        EATEST_REGISTER("TestBranchOffset", "TestBranchOffset", TestKDSubTree, TestBranchOffset);
        EATEST_REGISTER("TestSerialize", "Test Serialize()", TestKDSubTree, TestSerialize);
    }
    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        Volume::InitializeVTable();
    }
    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        tests::TestSuiteBase::TeardownSuite();
    }

private:

    void TestKDSubTreeArray();
    void TestBranchOffset();
    void TestSerialize();

} TestKDSubTree_Singleton;


//-----------------------------------------------------------------------------------------------------
void TestKDSubTree::TestBranchOffset()
{
    KDSubTree testKDSubTree;
    testKDSubTree.SetBranchNodeOffset(10);
    EATESTAssert(testKDSubTree.GetBranchNodeOffset()==10, "BranchNodeOffset not set");
}

void TestKDSubTree::TestKDSubTreeArray()
{
    //Load ClusteredMesh
    Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(COURTYARD);
    EATESTAssert(clusteredMeshVolume, "Expected to be able to load test mesh");

    //Generate KDSubTree Array
    ClusteredMesh &clusteredMesh = *((ClusteredMesh*)((AggregateVolume*)clusteredMeshVolume)->GetAggregate());
    EA::Physics::SizeAndAlignment arrayDesc = EA::Physics::SizeAndAlignment(sizeof(KDSubTree)*clusteredMesh.GetNumCluster(),16);
    EA::Physics::SizeAndAlignment workspaceDesc = rw::collision::GetKDSubTreeWorkSpaceResourceDescriptor(clusteredMesh);
    EA::Physics::MemoryPtr arrayRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(arrayDesc.GetSize(), 0, 0, arrayDesc.GetAlignment());

    EA::Physics::MemoryPtr workspaceRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(workspaceDesc.GetSize(), 0, 0, workspaceDesc.GetAlignment());
    KDSubTree *kdSubTreeArray = (KDSubTree*)arrayRes.GetMemory();
    rw::collision::CreateKDSubTreeArray(kdSubTreeArray,workspaceRes,clusteredMesh);

    //Check each KDSubTree
    uint32_t numClusters = clusteredMesh.GetNumCluster();
    for(uint32_t clusterNo = 0; clusterNo<numClusters; clusterNo++)
    {
        uint32_t kdSubTreeSize = sizeof(KDSubTree) + sizeof(KDTree::BranchNode)*kdSubTreeArray[clusterNo].GetNumBranchNodes();
        EA::Physics::SizeAndAlignment kdSubTreeDesc = EA::Physics::SizeAndAlignment(kdSubTreeSize,16);
        EA::Physics::MemoryPtr kdSubTreeRes = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(kdSubTreeDesc.GetSize(), 0, 0, kdSubTreeDesc.GetAlignment());
        KDSubTree *currentSubTree = (KDSubTree*)kdSubTreeRes.GetMemory();
        memcpy((void*)currentSubTree,(void*)&kdSubTreeArray[clusterNo],sizeof(KDSubTree));
        memcpy((void*)((uintptr_t)currentSubTree+sizeof(KDSubTree)),(void*)kdSubTreeArray[clusterNo].GetRootNode(),sizeof(KDTree::BranchNode)*kdSubTreeArray[clusterNo].GetNumBranchNodes());
        currentSubTree->SetRootNode((KDTree::BranchNode*)((uintptr_t)currentSubTree+sizeof(KDSubTree)));
        ClusteredMeshCluster &currentCluster = clusteredMesh.GetCluster(clusterNo);

        //Check Copy
        bool copiedOkay = true;
        copiedOkay = copiedOkay && kdSubTreeArray[clusterNo].GetNumBranchNodes()==currentSubTree->GetNumBranchNodes();
        copiedOkay = copiedOkay && kdSubTreeArray[clusterNo].GetBranchNodeOffset()==currentSubTree->GetBranchNodeOffset();

        for(uint32_t nodeNumber =0; nodeNumber<kdSubTreeArray[clusterNo].GetNumBranchNodes();nodeNumber++)
        {
            copiedOkay = kdSubTreeArray[clusterNo].m_branchNodes[nodeNumber].m_parent == currentSubTree->m_branchNodes[nodeNumber].m_parent;
        }
        EATESTAssert(copiedOkay, "Check All Nodes have been copied");

        //Check each KDSubTree isValid()
        EATESTAssert(currentSubTree->IsValid(),"KDSubTree isValid()");

        //Check each KDSubTree contains the correct number of entries
        EATESTAssert(currentSubTree->m_numEntries==currentCluster.unitCount,"Correct Number of Entries");

        //Check each KDSubTree has the correct bounding box
        rw::collision::AABBox clusterBBox;
        VecFloat compressionGranularity = clusteredMesh.GetVertexCompressionGranularity();
        clusterBBox.m_min=Vector3(MAX_FLOAT,MAX_FLOAT,MAX_FLOAT);
        clusterBBox.m_max=Vector3(-MAX_FLOAT,-MAX_FLOAT,-MAX_FLOAT);
        for(uint8_t vertexNo=0;vertexNo<currentCluster.vertexCount;vertexNo++)
        {
            clusterBBox.Union(currentCluster.GetVertex(vertexNo,clusteredMesh.GetVertexCompressionGranularity()));
        }
        clusterBBox.m_max+=compressionGranularity;
        clusterBBox.m_min-=compressionGranularity;
        EATESTAssert(clusterBBox.m_max==currentSubTree->m_bbox.m_max &&
                     clusterBBox.m_min==currentSubTree->m_bbox.m_min,"Incorrect Bounding Box");

        //Check Bounding Box Query of whole cluster returns all entries
        uint32_t entry=0;
        uint32_t numEntries=0;
        uint32_t totalEntries=0;
        KDTreeBBoxQuery bboxquery(currentSubTree,clusterBBox);

        while (bboxquery.GetNext(entry,numEntries))
        {
            uint32_t index;
            uint32_t unit;
            clusteredMesh.GetClusterIndexAndUnitFromNode(entry,index,unit);
            //Ensure entries belong to correct cluster on KDTree
            if(index==clusterNo)
            {
                totalEntries+=numEntries;
            }
        }
        EATESTAssert(totalEntries==currentCluster.unitCount, "BBox Query returns all entries");

        //Check Line Query on Cluster returns the same results as on the ClusteredMesh
        //Construct Line through KDSubTree (from min to max bbox extents)
        uint32_t totalClusterQueryEntries=0;
        uint32_t totalKDTreeQueryEntries=0;
        Vector3 bboxMin= clusterBBox.Min();
        Vector3 bboxMax= clusterBBox.Max();
        //Run Query on KDSubTree
        entry=0;
        numEntries=0;
        KDTreeLineQuery clusterLineQuery(currentSubTree,bboxMin,bboxMax);
        while(clusterLineQuery.GetNext(entry,numEntries))
        {
            uint32_t index;
            uint32_t unit;
            clusteredMesh.GetClusterIndexAndUnitFromNode(entry,index,unit);
            //Ensure entries belong to correct cluster on KDTree
            if(index==clusterNo)
            {
                totalClusterQueryEntries+=numEntries;
            }
        }
        //Run Query on KDTree
        entry=0;
        numEntries=0;
        KDTreeLineQuery kdtreeLineQuery(clusteredMesh.GetKDTree(),bboxMin,bboxMax);
        while(kdtreeLineQuery.GetNext(entry,numEntries))
        {
            uint32_t index;
            uint32_t unit;
            clusteredMesh.GetClusterIndexAndUnitFromNode(entry,index,unit);
            //Ensure entries belong to correct cluster on KDTree
            if(index==clusterNo)
            {
                totalKDTreeQueryEntries+=numEntries;
            }
        }
        //Check same number of entries
        EATESTAssert(totalClusterQueryEntries==totalKDTreeQueryEntries, "LineQuery returns same results on Cluster as on ClusteredMesh")

        //Free the copy
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(kdSubTreeRes.GetMemory());
    }

    //Free Memory
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(&clusteredMesh);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(arrayRes.GetMemory());
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(workspaceRes.GetMemory());
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(clusteredMeshVolume);
}

void TestKDSubTree::TestSerialize()
{
    uint32_t branchNodeIndex = 4;
    uint32_t numBranchNodes = 7;
    uint32_t numEntries = 6;
    uint32_t defaultEntry = 5;
    rw::collision::AABBox bbox(rwpmath::Vector3(-1.0f, -2.0f, 3.0f), rwpmath::Vector3(4.0f, -1.5f, 6.0f));

    static const uint32_t bufferSize = 500;
    uint8_t buffer[bufferSize];
    {
        KDTree * parent = EA::Physics::UnitFramework::Creator<KDTree>().New(12u, 14u, bbox);
        KDSubTree testKDSubTree;
        testKDSubTree.Initialize(parent, branchNodeIndex, numBranchNodes, numEntries, defaultEntry, bbox);

        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_oarchive<SimpleStream,
            EA::Serialization::Endian::LittleEndianConverter> oArchive(strm);
        oArchive & testKDSubTree;
        EATESTAssert(oArchive.Close(), "Should be able to close archive");
        EATESTAssert(!oArchive.fail(), "Should not have failed to write");
    }

    {
        rw::collision::AABBox bbox2(2.0f*bbox.Min(), 3.0f*bbox.Max());
        KDTree * parent2 = EA::Physics::UnitFramework::Creator<KDTree>().New(12u, 14u, bbox2);
        KDSubTree testKDSubTree2;
        testKDSubTree2.Initialize(parent2, branchNodeIndex*2u, numBranchNodes*2u, numEntries*2u, defaultEntry*2u, bbox2);

        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_iarchive<SimpleStream,
            EA::Serialization::Endian::LittleEndianConverter> iArchive(strm);
        iArchive & testKDSubTree2;
        EATESTAssert(iArchive.Close(), "Should be able to close archive");
        EATESTAssert(!iArchive.fail(), "Should not have failed to write");

        EATESTAssert(branchNodeIndex == testKDSubTree2.GetBranchNodeOffset(), "Should have read BranchNodeOffset");
        EATESTAssert(numBranchNodes == testKDSubTree2.GetNumBranchNodes(), "Should have read NumBranchNodes");
        EATESTAssert(numEntries == testKDSubTree2.GetNumEntries(), "Should have read NumEntries");
        EATESTAssert(defaultEntry == testKDSubTree2.GetDefaultEntry(), "Should have read DefaultEntry");
        EATESTAssert(bbox.Min() == testKDSubTree2.m_bbox.Min(), "Should have read BBox.Min");
        EATESTAssert(bbox.Max() == testKDSubTree2.m_bbox.Max(), "Should have read BBox.Max");

        testKDSubTree2.AttachToKDTree(parent2);
        EATESTAssert(parent2->m_branchNodes + testKDSubTree2.GetBranchNodeOffset() == testKDSubTree2.GetRootNode(),
            "Should have attached the branch nodes to parent");
    }
}
