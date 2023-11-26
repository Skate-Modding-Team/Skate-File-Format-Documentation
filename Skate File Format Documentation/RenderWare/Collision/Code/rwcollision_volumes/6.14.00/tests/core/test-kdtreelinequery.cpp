// (c) Electronic Arts. All Rights Reserved.
#ifdef _MSC_VER
#pragma warning(disable: 4700)
#endif

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/kdtreelinequery.h>

#include <rw/collision/aabbox.h>
#include <rw/collision/kdtree.h>
#include <rw/collision/volumelinequery.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/volume.h>
#include <rw/collision/aggregatevolume.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

#include "unit/unit.h"
#include "benchmarkenvironment/timer.h"
#include "benchmarkenvironment/statistics.h"

#include "clusteredmesh_test_helpers.hpp"
#include "fakekdtree.hpp"

#include "stdio.h"     // for sprintf()

using namespace rwpmath;
using namespace rw::collision;
using namespace rw::collision::unittest;

namespace
{

const float EPSILON = 1e-6f;

class TestKDTreeLineQuery : public tests::TestSuiteBase
{
public:
    TestKDTreeLineQuery()
    {
    }

    virtual ~TestKDTreeLineQuery()
    {
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
    }

    virtual void TeardownSuite()
    {
       EA::Physics::UnitFramework::ResetAllocator();
       tests::TestSuiteBase::TeardownSuite();
    }

#define REGISTER_KDTREE_LINEQUERYTEST(M,D) EATEST_REGISTER(#M, D, TestKDTreeLineQuery, M)

    virtual void Initialize()
    {
        SuiteName("TestKDTreeLineQuery");
        REGISTER_KDTREE_LINEQUERYTEST(TestInitializationWithLineOutsideKDTree,
                        "Test the initialization of a KDTreeLineQuery with a line outside the KDTree.");
        REGISTER_KDTREE_LINEQUERYTEST(TestInitializationWithKDTreeWithNoBranchNodes,
                        "Test the initialization of a KDTreeLineQuery with a KDTree with no branch nodes.");
        REGISTER_KDTREE_LINEQUERYTEST(TestInitializationWithKDTreeWithBranchNodes,
                        "Test the initialization of a KDTreeLineQuery with a KDTree with branch nodes.");
        REGISTER_KDTREE_LINEQUERYTEST(TestProcessBranchNodeWithKDTreeWithSingleBranchNode,
                        "Test ProcessBranchNode with a KDTree with only a single branch node.");
        REGISTER_KDTREE_LINEQUERYTEST(TestProcessBranchNode,
                        "Test ProcessBranchNode.");
        REGISTER_KDTREE_LINEQUERYTEST(TestProcessBranchNodeWithLineInLeftChildOnly,
                        "Test ProcessBranchNode with a line only in the left child of the root node.");
        REGISTER_KDTREE_LINEQUERYTEST(TestProcessBranchNodeWithLineInRightChildOnly,
                        "Test ProcessBranchNode with a line only in the right child of the root node.");
        REGISTER_KDTREE_LINEQUERYTEST(TestProcessBranchNodeWithLineFatness,
                        "Test ProcessBranchNode with a line with non-zero fatness.");
        REGISTER_KDTREE_LINEQUERYTEST(TestProcessBranchNodeWithLineFromRightToLeft,
                        "Test ProcessBranchNode with a line from the right child to the left.");
        REGISTER_KDTREE_LINEQUERYTEST(BenchmarkLineQuery,
                        "Benchmark KDTree line query");
        REGISTER_KDTREE_LINEQUERYTEST(BenchmarkLineClipper, 
                        "Benchmark AALineClipper.ClipToAABBox()");
        REGISTER_KDTREE_LINEQUERYTEST(BenchmarkMeshLineQuery, 
                        "Benchmark realistic KDTree line query within ClusteredMesh");
    }

#undef REGISTER_KDTREE_LINEQUERYTEST

    void TestInitializationWithLineOutsideKDTree()
    {
        KDTree *kdtree = GetKDTreeWithNoBranchNodes();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start(-2.0f, -2.0f, -2.0f);
        Vector3 end(-1.0f, -1.0f, -1.0f);
        float fatness = 0.1f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        EATESTAssert(0 == query.m_top, "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }


    void TestInitializationWithKDTreeWithNoBranchNodes()
    {
        KDTree *kdtree = GetKDTreeWithNoBranchNodes();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start(-0.2f, -0.2f, -0.2f);
        Vector3 end( 0.2f,  0.2f,  0.2f);
        float fatness = 0.1f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        EATESTAssert(0 == query.m_top, "");
        EATESTAssert(kdtree->GetNumEntries() == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");

    }


    void TestInitializationWithKDTreeWithBranchNodes()
    {
        KDTree *kdtree = GetKDTreeWithBranchNodes();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start(-0.2f, -0.2f, -0.2f);
        Vector3 end( 0.2f,  0.2f,  0.2f);
        float fatness = 0.1f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        EATESTAssert(1 == query.m_top, "");
        EATESTAssert(rwcKDTREE_BRANCH_NODE == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(0 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }


    void TestProcessBranchNodeWithKDTreeWithSingleBranchNode()
    {
        KDTree *kdtree = GetKDTreeWithSingleBranchNode();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start(-0.2f, -0.2f, -0.2f);
        Vector3 end( 0.2f,  0.2f,  0.2f);
        float fatness = 0.0f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        query.ProcessBranchNode();

        EATESTAssert(2 == query.m_top, "");
        EATESTAssert(2 == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(1 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(1 == query.m_stack[1].m_nodeRef.m_content, "");
        EATESTAssert(0 == query.m_stack[1].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[1].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[1].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }


    void TestProcessBranchNode()
    {
        KDTree *kdtree = GetKDTreeWithBranchNodes();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start(-0.2f, -0.2f, -0.2f);
        Vector3 end( 0.2f,  0.2f,  0.2f);
        float fatness = 0.0f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        query.ProcessBranchNode();

        EATESTAssert(2 == query.m_top, "");
        EATESTAssert(rwcKDTREE_BRANCH_NODE == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(2 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(rwcKDTREE_BRANCH_NODE == query.m_stack[1].m_nodeRef.m_content, "");
        EATESTAssert(1 == query.m_stack[1].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[1].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[1].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");

        query.ProcessBranchNode();

        EATESTAssert(3 == query.m_top, "");
        EATESTAssert(rwcKDTREE_BRANCH_NODE == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(2 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(2 == query.m_stack[1].m_nodeRef.m_content, "");
        EATESTAssert(1 == query.m_stack[1].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.25f, query.m_stack[1].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[1].m_pb, EPSILON), "");
        EATESTAssert(1 == query.m_stack[2].m_nodeRef.m_content, "");
        EATESTAssert(0 == query.m_stack[2].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[2].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[2].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");

        query.m_top = 1; // remove the two leaf nodes on top of the stack
        query.ProcessBranchNode();

        EATESTAssert(2 == query.m_top, "");
        EATESTAssert(4 == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(6 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(3 == query.m_stack[1].m_nodeRef.m_content, "");
        EATESTAssert(3 == query.m_stack[1].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[1].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(0.75f, query.m_stack[1].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }


    void TestProcessBranchNodeWithLineInLeftChildOnly()
    {
        KDTree *kdtree = GetKDTreeWithSingleBranchNode();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start(-0.05f, -0.05f, -0.05f);
        Vector3 end(-0.25f, -0.25f, -0.25f);
        float fatness = 0.0f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        query.ProcessBranchNode();

        EATESTAssert(1 == query.m_top, "");
        EATESTAssert(1 == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(0 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }


    void TestProcessBranchNodeWithLineInRightChildOnly()
    {
        KDTree *kdtree = GetKDTreeWithSingleBranchNode();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start( 0.05f,  0.05f,  0.05f);
        Vector3 end( 0.25f,  0.25f,  0.25f);
        float fatness = 0.0f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        query.ProcessBranchNode();

        EATESTAssert(1 == query.m_top, "");
        EATESTAssert(2 == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(1 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }


    void TestProcessBranchNodeWithLineFatness()
    {
        KDTree *kdtree = GetKDTreeWithSingleBranchNode();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start(-0.2f, -0.2f, -0.2f);
        Vector3 end( 0.2f,  0.2f,  0.2f);
        float fatness = 0.1f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        query.ProcessBranchNode();

        EATESTAssert(2 == query.m_top, "");
        EATESTAssert(2 == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(1 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.25f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(1 == query.m_stack[1].m_nodeRef.m_content, "");
        EATESTAssert(0 == query.m_stack[1].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[1].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(0.75f, query.m_stack[1].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }

    void TestProcessBranchNodeWithLineFromRightToLeft()
    {
        KDTree *kdtree = GetKDTreeWithSingleBranchNode();
        KDTreeHolder kdtreeHolder(kdtree);
        Vector3 start( 0.2f,  0.2f,  0.2f);
        Vector3 end(-0.2f, -0.2f, -0.2f);
        float fatness = 0.0f;
        KDTreeLineQuery query(kdtree, start, end, fatness);

        query.ProcessBranchNode();

        EATESTAssert(2 == query.m_top, "");
        EATESTAssert(1 == query.m_stack[0].m_nodeRef.m_content, "");
        EATESTAssert(0 == query.m_stack[0].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[0].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(1.0f, query.m_stack[0].m_pb, EPSILON), "");
        EATESTAssert(2 == query.m_stack[1].m_nodeRef.m_content, "");
        EATESTAssert(1 == query.m_stack[1].m_nodeRef.m_index, "");
        EATESTAssert(IsSimilar(0.0f, query.m_stack[1].m_pa, EPSILON), "");
        EATESTAssert(IsSimilar(0.5f, query.m_stack[1].m_pb, EPSILON), "");
        EATESTAssert(0 == query.m_leafCount, "");
        EATESTAssert(0 == query.m_nextEntry, "");
    }

    void BenchmarkLineQuery()
    {
        rw::collision::KDTreeBase * kdtree = (rw::collision::KDTreeBase *) GetComplexKDTree();
        rw::collision::AABBox allBBox = kdtree->GetBBox();
        rwpmath::Vector3 start = allBBox.Min();
        rwpmath::Vector3 end = allBBox.Max();

        benchmarkenvironment::Sample samples(mMaxSamples);
        bool moreSamplesNeeded = true;
        do
        {
            benchmarkenvironment::Timer timer;
            timer.Start();
            uint32_t checksum = PerformSingleLineQuery(kdtree, start, end);
            timer.Stop();
            EATESTAssert(checksum == 0x51 + 0x4e18f8, "Didn't extract expected leaves");
            moreSamplesNeeded = samples.AddElement(timer.AsSeconds()*1000.0f*1000.0f);
        } while (moreSamplesNeeded);
        SendBenchmark(samples, "avg microseconds to perform 1 query");
    }

    void BenchmarkLineClipper()
    {
        rw::collision::AABBox bbox(rwpmath::Vector3(-1.0f, -2.0f, -3.0f), rwpmath::Vector3(1.0f, 2.0f, 3.0f));

        benchmarkenvironment::Sample samples(mMaxSamples);
        bool moreSamplesNeeded = true;
        do
        {
            benchmarkenvironment::Timer timer;
            timer.Start();
            uint32_t numHits = PerformClipToAABBox(bbox, 100);
            timer.Stop();
            EATESTAssert(0x5c == numHits, "Should return expected number of hits");
            moreSamplesNeeded = samples.AddElement(timer.AsSeconds()*1000.0f*1000.0f);
        } while (moreSamplesNeeded);
        SendBenchmark(samples, "avg microseconds to perform 100 clips");
    }

    // Benchmark a complete query against a mesh to track effects of any optimizations at the user level.
    void BenchmarkMeshLineQuery()
    {
        const uint32_t STACKSIZE = 10;
        const uint32_t RESBUFSIZE = 10;
        rw::collision::VolumeLineQuery *volLineQuery = 
            EA::Physics::UnitFramework::Creator<rw::collision::VolumeLineQuery>().New(STACKSIZE, RESBUFSIZE); 

        const rw::collision::Volume * volumeArray[1];
        volumeArray[0] = GetClusteredMeshVolume();
        rwpmath::Matrix44Affine mtx = rwpmath::GetMatrix44Affine_Identity();
        const rwpmath::Matrix44Affine * volumeMtxPtrArray[1];
        volumeMtxPtrArray[0] = & mtx;

        rw::collision::AABBox allBBox;
        volumeArray[0]->GetBBox(&mtx, true, allBBox);
        const rwpmath::Vector3 start = allBBox.Min();
        const rwpmath::Vector3 end = allBBox.Max();

        benchmarkenvironment::Sample samples(mMaxSamples);
        bool moreSamplesNeeded = true;
        do
        {
            benchmarkenvironment::Timer timer;
            timer.Start();
            uint32_t checksum = PerformMeshLineQuery(volLineQuery, volumeArray, volumeMtxPtrArray, start, end);
            timer.Stop();
            EATESTAssert(checksum == 1 + 0x901, "Should return expected number of hits");
            moreSamplesNeeded = samples.AddElement(timer.AsSeconds()*1000.0f*1000.0f);
        } while (moreSamplesNeeded);
        SendBenchmark(samples, "avg microseconds to perform 1 mesh query");
    }

private:

    void SendBenchmark(benchmarkenvironment::Sample & timer, const char * benchmarkName, const double factor = 1.0f)
    {
        char str[256];
#if defined(EA_PLATFORM_PS3_SPU)
        // Distinguish SPU metrics on PS3 from PPU metrics
        const char * platform = "spu - ";
#else
        const char * platform = "";
#endif
        sprintf(str, "%s%s - %s", platform, "TestKDTreeLineQuery", benchmarkName);
        EATESTSendBenchmark(str, factor*timer.GetMean(), factor*timer.GetMin(), factor*timer.GetMax());
    }

    uint32_t PerformSingleLineQuery(const rw::collision::KDTreeBase * kdtree, 
        rwpmath::Vector3::InParam start,
        rwpmath::Vector3::InParam end)
    {
        uint32_t checksum = 0, count = 0;
        rw::collision::KDTree::LineQuery mapQuery(kdtree, start, end);
        uint32_t index = rwcKDTREE_INVALID_INDEX;
        while (mapQuery.GetNext(index))
        {
            ++count;
            checksum += index;
        }
        return count + checksum;
    }

    uint32_t PerformClipToAABBox(const rw::collision::AABBox & bbox, uint32_t num)
    {
        rwpmath::Vector3 d = bbox.Max() - bbox.Min();
        rwpmath::Vector3 end = bbox.Max() + 5.0f * d;
        rwpmath::Vector3 start = bbox.Min() - 5.0f * d;
        rwpmath::Vector3 incr(0.0f, 0.0f, 0.0f);
        incr.SetZ(2.0f*(bbox.Max().GetZ() - bbox.Min().GetZ())/(float) num);
        rw::collision::AABBox allBBox(bbox.Min() * 10.0f, bbox.Max() * 5.0f);
        uint32_t numHits = 0;
        for (uint32_t i = 0; i < num; ++i)
        {
            rw::collision::AALineClipper clipper(start, end, allBBox);
            float a = 0.0f;
            float b = 1.0f;
            if (clipper.ClipToAABBox(a, b, bbox))
            {
                numHits++;
            }
            start += incr;
        }
        return numHits;
    }

    uint32_t PerformMeshLineQuery(rw::collision::VolumeLineQuery *volLineQuery,
        const rw::collision::Volume ** volumeArray, 
        const rwpmath::Matrix44Affine ** volumeMtxPtrArray,
        rwpmath::Vector3::InParam lineStart, rwpmath::Vector3::InParam lineEnd)
    {
        uint32_t checksum = 0, count = 0;
        rw::collision::VolumeLineSegIntersectResult *results = volLineQuery->GetIntersectionResultsBuffer();

        //Initialize the specific query parameters
        volLineQuery->InitQuery(volumeArray, volumeMtxPtrArray, 1, lineStart, lineEnd);

        //Continue while there are still volumes left to query
        while(!volLineQuery->Finished())
        {
            //Get as many results as possible
            uint32_t numRes = volLineQuery->GetAllIntersections();

            //Process the results
            for(uint32_t i=0; i<numRes; i++)
            {
                count++;
                checksum += results[i].vRef.tag;
            }
        }
        return count + checksum;
    }

    const rw::collision::KDTreeBase * GetComplexKDTree()
    {
        const rw::collision::AggregateVolume * clusteredMeshVolume = 
            static_cast<const rw::collision::AggregateVolume*>(GetClusteredMeshVolume());
        const rw::collision::ClusteredMesh * clusteredMesh = 
            static_cast<const rw::collision::ClusteredMesh*>(clusteredMeshVolume->GetAggregate());
        return clusteredMesh->GetKDTreeBase();
    }

    const rw::collision::Volume * GetClusteredMeshVolume()
    {
        const char * file = "courtyard.dat";

        // Load ClusteredMesh on platforms supporting file loads
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(file);
        EA_ASSERT(clusteredMeshVolume);
        return clusteredMeshVolume;
    }

    static const uint32_t mMaxSamples = 100u;   // Max allowed by benchmarkenvironment

} testKDTreeLineQueryInstance;

}
