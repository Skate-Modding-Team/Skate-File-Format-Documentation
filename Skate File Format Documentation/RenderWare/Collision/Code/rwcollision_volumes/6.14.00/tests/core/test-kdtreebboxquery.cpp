// (c) Electronic Arts. All Rights Reserved.
#ifdef _MSC_VER
#pragma warning(disable: 4700)
#endif

#include <new>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/initialize.h>
#include <rw/collision/volume.h>
#include <rw/collision/aggregatevolume.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/aabbox.h>
#include <rw/collision/kdtree.h>
#include <rw/collision/kdtreebboxquery.h>

#include "unit/unit.h"
#include "benchmarkenvironment/timer.h"
#include "benchmarkenvironment/statistics.h"

#include "testsuitebase.h" // For TestSuiteBase

#include "clusteredmesh_test_helpers.hpp"
#include "fakekdtree.hpp"

#include "stdio.h"     // for sprintf()

using namespace rwpmath;
using namespace rw::collision;
using namespace rw::collision::unittest;

namespace
{

class TestKDTreeBBoxQuery : public tests::TestSuiteBase
{
public:
    TestKDTreeBBoxQuery()
    {
    }

    virtual ~TestKDTreeBBoxQuery()
    {
    }

    virtual void Initialize()
    {
        rw::collision::InitializeVTables();

        SuiteName("TestKDTreeBBoxQuery");
        EATEST_REGISTER("TestKDTreeWithNoBranchNodes",
                        "Test the KDTreeBBoxQuery with a KDTree with no branch nodes.",
                        TestKDTreeBBoxQuery,
                        TestKDTreeWithNoBranchNodes);
        EATEST_REGISTER("TestKDTreeWithBranchNodes",
                        "Test the KDTreeBBoxQuery with a KDTree with branch nodes.",
                        TestKDTreeBBoxQuery,
                        TestKDTreeWithBranchNodes);
        EATEST_REGISTER("TestComplexKDTree",
                        "Test complete query with a KDTree with branch nodes.",
                        TestKDTreeBBoxQuery,
                        TestComplexKDTree);
        EATEST_REGISTER("BenchmarkComplexQuery",
                        "Benchmark complete query with a KDTree with branch nodes.",
                        TestKDTreeBBoxQuery,
                        BenchmarkComplexQuery);
    }

    void TestKDTreeWithNoBranchNodes()
    {
        KDTree *kdtree = GetKDTreeWithNoBranchNodes();
        KDTreeHolder kdtreeHolder(kdtree);
        rw::collision::AABBox queryBbox(-0.2f, -0.2f, -0.2f, 0.2f, 0.2f, 0.2f);
        KDTreeBBoxQuery query(kdtree, queryBbox);

        uint32_t entry = 0xcdcdcdcd, count = 0xefefeeff;
        RwpBool more = query.GetNext(entry, count);

        EATESTAssert(more, "Should return leaf first if no branches");
        EATESTAssert(kdtree->GetNumEntries() == count, "Should return all leaf entries");
        EATESTAssert(0 == entry, "Should return first entry");

        more = query.GetNext(entry, count);
        EATESTAssert(!more, "Should be nothing more if no branches");
    }


    void TestKDTreeWithBranchNodes()
    {
        KDTree *kdtree = GetKDTreeWithSingleBranchNode();
        KDTreeHolder kdtreeHolder(kdtree);
        rw::collision::AABBox queryBbox(-0.2f, -0.2f, -0.2f, 0.2f, 0.2f, 0.2f);
        KDTreeBBoxQuery query(kdtree, queryBbox);

        uint32_t entry = 0xcdcdcdcd, count = 0xefefeeff;
        RwpBool more = query.GetNext(entry, count);

        EATESTAssert(more, "Should be more than single leaf");
        EATESTAssert(0 < count, "Should return first results");
        EATESTAssert(0 == entry, "Should return first entry");

        uint32_t entry2 = 0xcdcdcdcd, count2 = 0xefefeeff;
        RwpBool more2 = query.GetNext(entry2, count2);

        EATESTAssert(!more2, "Should be no more leaves");
    }

    void TestComplexKDTree()
    {
        const rw::collision::KDTreeBase * kdtree = GetComplexKDTree();
        rw::collision::AABBox allBBox = kdtree->GetBBox();
        rw::collision::AABBox bbox = SubBBox(allBBox, 0.7f, 1.0f);

        uint32_t checksum = CheckSingleBBoxQuery(kdtree, bbox);
        EATESTAssert(checksum == 0x104744 + 0x8, "Didn't extract expected leaves");
    }

    void BenchmarkComplexQuery()
    {
        const rw::collision::KDTreeBase * kdtree = GetComplexKDTree();
        rw::collision::AABBox allBBox = kdtree->GetBBox();
        rw::collision::AABBox bbox = SubBBox(allBBox, 0.7f, 1.0f);

        benchmarkenvironment::Sample samples(mMaxSamples);
        bool moreSamplesNeeded = true;
        do
        {
            benchmarkenvironment::Timer timer;
            timer.Start();
            uint32_t checksum = CheckSingleBBoxQuery(kdtree, bbox);
            timer.Stop();
            EATESTAssert(checksum == 0x104744 + 0x8, "Didn't extract expected leaves");
            moreSamplesNeeded = samples.AddElement(timer.AsSeconds()*1000.0f*1000.0f);
        } while (moreSamplesNeeded);
        SendBenchmark(samples, "avg microseconds to perform 1 query");
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
        sprintf(str, "%s%s - %s", platform, "TestKDTreeBBoxQuery", benchmarkName);
        EATESTSendBenchmark(str, factor*timer.GetMean(), 
            factor*timer.GetMin(), factor*timer.GetMax());
    }

    rwpmath::Vector3 Interpolate(const rw::collision::AABBox & bbox, rwpmath::VecFloatInParam f)
    {
        EA_ASSERT(f >= rwpmath::GetVecFloat_Zero() && f <= rwpmath::GetVecFloat_One());
        return f * bbox.Max() + (rwpmath::GetVecFloat_One() - f) * bbox.Min();
    }

    rw::collision::AABBox SubBBox(const rw::collision::AABBox & bbox, 
        rwpmath::VecFloatInParam f, rwpmath::VecFloatInParam g)
    {
        EA_ASSERT(g > f);
        return rw::collision::AABBox (Interpolate(bbox, f), Interpolate(bbox, g));
    }

    uint32_t CheckSingleBBoxQuery(
        const rw::collision::KDTreeBase * kdtree, 
        const rw::collision::AABBox & bbox)
    {
        uint32_t checksum = 0, count = 0;
        rw::collision::KDTree::BBoxQuery mapQuery(kdtree, bbox);
        uint32_t index = rwcKDTREE_INVALID_INDEX;
        while (mapQuery.GetNext(index))
        {
            ++count;
            checksum += index;
        }
        return count + checksum;
    }

    KDTreeBase * GetComplexKDTree()
    {
        const char * file = "courtyard.dat";

        // Load ClusteredMesh on platforms supporting file loads
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(file);
        EA_ASSERT(clusteredMeshVolume);
        ClusteredMesh &clusteredMesh = *((ClusteredMesh*)((AggregateVolume*)clusteredMeshVolume)->GetAggregate());
        return clusteredMesh.GetKDTreeBase();
    }

    static const uint32_t mMaxSamples = 100u;   // Max allowed by benchmarkenvironment

} testKDTreeBBoxQuery;

}
