// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/clusteredmesh.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include "eaphysics/unitframework/serialization_test_helpers.hpp"

#include "testsuitebase.h" // For TestSuiteBase

#include "clusteredmesh_test_helpers.hpp"

#include "benchmark_timer.hpp"

using namespace rw::collision;

namespace
{
    const char *g_clusteredMeshBenchmarkFilenames[] =
    {
        "courtyard.dat",
        "skatemesh_compressed_quads_ids.dat"
    };
}

// Unit tests for clustered mesh line queries
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.

class BenchmarkClusteredMeshBoxQuery: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("BenchmarkClusteredMeshBoxQuery");

#define CLUSTERED_MESH_TEST(F, D) EATEST_REGISTER(#F, D, BenchmarkClusteredMeshBoxQuery, F)

        CLUSTERED_MESH_TEST(BenchmarkBoxQuery, "Test a VolumeLineQuery against a clustered mesh");
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        // Initialise the collision system
        Volume::InitializeVTable();
    }

    virtual void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        tests::TestSuiteBase::TeardownSuite();
    }

private:

    void BenchmarkBoxQuery();


} BenchmarkClusteredMeshBoxQuerySingleton;


void BenchmarkClusteredMeshBoxQuery::BenchmarkBoxQuery()
{
    const uint32_t STACKSIZE = 1;

    for (uint32_t cm = 0; cm < EAArrayCount(g_clusteredMeshBenchmarkFilenames); ++cm)
    {
        //Load ClusteredMesh
        Volume *clusteredMeshVolume = LoadSerializedClusteredMesh(g_clusteredMeshBenchmarkFilenames[cm]);
        EATESTAssert(clusteredMeshVolume, "Failed to load clustered mesh.");

        AggregateVolume *aggVol = static_cast<AggregateVolume *>(clusteredMeshVolume);
        ClusteredMesh *mesh = static_cast<ClusteredMesh *>(aggVol->GetAggregate());

        // Create bbox quey to extract all triangle from the clustered mesh
        VolumeBBoxQuery* bboxQuery = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(STACKSIZE, mesh->GetVolumeCount() * 2);
        EATESTAssert(bboxQuery, "Failed to create BBox query.");

        const Volume * clusteredMeshLineTestVolumeArray[] = { clusteredMeshVolume };

        AABBox volBBox;
        clusteredMeshVolume->GetBBox(0, TRUE, volBBox);
        bboxQuery->InitQuery(clusteredMeshLineTestVolumeArray, 0, 1, volBBox);

        rw::collision::Tests::BenchmarkTimer timer;

        timer.Start();
        const uint32_t numResults = bboxQuery->GetOverlaps();
        timer.Stop();

        EATESTAssert(numResults, "No results found.");
        EATESTAssert(bboxQuery->Finished(), "More results found. Increase result buffer size.");

        const double time = timer.GetAverageDurationMilliseconds();
        char buffer[256];
        sprintf(buffer, "BenchmarkClusteredMeshBoxQuery_%s", g_clusteredMeshBenchmarkFilenames[cm]);
        EATESTSendBenchmark(buffer, time, time, time);

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(bboxQuery);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(aggVol);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
    }
}
