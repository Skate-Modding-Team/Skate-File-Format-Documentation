// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: benchmark-cluster.cpp

Purpose: unit tests to benchmark extraction of triangles from uncompressed clusters

*/

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <EAAssert/eaassert.h>

#include "benchmark-cluster.h"

namespace rw
{
    namespace collision
    {
        namespace Tests
        {
            // Currently only allowed one test suite per SPU executable.
            ClusterBenchmark gClusterBenchmark("BenchmarkCluster",  "benchmark-cluster.elf",
                rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, false, true);

            EA_ALIGNED(uint8_t, ClusterBenchmark::sResultsBuffer[ClusterBenchmark::RESULTS_SIZE], 16);
        }
    }
}
