// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: benchmark-cluster-noids.cpp

Purpose: unit tests to benchmark extraction of triangles from uncompressed clusters with triangles but no ids

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
            ClusterBenchmark gClusterBenchmarkNoIDs("BenchmarkClusterNoIDs", "benchmark-cluster-noids.elf", 
                rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, false, false);

#ifdef  EA_PLATFORM_PS3_SPU
            // On non-SPU this is already defined in benchmark-cluster.cpp
            EA_ALIGNED(uint8_t, ClusterBenchmark::sResultsBuffer[ClusterBenchmark::RESULTS_SIZE], 16);
#endif

        }
    }
}
