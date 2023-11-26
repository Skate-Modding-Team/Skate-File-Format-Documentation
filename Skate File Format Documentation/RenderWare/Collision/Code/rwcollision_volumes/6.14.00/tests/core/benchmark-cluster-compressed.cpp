// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************
Purpose: unit tests to benchmark extraction of triangles from compressed clusters
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
            ClusterBenchmark gClusterBenchmarkCompressed("BenchmarkClusterCompressed", "benchmark-cluster-compressed.elf", 
                rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED, false, true);

#ifdef  EA_PLATFORM_PS3_SPU
            // On non-SPU this is already defined in benchmark-cluster.cpp
            EA_ALIGNED(uint8_t, ClusterBenchmark::sResultsBuffer[ClusterBenchmark::RESULTS_SIZE], 16);
#endif

        }
    }
}
