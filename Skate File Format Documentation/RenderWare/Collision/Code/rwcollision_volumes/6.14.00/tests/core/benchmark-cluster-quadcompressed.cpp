// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: benchmark-cluster-quadcompressed.cpp

Purpose: unit tests to benchmark extraction of triangles from compressed clusters with quads

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
            ClusterBenchmark gClusterBenchmarkQuadCompressed("BenchmarkClusterQuadCompressed", "benchmark-cluster-quadcompressed.elf", 
                rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED, true, true);

#ifdef  EA_PLATFORM_PS3_SPU
            // On non-SPU this is already defined in benchmark-cluster.cpp
            EA_ALIGNED(uint8_t, ClusterBenchmark::sResultsBuffer[ClusterBenchmark::RESULTS_SIZE], 16);
#endif

        }
    }
}
