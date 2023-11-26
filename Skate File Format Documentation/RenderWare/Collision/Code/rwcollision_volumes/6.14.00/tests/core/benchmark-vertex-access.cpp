// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: benchmark-vertex-access.cpp

Purpose: unit tests to benchmark extraction of vertices from clusters

*/

#include "benchmark_timer.hpp"

#include <rw/collision/common.h>
#include <rw/collision/volumedata.h>
#include <rw/collision/aabbox.h>
#include <rw/collision/volume.h>
#include <rw/collision/triangle.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/clusteredmeshcluster.h>

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <EAAssert/eaassert.h>

#include "mesh-test-base.h"

#include <rw/collision/triangleunit.h>
#include <rw/collision/genericclusterunit.h>
#include <rw/collision/clustertriangleiterator.h>

#include "stdio.h"     // for sprintf()

#include "rw/math/vpl.h"

// ***********************************************************************************************************

namespace rw 
{
    namespace collision
    {

        namespace Tests
        {
            /// Unit tests to benchmark extraction of triangle data from clustered mesh clusters.
            class BenchmarkVertexAccess : public ClusteredMeshTestBase
            {
            public:

                BenchmarkVertexAccess(): ClusteredMeshTestBase(), mCompressedMesh(0)
                {
                }

            protected:

                virtual const char * GetMeshFileName() const
                {
                    return UNITTEST_DATA_FILE("skatemesh_ids.dat");
                }
                virtual const char * GetCompressedMeshFileName() const
                {
                    return UNITTEST_DATA_FILE("skatemesh_compressed_ids.dat");
                }
                virtual const char * GetSuiteName() const
                {
                    return "BenchmarkVertexAccess";
                }

                virtual void Initialize()
                {
#define REGISTER_VERTEX_ACCESS_TEST(M,D) EATEST_REGISTER(#M, D, BenchmarkVertexAccess, M)

                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGetVertexDynamic, "Test extracting vertex with dynamic compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGetVertexStatic, "Test extracting vertex with static compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet3VerticesDynamic, "Test extracting 3 vertices with dynamic compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet3VerticesStatic, "Test extracting 3 vertices with static compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGetVertexCompressedDynamic, "Test extracting compressed vertex with dynamic compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGetVertexCompressedStatic, "Test extracting compressed vertex with static compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet3VerticesCompressedDynamic, "Test extracting 3 compressed vertices with dynamic compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet3VerticesCompressedStatic, "Test extracting 3 compressed vertices with static compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet4VerticesDynamic, "Test extracting 4 vertices with dynamic compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet4VerticesStatic, "Test extracting 4 vertices with static compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet4VerticesCompressedDynamic, "Test extracting 4 compressed vertices with dynamic compression");
                    REGISTER_VERTEX_ACCESS_TEST(BenchmarkGet4VerticesCompressedStatic, "Test extracting 4 compressed vertices with static compression");
                    // Run the whole tests suite on SPU too
                    // EATEST_REGISTER_SPU_ARG("ClusterVertexBenchmarkSPU", "SPU vertex access benchmarks", "benchmark-vertex-access.elf", (uint64_t) &mClusterInfo);

                    ClusteredMeshTestBase::Initialize();
                }

                virtual void Setup()
                {
                    ClusteredMeshTestBase::Setup();
#if !defined(EA_PLATFORM_PS3_SPU)
                    if (!mCompressedMesh)
                    {
                        // Load a compressed version of the mesh
                        mCompressedMesh = LoadSerializedMesh(GetCompressedMeshFileName());
                    }
                    CreateClusterInfo(mClusterInfo[1], mCompressedMesh);
#endif
                }

                virtual void Setup(void *)
                {
                }

                virtual void Uninitialize()
                {
#if !defined(EA_PLATFORM_PS3_SPU)
                    // free mesh created by ConvertMesh()
                    if (mCompressedMesh)
                    {
                        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mCompressedMesh);
                        mCompressedMesh = 0;
                    }
#endif
                    ClusteredMeshTestBase::Uninitialize();
                }

            private:

                template <uint8_t COMPRESSION>
                void BenchmarkGetVertex(ClusterInfo & clusterInfo, const char * name)
                {
                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    float vertexCompressionGranularity = clusterInfo.clusterParams.mVertexCompressionGranularity;
                    uint8_t numVertices = cluster->vertexCount;

                    rwpmath::Vector3 min = GetVector3_Large();
                    rwpmath::Vector3 max = -min;

                    BenchmarkTimer timer;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        min = GetVector3_Large();
                        max = -min;

                        timer.Start();
                        for (uint8_t v = 0; v < numVertices; ++v)
                        {
                            uint8_t v0 = (uint8_t) (v>>1);
                            uint8_t v1 = v;
                            uint8_t v2 = (uint8_t) (v>>2);
                            rwpmath::Vector3 p0 = cluster->GetVertexBase<COMPRESSION>(v0, vertexCompressionGranularity);
                            rwpmath::Vector3 p1 = cluster->GetVertexBase<COMPRESSION>(v1, vertexCompressionGranularity);
                            rwpmath::Vector3 p2 = cluster->GetVertexBase<COMPRESSION>(v2, vertexCompressionGranularity);
                            // Do something with the result to ensure it gets used
                            min = rwpmath::Min(rwpmath::Min(min, p0), rwpmath::Min(p1, p2));
                            max = rwpmath::Max(rwpmath::Max(max, p0), rwpmath::Max(p1, p2));
                        }
                        timer.Stop();
                    }

                    SendBenchmark(timer, "ms using GetVertex", name);

                    EATESTAssert((float) min.GetX() < (float) max.GetX(), "Non-zero bounds in X");
                    EATESTAssert((float) min.GetY() < (float) max.GetY(), "Non-zero bounds in Y");
                    EATESTAssert((float) min.GetZ() < (float) max.GetZ(), "Non-zero bounds in Z");
                }

                void BenchmarkGetVertexDynamic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    BenchmarkGetVertex<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC>(clusterInfo, "UNCOMPRESSED-DYNAMIC");
                }
                void BenchmarkGetVertexStatic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    BenchmarkGetVertex<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>(clusterInfo, "UNCOMPRESSED");
                }
                void BenchmarkGetVertexCompressedDynamic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[1];
                    BenchmarkGetVertex<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC>(clusterInfo, "16BIT-DYNAMIC");
                }
                void BenchmarkGetVertexCompressedStatic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[1];
                    BenchmarkGetVertex<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(clusterInfo, "16BIT");
                }

                template <uint8_t COMPRESSION>
                void BenchmarkGet3Vertices(ClusterInfo & clusterInfo, const char * parameters)
                {
                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    float vertexCompressionGranularity = clusterInfo.clusterParams.mVertexCompressionGranularity;
                    uint8_t numVertices = cluster->vertexCount;

                    rwpmath::Vector3 min = GetVector3_Large();
                    rwpmath::Vector3 max = -min;

                    BenchmarkTimer timer;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        min = GetVector3_Large();
                        max = -min;

                        timer.Start();
                        for (uint8_t v = 0; v < numVertices; ++v)
                        {
                            rwpmath::Vector3 p0, p1, p2;
                            uint8_t v0 = (uint8_t) (v>>1);
                            uint8_t v1 = v;
                            uint8_t v2 = (uint8_t) (v>>2);
                            cluster->Get3VerticesBase<COMPRESSION>(p0, p1, p2, v0, v1, v2, vertexCompressionGranularity);
                            // Do something with the result to ensure it gets used
                            min = rwpmath::Min(rwpmath::Min(min, p0), rwpmath::Min(p1, p2));
                            max = rwpmath::Max(rwpmath::Max(max, p0), rwpmath::Max(p1, p2));
                        }
                        timer.Stop();
                    }

                    SendBenchmark(timer, "Get3Vertices", "ms using Get3Vertices", parameters);

                    EATESTAssert((float) min.GetX() < (float) max.GetX(), "Non-zero bounds in X");
                    EATESTAssert((float) min.GetY() < (float) max.GetY(), "Non-zero bounds in Y");
                    EATESTAssert((float) min.GetZ() < (float) max.GetZ(), "Non-zero bounds in Z");
                }

                void BenchmarkGet3VerticesDynamic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    BenchmarkGet3Vertices<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC>(clusterInfo, "Decompression:UNCOMPRESSED-DYNAMIC");
                }
                void BenchmarkGet3VerticesStatic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    BenchmarkGet3Vertices<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>(clusterInfo, "Decompression:RAW");
                }
                void BenchmarkGet3VerticesCompressedDynamic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[1];
                    BenchmarkGet3Vertices<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC>(clusterInfo, "Decompression:16BIT-DYNAMIC");
                }
                void BenchmarkGet3VerticesCompressedStatic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[1];
                    BenchmarkGet3Vertices<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(clusterInfo, "Decompression:16BIT");
                }

                template <uint8_t COMPRESSION>
                void BenchmarkGet4Vertices(ClusterInfo & clusterInfo, const char * name)
                {
                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    float vertexCompressionGranularity = clusterInfo.clusterParams.mVertexCompressionGranularity;
                    uint8_t numVertices = cluster->vertexCount;

                    rwpmath::Vector3 min = GetVector3_Large();
                    rwpmath::Vector3 max = -min;

                    BenchmarkTimer timer;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        min = GetVector3_Large();
                        max = -min;

                        timer.Start();
                        for (uint8_t v = 0; v < numVertices; ++v)
                        {
                            rwpmath::Vector3 p0, p1, p2, p3;
                            uint8_t v0 = (uint8_t) (v>>1);
                            uint8_t v1 = v;
                            uint8_t v2 = (uint8_t) (v>>2);
                            uint8_t v3 = (uint8_t) ((v>>2)+1u);
                            cluster->Get4VerticesBase<COMPRESSION>(p0, p1, p2, p3, v0, v1, v2, v3, vertexCompressionGranularity);
                            // Do something with the result to ensure it gets used
                            min = rwpmath::Min(rwpmath::Min(rwpmath::Min(p3, p0), rwpmath::Min(p1, p2)), min);
                            max = rwpmath::Max(rwpmath::Max(rwpmath::Max(p3, p0), rwpmath::Max(p1, p2)), max);
                        }
                        timer.Stop();
                    }

                    SendBenchmark(timer, "ms using Get4Vertices", name);

                    EATESTAssert((float) min.GetX() < (float) max.GetX(), "Non-zero bounds in X");
                    EATESTAssert((float) min.GetY() < (float) max.GetY(), "Non-zero bounds in Y");
                    EATESTAssert((float) min.GetZ() < (float) max.GetZ(), "Non-zero bounds in Z");
                }

                void BenchmarkGet4VerticesDynamic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    BenchmarkGet4Vertices<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC>(clusterInfo, "UNCOMPRESSED-DYNAMIC");
                }
                void BenchmarkGet4VerticesStatic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    BenchmarkGet4Vertices<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>(clusterInfo, "UNCOMPRESSED");
                }
                void BenchmarkGet4VerticesCompressedDynamic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[1];
                    BenchmarkGet4Vertices<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC>(clusterInfo, "16BIT-DYNAMIC");
                }
                void BenchmarkGet4VerticesCompressedStatic()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[1];
                    BenchmarkGet4Vertices<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(clusterInfo, "16BIT");
                }
            private:

                /// How many iterations to do to get a semi-reliable timing result
#if defined(EA_PLATFORM_WINDOWS)
                static const uint32_t mNumIterations = 50;
#else
                static const uint32_t mNumIterations = 10;
#endif

#if !defined(EA_PLATFORM_PS3_SPU)
                /// Compressed version of the mesh we're testing
                rw::collision::ClusteredMesh * mCompressedMesh;
#endif


            } benchmarkVertexAccess;
        }
    }
}

// ***********************************************************************************************************
