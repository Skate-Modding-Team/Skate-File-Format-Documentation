// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: benchmark-cluster.h

Purpose: unit tests to benchmark extraction of triangles from clusters

*/

// Use simulated VPL to see how slow it goes
#define RW_MATH_ENABLE_SIMULATED_VPL_OPERATIONS 

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
            class ClusterBenchmark : public ClusteredMeshTestBase
            {
            public:

                ClusterBenchmark(const char * name, 
                    const char * spuElf,
                    uint8_t compression,
                    bool useQuads, bool useIDs)
                    : mName(name), mSpuElf(spuElf), mCompression(compression), mUseQuads(useQuads), mUseIDs(useIDs)
                {
                    const char * meshFilename = "skatemesh";
                    sprintf(mMeshFilename, UNITTEST_DATA_FILE("%s%s%s%s.dat"), 
                        meshFilename,
                        (mCompression != 0) ? "_compressed" : "", 
                        mUseQuads ? "_quads" : "", 
                        mUseIDs ? "_ids" : "");
                    EA_UNUSED(mSpuElf);
                }

            protected:

                virtual const char * GetSuiteName() const
                {
                    return mName;
                }
                virtual const char * GetMeshFileName() const
                {
                    return mMeshFilename;
                }

                /// Return whether two axis aligned bounding boxes overlap
                EA_FORCE_INLINE
                    rwpmath::MaskScalar Overlaps(rwpmath::Vector3::InParam bboxAMin,
                    rwpmath::Vector3::InParam bboxAMax,
                    rwpmath::Vector3::InParam bboxBMin,
                    rwpmath::Vector3::InParam bboxBMax)
                {
                    rwpmath::Vector3 temp = rwpmath::Max(bboxAMin - bboxBMax, bboxBMin - bboxAMax);
                    return rwpmath::CompAllTrue(rwpmath::CompLessThan(temp, rwpmath::GetVector3_Zero()));
                }

                virtual void Initialize()
                {
#define REGISTER_CLUSTER_TEST(M,D) EATEST_REGISTER(#M, D, ClusterBenchmark, M)

#if !defined(EA_PLATFORM_PS3_SPU)
                    REGISTER_CLUSTER_TEST(TestClusterSize, "benchmark size of clusters");
#endif //!defined(EA_PLATFORM_PS3_SPU)

                    // Following tests are run in 3 cases:
                    // 1. Using GPTriangle API (UnitGetOverlappingGPTriangles())
                    // 2. Using most appropriate new clustertriangleiterator with generic unit API
                    // 3. Using most appropriate new clustertriangleiterator with unit type specific to the mesh format in use
                    // Ideally 2 should be no worse than 1 and 3 should be better than both.

                    // benchmark extracting a single triangle
                    REGISTER_CLUSTER_TEST(TestExtractOneExisting,     "Test extracting single triangle using existing code");
                    REGISTER_CLUSTER_TEST(TestExtractOneGenericUnit,  "Test extracting single triangle using generic code");
                    if (!mUseQuads)
                    {
                        REGISTER_CLUSTER_TEST(TestExtractOneSpecificUnit, "Test extracting single triangle using specific code");
                    }

                    // benchmark extracting triangles into format suitable for eacollision_primitives queries
                    REGISTER_CLUSTER_TEST(TestExtractPPQExisting,     "Test extracting range of triangle data using existing code");
                    REGISTER_CLUSTER_TEST(TestExtractPPQGenericUnit,  "Test extracting range of triangle data using generic code");
                    if (!mUseQuads)
                    {
                        REGISTER_CLUSTER_TEST(TestExtractPPQSpecificUnit, "Test extracting range of triangle data using specific code");
                    }

                    // benchmark extracting GP triangles
                    REGISTER_CLUSTER_TEST(TestExtractGPExisting,     "Test extracting GPTriangles in bbox using existing code");
                    REGISTER_CLUSTER_TEST(TestExtractGPGenericUnit,  "Test extracting GPTriangles in bbox using generic unit code");
                    if (!mUseQuads)
                    {
                        REGISTER_CLUSTER_TEST(TestExtractGPSpecificUnit, "Test extracting GPTriangles in bbox using specific unit code");
                    }

                    // benchmark computing bounding box only
                    REGISTER_CLUSTER_TEST(TestComputeBBoxExisting,     "Test computing bbox using existing code");
                    REGISTER_CLUSTER_TEST(TestComputeBBoxGenericUnit,  "Test computing bbox using generic unit code");
                    if (!mUseQuads)
                    {
                        REGISTER_CLUSTER_TEST(TestComputeBBoxSpecificUnit, "Test computing bbox using specific unit code");
                    }

                    // Run the whole tests suite on SPU too
                    // EATEST_REGISTER_SPU_ARG("ClusterBenchmarkSPU", "SPU cluster benchmarks", mSpuElf, (uint64_t) &mClusterInfo);

                    ClusteredMeshTestBase::Initialize();

                    if (mUseQuads)
                    {
                        numExpectedTriangles = 269;
                        expectedNumTrisInBBox = 22;
                        expectedNumUnitsInBBox = 26;
                    }
                    else
                    {
                        numExpectedTriangles = 394;
                        expectedNumTrisInBBox = 31;
                        expectedNumUnitsInBBox = expectedNumTrisInBBox;
                    }
                }

                virtual void Setup()
                {
                    ClusteredMeshTestBase::Setup();
                }

                virtual void Setup(void *)
                {
                }

                // The generic unit
                typedef rw::collision::GenericClusterUnit<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC> GenericUnit;
                // The specific unit in use is this one. Need a class to hide the IDs in template.
                template <uint8_t COMPRESSION>
                class SpecificUnit : public rw::collision::TriangleUnitWithEdgeCosinesAndIDs<COMPRESSION, 2,2>
                {
                    typedef rw::collision::TriangleUnitWithEdgeCosinesAndIDs<COMPRESSION,2,2> BaseClass;
                public:
                    SpecificUnit(const rw::collision::ClusteredMeshCluster & cluster, 
                        const rw::collision::ClusterParams & clusterParams, uint32_t offset = 0) : 
                    BaseClass(cluster, clusterParams, offset)
                    {}
                   
                };

                virtual void Uninitialize()
                {
                    ClusteredMeshTestBase::Uninitialize();
                }

            private:

                /// Benchmark the total size used to store the mesh
#if !defined(EA_PLATFORM_PS3_SPU)
                void TestClusterSize()
                {
                    uint32_t totalSize = 0;
                    // Process all clusters
                    EATESTAssert(mMesh->GetNumCluster(), "Should be some clusters in the mesh");
                    uint32_t numClusterIndexs = mMesh->GetNumCluster();
                    for (uint32_t c = 0; c < numClusterIndexs; ++c)
                    {
                        rw::collision::ClusteredMeshCluster & cluster = mMesh->GetCluster(c);
                        totalSize += mMesh->GetClusterSize(cluster);
                    }
                    char str[256];
                    sprintf(str, "mesh:%s,name:TotalSize,description:Kb to store all clusters", mMeshFilename);
                    EATESTSendBenchmark(str, (double) totalSize / 1024.0);
                    sprintf(str, "mesh:%s,name:ClusterSize,description:bytes to store ClusteredMeshCluster", mMeshFilename);
                    EATESTSendBenchmark(str, (double) mClusterInfo[0].clusterSize);
                }
#endif // !defined(EA_PLATFORM_PS3_SPU)

                // ********************** TestExtractOne
                // Simple test of time it takes to extract data about a single triangle from the cluster.
                // Test does this many times in order to make it measurable.
                // This isn't likely to result in a benchmark that can be scaled since it'll have a very 
                // particular cache behavior.
                // Bounding box of triangle is computed to ensure compiler doesn't optimize away.
                // Currently, only vertex data is used.

                /// Benchmark extracting a single triangle from a cluster.
                /// Slightly hampered by not having an appropriate API for this.
                void TestExtractOneExisting()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    rwpmath::Vector3 min = GetVector3_Large();
                    rwpmath::Vector3 max = -min;
                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;
                    uint32_t offset = clusterInfo.unitOffset;
                    rw::collision::AABBox bbox = clusterInfo.bbox;
                    rwpmath::Matrix44Affine transform = rwpmath::GetMatrix44Affine_Identity();

                    BenchmarkTimer timer;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        min = GetVector3_Large();
                        max = -min;

                        timer.Start();
                        // Extract 1000 times to make it measurable
                        for (uint32_t t = 0; t < 1000; ++t)
                        {
                            rw::collision::GPTriangle tris[2];
                            rwpmath::Vector3 v0, v1, v2;
                            uint32_t numTris = 0;
                            cluster->UnitGetOverlappingGPInstances(offset, bbox, &transform, tris, numTris, clusterParams);
                            EA_ASSERT(numTris <= 2);
                            for (uint32_t tt = 0; tt < numTris; ++tt)
                            {
                                // Do something reasonable with the results - compute bbox of triangles
                                v0 = tris[tt].Vertex0();
                                v1 = tris[tt].Vertex1();
                                v2 = tris[tt].Vertex2();
                                min = rwpmath::Min(min, v0);
                                min = rwpmath::Min(min, v1);
                                min = rwpmath::Min(min, v2);
                                max = rwpmath::Max(max, v0);
                                max = rwpmath::Max(max, v1);
                                max = rwpmath::Max(max, v2);
                            }
                        }
                        timer.Stop();
                    }
                    SendBenchmark(timer, "Extract1000Triangles", "ms to extract one triangle 1000 times", "method:existing");

                    // Check results
                    EATESTAssert((float) min.GetX() < (float) max.GetX(), "Non-zero bounds in X");
                    EATESTAssert((float) min.GetY() < (float) max.GetY(), "Non-zero bounds in Y");
                    EATESTAssert((float) min.GetZ() < (float) max.GetZ(), "Non-zero bounds in Z");
                }

                template <class Unit>
                void TestExtractOneUnit(ClusterInfo & clusterInfo, const char * parameters)
                {
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    rwpmath::Vector3 min = GetVector3_Large();
                    rwpmath::Vector3 max = -min;
                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;
                    uint32_t offset = clusterInfo.unitOffset;

                    BenchmarkTimer timer;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        min = GetVector3_Large();
                        max = -min;

                        timer.Start();
                        // Extract 1000 times to make it measurable
                        for (uint32_t t = 0; t < 1000; ++t)
                        {
                            rwpmath::Vector3 v0, v1, v2, edgeCosines;
                            rwpmath::Mask3 edgeIsConvex, disableVertices;
                            const Unit unit(*cluster, clusterParams, offset);
                            unit.GetTriVertices(v0, v1, v2);
                            // Do something reasonable with the results - compute bbox of triangles
                            min = rwpmath::Min(min, v0);
                            min = rwpmath::Min(min, v1);
                            min = rwpmath::Min(min, v2);
                            max = rwpmath::Max(max, v0);
                            max = rwpmath::Max(max, v1);
                            max = rwpmath::Max(max, v2);
                        }
                        timer.Stop();
                    }

                    SendBenchmark(timer, "Extract1000Triangles", "ms to extract one triangle 1000 times", parameters);

                    // Check results
                    EATESTAssert((float) min.GetX() < (float) max.GetX(), "Non-zero bounds in X");
                    EATESTAssert((float) min.GetY() < (float) max.GetY(), "Non-zero bounds in Y");
                    EATESTAssert((float) min.GetZ() < (float) max.GetZ(), "Non-zero bounds in Z");
                }

                /// Benchmark extracting a single triangle from a cluster using new generic unit code.
                void TestExtractOneGenericUnit()
                {
                    TestExtractOneUnit<GenericUnit>(mClusterInfo[0], "method:generic unit");
                }                    
                /// Benchmark extracting a single triangle from a cluster using new specialized unit code.
                void TestExtractOneSpecificUnit()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    switch (clusterInfo.cluster->compressionMode)
                    {
                    case rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED:
                        TestExtractOneUnit<SpecificUnit< 
                            rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED:
                        TestExtractOneUnit<SpecificUnit< 
                            rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED:
                        TestExtractOneUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    }
                }                    

                // ********************** TestExtractPPQ
                // This benchmark is meant to model what we'd need to do to prepare data for use in the 
                // new collision_primitives style tests.
                // Data is written to an array of memory in a raw form.
                // Again we extract all triangles from the cluster, but the code should work with any range.

                /// Triangle data that is roughly what is needed for collision_primitives operations.
                struct PPQTriangle
                {
                    rwpmath::Vector3 vs[3];
                    rwpmath::Vector3 edgeCosines;
                    rwpmath::Mask3 edgeIsConvex, disableVertices;
                    rwpmath::MaskScalar oneSided;
                    uint32_t id;
                };

                static const uint32_t MAX_TRIANGLES = 100;

                void * GetResultsBuffer() const
                {
                    return sResultsBuffer;
                }

                /// Get a range of triangles from a cluster and store data in form needed by collision_primitives queries.
                void TestExtractPPQExisting()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    PPQTriangle * results = (PPQTriangle *) GetResultsBuffer();

                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;
                    rw::collision::AABBox bbox = clusterInfo.bbox;
                    rwpmath::Matrix44Affine transform = rwpmath::GetMatrix44Affine_Identity();

                    BenchmarkTimer timer;
                    uint32_t numTriangles = 0;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        numTriangles = 0;

                        uint32_t numUnitsInCluster = clusterInfo.numUnitsInCluster;
                        EATESTAssert(numUnitsInCluster, "Should have some units in cluster");
                        // Extract all the triangles in the cluster (could be a range)
                        timer.Start();
                        uint32_t offset = 0;
                        for (uint32_t u = 0; u < numUnitsInCluster; ++u)
                        {
                            uint32_t numTris = 0;
                            rw::collision::GPTriangle tris[2];
                            offset += cluster->UnitGetOverlappingGPInstances(offset, bbox, &transform, tris, numTris, clusterParams);
                            EA_ASSERT(numTris <= 2);
                            for (uint32_t t = 0; t < numTris; ++t)
                            {
                                const rw::collision::GPTriangle & tri = tris[t];
                                PPQTriangle & res = results[(numTriangles++) % MAX_TRIANGLES];
                                res.vs[0] = tri.Vertex0();
                                res.vs[1] = tri.Vertex1();
                                res.vs[2] = tri.Vertex2();
                                res.edgeCosines = tri.EdgeCosines();
                                const uint32_t flags = tri.Flags();
                                res.edgeIsConvex = rwpmath::Mask3(
                                    (flags & rw::collision::GPInstance::FLAG_TRIANGLEEDGE0CONVEX) != 0,
                                    (flags & rw::collision::GPInstance::FLAG_TRIANGLEEDGE1CONVEX) != 0,
                                    (flags & rw::collision::GPInstance::FLAG_TRIANGLEEDGE2CONVEX) != 0);
                                res.disableVertices = rwpmath::Mask3(
                                    (flags & rw::collision::GPInstance::FLAG_TRIANGLEVERT0DISABLE) != 0,
                                    (flags & rw::collision::GPInstance::FLAG_TRIANGLEVERT1DISABLE) != 0,
                                    (flags & rw::collision::GPInstance::FLAG_TRIANGLEVERT2DISABLE) != 0);
                                res.oneSided = rwpmath::MaskScalar(
                                    (flags & rw::collision::GPInstance::FLAG_TRIANGLEONESIDED) != 0);
                                res.id = tri.mUserTag;
                            }
                        }
                        timer.Stop();
                    }
                    SendBenchmark(timer, "ExtractAllTriData", "ms to extract all triangles for ppq", "method:existing");

                    // Check results
                    EATESTAssert(numTriangles == numExpectedTriangles, "Extracted expected number of triangles");
                }

                /// Get all the triangles from a cluster.
                /// This could work differently from extracting a single triangle, for example by decompressing
                /// all vertices in advance.
                template <class Unit>
                void TestExtractPPQUnit(ClusterInfo & clusterInfo, const char * parameters)
                {
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    typedef rw::collision::ClusterTriangleIterator<Unit> TriangleIterator;

                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;

                    PPQTriangle * results = (PPQTriangle *) GetResultsBuffer();

                    BenchmarkTimer timer;
                    uint32_t numTriangles = 0;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        numTriangles = 0;
                        uint32_t numUnitsInCluster = clusterInfo.numUnitsInCluster;
                        EATESTAssert(numUnitsInCluster, "Should have some units in cluster");

                        // Extract all the triangles in the cluster.
                        timer.Start();
                        for (TriangleIterator it(*cluster, clusterParams);
                             !it.AtEnd(); it.Next())
                        {
                            rwpmath::Vector3 v0, v1, v2, edgeCosines;
                            rwpmath::Mask3 edgeIsConvex, disableVertices;
                            rwpmath::MaskScalar oneSided;
                            uint32_t id;
                            it.GetTriangle(v0, v1, v2, edgeCosines, oneSided, edgeIsConvex, disableVertices, id);

                            PPQTriangle & res = results[(numTriangles++) % MAX_TRIANGLES];
                            res.vs[0] = v0;
                            res.vs[1] = v1;
                            res.vs[2] = v2;
                            res.edgeCosines = edgeCosines;
                            res.id = id;
                            // In practice, you'd store the compressed GP flags and expand on use.
                            res.edgeIsConvex = edgeIsConvex;
                            res.disableVertices = disableVertices;
                            res.oneSided = oneSided;
                        }
                        timer.Stop();
                    }
                    SendBenchmark(timer, "ExtractAllTriData", "ms to extract all triangles for ppq", parameters);

                    // Check results
                    EATESTAssert(numTriangles == numExpectedTriangles, "Extracted expected number of triangles");
                }

                /// Benchmark extracting all triangles from a cluster using new generic unit code.
                void TestExtractPPQGenericUnit()
                {
                    TestExtractPPQUnit<GenericUnit>(mClusterInfo[0], "method:generic unit");
                }                    
                /// Benchmark extracting all triangles from a cluster using new specialized unit code.
                void TestExtractPPQSpecificUnit()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    switch (clusterInfo.cluster->compressionMode)
                    {
                    case rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED:
                        TestExtractPPQUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED:
                        TestExtractPPQUnit<SpecificUnit< 
                            rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED:
                        TestExtractPPQUnit<SpecificUnit< 
                            rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    }
                }                    

                // ********************** TestExtractGP
                // This benchmark stores all GPTriangles that overlap the given bounding box.
                // This is basically what the existing UnitGetOverlappingGPInstances() does.
                // We run this over the whole cluster, but it could be run over a sub range.

                /// Return the bounding box to use for the cluster queries
                rw::collision::AABBox GetBBoxForQuery(const ClusterInfo & clusterInfo)
                {
                    // Query triangles from centre of bbox
                    rwpmath::Vector3 min = clusterInfo.bbox.Min();
                    rwpmath::Vector3 max = clusterInfo.bbox.Max();
                    rwpmath::Vector3 center = (min + max)/2.0f;
                    rwpmath::Vector3 size = max - center;
                    rwpmath::VecFloat fraction = 0.5f;
                    return rw::collision::AABBox(center - size * fraction, center + size * fraction);
                }

                /// Get a range of triangles from a cluster and store data in form needed by collision_primitives queries.
                void TestExtractGPExisting()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;
                    rw::collision::AABBox bbox = GetBBoxForQuery(clusterInfo);
                    rwpmath::Matrix44Affine transform = rwpmath::GetMatrix44Affine_Identity();

                    rw::collision::GPTriangle * results = (rw::collision::GPTriangle *) GetResultsBuffer();

                    BenchmarkTimer timer;
                    uint32_t numTriangles = 0;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        numTriangles = 0;
                        uint32_t numUnitsInCluster = clusterInfo.numUnitsInCluster;
                        EATESTAssert(numUnitsInCluster, "Should have some units in cluster");

                        // Extract all the triangles in the cluster (could be a range)
                        timer.Start();

                        uint32_t offset = 0;
                        for (uint32_t u = 0; u < numUnitsInCluster; ++u)
                        {
                            uint32_t numTris = 0;
                            uint32_t size = cluster->UnitGetOverlappingGPInstances(offset, 
                                bbox, &transform, results + (numTriangles % MAX_TRIANGLES), numTris, clusterParams);
                            EA_ASSERT(numTris <= 2);
#if 0
                            // UnitGetOverlappingGPInstances() returns some triangles that may not overlap
                            // because it does the bounding box test on the quad not the individual triangles.
                            // This code can be used to identify the differences.
                            for (uint32_t t = 0; t < numTris; ++t)
                            {
                                rwpmath::Vector3 v0 = results[numTriangles+t].Vertex0();
                                rwpmath::Vector3 v1 = results[numTriangles+t].Vertex1();
                                rwpmath::Vector3 v2 = results[numTriangles+t].Vertex2();
                                rwpmath::Vector3 triBBoxMin = rwpmath::Min(v0, rwpmath::Min(v1, v2));
                                rwpmath::Vector3 triBBoxMax = rwpmath::Max(v0, rwpmath::Max(v1, v2));
                                rw::collision::AABBox triBBox(triBBoxMin, triBBoxMax);
                                if (bbox.Overlaps(triBBox))
                                {
                                    printf("unit at %d\n", offset);
                                }
                            }
#endif
                            offset += size;
                            numTriangles += numTris;
                        }
                        timer.Stop();
                    }
                    SendBenchmark(timer, "ExtractGPTriangles", "ms to extract all GPTriangles", "method:existing");

                    // Check results
                    EATESTAssert(numTriangles == expectedNumUnitsInBBox, "Extracted expected number of triangles");
                }

                /// Get all the triangles from a cluster.
                /// This could work differently from extracting a single triangle, for example by decompressing
                /// all vertices in advance.
                template <class Unit>
                void TestExtractGPUnit(ClusterInfo & clusterInfo, const char * parameters)
                {
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    typedef rw::collision::ClusterTriangleIterator<Unit> TriangleIterator;

                    const rw::collision::AABBox bbox = GetBBoxForQuery(clusterInfo);
                    const rwpmath::Vector3 bboxMin = bbox.Min();
                    const rwpmath::Vector3 bboxMax = bbox.Max();

                    const rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;
                    rw::collision::GPTriangle * results = (rw::collision::GPTriangle *) GetResultsBuffer();

                    BenchmarkTimer timer;
                    uint32_t numTriangles = 0;
                    uint32_t numTrianglesConsidered = 0;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        numTriangles = 0;
                        numTrianglesConsidered = 0;
                        uint32_t numUnitsInCluster = clusterInfo.numUnitsInCluster;
                        EATESTAssert(numUnitsInCluster, "Should have some units in cluster");

                        // Extract all the triangles in the cluster.
                        timer.Start();

                        for (TriangleIterator it(*cluster, clusterParams);
                            !it.AtEnd(); it.Next())
                        {
                            rwpmath::Vector3 v0, v1, v2, edgeCosines;
                            uint32_t id, flags;
                            it.GetVertices(v0, v1, v2);
                            numTrianglesConsidered++;

                            // Bounding box test on all triangles, rather than any quads
                            rwpmath::Vector3 triBBoxMin = rwpmath::Min(v0, rwpmath::Min(v1, v2));
                            rwpmath::Vector3 triBBoxMax = rwpmath::Max(v0, rwpmath::Max(v1, v2));
                            if (Overlaps(bboxMin, bboxMax, triBBoxMin, triBBoxMax).GetBool())
                            {
                                rw::collision::GPTriangle & res = results[(numTriangles++) % MAX_TRIANGLES];
                                // Let units compute edge cosines only if needed
                                flags = it.GetEdgeCosinesAndFlags(edgeCosines);
                                id = it.GetID();
                                res.Initialize(v0, v1, v2, 0.0f, flags, edgeCosines, 0, id);
                            }
                        }

                        timer.Stop();
                    }
                    SendBenchmark(timer, "ExtractGPTriangles", "ms to extract all GPTriangles", parameters);

                    // Check results
                    EATESTAssert(numTrianglesConsidered == numExpectedTriangles, "Expected number of triangles considered");
                    EATESTAssert(numTriangles == expectedNumTrisInBBox, "Extracted expected number of triangles");
                }

                /// Benchmark extracting all triangles from a cluster using new generic unit code.
                void TestExtractGPGenericUnit()
                {
                    TestExtractGPUnit<GenericUnit>(mClusterInfo[0], "generic unit");
                }                    
                /// Benchmark extracting all triangles from a cluster using new specialized unit code.
                void TestExtractGPSpecificUnit()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    switch (clusterInfo.cluster->compressionMode)
                    {
                    case rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED:
                        TestExtractGPUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED:
                        TestExtractGPUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED:
                        TestExtractGPUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    }
                }                    
           

                // ********************** TestExtractBBox
                // Benchmark time to compute the bounding box of a range of units.
                // We'll run this on all the units in the cluster, but the intention is that this would
                // work on a subrange, so we aren't allowed to just get the min & max of all the vertices
                // but instead we have to iterate over units.

                /// Compute bounding box of set of units.
                /// Existing API is not ideal for this since UnitGetOverlappingGPInstances() does a lot more.
                void TestComputeBBoxExisting()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    // Query triangles from centre of bbox
                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;
                    rw::collision::AABBox bbox = clusterInfo.bbox;
                    rwpmath::Matrix44Affine transform = rwpmath::GetMatrix44Affine_Identity();
                    // Extract all the triangles in the cluster
                    uint32_t startOffset = 0;
                    uint32_t numUnits = clusterInfo.numUnitsInCluster;
                    EATESTAssert(numUnits, "Should have some units in cluster");

                    BenchmarkTimer timer;
                    rwpmath::Vector3 min, max;
                    uint32_t numTriangles = 0;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        numTriangles = 0;
                        min = GetVector3_Large();
                        max = -min;

                        timer.Start();
                        uint32_t offset = startOffset;
                        for (uint32_t u = 0; u < numUnits; ++u)
                        {
                            uint32_t numTris = 0;
                            rw::collision::GPTriangle tris[2];
                            offset += cluster->UnitGetOverlappingGPInstances(offset, bbox, &transform, tris, numTris, clusterParams);
                            EA_ASSERT(numTris <= 2);
                            for (uint32_t t = 0; t < numTris; ++t)
                            {
                                // Do something reasonable with the results - compute bbox of triangles
                                UpdateBBox(min, max, tris[t].Vertex0(), tris[t].Vertex1(), tris[t].Vertex2());
                            }
                            numTriangles += numTris;
                        }
                        timer.Stop();
                    }

                    SendBenchmark(timer, "ComputeBBox", "ms to compute all triangle bbox", "method:existing");

                    // Check results
                    EATESTAssert(numTriangles == clusterInfo.numTrianglesInCluster, "Should have extracted some triangles");
                    EATESTAssert((float) min.GetX() < (float) max.GetX(), "Non-zero bounds in X");
                    EATESTAssert((float) min.GetY() < (float) max.GetY(), "Non-zero bounds in Y");
                    EATESTAssert((float) min.GetZ() < (float) max.GetZ(), "Non-zero bounds in Z");
                }

                /// Compute bounding box of set of units.
                /// Use new triangle iterator to access just vertices.
                template <class Unit>
                void TestComputeBBoxUnit(ClusterInfo & clusterInfo, const char * parameters)
                {
                    EATESTAssert(clusterInfo.cluster, "Should have loaded mesh cluster");

                    typedef rw::collision::ClusterTriangleIterator<Unit> TriangleIterator;

                    rw::collision::ClusteredMeshCluster * cluster = clusterInfo.cluster;
                    const rw::collision::ClusterParams & clusterParams = clusterInfo.clusterParams;
                    // Extract all the triangles in the cluster
                    uint32_t startOffset = 0;
                    uint32_t numUnits = clusterInfo.numUnitsInCluster;
                    EATESTAssert(numUnits, "Should have some units in cluster");

                    BenchmarkTimer timer;
                    uint32_t numTriangles = 0;
                    rwpmath::Vector3 min, max;
                    for (uint32_t iteration = 0; iteration < mNumIterations; ++iteration)
                    {
                        numTriangles = 0;
                        min = GetVector3_Large();
                        max = -min;

                        timer.Start();
                        for (TriangleIterator it(*cluster, clusterParams, startOffset, numUnits);
                            !it.AtEnd(); it.Next())
                        {
                            rwpmath::Vector3 v0, v1, v2;
                            it.GetVertices(v0, v1, v2);
                            // Do something reasonable with the results - compute bbox of triangles
                            UpdateBBox(min, max, v0, v1, v2);
                            numTriangles ++;
                        }
                        timer.Stop();
                    }

                    SendBenchmark(timer, "ComputeBBox", "ms to compute all triangle bbox", parameters);

                    // Check results
                    EATESTAssert(numTriangles == clusterInfo.numTrianglesInCluster, "Should have extracted some triangles");
                    EATESTAssert((float) min.GetX() < (float) max.GetX(), "Non-zero bounds in X");
                    EATESTAssert((float) min.GetY() < (float) max.GetY(), "Non-zero bounds in Y");
                    EATESTAssert((float) min.GetZ() < (float) max.GetZ(), "Non-zero bounds in Z");
                }

                /// Benchmark extracting subset of triangles from a cluster using new generic unit code.
                void TestComputeBBoxGenericUnit()
                {
                    TestComputeBBoxUnit<GenericUnit>(mClusterInfo[0], "method:generic unit");
                }                    
                /// Benchmark extracting subset triangles from a cluster using new specialized unit code.
                void TestComputeBBoxSpecificUnit()
                {
                    ClusterInfo & clusterInfo = mClusterInfo[0];
                    switch (clusterInfo.cluster->compressionMode)
                    {
                    case rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED:
                        TestComputeBBoxUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED:
                        TestComputeBBoxUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED:
                        TestComputeBBoxUnit<SpecificUnit<
                            rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED> >(clusterInfo, "method:specific unit");
                        break;
                    }
                }                    

            private:

                EA_FORCE_INLINE void UpdateBBox(rwpmath::Vector3 & min, rwpmath::Vector3 & max,
                    rwpmath::Vector3::InParam v0, rwpmath::Vector3::InParam v1, rwpmath::Vector3::InParam v2)
                {
                    min = rwpmath::Min(min, v0);
                    min = rwpmath::Min(min, v1);
                    min = rwpmath::Min(min, v2);
                    max = rwpmath::Max(max, v0);
                    max = rwpmath::Max(max, v1);
                    max = rwpmath::Max(max, v2);
                }


            private:

                const char * mName;
                const char * mSpuElf;
                char mMeshFilename[256];
                uint8_t mCompression;
                bool mUseQuads;
                bool mUseIDs;
                uint32_t numExpectedTriangles;
                uint32_t expectedNumTrisInBBox;
                uint32_t expectedNumUnitsInBBox;


                /// How many iterations to do to get a semi-reliable timing result
#if defined(EA_PLATFORM_WINDOWS)
                static const uint32_t mNumIterations = 50;
#else
                static const uint32_t mNumIterations = 10;
#endif

                static const uint32_t RESULTS_SIZE = MAX_TRIANGLES*sizeof(PPQTriangle);
                static EA_ALIGNED(uint8_t, sResultsBuffer[RESULTS_SIZE], 16);

            };
        }
    }
}

// ***********************************************************************************************************
