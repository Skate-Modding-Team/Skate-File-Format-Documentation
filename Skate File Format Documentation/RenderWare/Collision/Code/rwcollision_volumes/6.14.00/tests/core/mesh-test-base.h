// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: mesh-test-base.h

Purpose: base class for testing clustered meshes

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

#include <coreallocator/icoreallocator_interface.h>

#if defined(EA_PLATFORM_PS3_SPU)
#include <job_util/dma.h>
#else // !defined(EA_PLATFORM_PS3_SPU)
#include <eaphysics/unitframework/serialization_test_helpers.hpp>
#include <rw/collision/initialize.h>
#include <rw/collision/aggregate.h>
#include <rw/collision/aggregatevolume.h>
#include <eaphysics/hlserializable.h>
#include <serialization/serialization.h>
#include <serialization/binary_stream_iarchive.h>
#endif // !defined(EA_PLATFORM_PS3_SPU)

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include <string.h>     // for memset()
#include "stdio.h"     // for sprintf()

#include "SimpleStream.hpp"

// ***********************************************************************************************************

namespace rw
{
    namespace collision
    {
        namespace Tests
        {
            /// Unit tests to benchmark extraction of triangle data from clustered mesh clusters.
            class ClusteredMeshTestBase : public EATest::TestSuite
            {
            public:

                ClusteredMeshTestBase() : EATest::TestSuite()
                {
#if !defined(EA_PLATFORM_PS3_SPU)
                    mVolume = 0;
                    mMesh = 0;
#endif
                    // Zero all unused cluster info structures
                    memset(mClusterInfo, 0, sizeof(ClusterInfo)*MAX_CLUSTER_INFO);
                }

                struct EA_PREFIX_ALIGN(16) ClusterInfo
                {
                    static const uint32_t MAX_NAME = 32;
                    /// Name for results from this cluster
                    char name[MAX_NAME];
                    /// The index of the cluster to use for testing
                    uint32_t index;
                    /// Pointer to cluster if in use, 0 otherwise.
                    rw::collision::ClusteredMeshCluster * cluster;
                    rw::collision::ClusteredMeshCluster * clusterEA;
                    uint32_t clusterSize;
                    uint32_t numUnitsInCluster;
                    uint32_t numTrianglesInCluster;
                    rw::collision::ClusterParams clusterParams;
                    rw::collision::AABBox bbox;
                    /// Offset for the unit we'll test with
                    uint32_t unitOffset;
                    uint32_t unitID;
                } EA_POSTFIX_ALIGN(16);

            protected:

                // Derived classes must implement this to return the filename of the binary archive containing the clustered mesh volume.
                virtual const char * GetMeshFileName() const = 0;
                // Derived classes should implement this to return the name of the TestSuite
                virtual const char * GetSuiteName() const = 0;

                /// Derived classes should override to register their unit tests and then call this implementation
                virtual void Initialize()
                {
                    SuiteName(GetSuiteName());
#if !defined(EA_PLATFORM_PS3_SPU)
                    rw::collision::InitializeVTables();
#endif
                }

                virtual void Uninitialize()
                {
#if !defined(EA_PLATFORM_PS3_SPU)
                    if (mVolume)
                    {
                       EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mVolume);
                        mVolume = 0;
                    }
                    if (mMesh)
                    {
                        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mMesh);
                        mMesh = 0;
                    }

                    EA::Physics::UnitFramework::ResetAllocator();
#endif
                }

#if !defined(EA_PLATFORM_PS3_SPU)
                virtual uint32_t PickCluster()
                {
                    return mMesh->GetNumCluster()/2;
                }
                virtual uint32_t PickUnit(ClusterInfo & clusterInfo)
                {
                    return clusterInfo.numUnitsInCluster/2;
                }
                virtual void CreateClusterInfo(ClusterInfo & clusterInfo, rw::collision::ClusteredMesh * mesh, const char * name = "<UNCOMPRESSED,TRI,COS>")
                {
                    // Extract all the information we need from the mesh and put into ClusterInfo
                    EATESTAssert(mesh->GetNumCluster() > 0, "Should have some clusters");
                    EA_ASSERT(strlen(name) < ClusterInfo::MAX_NAME-1);
                    strcpy(clusterInfo.name, name);
                    clusterInfo.index = PickCluster();
                    clusterInfo.cluster = &mesh->GetCluster(clusterInfo.index);
                    clusterInfo.clusterEA = clusterInfo.cluster;
                    clusterInfo.clusterSize = mesh->GetClusterSize(*clusterInfo.cluster);
                    clusterInfo.clusterSize = EA::Physics::SizeAlign<uint32_t>(clusterInfo.clusterSize, 16);
                    clusterInfo.numUnitsInCluster = mesh->GetNumUnitInCluster(clusterInfo.index);
                    clusterInfo.numTrianglesInCluster = CountTriangles(mesh, clusterInfo.index);
                    clusterInfo.clusterParams = mesh->GetClusterParams();
                    clusterInfo.bbox = GetClusterBBox(clusterInfo, mesh);
                    EATESTAssert(clusterInfo.numUnitsInCluster > 0, "Should have some units in chosen cluster");
                    uint32_t unit = PickUnit(clusterInfo);
                    clusterInfo.unitID = unit;
                    clusterInfo.unitOffset = GetUnitOffset(clusterInfo.index, unit);
                }
#endif // defined(EA_PLATFORM_PS3_SPU)

                virtual void Setup()
                {
#if !defined(EA_PLATFORM_PS3_SPU)
                    if (!mMesh) // load mesh once for first test that uses it
                    {
                        mMesh = LoadSerializedMesh(GetMeshFileName());
                        mVolume = static_cast<rw::collision::Volume*>(EA::Physics::UnitFramework::Creator<rw::collision::AggregateVolume>().New(mMesh));
                        EATESTAssert(mVolume, "Should be an aggregate");
                    }
                    // Default to creating a single test cluster from our standard mesh
                    CreateClusterInfo(mClusterInfo[0], mMesh);
#else // defined(EA_PLATFORM_PS3_SPU)
                    uint64_t dataEA = GetSPUArg();
                    EA::Jobs::DmaGet(mClusterInfo, (const void *) dataEA, sizeof(ClusterInfo)*MAX_CLUSTER_INFO);
                    uint8_t * clusterData = sClusterData;
                    uint32_t dataLeft = CLUSTER_MAX_SIZE;
                    for (uint32_t c = 0; c < MAX_CLUSTER_INFO; ++c)
                    {
                        if (mClusterInfo[c].cluster)
                        {
                            uint32_t size = mClusterInfo[c].clusterSize;
                            EATESTAssert(size <= dataLeft, "Next cluster larger than expected");
                            EA::Jobs::DmaGet(clusterData, mClusterInfo[c].cluster, size);
                            mClusterInfo[c].cluster = (rw::collision::ClusteredMeshCluster *) clusterData;
                            dataLeft -= size;
                            clusterData += size;
                        }
                    }
#endif // defined(EA_PLATFORM_PS3_SPU)
                }

                virtual void Setup(void *)
                {
                }

                virtual void Teardown()
                {
                    memset(mClusterInfo, 0, sizeof(ClusterInfo)*MAX_CLUSTER_INFO);
                }

                virtual void Teardown(void *)
                {
                }

                /// Format benchmark string and send result from a timer
                void SendBenchmark(BenchmarkTimer & timer, const char * name, const char * description = 0, const char * parameters = 0, const double factor = 1.0f)
                {
                    char str[256];
#if defined(EA_PLATFORM_PS3_SPU)
                    // Distinguish SPU metrics on PS3 from PPU metrics
                    const char * platform = "platform:spu,";
#else
                    const char * platform = "";
#endif
                    char descriptionString[100] = "";
                    if (description)
                    {
                        sprintf(descriptionString, ",description:%s", description);
                    }
                    char parametersString[100] = "";
                    if (parameters)
                    {
                        sprintf(parametersString, ",%s", parameters);
                    }
                    sprintf(str, "%ssuite:%s,benchmark:%s%s%s", platform, GetSuiteName(), name, parametersString, descriptionString);
                    double avgTime = factor*timer.GetAverageDurationMilliseconds();
                    double minTime = factor*timer.GetMinDurationMilliseconds();
                    double maxTime = factor*timer.GetMaxDurationMilliseconds();
                    EATESTSendBenchmark(str, avgTime, minTime < avgTime ? minTime : avgTime, maxTime > avgTime ? maxTime : avgTime);
                }

                /// Large vector3 suitable for initializing bounding box calculations
                static rwpmath::Vector3 GetVector3_Large()
                {   
                    rwpmath::VecFloat large(1e20f);
                    return rwpmath::Vector3(large, large, large);
                }

#if !defined(EA_PLATFORM_PS3_SPU)
                /// Load a collision volume from binary archive
                rw::collision::ClusteredMesh * LoadSerializedMesh(const char * filename)
                {
                    rw::collision::ClusteredMesh * loaded = 0;

                    SimpleStream strm(filename);

                    EA::Serialization::basic_binary_stream_iarchive<SimpleStream, EA::Serialization::Endian::LittleEndianConverter> iArchive(strm);
                    iArchive & EAPHYSICS_HL_SERIALIZABLE_WITH_ALLOCATOR(rw::collision::ClusteredMesh, loaded, *EA::Allocator::ICoreAllocator::GetDefaultAllocator());
                    if (!iArchive.Close())
                    {
                        return 0;
                    }

                    EA_ASSERT_MSG(loaded, "Failed high level file serialization (loading only).");
                    EA_ASSERT_MSG(loaded->IsValid(), "Failed high level file serialization (loading only).");
                    return loaded;
                }

                /// Return offset used to access a chosen unit from the cluster
                uint32_t GetUnitOffset(uint32_t cluster, uint32_t unit)
                {
                    rw::collision::Volume volume;
                    rw::collision::TriangleVolume * tri = (rw::collision::TriangleVolume *) &volume;
                    uint32_t offset = 0;
                    for (uint32_t u = 0; u < unit; ++u)
                    {
                        offset += mMesh->GetUnitVolume(cluster, offset, 0, tri);
                    }
                    return offset;
                }

                /// Return bounding box of a cluster for use in tests
                rw::collision::AABBox GetClusterBBox(ClusterInfo & clusterInfo, rw::collision::ClusteredMesh * mesh)
                {
                    EA_ASSERT(mMesh);

                    rw::collision::Volume volume;
                    rw::collision::TriangleVolume * tri = (rw::collision::TriangleVolume *) &volume;
                    rwpmath::Vector3 v0, v1, v2;
                    uint32_t numTriangles = 0;
                    rwpmath::Vector3 min = GetVector3_Large();
                    rwpmath::Vector3 max = -min;

                    uint32_t c = clusterInfo.index;
                    uint32_t numUnitsInCluster = clusterInfo.numUnitsInCluster;
                    EA_ASSERT(numUnitsInCluster);
                    // Extract all the triangles in the cluster
                    uint32_t offset = 0;
                    for (uint32_t u = 0; u < numUnitsInCluster; ++u)
                    {
                        offset += mesh->GetUnitVolume(c, offset, 0, tri);
                        // Do something reasonable with the results - compute bbox of triangles
                        tri->GetPoints(v0, v1, v2);
                        min = rwpmath::Min(min, v0);
                        min = rwpmath::Min(min, v1);
                        min = rwpmath::Min(min, v2);
                        max = rwpmath::Max(max, v0);
                        max = rwpmath::Max(max, v1);
                        max = rwpmath::Max(max, v2);
                        numTriangles++;
                    }
                    return rw::collision::AABBox(min, max);
                }

                uint32_t CountTriangles(rw::collision::ClusteredMesh * mesh, uint32_t cluster)
                {
                    uint32_t numTriangles = 0;
                    for (rw::collision::ClusteredMeshUnit u(mesh, cluster); !u.AtEnd(); u.Next())
                    {
                        numTriangles += u.GetTriangleCount();
                    }
                    return numTriangles;
                }

#endif // !defined(EA_PLATFORM_PS3_SPU)

            protected:

#if !defined(EA_PLATFORM_PS3_SPU)
                /// Collision volume loaded in Setup()
                rw::collision::Volume * mVolume;
                /// The mesh we're testing - loaded in Setup()
                rw::collision::ClusteredMesh * mMesh;
#else  // defined(EA_PLATFORM_PS3_SPU)
                static const uint32_t CLUSTER_MAX_SIZE = 30<<10;    // 30k max cluster size
                static uint8_t sClusterData[CLUSTER_MAX_SIZE];
#endif // defined(EA_PLATFORM_PS3_SPU)

                /// Information about the cluster to help loading onto SPU
                static const uint32_t MAX_CLUSTER_INFO = 4;
                ClusterInfo mClusterInfo[MAX_CLUSTER_INFO];

            };

#if defined(EA_PLATFORM_PS3_SPU)
            // Although this is static data and should normally go in a .cpp file, because it is only needed 
            // on SPU and because we can only have one unit test suite in an SPU elf, this can only get 
            // instantiated once per binary from the header file.
            uint8_t ClusteredMeshTestBase::sClusterData[ClusteredMeshTestBase::CLUSTER_MAX_SIZE];
#endif // defined(EA_PLATFORM_PS3_SPU)

        }
    }
}

// ***********************************************************************************************************
