// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: test-clusterunit.h

Purpose: unit tests to for extraction of triangles from clusters

*/

#ifndef TEST_CLUSTERUNIT_H
#define TEST_CLUSTERUNIT_H

#include <EABase/eabase.h>

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/volume.h"
#include "rw/collision/triangle.h"
#include "rw/collision/clusteredmesh.h"
#include "rw/collision/clusteredmeshcluster.h"

#include <unit/unit.h>
#include "EABase/eabase.h"
#include "EAAssert/eaassert.h"

// ***********************************************************************************************************

namespace EA 
{
    namespace Collision
    {
        namespace Tests
        {
            /// Unit tests to benchmark extraction of triangle data from clustered mesh clusters.
            /// Templated on the unit class we want to test.
            /// Use constructor parameters to control which tests are appropriate for particular unit class.
            template <class TestUnit>
            class TestClusterUnit : public EATest::TestSuite
            {
            public:

                TestClusterUnit(const char * suiteName = "TestClusterUnit", const char * spuElf = "test-clusterunit.elf", bool supportQuads = true, bool assumesEdgeCosines = false, bool supportsIDs = true)
                    : mSuiteName(suiteName), mSpuElf(spuElf),
                    mSupportQuads(supportQuads), mAssumesEdgeCosines(assumesEdgeCosines), mSupportsIDs(supportsIDs)
                {}


            protected:

                virtual void Initialize()
                {
                    SuiteName(mSuiteName);

#define REGISTER_CLUSTER_TEST(M,D) EATEST_REGISTER_TEMPLATED(#M, D, TestClusterUnit<TestUnit>, M)

                    // Run the whole tests suite on SPU too
                    EATEST_REGISTER_SPU("TestClusterUnitSPU", "SPU cluster benchmarks", mSpuElf);

                    // Tests that use just triangles
                    REGISTER_CLUSTER_TEST(CheckGetTriSize, "Check size of triangle units");
                    REGISTER_CLUSTER_TEST(CheckAdvanceAndReset, "Check advancing and resetting");
                    REGISTER_CLUSTER_TEST(CheckGetVertexFromTri, "Check getting single vertex");
                    REGISTER_CLUSTER_TEST(CheckGetVerticesFromTri, "Check getting vertices from triangle");
                    REGISTER_CLUSTER_TEST(CheckGetVertexIndicesFromTri, "Check getting vertex indices from triangle");
                    REGISTER_CLUSTER_TEST(CheckGetEdgeCosinesFromTri, "Check getting edge cosines from tri");
                    if (!mAssumesEdgeCosines)
                    {
                        REGISTER_CLUSTER_TEST(CheckGetDefaultEdgeCosinesFromTri, "Check getting default edge cosines from tri");
                        REGISTER_CLUSTER_TEST(CheckGetDefaultEdgeCosinesFromTwoSidedTri, "Check getting default edge cosines from two sided tri");
                    }
                    REGISTER_CLUSTER_TEST(CheckGetEdgeCosinesFromTwoSidedTri, "Check getting edge cosines from two-sided tri");
                    if (mSupportsIDs)
                    {
                        if (!mAssumesEdgeCosines)
                        {
                            REGISTER_CLUSTER_TEST(CheckGetIDsFromTri, "Check getting IDs from a tri");
                        }
                        REGISTER_CLUSTER_TEST(CheckGetIDsFromTriWithEdgeCosines, "Check getting IDs from a tri with edge cosines");
                    }

                    // Tests that also use quads.
                    // This code should still compile for all unit class, but will fail at runtime
                    if (mSupportQuads)
                    {
                        REGISTER_CLUSTER_TEST(CheckGetQuadSize, "Check size of quad units");
                        REGISTER_CLUSTER_TEST(CheckGetVertexFromQuad, "Check getting single vertex");
                        REGISTER_CLUSTER_TEST(CheckGetVerticesFromQuad, "Check getting vertices from quad");
                        REGISTER_CLUSTER_TEST(CheckGetVertexIndicesFromQuad, "Check getting vertex indices from quad");
                        REGISTER_CLUSTER_TEST(CheckGetEdgeCosinesFromQuad, "Check getting edge cosines from quad");
                        if (!mAssumesEdgeCosines)
                        {
                            REGISTER_CLUSTER_TEST(CheckGetDefaultEdgeCosinesFromQuad, "Check getting default edge cosines from quad");
                            REGISTER_CLUSTER_TEST(CheckGetDefaultEdgeCosinesFromTwoSidedQuad, "Check getting default edge cosines from two sided quad");
                        }
                        if (mSupportsIDs)
                        {
                            if (!mAssumesEdgeCosines)
                            {
                                REGISTER_CLUSTER_TEST(CheckGetIDsFromQuad, "Check getting IDs from a quad");
                            }
                            REGISTER_CLUSTER_TEST(CheckGetIDsFromQuadWithEdgeCosines, "Check getting IDs from a quad with edge cosines");
                        }
                    }

                    // TODO: add tests for compression?
                }

                virtual void Uninitialize()
                {
                }

            private:

                virtual void CheckGetTriSize()
                {
                    if (!mAssumesEdgeCosines)
                    {
                        CreateTriUnit();
                        TestUnit unit(*mCluster, mClusterParams);
                        EATESTAssert(unit.IsValid(), "Should be valid");
                        EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices");
                        EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle");
                        EATESTAssert(unit.GetSize() == 4u, "Triangle unit with no extra data should be 4 bytes");
                    }
                    {
                        CreateTriUnitWithEdgeCosines();
                        TestUnit unit(*mCluster, mClusterParams);
                        EATESTAssert(unit.IsValid(), "Should be valid");
                        EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices");
                        EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle");
                        EATESTAssert(unit.GetSize() == 7u, "Triangle unit with edge data should be 7 bytes");
                    }
                    for (uint8_t s = 0; s < 2; ++s)
                    {
                        for (uint8_t g = 0; g < 2; ++g)
                        {
                            if (!mAssumesEdgeCosines)
                            {
                                CreateTriUnitWithIDs(g,s);
                                TestUnit unit(*mCluster, mClusterParams);
                                EATESTAssert(unit.IsValid(), "Should be valid");
                                EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices");
                                EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle");
                                EATESTAssert(unit.GetSize() == (uint32_t) (4+s+g), 
                                    "Triangle unit with IDs should be 4 or more bytes");
                            }
                            if (mSupportsIDs)
                            {
                                CreateTriUnitWithEdgeCosinesAndIDs(g,s);
                                TestUnit unit(*mCluster, mClusterParams);
                                EATESTAssert(unit.IsValid(), "Should be valid");
                                EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices");
                                EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle");
                                EATESTAssert(unit.GetSize() == (uint32_t) (7+s+g), 
                                    "Triangle unit with IDs should be 7 or more bytes");
                            }
                        }
                    }
                }

                void CheckGetQuadSize()
                {
                    if (!mAssumesEdgeCosines)
                    {
                        CreateQuadUnit();
                        TestUnit unit(*mCluster, mClusterParams);
                        EATESTAssert(unit.IsValid(), "Should be valid");
                        EATESTAssert(unit.GetVertexCount() == 4u, "Should hold four vertices");
                        EATESTAssert(unit.GetTriCount() == 2u, "Should hold two triangles");
                        EATESTAssert(unit.GetSize() == 5u, "Quad unit with no extra data should be 4 bytes");
                    }
                    {
                        CreateQuadUnitWithEdgeCosines();
                        TestUnit unit(*mCluster, mClusterParams);
                        EATESTAssert(unit.IsValid(), "Should be valid");
                        EATESTAssert(unit.GetVertexCount() == 4u, "Should hold four vertices");
                        EATESTAssert(unit.GetTriCount() == 2u, "Should hold two triangles");
                        EATESTAssert(unit.GetSize() == 9u, "Quad unit with edge data should be 7 bytes");
                    }
                    for (uint8_t s = 0; s < 2; ++s)
                    {
                        for (uint8_t g = 0; g < 2; ++g)
                        {
                            if (!mAssumesEdgeCosines)
                            {
                                CreateQuadUnitWithIDs(g,s);
                                TestUnit unit(*mCluster, mClusterParams);
                                EATESTAssert(unit.IsValid(), "Should be valid");
                                EATESTAssert(unit.GetVertexCount() == 4u, "Should hold four vertices");
                                EATESTAssert(unit.GetTriCount() == 2u, "Should hold two triangles");
                                EATESTAssert(unit.GetSize() == (uint32_t) (5+s+g), 
                                    "Quad unit with IDs should be 5 or more bytes");
                            }
                            if (mSupportsIDs)
                            {
                                CreateQuadUnitWithEdgeCosinesAndIDs(g,s);
                                TestUnit unit(*mCluster, mClusterParams);
                                EATESTAssert(unit.IsValid(), "Should be valid");
                                EATESTAssert(unit.GetVertexCount() == 4u, "Should hold four vertices");
                                EATESTAssert(unit.GetTriCount() == 2u, "Should hold two triangles");
                                EATESTAssert(unit.GetSize() == (uint32_t) (9+s+g), 
                                    "Quad unit with IDs should be 9 or more bytes");
                            }
                        }
                    }
                }

                void CheckAdvanceAndReset()
                {
                    InitializeCluster();
                    // Write edge cosines so can test with units that assume it
                    uint32_t s = 0;
                    s += WriteUnit(mCluster->UnitData() + s, mClusterParams, 3, 6,2,5,0, true);
                    s += WriteUnit(mCluster->UnitData() + s, mClusterParams, 3, 4,1,2,0, true);
                    s += WriteUnit(mCluster->UnitData() + s, mClusterParams, 3, 1,3,5,0, true);
                    mCluster->unitCount = 3;

                    TestUnit unit(*mCluster, mClusterParams);
                    EATESTAssert(unit.IsValid(), "Should be valid");
                    EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices");
                    EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle");
                    EATESTAssert(unit.GetSize() == 7u, "Triangle unit with edge cosines should be 7 bytes");
                    {
                        rwpmath::Vector3 v0 = unit.GetVertex(0);
                        EATESTAssert(v0 == GetExpectedVertex(6), "First vertex should be vertexArray[6]");
                        rwpmath::Vector3 v1 = unit.GetVertex(1);
                        EATESTAssert(v1 == GetExpectedVertex(2), "Second vertex should be vertexArray[2]");
                        rwpmath::Vector3 v2 = unit.GetVertex(2);
                        EATESTAssert(v2 == GetExpectedVertex(5), "Third vertex should be vertexArray[5]");
                    }
                    unit.Advance();
                    EATESTAssert(unit.IsValid(), "Should be valid again");
                    EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices again");
                    EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle again");
                    EATESTAssert(unit.GetSize() == 7u, "Triangle unit with edge cosines should be 7 bytes again");
                    {
                        rwpmath::Vector3 v0 = unit.GetVertex(0);
                        EATESTAssert(v0 == GetExpectedVertex(4), "First vertex should now be vertexArray[4]");
                        rwpmath::Vector3 v1 = unit.GetVertex(1);
                        EATESTAssert(v1 == GetExpectedVertex(1), "Second vertex should now be vertexArray[1]");
                        rwpmath::Vector3 v2 = unit.GetVertex(2);
                        EATESTAssert(v2 == GetExpectedVertex(2), "Third vertex should now be vertexArray[2]");
                    }
                    unit.Advance();
                    EATESTAssert(unit.IsValid(), "Should still be valid now");
                    unit.Reset(7u);
                    EATESTAssert(unit.IsValid(), "Should be valid again");
                    EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices once again");
                    EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle once again");
                    EATESTAssert(unit.GetSize() == 7u, "Triangle unit with edge cosines should be 7 bytes once again");
                    {
                        rwpmath::Vector3 v0 = unit.GetVertex(0);
                        EATESTAssert(v0 == GetExpectedVertex(4), "First vertex should now be vertexArray[4] again");
                        rwpmath::Vector3 v1 = unit.GetVertex(1);
                        EATESTAssert(v1 == GetExpectedVertex(1), "Second vertex should now be vertexArray[1] again");
                        rwpmath::Vector3 v2 = unit.GetVertex(2);
                        EATESTAssert(v2 == GetExpectedVertex(2), "Third vertex should now be vertexArray[2] again");
                    }
                    unit.Reset();
                    EATESTAssert(unit.IsValid(), "Should be valid");
                    EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices yet again");
                    EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle yet again");
                    EATESTAssert(unit.GetSize() == 7u, "Triangle unit with edge cosines should be 7 bytes yet again");
                    {
                        rwpmath::Vector3 v0 = unit.GetVertex(0);
                        EATESTAssert(v0 == GetExpectedVertex(6), "First vertex should be vertexArray[6] again");
                        rwpmath::Vector3 v1 = unit.GetVertex(1);
                        EATESTAssert(v1 == GetExpectedVertex(2), "Second vertex should be vertexArray[2] again");
                        rwpmath::Vector3 v2 = unit.GetVertex(2);
                        EATESTAssert(v2 == GetExpectedVertex(5), "Third vertex should be vertexArray[5] again");
                    }
                }

                template <uint8_t COMPRESSION>
                void CheckGetVertexCompression(uint32_t vertices)
                {
                    // Inclusion of edge data and/or IDs should not be relevant, so just test once with them.
                    // Compression is important, so test with different compressions.
                    InitializeCluster(COMPRESSION, 2, 2);
                    WriteUnit(mCluster->UnitData(), mClusterParams, vertices, 6, 2, 5, 4, true);
                    TestUnit unit(*mCluster, mClusterParams);
                    rwpmath::Vector3 v0 = unit.GetVertex(0);
                    EATESTAssert(v0 == GetExpectedVertex(6), "First vertex should be vertexArray[6]");
                    rwpmath::Vector3 v1 = unit.GetVertex(1);
                    EATESTAssert(v1 == GetExpectedVertex(2), "Second vertex should be vertexArray[2]");
                    rwpmath::Vector3 v2 = unit.GetVertex(2);
                    EATESTAssert(v2 == GetExpectedVertex(5), "Third vertex should be vertexArray[5]");
                    if (vertices > 3)
                    {
                        rwpmath::Vector3 v3 = unit.GetVertex(3);
                        EATESTAssert(v3 == GetExpectedVertex(4), "Fourth vertex should be vertexArray[4]");
                    }
                }
                template <uint8_t COMPRESSION>
                void CheckGetVerticesFromTriCompression()
                {
                    rwpmath::Vector3 undefined(-1.0f, -1.0f, -1.0f);
                    // Inclusion of edge data and/or IDs should not be relevant, so just test once with them.
                    // Compression is important, so test with different compressions.
                    InitializeCluster(COMPRESSION, 2, 2);
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 6, 2, 5, 0, true);
                    TestUnit unit(*mCluster, mClusterParams);
                    rwpmath::Vector3 v0 = undefined;
                    rwpmath::Vector3 v1 = undefined;
                    rwpmath::Vector3 v2 = undefined;
                    unit.GetTriVertices(v0, v1, v2);
                    EATESTAssert(v0 == GetExpectedVertex(6), "First vertex should be vertexArray[6]");
                    EATESTAssert(v1 == GetExpectedVertex(2), "Seond vertex should be vertexArray[2]");
                    EATESTAssert(v2 == GetExpectedVertex(5), "Third vertex should be vertexArray[5]");
                }
                template <uint8_t COMPRESSION>
                void CheckGetVerticesFromQuadCompression()
                {
                    rwpmath::Vector3 undefined(-1.0f, -1.0f, -1.0f);
                    // Inclusion of edge data and/or IDs should not be relevant, so just test once with them.
                    // Compression is important, so test with different compressions.
                    InitializeCluster(COMPRESSION, 2, 2);
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 7, 9, 0, 4, true);
                    TestUnit unit(*mCluster, mClusterParams);
                    // First triangle is 0,1,2
                    rwpmath::Vector3 v0 = undefined;
                    rwpmath::Vector3 v1 = undefined;
                    rwpmath::Vector3 v2 = undefined;
                    unit.GetTriVertices(v0, v1, v2, 0);
                    EATESTAssert(v0 == GetExpectedVertex(7), "First vertex should be vertexArray[7]");
                    EATESTAssert(v1 == GetExpectedVertex(9), "Second vertex should be vertexArray[9]");
                    EATESTAssert(v2 == GetExpectedVertex(0), "Third vertex should be vertexArray[0]");
                    rwpmath::Vector3 w0 = undefined;
                    rwpmath::Vector3 w1 = undefined;
                    rwpmath::Vector3 w2 = undefined;
                    // 2nd triangle is 3,2,1
                    unit.GetTriVertices(w0, w1, w2, 1);
                    EATESTAssert(w0 == GetExpectedVertex(4), "Fourth vertex should be vertexArray[4]");
                    EATESTAssert(w1 == GetExpectedVertex(0), "Third vertex should be vertexArray[0]");
                    EATESTAssert(w2 == GetExpectedVertex(9), "Second vertex should be vertexArray[9]");
                }

                template <uint8_t COMPRESSION>
                void CheckGetVertexIndicesFromTriCompression()
                {
                    uint8_t undefined = 0xFF;
                    // Inclusion of edge data and/or IDs should not be relevant, so just test once with them.
                    // Compression is important, so test with different compressions.
                    InitializeCluster(COMPRESSION, 2, 2);
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 6, 2, 5, 0, true);
                    TestUnit unit(*mCluster, mClusterParams);
                    uint8_t v0 = undefined;
                    uint8_t v1 = undefined;
                    uint8_t v2 = undefined;
                    unit.GetTriVertexIndices(v0, v1, v2);
                    EATESTAssert(v0 == 6, "First vertex should have index 6");
                    EATESTAssert(v1 == 2, "Second vertex should have index 2");
                    EATESTAssert(v2 == 5, "Third vertex should have index 5");
                }

                template <uint8_t COMPRESSION>
                void CheckGetVertexIndicesFromQuadCompression()
                {
                    uint8_t undefined = 0xFF;
                    // Inclusion of edge data and/or IDs should not be relevant, so just test once with them.
                    // Compression is important, so test with different compressions.
                    InitializeCluster(COMPRESSION, 2, 2);
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 7, 9, 0, 4, true);
                    TestUnit unit(*mCluster, mClusterParams);
                    // First triangle is 0,1,2
                    uint8_t v0 = undefined;
                    uint8_t v1 = undefined;
                    uint8_t v2 = undefined;
                    unit.GetTriVertexIndices(v0, v1, v2, 0);
                    EATESTAssert(v0 == 7, "First vertex should have index 7");
                    EATESTAssert(v1 == 9, "Seond vertex should have index 9");
                    EATESTAssert(v2 == 0, "Third vertex should have index 0");
                    uint8_t w0 = undefined;
                    uint8_t w1 = undefined;
                    uint8_t w2 = undefined;
                    // 2nd triangle is 3,2,1
                    unit.GetTriVertexIndices(w0, w1, w2, 1);
                    EATESTAssert(w0 == 4, "First vertex should have index 4");
                    EATESTAssert(w1 == 0, "Seond vertex should have index 0");
                    EATESTAssert(w2 == 9, "Third vertex should have index 9");
                }

                void CheckGetVertexIndicesFromTri()
                {
                    CheckGetVertexIndicesFromTriCompression<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>();
                    CheckGetVertexIndicesFromTriCompression<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>();
                    CheckGetVertexIndicesFromTriCompression<rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>();
                }

                void CheckGetVertexIndicesFromQuad()
                {
                    CheckGetVertexIndicesFromQuadCompression<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>();
                    CheckGetVertexIndicesFromQuadCompression<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>();
                    CheckGetVertexIndicesFromQuadCompression<rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>();
                }

                void CheckGetVertexFromTri()
                {
                    CheckGetVertexCompression<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>(3);
                    CheckGetVertexCompression<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(3);
                    CheckGetVertexCompression<rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>(3);
                }
                void CheckGetVertexFromQuad()
                {
                    CheckGetVertexCompression<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>(4);
                    CheckGetVertexCompression<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(4);
                    CheckGetVertexCompression<rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>(4);
                }

                void CheckGetVerticesFromTri()
                {
                    CheckGetVerticesFromTriCompression<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>();
                    CheckGetVerticesFromTriCompression<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>();
                    CheckGetVerticesFromTriCompression<rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>();
                }

                void CheckGetVerticesFromQuad()
                {
                    CheckGetVerticesFromQuadCompression<rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED>();
                    CheckGetVerticesFromQuadCompression<rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>();
                    CheckGetVerticesFromQuadCompression<rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>();
                }

                void CheckGetEdgeCosinesFromTri()
                {
                    // First case - edge cosine data exists
                    InitializeCluster();
                    // Try all combination of edge data flags
                    for (uint32_t d = 0; d < 8; ++d)
                    {
                        for (uint32_t c = 0; c < 8; ++c)
                        {
                            uint32_t expectedFlags = rw::collision::VOLUMEFLAG_ISENABLED|rw::collision::VOLUMEFLAG_TRIANGLEUSEEDGECOS;
                            expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEONESIDED;
                            expectedFlags |= c*rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX;
                            expectedFlags |= d*rw::collision::VOLUMEFLAG_TRIANGLEVERT0DISABLE;
                            uint8_t ed0 = 0, ed1 = 0, ed2 = 0;
                            if (d & 1) ed0 |= rw::collision::EDGEFLAG_VERTEXDISABLE;
                            if (d & 2) ed1 |= rw::collision::EDGEFLAG_VERTEXDISABLE;
                            if (d & 4) ed2 |= rw::collision::EDGEFLAG_VERTEXDISABLE;
                            if (c & 1) ed0 |= rw::collision::EDGEFLAG_EDGECONVEX;
                            if (c & 2) ed1 |= rw::collision::EDGEFLAG_EDGECONVEX;
                            if (c & 4) ed2 |= rw::collision::EDGEFLAG_EDGECONVEX;
                            WriteUnit(mCluster->UnitData(), mClusterParams, 3, 
                                6, 2, 5, 0, true, 
                                (uint8_t) (9 | ed0), 
                                (uint8_t) (19 | ed1),
                                (uint8_t) (25 | ed2));
                            TestUnit unit(*mCluster, mClusterParams);
                            rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                            uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines);
                            EATESTAssert(flags == expectedFlags, "Should have expected flags");
                            rwpmath::Vector3 expectedCosines(
                                rw::collision::DecodeEdgeCos(9), 
                                rw::collision::DecodeEdgeCos(19), 
                                rw::collision::DecodeEdgeCos(25));
                            EATESTAssert(edgeCosines == expectedCosines, "Should have expected edge cosines");
                        }
                    }
                }

                void CheckGetDefaultEdgeCosinesFromTri()
                {
                    // Second case - no edge cosine data
                    InitializeCluster();
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 6, 2, 5, 0, false);
                    TestUnit unit(*mCluster, mClusterParams);
                    rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                    const uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines);
                    const uint32_t expectedFlags = rw::collision::GPTriangle::FLAG_TRIANGLEONESIDED;
                    EATESTAssert(flags == expectedFlags, "Should have one sided flag");
                    EATESTAssert(edgeCosines == rwpmath::GetVector3_Zero(), "Should have zero edge cosines");
                }

                void CheckGetDefaultEdgeCosinesFromTwoSidedTri()
                {
                    // Second case - no edge cosine data
                    InitializeCluster();
                    mClusterParams.mFlags = 0;
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 6, 2, 5, 0, false);
                    TestUnit unit(*mCluster, mClusterParams);
                    rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                    const uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines);
                    const uint32_t expectedFlags = 0;
                    EATESTAssert(flags == expectedFlags, "Should have one sided flag");
                    EATESTAssert(edgeCosines == rwpmath::GetVector3_Zero(), "Should have zero edge cosines");
                }

                void CheckGetEdgeCosinesFromTwoSidedTri()
                {
                    // Check one-sided flag is propagated, but no other bits from ClusterParams.mFlags
                    InitializeCluster();
                    mClusterParams.mFlags = (uint16_t) ~rw::collision::CMFLAG_ONESIDED;
                    // Set some different flags for increased coverage
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 
                        6, 2, 5, 0, true, 
                        9 | rw::collision::EDGEFLAG_EDGECONVEX, 
                        19 | rw::collision::EDGEFLAG_VERTEXDISABLE,
                        25);
                    TestUnit unit(*mCluster, mClusterParams);
                    rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                    uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines);
                    uint32_t expectedFlags = rw::collision::VOLUMEFLAG_ISENABLED|rw::collision::VOLUMEFLAG_TRIANGLEUSEEDGECOS;
                    expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX;
                    expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEVERT1DISABLE;
                    EATESTAssert(flags == expectedFlags, "Should have expected flags");
                    rwpmath::Vector3 expectedCosines(
                        rw::collision::DecodeEdgeCos(9), 
                        rw::collision::DecodeEdgeCos(19), 
                        rw::collision::DecodeEdgeCos(25));
                    EATESTAssert(edgeCosines == expectedCosines, "Should have expected edge cosines");
                }

                void CheckGetEdgeCosinesFromQuad()
                {
                    // First case - edge cosine data exists
                    InitializeCluster();
                    // Try all combination of edge data flags
                    for (uint32_t d = 0; d < 16; ++d)
                    {
                        for (uint32_t c = 0; c < 16; ++c)
                        {
                            uint8_t ed0 = 0, ed1 = 0, ed2 = 0, ed3 = 0;
                            if (d & 1) ed0 |= rw::collision::EDGEFLAG_VERTEXDISABLE;
                            if (d & 2) ed1 |= rw::collision::EDGEFLAG_VERTEXDISABLE;
                            if (d & 4) ed2 |= rw::collision::EDGEFLAG_VERTEXDISABLE;
                            if (d & 8) ed3 |= rw::collision::EDGEFLAG_VERTEXDISABLE;
                            if (c & 1) ed0 |= rw::collision::EDGEFLAG_EDGECONVEX;
                            if (c & 2) ed1 |= rw::collision::EDGEFLAG_EDGECONVEX;
                            if (c & 4) ed2 |= rw::collision::EDGEFLAG_EDGECONVEX;
                            if (c & 8) ed3 |= rw::collision::EDGEFLAG_EDGECONVEX;
                            WriteUnit(mCluster->UnitData(), mClusterParams, 4, 
                                6, 2, 5, 0, true, 
                                (uint8_t) (9 | ed0), 
                                (uint8_t) (19 | ed1),
                                (uint8_t) (25 | ed2),
                                (uint8_t) (14 | ed3));

                            // This is complex - we'll double check against the existing implementation.
                            rw::collision::GPTriangle tris[2];
                            rwpmath::Matrix44Affine identity = rwpmath::GetMatrix44Affine_Identity();
                            rw::collision::AABBox bbox(rwpmath::Vector3(-1000.0f,-1000.0f,-1000.0f), rwpmath::Vector3(1000.0f,1000.0f,1000.0f));
                            uint32_t numTris = 0;
                            mCluster->UnitGetOverlappingGPInstances(0, bbox, &identity, tris, numTris, mClusterParams);
                            EATESTAssert(numTris == 2, "Should get both tris");
                            int8_t centralFlag = 0;
                            rwpmath::Vector3 vs[4];
                            mCluster->Get4Vertices(vs, 6, 2, 5, 0, mClusterParams.mVertexCompressionGranularity);
                            float centralEdgeCosine = rw::collision::ComputeEdgeCos(centralFlag, vs[0], vs[1], vs[2], vs[3]);
                            EATESTAssert(centralFlag == 0, "Central edge should not be convex");

                            TestUnit unit(*mCluster, mClusterParams);
                            // Edge cosines and flags for first tri (v0,v1,v2)
                            {
                                rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                                uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines, 0);
                                uint32_t expectedFlags = rw::collision::VOLUMEFLAG_ISENABLED|rw::collision::VOLUMEFLAG_TRIANGLEUSEEDGECOS;
                                expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEONESIDED;
                                if (c & 1) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX;
                                if (d & 1) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEVERT0DISABLE;
                                if (centralFlag) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX;
                                if (d & 2) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEVERT1DISABLE;
                                if (c & 4) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX;
                                if (d & 4) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEVERT2DISABLE;
                                rwpmath::Vector3 expectedCosines(
                                    rw::collision::DecodeEdgeCos(9), 
                                    centralEdgeCosine, 
                                    rw::collision::DecodeEdgeCos(25));
                                EATESTAssert(tris[0].mFlags == expectedFlags, "Expected flags on first tri should match GP");
                                EATESTAssert(tris[0].EdgeCosines() == expectedCosines, "Expected edge cosines on first tri should match GP");
                                EATESTAssert(flags == expectedFlags, "Should have expected flags on first tri");
                                EATESTAssert(edgeCosines == expectedCosines, "Should have expected edge cosines on first tri");
                            }
                            // Edge cosines and flags for second tri (v3,v2,v1)
                            {
                                rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                                uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines, 1);
                                uint32_t expectedFlags = rw::collision::VOLUMEFLAG_ISENABLED|rw::collision::VOLUMEFLAG_TRIANGLEUSEEDGECOS;
                                expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEONESIDED;
                                if (c & 8) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX;
                                if (d & 8) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEVERT0DISABLE;
                                if (centralFlag) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX;
                                if (d & 4) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEVERT1DISABLE;
                                if (c & 2) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX;
                                if (d & 2) expectedFlags |= rw::collision::VOLUMEFLAG_TRIANGLEVERT2DISABLE;
                                rwpmath::Vector3 expectedCosines(
                                    rw::collision::DecodeEdgeCos(14), 
                                    centralEdgeCosine, 
                                    rw::collision::DecodeEdgeCos(19));
                                EATESTAssert(tris[1].mFlags == expectedFlags, "Expected flags on second tri should match GP");
                                EATESTAssert(tris[1].EdgeCosines() == expectedCosines, "Expected edge cosines on second tri should match GP");
                                EATESTAssert(flags == expectedFlags, "Should have expected flags on second tri");
                                EATESTAssert(edgeCosines == expectedCosines, "Should have expected edge cosines on second tri");
                            }
                        }
                    }
                }

                void CheckGetDefaultEdgeCosinesFromQuad()
                {
                    // Second case - no edge cosine data
                    InitializeCluster();
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 
                        6, 2, 5, 0, false);
                    TestUnit unit(*mCluster, mClusterParams);
                    // First tri
                    {
                        rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                        const uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines, 0);
                        const uint32_t expectedFlags = rw::collision::GPTriangle::FLAG_TRIANGLEONESIDED;
                        EATESTAssert(flags == expectedFlags, "Should have one sided flag for first tri");
                        EATESTAssert(edgeCosines == rwpmath::GetVector3_Zero(), "Should have zero edge cosines for first tri");
                    }
                    // Second tri
                    {
                        rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                        const uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines, 1);
                        const uint32_t expectedFlags = rw::collision::GPTriangle::FLAG_TRIANGLEONESIDED;
                        EATESTAssert(flags == expectedFlags, "Should have one sided flag for second tri");
                        EATESTAssert(edgeCosines == rwpmath::GetVector3_Zero(), "Should have zero edge cosines for second tri");
                    }
                }

                void CheckGetDefaultEdgeCosinesFromTwoSidedQuad()
                {
                    // Second case - no edge cosine data
                    InitializeCluster();
                    mClusterParams.mFlags = 0;
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 6, 2, 5, 0, false);
                    TestUnit unit(*mCluster, mClusterParams);
                    // First tri
                    {
                        rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                        const uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines, 0);
                        const uint32_t expectedFlags = 0;
                        EATESTAssert(flags == expectedFlags, "Should have no flags for first tri");
                        EATESTAssert(edgeCosines == rwpmath::GetVector3_Zero(), "Should have zero edge cosines for first tri");
                    }
                    // Second tri
                    {
                        rwpmath::Vector3 edgeCosines(-10.0f, -10.0f, -10.0f);
                        const uint32_t flags = unit.GetEdgeCosinesAndFlags(edgeCosines, 1);
                        const uint32_t expectedFlags = 0;
                        EATESTAssert(flags == expectedFlags, "Should have no flags for second tri");
                        EATESTAssert(edgeCosines == rwpmath::GetVector3_Zero(), "Should have zero edge cosines for second tri");
                    }
                }

                virtual void CheckGetIDsFromUnit(uint8_t groupIdBytes, uint8_t surfaceIdBytes, uint32_t numVertices, bool includeEdgeCosines = false)
                {
                    uint32_t masks[3] = { 0x0, 0xff, 0xffff };
                    uint32_t mask = masks[groupIdBytes] | (masks[surfaceIdBytes])<<16;
                    InitializeCluster(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED,
                        groupIdBytes, surfaceIdBytes);
                    {
                        const uint16_t groupID = 0x1234;
                        const uint16_t surfaceID = 0xfdeb;
                        // Tri unit, no edge cosines
                        WriteUnit(mCluster->UnitData(), mClusterParams, numVertices, 4,1,3,9,
                            includeEdgeCosines, 0,0,0,0, groupID, surfaceID);
                        TestUnit unit(*mCluster, mClusterParams);

                        uint32_t id;
                        id = unit.GetID();
                        EATESTAssert((0xfdeb1234 & mask) == id, "Should combine 2 IDs");

                        id = unit.GetGroupID();
                        EATESTAssert((groupID & masks[groupIdBytes]) == id, "Invalid group ID");

                        id = unit.GetSurfaceID();
                        EATESTAssert((surfaceID & masks[surfaceIdBytes]) == id, "Invalid surface ID");

                        // Double check its the same as GPTriangles use
                        rw::collision::GPTriangle tris[2];
                        rwpmath::Matrix44Affine identity = rwpmath::GetMatrix44Affine_Identity();
                        rw::collision::AABBox bbox(rwpmath::Vector3(-1000.0f,-1000.0f,-1000.0f), rwpmath::Vector3(1000.0f,1000.0f,1000.0f));
                        uint32_t numTris = 0;
                        mCluster->UnitGetOverlappingGPInstances(0, bbox, &identity, tris, numTris, mClusterParams);
                        EATESTAssert(numTris == numVertices-2, "Should get all tris");
                        EATESTAssert(tris[0].mUserTag == (0xfdeb1234 & mask), "Should be same as GP");
                    }
                    // Default surfaceID
                    {
                        const uint16_t groupID = 0x1234;
                        const uint16_t surfaceID = 0x0;
                        // Tri unit, no edge cosines
                        WriteUnit(mCluster->UnitData(), mClusterParams, numVertices, 4,1,3,9,
                            includeEdgeCosines, 0,0,0,0, groupID, surfaceID);
                        TestUnit unit(*mCluster, mClusterParams);

                        uint32_t id;
                        id = unit.GetID();
                        EATESTAssert((0x00001234 & mask) == id, "Should combine 2 IDs when surfaceID default");

                        id = unit.GetGroupID();
                        EATESTAssert((groupID & masks[groupIdBytes]) == id, "Invalid group ID when surfaceID default");

                        id = unit.GetSurfaceID();
                        EATESTAssert((surfaceID & masks[surfaceIdBytes]) == id, "Invalid surface ID when surfaceID default");
                    }
                    // Both default
                    {
                        const uint16_t groupID = 0x0;
                        const uint16_t surfaceID = 0x0;
                        // Tri unit, no edge cosines
                        WriteUnit(mCluster->UnitData(), mClusterParams, numVertices, 4,1,3,9,
                            includeEdgeCosines, 0,0,0,0, groupID, surfaceID);
                        TestUnit unit(*mCluster, mClusterParams);

                        uint32_t id;
                        id = unit.GetID();
                        EATESTAssert((0x00000000 & mask) == id, "Should combine 2 IDs when both default");

                        id = unit.GetGroupID();
                        EATESTAssert((groupID & masks[groupIdBytes]) == id, "Invalid group ID when both default");

                        id = unit.GetSurfaceID();
                        EATESTAssert((surfaceID & masks[surfaceIdBytes]) == id, "Invalid surface ID when both default");
                    }
                    // Default groupID
                    {
                        const uint16_t groupID = 0x0;
                        const uint16_t surfaceID = 0xfdeb;
                        // Tri unit, no edge cosines
                        WriteUnit(mCluster->UnitData(), mClusterParams, numVertices, 4,1,3,9,
                            includeEdgeCosines, 0,0,0,0, groupID, surfaceID);
                        TestUnit unit(*mCluster, mClusterParams);

                        uint32_t id;
                        id = unit.GetID();
                        EATESTAssert((0xfdeb0000 & mask) == id, "Should combine 2 IDs when groupID default");

                        id = unit.GetGroupID();
                        EATESTAssert((groupID & masks[groupIdBytes]) == id, "Invalid group ID when groupID default");

                        id = unit.GetSurfaceID();
                        EATESTAssert((surfaceID & masks[surfaceIdBytes]) == id, "Invalid surface ID when groupID default");
                    }
                }

                void CheckGetIDsFromUnit(uint32_t numVertices, bool includeEdgeCosines = false)
                {
                    // Loop over possible ID sizes
                    for (uint8_t groupIdBytes = 0; groupIdBytes <= 2; ++groupIdBytes)
                    {
                        for (uint8_t surfaceIdBytes = 0; surfaceIdBytes <= 2; ++surfaceIdBytes)
                        {
                            CheckGetIDsFromUnit(groupIdBytes, surfaceIdBytes, numVertices, includeEdgeCosines);
                        }
                    }
                }

                void CheckGetIDsFromTri()
                {
                    CheckGetIDsFromUnit(3);
                }

                void CheckGetIDsFromTriWithEdgeCosines()
                {
                    CheckGetIDsFromUnit(3, true);
                }

                void CheckGetIDsFromQuad()
                {
                    CheckGetIDsFromUnit(4);
                }

                void CheckGetIDsFromQuadWithEdgeCosines()
                {
                    CheckGetIDsFromUnit(4, true);
                }

            protected:

                rwpmath::Vector3 GetExpectedVertex(uint8_t i)
                {
                    return mCluster->template GetVertexBase<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC>(i, mClusterParams.mVertexCompressionGranularity);
                }

                void InitializeCluster(uint8_t compression = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED,
                    uint8_t groupIDBytes = 0, uint8_t surfaceIDBytes = 0)
                {
                    uint8_t nx = 4;
                    uint8_t ny = 4;
                    uint8_t numVertices = (uint8_t) (nx*ny);

                    // Initialize cluster parameters
                    mClusterParams.mFlags = rw::collision::CMFLAG_ONESIDED;
                    mClusterParams.mGroupIdSize = groupIDBytes;
                    mClusterParams.mSurfaceIdSize = surfaceIDBytes;
                    mClusterParams.mVertexCompressionGranularity = 0.01f;

                    // Initialize cluster
                    mCluster = reinterpret_cast<rw::collision::ClusteredMeshCluster *>(mClusterData);
                    mCluster->unitCount = 1;
                    mCluster->unitDataSize = 100;
                    mCluster->unitDataStart = numVertices;
                    mCluster->normalStart = 0;
                    mCluster->totalSize = 1000;
                    mCluster->vertexCount = numVertices;
                    mCluster->normalCount = 0;
                    mCluster->compressionMode = compression;

                    // Initialize vertices
                    for (uint8_t i = 0; i < nx; ++i)
                    {
                        for (uint8_t j = 0; j < ny; ++j)
                        {
                            float x = (float) (i);
                            float y = (float) (j);
                            rwpmath::Vector3 v(rwpmath::Cos(x/10.0f), rwpmath::Sin(x/10.0f)*rwpmath::Cos(y/10.0f), rwpmath::Cos(y/10.0f));
                            uint8_t index = (uint8_t) (i*ny + j);
                            SetVertex(*mCluster, v, index, mClusterParams.mVertexCompressionGranularity);
                        }
                    }
                }

            protected:    // Functions to initialize the cluster with all different types of units

                void CreateTriUnit()
                {
                    InitializeCluster();
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 6, 2, 5);
                }

                void CreateTriUnitWithEdgeCosines()
                {
                    InitializeCluster();
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 
                        6, 2, 5, 0, true, 
                        9, 
                        19 | rw::collision::EDGEFLAG_EDGECONVEX,
                        25 | rw::collision::EDGEFLAG_VERTEXDISABLE);
                }

                void CreateTriUnitWithIDs(uint8_t groupIDBytes, uint8_t surfaceIDBytes)
                {
                    InitializeCluster(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, groupIDBytes, surfaceIDBytes);
                    uint16_t groupID = 0x1234;
                    uint16_t surfaceID = 0xfedc;
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 
                        6, 2, 5, 0, false, 0, 0, 0, 0, groupID, surfaceID);
                }

                void CreateTriUnitWithEdgeCosinesAndIDs(uint8_t groupIDBytes, uint8_t surfaceIDBytes)
                {
                    InitializeCluster(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, groupIDBytes, surfaceIDBytes);
                    uint16_t groupID = 0x1234;
                    uint16_t surfaceID = 0xfedc;
                    WriteUnit(mCluster->UnitData(), mClusterParams, 3, 
                        6, 2, 5, 0, true,
                        9, 
                        19 | rw::collision::EDGEFLAG_EDGECONVEX,
                        25 | rw::collision::EDGEFLAG_VERTEXDISABLE,
                        0,
                        groupID, surfaceID);
                }

                void CreateQuadUnit()
                {
                    InitializeCluster();
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 1, 5, 0, 3);
                }

                void CreateQuadUnitWithEdgeCosines()
                {
                    InitializeCluster();
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 
                        1, 5, 0, 3, true, 
                        9, 
                        19 | rw::collision::EDGEFLAG_EDGECONVEX,
                        25 | rw::collision::EDGEFLAG_VERTEXDISABLE,
                        rw::collision::EDGEFLAG_EDGECONVEX | rw::collision::EDGEFLAG_VERTEXDISABLE);
                }

                void CreateQuadUnitWithIDs(uint8_t groupIDBytes, uint8_t surfaceIDBytes)
                {
                    InitializeCluster(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, groupIDBytes, surfaceIDBytes);
                    uint16_t groupID = 0x1234;
                    uint16_t surfaceID = 0xfedc;
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 
                        1, 5, 0, 3, false, 0, 0, 0, 0, groupID, surfaceID);
                }

                void CreateQuadUnitWithEdgeCosinesAndIDs(uint8_t groupIDBytes, uint8_t surfaceIDBytes)
                {
                    InitializeCluster(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, groupIDBytes, surfaceIDBytes);
                    uint16_t groupID = 0x1234;
                    uint16_t surfaceID = 0xfedc;
                    WriteUnit(mCluster->UnitData(), mClusterParams, 4, 
                        1, 5, 0, 3, true,
                        9, 
                        19 | rw::collision::EDGEFLAG_EDGECONVEX,
                        25 | rw::collision::EDGEFLAG_VERTEXDISABLE,
                        rw::collision::EDGEFLAG_EDGECONVEX | rw::collision::EDGEFLAG_VERTEXDISABLE,
                        groupID, surfaceID);
                }

            protected:    // Static helper functions. Probably belong elsewhere.

                static uint32_t WriteUnit(uint8_t * data,
                    const rw::collision::ClusterParams & clusterParams,
                    uint32_t numVertices,
                    uint8_t vi0, uint8_t vi1, uint8_t vi2, uint8_t vi3 = 0,
                    bool useEdgeCosines = false,
                    uint8_t ed0 = 0, uint8_t ed1 = 0, uint8_t ed2 = 0, uint8_t ed3 = 0,
                    uint16_t groupID = 0,
                    uint16_t surfaceID = 0)
                {
                    EA_ASSERT(numVertices == 3 || numVertices == 4);
                    uint32_t s = 0;
                    uint8_t type = rw::collision::UNITTYPE_TRIANGLE;
                    if (numVertices == 4)
                    {
                        type = rw::collision::UNITTYPE_QUAD;
                    }
                    data[++s] = vi0;
                    data[++s] = vi1;
                    data[++s] = vi2;
                    if (numVertices == 4)
                    {
                        data[++s] = vi3;
                    }
                    if (useEdgeCosines)
                    {
                        data[++s] = ed0;
                        data[++s] = ed1;
                        data[++s] = ed2;
                        if (numVertices == 4)
                        {
                            data[++s] = ed3;
                        }
                        type |= rw::collision::UNITFLAG_EDGEANGLE;
                    }
                    if (groupID)
                    {
                        for (uint32_t b = 0; b < clusterParams.mGroupIdSize; ++b)
                        {
                            data[++s] = (uint8_t) (groupID & 255);
                            groupID = (uint8_t) (groupID >> 8);
                            type |= rw::collision::UNITFLAG_GROUPID;
                        }
                    }
                    if (surfaceID)
                    {
                        for (uint32_t b = 0; b < clusterParams.mSurfaceIdSize; ++b)
                        {
                            data[++s] = (uint8_t) (surfaceID & 255);
                            surfaceID = (uint8_t) (surfaceID >> 8);
                            type |= rw::collision::UNITFLAG_SURFACEID;
                        }
                    }
                    data[0] = type;
                    return s+1;
                }

                // Utility code to write a vertex with current compression.
                static void SetVertex(rw::collision::ClusteredMeshCluster & cluster,
                    rwpmath::Vector3::InParam v, uint8_t vertid, const float & vertexGranularity)
                {
                    EA_ASSERT(vertid < cluster.vertexCount);
                    rwpmath::Vector3 * vertexArray = cluster.vertexArray;
                    switch (cluster.compressionMode)
                    {
                    default:
                        EA_FAIL_MSG("Unknown compression mode");
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED:
                        vertexArray[vertid] = v;
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED:
                        {
                            const int32_t *vertexOffsetData = reinterpret_cast<const int32_t*>(vertexArray);
                            rw::collision::ClusteredMeshCluster::Vertex16 *vertData = reinterpret_cast<rw::collision::ClusteredMeshCluster::Vertex16*>(vertexArray);
                            vertData += 2; // skip the first 6 bytes taken up by cluster offset
                            const int32_t offsetX = vertexOffsetData[0], offsetY = vertexOffsetData[1], offsetZ = vertexOffsetData[2];
                            int32_t x = (int32_t) ((v.X() / vertexGranularity));
                            int32_t y = (int32_t) ((v.Y() / vertexGranularity));
                            int32_t z = (int32_t) ((v.Z() / vertexGranularity));
                            vertData[vertid].x = (uint16_t) (x - offsetX);
                            vertData[vertid].y = (uint16_t) (y - offsetY);
                            vertData[vertid].z = (uint16_t) (z - offsetZ);
                        }
                        break;
                    case rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED:
                        {
                            rw::collision::ClusteredMeshCluster::Vertex32 *vertData = reinterpret_cast<rw::collision::ClusteredMeshCluster::Vertex32*>(vertexArray);
                            int32_t x = (int32_t) ((v.X() / vertexGranularity));
                            int32_t y = (int32_t) ((v.Y() / vertexGranularity));
                            int32_t z = (int32_t) ((v.Z() / vertexGranularity));
                            vertData[vertid].x = x;
                            vertData[vertid].y = y;
                            vertData[vertid].z = z;
                        }
                        break;
                    }
                }

                const char * mSuiteName;
                const char * mSpuElf;
                bool mSupportQuads;
                bool mAssumesEdgeCosines;
                bool mSupportsIDs;
                rw::collision::ClusterParams mClusterParams;
                rw::collision::ClusteredMeshCluster * mCluster;
                EA_ALIGNED(uint8_t, mClusterData[4000], 16);

            };
        }
    }
}

// ***********************************************************************************************************

#endif  // !TEST_CLUSTERUNIT_H
