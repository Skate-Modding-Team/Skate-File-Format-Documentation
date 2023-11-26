// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

File: test-cluster.cpp

Purpose: Test for clustered mesh cluster operations.

*/

#include <EABase/eabase.h>

#include <rw/collision/common.h>
#include <rw/collision/volumedata.h>
#include <rw/collision/aabbox.h>
#include <rw/collision/volume.h>
#include <rw/collision/triangle.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/clusteredmeshcluster.h>

#if !defined(EA_PLATFORM_PS3_SPU)
#include <rw/collision/initialize.h>
#endif // !defined(EA_PLATFORM_PS3_SPU)

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <EAAssert/eaassert.h>

#include <rw/collision/clusterunitbase.h>

#include <eaphysics/unitframework/creator.h>

#include "testsuitebase.h" // For TestSuiteBase

#include "stdio.h"     // for sprintf()

namespace EA 
{
namespace Collision
{
namespace Tests
{

using rw::collision::UnitParameters;

class TestCluster : public rw::collision::tests::TestSuiteBase
{
public:     // TestSuite overrides

#define REGISTER_CLUSTER_TEST(M, D) EATEST_REGISTER(#M, D, TestCluster, M)

    virtual void Initialize()
    {
        SuiteName("TestCluster");
        REGISTER_CLUSTER_TEST(TestDecodeEdgeCosine, "Check DecodeEdgeCosine()");
        REGISTER_CLUSTER_TEST(TestDecodeEdgeCosines3, "Check DecodeEdgeCosines(Vector3)");
        REGISTER_CLUSTER_TEST(TestDecodeEdgeCosines4, "Check DecodeEdgeCosines(Vector4)");
        REGISTER_CLUSTER_TEST(TestExtractTriEdgeData, "Check ExtractTriEdgeData()");
        REGISTER_CLUSTER_TEST(TestExtractQuadEdgeData, "Check ExtractQuadEdgeData()");

        // GetSize Method unit tests
        REGISTER_CLUSTER_TEST(TestGetSizeZero, "Check GetSize() Zero");
        REGISTER_CLUSTER_TEST(TestGetSizeVerticesOnly, "Check GetSize() Vertices Only");
        REGISTER_CLUSTER_TEST(TestGetSizeTriangleUnitsOnly, "Check GetSize() Triangle Units Only");
        REGISTER_CLUSTER_TEST(TestGetSizeQuadUnitsOnly, "Check GetSize() Quad Units Only");
        REGISTER_CLUSTER_TEST(TestGetSizeUnitsOnly, "Check GetSize() Both Units Only");
        REGISTER_CLUSTER_TEST(TestGetSizeEdgeCosineOnly, "Check GetSize() EdgeCosine Only");
        REGISTER_CLUSTER_TEST(TestGetSizeGroupIDSizeOne, "Check GetSize() GroupID Size One");
        REGISTER_CLUSTER_TEST(TestGetSizeGroupIDSizeTwo, "Check GetSize() GroupID Size Two");
        REGISTER_CLUSTER_TEST(TestGetSizeSurfaceIDSizeOne, "Check GetSize() GroupID Size One");
        REGISTER_CLUSTER_TEST(TestGetSizeSurfaceIDSizeTwo, "Check GetSize() GroupID Size Two");
        REGISTER_CLUSTER_TEST(TestGetSizeFull, "Check GetSize() Full");
        REGISTER_CLUSTER_TEST(TestGetSize16BitVertexCompression, "Check GetSize() 16Bit vertex compression");
        REGISTER_CLUSTER_TEST(TestGetSize32BitVertexCompression, "Check GetSize() 32Bit vertex compression");

        // These unit tests use rw::GetDefaultAllocator and do not build for SPU
#if !defined(EA_PLATFORM_PS3_SPU)
        // Initialize Method unit tests
        REGISTER_CLUSTER_TEST(TestInitializeEmpty, "Check Initialize() Empty");
        REGISTER_CLUSTER_TEST(TestInitialize16BitCompressed, "Check Initialize() 16Bit Vertices");
        REGISTER_CLUSTER_TEST(TestInitialize32BitCompressed, "Check Initialize() 32Bit Vertices");
        REGISTER_CLUSTER_TEST(TestInitializeUncompressed, "Check Initialize() Uncompressed Vertices");
#endif // !defined(EA_PLATFORM_PS3_SPU)

        // GetUnitSize Method unit tests
        REGISTER_CLUSTER_TEST(TestGetUnitSizeSimpleTriangle, "Check GetUnitSize() Simple Triangle");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeSimpleQuad, "Check GetUnitSize() Simple Quad");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeTriangleEdgeCos, "Check GetUnitSize() Triangle EdgeCosine");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeQuadEdgeCos, "Check GetUnitSize() Quad EdgeCosine");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeTriangleGroupID, "Check GetUnitSize() Triangle GroupID");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeTriangleSurfaceID, "Check GetUnitSize() Triangle SurfaceID");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeTriangleBothIDs, "Check GetUnitSize() Triangle Both IDs");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeQuadGroupID, "Check GetUnitSize() Quad GroupID");
        REGISTER_CLUSTER_TEST(TestGetUnitSizeQuadSurfaceID, "Check GetUnitSize() Quad SurfaceID");

        // These two tests use TriangleVolumes and do not build for SPU
#if !defined(EA_PLATFORM_PS3_SPU)
        // SetVertexOffset Method unit tests
        REGISTER_CLUSTER_TEST(TestSetVertexOffset16BitCompression, "Check SetVertexOffset() 16Bit Compression");
        REGISTER_CLUSTER_TEST(TestSetVertexOffset32BitCompression, "Check SetVertexOffset() 32Bit Compression");
        REGISTER_CLUSTER_TEST(TestSetVertexOffsetNoCompression, "Check SetVertexOffset() No Compression");

        // SetVertex Method unit tests
        REGISTER_CLUSTER_TEST(TestSetVertex16BitCompression, "Check SetVertex() 16Bit Compression");
        REGISTER_CLUSTER_TEST(TestSetVertex32BitCompression, "Check SetVertex() 32Bit Compression");
        REGISTER_CLUSTER_TEST(TestSetVertexNoCompressionSingle, "Check SetVertex() No Compression Single Vertex");
        REGISTER_CLUSTER_TEST(TestSetVertexNoCompressionMultiple, "Check SetVertex() No Compression Multiple Vertices");

        // SetTriangle Method unit tests
        REGISTER_CLUSTER_TEST(TestSetTriangle, "Check SetTriangle()");

        // GetTriangleVolume unit tests
        REGISTER_CLUSTER_TEST(TestGetTriangleVolumeTriangle, "Check GetTriangleVolume()");
        REGISTER_CLUSTER_TEST(TestGetTriangleVolumeQuad, "Check GetTriangleVolume()");

        // GetTriangleIndices unit tests
        REGISTER_CLUSTER_TEST(TestGetTriangleIndicesTriangle, "Check GetTriangleIndices()");
        REGISTER_CLUSTER_TEST(TestGetTriangleIndicesQuad, "Check GetTriangleIndices()");

#endif // !defined(EA_PLATFORM_PS3_SPU)

        // Run the whole tests suite on SPU too
        EATEST_REGISTER_SPU("TestClusterSPU", "SPU cluster tests", "test-cluster.elf");
    }

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();
#if !defined(EA_PLATFORM_PS3_SPU)
        // Initialize the volume virtual function table
        rw::collision::InitializeVTables();
#endif // !defined(EA_PLATFORM_PS3_SPU)

        // We don't read any unit or vertex data from cluster since our MockUnit doesn't, 
        // but we do need some bits of the header stuff
        mCluster.unitCount = 5u;
        mCluster.unitDataSize = 123u;
        mCluster.unitDataStart = 2u;
        // These members shouldn't be used, so we'll set them to something unusual
        mCluster.normalStart = 3u;
        mCluster.totalSize = 723u;
        mCluster.vertexCount = 44u;
        mCluster.normalCount = 196u;
        mCluster.compressionMode = 57u;
        mClusterParams.mVertexCompressionGranularity = 0.0f;
        mClusterParams.mFlags = 33u;
        mClusterParams.mGroupIdSize = 33u;
        mClusterParams.mSurfaceIdSize = 33u;
    }

protected:

    union DataUnion
    {
        // can't put an rwpmath::Vector3 in here since it has a copy constructor
        uint32_t u[4];
        float f[4];
    };


private:

    void TestDecodeEdgeCosine();
    void TestDecodeEdgeCosines3();
    void TestDecodeEdgeCosines4();
    void TestExtractTriEdgeData();
    void TestExtractQuadEdgeData();

    // GetSize unit tests
    void TestGetSizeZero();
    void TestGetSizeVerticesOnly();
    void TestGetSizeTriangleUnitsOnly();
    void TestGetSizeQuadUnitsOnly();
    void TestGetSizeUnitsOnly();
    void TestGetSizeEdgeCosineOnly();
    void TestGetSizeGroupIDSizeOne();
    void TestGetSizeGroupIDSizeTwo();
    void TestGetSizeSurfaceIDSizeOne();
    void TestGetSizeSurfaceIDSizeTwo();
    void TestGetSizeFull();
    void TestGetSize16BitVertexCompression();
    void TestGetSize32BitVertexCompression();

#if !defined(EA_PLATFORM_PS3_SPU)
    // Initialize unit tests
    void TestInitializeEmpty();
    void TestInitialize16BitCompressed();
    void TestInitialize32BitCompressed();
    void TestInitializeUncompressed();
#endif // !defined(EA_PLATFORM_PS3_SPU)

    // GetUnitSize unit tests
    void TestGetUnitSizeSimpleTriangle();
    void TestGetUnitSizeSimpleQuad();
    void TestGetUnitSizeTriangleEdgeCos();
    void TestGetUnitSizeQuadEdgeCos();
    void TestGetUnitSizeTriangleGroupID();
    void TestGetUnitSizeTriangleSurfaceID();
    void TestGetUnitSizeTriangleBothIDs();
    void TestGetUnitSizeQuadGroupID();
    void TestGetUnitSizeQuadSurfaceID();

#if !defined(EA_PLATFORM_PS3_SPU)
    // SetVertexOffset unit tests
    void TestSetVertexOffset16BitCompression();
    void TestSetVertexOffset32BitCompression();
    void TestSetVertexOffsetNoCompression();

    // SetVertex unit tests
    void TestSetVertex16BitCompression();
    void TestSetVertex32BitCompression();
    void TestSetVertexNoCompressionSingle();
    void TestSetVertexNoCompressionMultiple();

    // SetTriangle unit tests
    void TestSetTriangle();

    // GetVolume unit tests
    void TestGetTriangleVolumeTriangle();
    void TestGetTriangleVolumeQuad();

    // GetTriangleIndices unit tests
    void TestGetTriangleIndicesTriangle();
    void TestGetTriangleIndicesQuad();

#endif // !defined(EA_PLATFORM_PS3_SPU)

    template <class COMPARISON_TYPE>
    void CheckValue(COMPARISON_TYPE actual, COMPARISON_TYPE expected, const char * msg);

    void CheckFlag(bool actual, bool expected, const char * msg)
    {
        char str[256];
        sprintf(str, "%s flag should be %s", msg, expected ? "true" : "false");
        EATESTAssert(actual == expected, str);
    }
    void CheckEdgeFlag(uint32_t edge, uint32_t flags, uint32_t expected, const char * msg)
    {
        bool a = ((flags & (rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX<<edge)) != 0);
        bool e = ((expected & rw::collision::EDGEFLAG_EDGECONVEX) != 0);
        CheckFlag(a,e,msg);
    }
    void CheckEdgeFlag(rwpmath::MaskScalar::InParam flag, uint32_t expected, const char * msg)
    {
        bool a = flag.GetBool();
        bool e = ((expected & rw::collision::EDGEFLAG_EDGECONVEX) != 0);
        CheckFlag(a,e,msg);
    }

    void CheckVertexFlag(uint32_t vertex, uint32_t flags, uint32_t expected, const char * msg)
    {
        bool a = ((flags & (rw::collision::VOLUMEFLAG_TRIANGLEVERT0DISABLE<<vertex)) != 0);
        bool e = ((expected & rw::collision::EDGEFLAG_VERTEXDISABLE) != 0);
        CheckFlag(a,e,msg);
    }
    void CheckVertexFlag(rwpmath::MaskScalar::InParam flag, uint32_t expected, const char * msg)
    {
        bool a = flag.GetBool();
        bool e = ((expected & rw::collision::EDGEFLAG_VERTEXDISABLE) != 0);
        CheckFlag(a,e,msg);
    }

    void CheckTriFlag(uint32_t flags, uint32_t expected, const char * msg)
    {
        bool a = ((flags & rw::collision::VOLUMEFLAG_TRIANGLEONESIDED) != 0);
        bool e = ((expected & rw::collision::CMFLAG_ONESIDED) != 0);
        CheckFlag(a,e,msg);
    }
    void CheckTriFlag(rwpmath::MaskScalar::InParam flag, uint32_t expected, const char * msg)
    {
        bool a = flag.GetBool();
        bool e = ((expected & rw::collision::CMFLAG_ONESIDED) != 0);
        CheckFlag(a,e,msg);
    }

    static uint8_t EdgeFlags(uint32_t d)
    {
        return (uint8_t) (d << 5);
    }
    static uint32_t TriFlags(uint32_t d)
    {
        switch (d)
        {

        case 0: return 0;
        case 1: return rw::collision::VOLUMEFLAG_TRIANGLEONESIDED;
        case 2: return 0xffffffff;
        case 3: return 0x12345600u;
        case 4: return 0x12345601u;
        default: return 0x12345678u;
        }
    }

    static float DecodeEdgeCosineReference(uint8_t i)
    {
        const float PI_SQUARED = rwpmath::PI*rwpmath::PI;
        uint32_t e = (uint32_t) (i & ((uint8_t) rw::collision::EDGEFLAG_ANGLEMASK));
        return (1.0f - ldexp(PI_SQUARED, -((int) e + 3)));
    }

    void * GetResultsBuffer() const
    {
        return sResultsBuffer;
    }

    rw::collision::ClusteredMeshCluster mCluster;
    rw::collision::ClusterParams mClusterParams;

    // Number of iterations for benchmarks
    static const uint32_t mNumIterations = 50;

    static const uint32_t RESULTS_SIZE = 300*16 + 1300;
    static uint8_t sResultsBuffer[RESULTS_SIZE];

    // Collection of Unit data Sizes
    static const uint16_t mSizeOfTriangleUnit = 4;
    static const uint16_t mSizeOfQuadUnit = 5;
    static const uint16_t mSizeOfTriangleEdgeCosines = 3;
    static const uint16_t mSizeOfQuadEdgeCosines = 4;

} gTestCluster;


template <class COMPARISON_TYPE>
void
TestCluster::CheckValue(COMPARISON_TYPE actual, COMPARISON_TYPE expected, const char * msg)
{
    char str[256];
    sprintf(str, "%s should be %u", msg, expected);
    EATESTAssert(actual == expected, str);
}

template<>
void
TestCluster::CheckValue<uint32_t>(uint32_t actual, uint32_t expected, const char * msg)
{
    char str[256];
    sprintf(str, "%s should be %u", msg, expected);
    EATESTAssert(actual == expected, str);
}

template<>
void
TestCluster::CheckValue<uint16_t>(uint16_t actual, uint16_t expected, const char * msg)
{
    char str[256];
    sprintf(str, "%s should be %u", msg, expected);
    EATESTAssert(actual == expected, str);
}

template<>
void
TestCluster::CheckValue<rwpmath::Vector3::InParam>(rwpmath::Vector3::InParam actual, rwpmath::Vector3::InParam expected, const char * msg)
{
    char str[256];
    sprintf(str, "%s should be (%f, %f, %f)", msg, static_cast<float>(expected.GetX()), static_cast<float>(expected.GetY()), static_cast<float>(expected.GetZ()));
    EATESTAssert(actual == expected, str);
}

template<>
void
TestCluster::CheckValue<const rw::collision::ClusteredMeshCluster::Vertex32&>(const rw::collision::ClusteredMeshCluster::Vertex32& actual,
                                                                              const rw::collision::ClusteredMeshCluster::Vertex32& expected,
                                                                              const char * msg)
{
    char str[256];
    sprintf(str, "%s should be (%d, %d, %d)", msg, expected.x, expected.y, expected.z);
    EATESTAssert( (actual.x == expected.x) &&
                  (actual.y == expected.y) &&
                  (actual.z == expected.z), str);
}


void
TestCluster::TestDecodeEdgeCosine()
    {
        for (uint8_t i = 0; i < 26; ++i)
        {
            for (uint8_t b = 0; b < 8; ++b)
            {
                uint8_t e = (uint8_t) (i | (b<<5));
                float ec = rw::collision::ClusterUnitBase::DecodeEdgeCosineUnmasked(i); // Expects flags to be masked out already
                float expected = DecodeEdgeCosineReference(i);
                char str[256];
                sprintf(str, "DecodeEdgeCosine(%d=0x%x|%d) should be %f but was %f",
                    e, b<<5, i, expected, ec);
                EATESTAssert(fabs(ec - expected) < 0.00001f, str);
            }
        }
    }


void
TestCluster::TestDecodeEdgeCosines3()
    {
        for (uint8_t i = 0; i < 26; i += 3)
        {
            for (uint8_t b = 0; b < 8; ++b)
            {
                uint8_t e[3];
                uint8_t i0 = (uint8_t) ((i+0) & rw::collision::EDGEFLAG_ANGLEMASK);
                uint8_t i1 = (uint8_t) ((i+1) & rw::collision::EDGEFLAG_ANGLEMASK);
                uint8_t i2 = (uint8_t) ((i+2) & rw::collision::EDGEFLAG_ANGLEMASK);
                e[0] = (uint8_t) ((i0) | (b<<5));
                e[1] = (uint8_t) ((i1) | (b<<5));
                e[2] = (uint8_t) ((i2) | (b<<5));
                rwpmath::Vector3 ec = rw::collision::ClusterUnitBase::DecodeEdgeCosinesUnmasked(e[0], e[1], e[2]);
                float ecs[3];
                ecs[0] = ec.GetX();
                ecs[1] = ec.GetY();
                ecs[2] = ec.GetZ();
                float expected[3];
                expected[0] = DecodeEdgeCosineReference(i0);
                expected[1] = DecodeEdgeCosineReference(i1);
                expected[2] = DecodeEdgeCosineReference(i2);
                for (uint8_t n = 0; n < 3; ++n)
                {
                    char str[256];
                    sprintf(str, "DecodeEdgeCosines(%d=0x%x|%d)[%d] should be %f but was %f",
                        e[n], b<<5, i+n, n, expected[n], ecs[n]);
                    EATESTAssert(fabs(ecs[n] - expected[n]) < 0.00001f, str);
                }
            }
        }
    }


void
TestCluster::TestDecodeEdgeCosines4()
    {
        for (uint8_t i = 0; i < 26; i += 3)
        {
            for (uint8_t b = 0; b < 8; ++b)
            {
                uint8_t e[4];
                uint8_t i0 = (uint8_t) ((i+0) & rw::collision::EDGEFLAG_ANGLEMASK);
                uint8_t i1 = (uint8_t) ((i+1) & rw::collision::EDGEFLAG_ANGLEMASK);
                uint8_t i2 = (uint8_t) ((i+2) & rw::collision::EDGEFLAG_ANGLEMASK);
                uint8_t i3 = (uint8_t) ((i+3) & rw::collision::EDGEFLAG_ANGLEMASK);
                e[0] = (uint8_t) ((i0) | (b<<5));
                e[1] = (uint8_t) ((i1) | (b<<5));
                e[2] = (uint8_t) ((i2) | (b<<5));
                e[3] = (uint8_t) ((i3) | (b<<5));
                rwpmath::Vector4 ec = rw::collision::ClusterUnitBase::DecodeEdgeCosinesUnmasked(e[0], e[1], e[2], e[3]);
                float ecs[4];
                ecs[0] = ec.GetX();
                ecs[1] = ec.GetY();
                ecs[2] = ec.GetZ();
                ecs[3] = ec.GetW();
                float expected[4];
                expected[0] = DecodeEdgeCosineReference(i0);
                expected[1] = DecodeEdgeCosineReference(i1);
                expected[2] = DecodeEdgeCosineReference(i2);
                expected[3] = DecodeEdgeCosineReference(i3);
                for (uint8_t n = 0; n < 4; ++n)
                {
                    char str[256];
                    sprintf(str, "DecodeEdgeCosines(%d=0x%x|%d)[%d] should be %f but was %f",
                        e[n], b<<5, i+n, n, expected[n], ecs[n]);
                    EATESTAssert(fabs(ecs[n] - expected[n]) < 0.00001f, str);
                }
            }
        }
    }


void
TestCluster::TestExtractTriEdgeData()
    {
        const float tolerance = 0.00001f;
        for (uint32_t j = 0; j < 8*8*8*8; ++j)
        {
            uint8_t i = (uint8_t) (j & rw::collision::EDGEFLAG_ANGLEMASK);
            if (i+2 > 26) i = 26-2;
            uint8_t ef0 = EdgeFlags((j>>3) & 0x7);
            uint8_t ef1 = EdgeFlags((j>>6) & 0x7);
            uint8_t ef2 = EdgeFlags((j>>9) & 0x7);
            uint32_t tf = TriFlags(j & 0x7);
            uint8_t edgeData[3];
            edgeData[0] = (uint8_t) ((i+0) | ef0);
            edgeData[1] = (uint8_t) ((i+1) | ef1);
            edgeData[2] = (uint8_t) ((i+2) | ef2);

            rwpmath::Vector3 edgeCosines = -10.f*rwpmath::GetVector3_One();
            uint32_t flagsU = rw::collision::ClusterUnitBase::ExtractTriEdgeData(edgeCosines, &edgeData[0], (uint16_t) (tf));  // Forward to fn to test

            uint32_t f = flagsU;
            CheckEdgeFlag(0, f, edgeData[0], "Tri0 edge0");
            CheckEdgeFlag(1, f, edgeData[1], "Tri0 edge1");
            CheckEdgeFlag(2, f, edgeData[2], "Tri0 edge2");
            CheckVertexFlag(0, f, edgeData[0], "Tri0 vertex0");
            CheckVertexFlag(1, f, edgeData[1], "Tri0 vertex1");
            CheckVertexFlag(2, f, edgeData[2], "Tri0 vertex2");
            CheckTriFlag(f, tf, "Tri0");

            float expected[3];
            expected[0] = DecodeEdgeCosineReference((uint8_t) (i+0));
            expected[1] = DecodeEdgeCosineReference((uint8_t) (i+1));
            expected[2] = DecodeEdgeCosineReference((uint8_t) (i+2));
            EATESTAssert(fabs(expected[0] - (float) edgeCosines.GetX()) < tolerance, "EdgeCosine0 should be as expected");
            EATESTAssert(fabs(expected[1] - (float) edgeCosines.GetY()) < tolerance, "EdgeCosine1 should be as expected");
            EATESTAssert(fabs(expected[2] - (float) edgeCosines.GetZ()) < tolerance, "EdgeCosine2 should be as expected");

            rwpmath::MaskScalar oneSidedFlag(false);
            rwpmath::Mask3 edgeIsConvex(false, true, false);
            rwpmath::Mask3 disableVertices(true, true, false);

            rw::collision::ClusterUnitBase::ComputeTriangleMasks(edgeIsConvex, disableVertices, oneSidedFlag, flagsU);

            CheckEdgeFlag(edgeIsConvex.GetX(), edgeData[0], "Extracted edge0");
            CheckEdgeFlag(edgeIsConvex.GetY(), edgeData[1], "Extracted edge1");
            CheckEdgeFlag(edgeIsConvex.GetZ(), edgeData[2], "Extracted edge2");
            CheckVertexFlag(disableVertices.GetX(), edgeData[0], "Extracted vertex0");
            CheckVertexFlag(disableVertices.GetY(), edgeData[1], "Extracted vertex1");
            CheckVertexFlag(disableVertices.GetZ(), edgeData[2], "Extracted vertex2");
            CheckTriFlag(oneSidedFlag, tf & 0xff, "Extracted oneSided");
        }
    }


void
TestCluster::TestExtractQuadEdgeData()
    {
        uint32_t uninitialized = 0x12345678u;
        const float tolerance = 0.00001f;
        for (uint32_t j = 0; j < 8*8*8*8*8; ++j)
        {
            uint8_t i = (uint8_t) (j & rw::collision::EDGEFLAG_ANGLEMASK);
            if (i+3 > 26) i = 26-3;
            uint8_t ef0 = EdgeFlags((j>>3) & 0x7);
            uint8_t ef1 = EdgeFlags((j>>6) & 0x7);
            uint8_t ef2 = EdgeFlags((j>>9) & 0x7);
            uint8_t ef3 = EdgeFlags((j>>12) & 0x7);
            uint32_t tf = TriFlags(j & 0x7);
            uint8_t edgeData[4];
            edgeData[0] = (uint8_t) ((i+0) | ef0);
            edgeData[1] = (uint8_t) ((i+1) | ef1);
            edgeData[2] = (uint8_t) ((i+2) | ef2);
            edgeData[3] = (uint8_t) ((i+3) | ef3);
            rwpmath::Vector3 v0(0.0f, 0.0f, 0.0f);
            rwpmath::Vector3 v1(1.0f, 0.0f, 0.0f);
            rwpmath::Vector3 v2(1.0f, 1.0f, 0.0f);
            rwpmath::Vector3 v3(0.0f, 1.0f, 0.1f);
            uint32_t centralFlag = (uint32_t) (((tf >> 31) == 0x1)<<5);
            if (centralFlag != 0)
            {
                v3.SetZ(-0.1f);
            }
            rwpmath::MaskScalar centralEdgeIsConvex;
            rwpmath::VecFloat centralEdgeCosine = rw::collision::ClusterUnitBase::ComputeCentralEdgeCosine(centralEdgeIsConvex, v0, v1, v2, v3);
            EATESTAssert(centralEdgeIsConvex.GetBool() == (centralFlag != 0), "Central edge is convex flag should be set correctly");

            rwpmath::Vector3 edgeCosinesA = -10.f*rwpmath::GetVector3_One();
            rwpmath::Vector3 edgeCosinesB = -10.f*rwpmath::GetVector3_One();
            uint32_t flagsAU = uninitialized;
            uint32_t flagsBU = uninitialized;
            rw::collision::ClusterUnitBase::ExtractQuadEdgeData(edgeCosinesA, flagsAU, edgeCosinesB, flagsBU, 
                v0, v1, v2, v3, &edgeData[0], (uint8_t) tf);
            {
                uint32_t fA = flagsAU;
                CheckEdgeFlag(0, fA, edgeData[0], "Tri0 edge0");
                CheckEdgeFlag(1, fA, centralFlag, "Tri0 edge1");
                CheckEdgeFlag(2, fA, edgeData[2], "Tri0 edge2");
                CheckVertexFlag(0, fA, edgeData[0], "Tri0 vertex0");
                CheckVertexFlag(1, fA, edgeData[1], "Tri0 vertex1");
                CheckVertexFlag(2, fA, edgeData[2], "Tri0 vertex2");
                CheckTriFlag(fA, tf, "Tri0");
            }

            {
                uint32_t fB = flagsBU;
                CheckEdgeFlag(0, fB, edgeData[3], "Tri1 edge0");
                CheckEdgeFlag(1, fB, centralFlag, "Tri1 edge1");
                CheckEdgeFlag(2, fB, edgeData[1], "Tri1 edge2");
                CheckVertexFlag(0, fB, edgeData[3], "Tri1 vertex0");
                CheckVertexFlag(1, fB, edgeData[2], "Tri1 vertex1");
                CheckVertexFlag(2, fB, edgeData[1], "Tri1 vertex2");
                CheckTriFlag(fB, tf, "Tri1");
            }

            {
                float expected[5];
                expected[0] = DecodeEdgeCosineReference((uint8_t) (i+0));
                expected[1] = DecodeEdgeCosineReference((uint8_t) (i+1));
                expected[2] = DecodeEdgeCosineReference((uint8_t) (i+2));
                expected[3] = DecodeEdgeCosineReference((uint8_t) (i+3));
                expected[4] = (float) centralEdgeCosine;
                EATESTAssert(fabs(expected[0] - (float) edgeCosinesA.GetX()) < tolerance, "EdgeCosineA0 should be as expected");
                EATESTAssert(fabs(expected[4] - (float) edgeCosinesA.GetY()) < tolerance, "EdgeCosineA1 should be as expected");
                EATESTAssert(fabs(expected[2] - (float) edgeCosinesA.GetZ()) < tolerance, "EdgeCosineA2 should be as expected");
                EATESTAssert(fabs(expected[3] - (float) edgeCosinesB.GetX()) < tolerance, "EdgeCosineB0 should be as expected");
                EATESTAssert(fabs(expected[4] - (float) edgeCosinesB.GetY()) < tolerance, "EdgeCosineB1 should be as expected");
                EATESTAssert(fabs(expected[1] - (float) edgeCosinesB.GetZ()) < tolerance, "EdgeCosineB2 should be as expected");
            }

            {
                rwpmath::MaskScalar oneSidedFlag(false);
                rwpmath::Mask3 edgeIsConvex(false, true, false);
                rwpmath::Mask3 disableVertices(true, true, false);

                rw::collision::ClusterUnitBase::ComputeTriangleMasks(edgeIsConvex, disableVertices, oneSidedFlag, flagsAU);

                CheckEdgeFlag(edgeIsConvex.GetX(), edgeData[0], "Extracted tri0 edge0");
                CheckEdgeFlag(edgeIsConvex.GetY(), centralFlag, "Extracted tri0 edge1");
                CheckEdgeFlag(edgeIsConvex.GetZ(), edgeData[2], "Extracted tri0 edge2");
                CheckVertexFlag(disableVertices.GetX(), edgeData[0], "Extracted tri0 vertex0");
                CheckVertexFlag(disableVertices.GetY(), edgeData[1], "Extracted tri0 vertex1");
                CheckVertexFlag(disableVertices.GetZ(), edgeData[2], "Extracted tri0 vertex2");
                CheckTriFlag(oneSidedFlag, tf, "Extracted tri0 oneSided");
            }

            {
                rwpmath::MaskScalar oneSidedFlag(false);
                rwpmath::Mask3 edgeIsConvex(false, true, false);
                rwpmath::Mask3 disableVertices(true, true, false);

                rw::collision::ClusterUnitBase::ComputeTriangleMasks(edgeIsConvex, disableVertices, oneSidedFlag, flagsBU);

                CheckEdgeFlag(edgeIsConvex.GetX(), edgeData[3], "Extracted tri1 edge0");
                CheckEdgeFlag(edgeIsConvex.GetY(), centralFlag, "Extracted tri1 edge1");
                CheckEdgeFlag(edgeIsConvex.GetZ(), edgeData[1], "Extracted tri1 edge2");
                CheckVertexFlag(disableVertices.GetX(), edgeData[3], "Extracted tri1 vertex0");
                CheckVertexFlag(disableVertices.GetY(), edgeData[2], "Extracted tri1 vertex1");
                CheckVertexFlag(disableVertices.GetZ(), edgeData[1], "Extracted tri1 vertex2");
                CheckTriFlag(oneSidedFlag, tf, "Extracted tri1 oneSided");
            }
        }
    }


void
TestCluster::TestGetSizeZero()
{
    rw::collision::ClusterConstructionParameters parameters;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t totalExpectedSize = sizeOfCluster;

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize == size, str);
}

void
TestCluster::TestGetSizeVerticesOnly()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 10;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfVertices = static_cast<uint16_t>(16u * parameters.mVertexCount);
    uint16_t totalExpectedSize =static_cast<uint16_t>( sizeOfCluster + sizeOfVertices - sizeof(rwpmath::Vector3));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeTriangleUnitsOnly()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mTriangleUnitCount = 10;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfTriangleUnits = static_cast<uint16_t>(mSizeOfTriangleUnit * parameters.mTriangleUnitCount);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + sizeOfTriangleUnits - sizeof(rwpmath::Vector3));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeQuadUnitsOnly()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mQuadUnitCount = 10;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfTriangleUnits = static_cast<uint16_t>(mSizeOfQuadUnit * parameters.mQuadUnitCount);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + sizeOfTriangleUnits - sizeof(rwpmath::Vector3));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeUnitsOnly()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mTriangleUnitCount = 5;
    parameters.mQuadUnitCount = 5;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfTriangleUnits = static_cast<uint16_t>(mSizeOfTriangleUnit * parameters.mTriangleUnitCount);
    uint16_t sizeOfQuadUnits = static_cast<uint16_t>(mSizeOfQuadUnit * parameters.mQuadUnitCount);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + sizeOfTriangleUnits + sizeOfQuadUnits - sizeof(rwpmath::Vector3));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeEdgeCosineOnly()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mEdgeCosineCount = 10;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfEdgeCosines = static_cast<uint16_t>(parameters.mEdgeCosineCount);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + ( sizeOfEdgeCosines > sizeof(rwpmath::Vector3) ? (sizeOfEdgeCosines - sizeof(rwpmath::Vector3)) : 0));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeGroupIDSizeOne()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mGroupIDCount = 10;
    parameters.mGroupIDSize = 1;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfGroupID = static_cast<uint16_t>(parameters.mGroupIDCount * parameters.mGroupIDSize);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + ( sizeOfGroupID > sizeof(rwpmath::Vector3) ? (sizeOfGroupID - sizeof(rwpmath::Vector3)) : 0));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeGroupIDSizeTwo()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mGroupIDCount = 10;
    parameters.mGroupIDSize = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfGroupID = static_cast<uint16_t>(parameters.mGroupIDCount * parameters.mGroupIDSize);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + ( sizeOfGroupID > sizeof(rwpmath::Vector3) ? (sizeOfGroupID - sizeof(rwpmath::Vector3)) : 0));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeSurfaceIDSizeOne()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mSurfaceIDCount = 10;
    parameters.mSurfaceIDSize = 1;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfSurfaceID = static_cast<uint16_t>(parameters.mSurfaceIDCount * parameters.mSurfaceIDSize);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + ( sizeOfSurfaceID > sizeof(rwpmath::Vector3) ? (sizeOfSurfaceID - sizeof(rwpmath::Vector3)) : 0));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeSurfaceIDSizeTwo()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mSurfaceIDCount = 10;
    parameters.mSurfaceIDSize = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfSurfaceID = static_cast<uint16_t>(parameters.mSurfaceIDCount * parameters.mSurfaceIDSize);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + ( sizeOfSurfaceID > sizeof(rwpmath::Vector3) ? (sizeOfSurfaceID - sizeof(rwpmath::Vector3)) : 0));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSizeFull()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 10;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 10;
    parameters.mQuadUnitCount = 10;
    parameters.mEdgeCosineCount = 10;
    parameters.mGroupIDCount = 10;
    parameters.mGroupIDSize = 2;
    parameters.mSurfaceIDCount = 10;
    parameters.mSurfaceIDSize = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfUnitData = static_cast<uint16_t>(parameters.mTriangleUnitCount * 4 +
                                                    parameters.mQuadUnitCount * 5 +
                                                    parameters.mEdgeCosineCount +
                                                    parameters.mSurfaceIDCount * parameters.mSurfaceIDSize +
                                                    parameters.mGroupIDCount * parameters.mGroupIDSize);
    uint16_t sizeOfVertexData = static_cast<uint16_t>((parameters.mVertexCount) * 16u);
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + sizeOfUnitData + sizeOfVertexData - sizeof(rwpmath::Vector3));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSize16BitVertexCompression()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 10;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfVertexData = static_cast<uint16_t>((3u * 4u) + sizeof(rw::collision::ClusteredMeshCluster::Vertex16) * parameters.mVertexCount);
    sizeOfVertexData = static_cast<uint16_t>(EA::Physics::SizeAlign<uint16_t>( sizeOfVertexData, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT ));
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + sizeOfVertexData - sizeof(rwpmath::Vector3));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

void
TestCluster::TestGetSize32BitVertexCompression()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 10;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    uint16_t sizeOfCluster = sizeof(rw::collision::ClusteredMeshCluster);
    uint16_t sizeOfVertexData = static_cast<uint16_t>(sizeof(rw::collision::ClusteredMeshCluster::Vertex32) * parameters.mVertexCount);
    sizeOfVertexData = static_cast<uint16_t>(EA::Physics::SizeAlign<uint16_t>( sizeOfVertexData, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT ));
    uint16_t totalExpectedSize = static_cast<uint16_t>(sizeOfCluster + sizeOfVertexData - sizeof(rwpmath::Vector3));

    char str[128];
    sprintf(str, "Size should be %u", totalExpectedSize);
    EATESTAssert(totalExpectedSize  == size, str);
}

#if !defined(EA_PLATFORM_PS3_SPU)
void
TestCluster::TestInitializeEmpty()
{
    rw::collision::ClusterConstructionParameters parameters;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);
    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        CheckValue<uint16_t>(cluster->unitCount, 0u, "Cluster unit count");
        CheckValue<uint16_t>(cluster->unitDataSize, 0u, "Cluster unit data size");
        CheckValue<uint16_t>(cluster->unitDataStart, 0u, "Cluster unit data start");
        CheckValue<uint16_t>(cluster->normalStart, 0u, "Cluster normal start");
        CheckValue<uint16_t>(cluster->totalSize, size, "Cluster size");
        CheckValue<uint16_t>(cluster->vertexCount, 0u, "Cluster vertex count");
        CheckValue<uint16_t>(cluster->normalCount, 0u, "Cluster normal count");
        CheckValue<uint16_t>(cluster->compressionMode, rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, "Cluster compression mode");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}

void
TestCluster::TestInitialize16BitCompressed()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 10;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED;
    parameters.mTriangleUnitCount = 10;
    parameters.mQuadUnitCount = 10;
    parameters.mEdgeCosineCount = 10;
    parameters.mGroupIDCount = 10;
    parameters.mGroupIDSize = 2;
    parameters.mSurfaceIDCount = 10;
    parameters.mSurfaceIDSize = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        uint16_t unitDataStart = static_cast<uint16_t>(3u * sizeof(int32_t) + sizeof(rw::collision::ClusteredMeshCluster::Vertex16) * parameters.mVertexCount);
        unitDataStart = static_cast<uint16_t>( EA::Physics::SizeAlign<uint16_t>( unitDataStart, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT ));
        unitDataStart = static_cast<uint16_t>( unitDataStart / rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );

        CheckValue<uint16_t>(cluster->unitCount, 0, "Cluster unit count");
        CheckValue<uint16_t>(cluster->unitDataSize, 0, "Cluster unit data size");
        CheckValue<uint16_t>(cluster->unitDataStart, unitDataStart, "Cluster unit data start");
        CheckValue<uint16_t>(cluster->normalStart, unitDataStart, "Cluster normal start");
        CheckValue<uint16_t>(cluster->totalSize, size, "Cluster size");
        CheckValue<uint16_t>(cluster->vertexCount, 0, "Cluster vertex count");
        CheckValue<uint16_t>(cluster->normalCount, 0, "Cluster normal count");
        CheckValue<uint16_t>(cluster->compressionMode, rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED, "Cluster compression mode");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}

void
TestCluster::TestInitialize32BitCompressed()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 10;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED;
    parameters.mTriangleUnitCount = 10;
    parameters.mQuadUnitCount = 10;
    parameters.mEdgeCosineCount = 10;
    parameters.mGroupIDCount = 10;
    parameters.mGroupIDSize = 2;
    parameters.mSurfaceIDCount = 10;
    parameters.mSurfaceIDSize = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        uint16_t unitDataStart = static_cast<uint16_t>(sizeof(rw::collision::ClusteredMeshCluster::Vertex32) * parameters.mVertexCount);
        unitDataStart = static_cast<uint16_t>(EA::Physics::SizeAlign<uint16_t>( unitDataStart, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT ));
        unitDataStart = static_cast<uint16_t>( unitDataStart / rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );

        CheckValue<uint16_t>(cluster->unitCount, 0, "Cluster unit count");
        CheckValue<uint16_t>(cluster->unitDataSize, 0, "Cluster unit data size");
        CheckValue<uint16_t>(cluster->unitDataStart, unitDataStart, "Cluster unit data start");
        CheckValue<uint16_t>(cluster->normalStart, unitDataStart, "Cluster normal start");
        CheckValue<uint16_t>(cluster->totalSize, size, "Cluster size");
        CheckValue<uint16_t>(cluster->vertexCount, 0, "Cluster vertex count");
        CheckValue<uint16_t>(cluster->normalCount, 0, "Cluster normal count");
        CheckValue<uint16_t>(cluster->compressionMode, rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED, "Cluster compression mode");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}

void
TestCluster::TestInitializeUncompressed()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 10;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 10;
    parameters.mQuadUnitCount = 10;
    parameters.mEdgeCosineCount = 10;
    parameters.mGroupIDCount = 10;
    parameters.mGroupIDSize = 2;
    parameters.mSurfaceIDCount = 10;
    parameters.mSurfaceIDSize = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        CheckValue<uint16_t>(cluster->unitCount, 0, "Cluster unit count");
        CheckValue<uint16_t>(cluster->unitDataSize, 0, "Cluster unit data size");
        CheckValue<uint16_t>(cluster->unitDataStart, parameters.mVertexCount, "Cluster unit data start");
        CheckValue<uint16_t>(cluster->normalStart, parameters.mVertexCount, "Cluster normal start");
        CheckValue<uint16_t>(cluster->totalSize, size, "Cluster size");
        CheckValue<uint16_t>(cluster->vertexCount, 0, "Cluster vertex count");
        CheckValue<uint16_t>(cluster->normalCount, 0, "Cluster normal count");
        CheckValue<uint16_t>(cluster->compressionMode, rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED, "Cluster compression mode");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}

#endif // !defined(EA_PLATFORM_PS3_SPU)
void
TestCluster::TestGetUnitSizeSimpleTriangle()
{
    const uint8_t unitType = rw::collision::UNITTYPE_TRIANGLE;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = 0;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 0;

    const uint32_t groupID = 0;

    const uint32_t surfaceID = 0;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
                        unitType,
                        unitParameters,
                        groupID,
                        surfaceID);

    CheckValue(size, uint32_t(4), "Unit size");
}

void
TestCluster::TestGetUnitSizeSimpleQuad()
{
    const uint8_t unitType = rw::collision::UNITTYPE_QUAD;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = 0;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 0;

    const uint32_t groupID = 0;

    const uint32_t surfaceID = 0;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(5u), "Unit size");
}

void
TestCluster::TestGetUnitSizeTriangleEdgeCos()
{
    const uint8_t unitType = rw::collision::UNITTYPE_TRIANGLE;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_EDGEANGLE;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 0;

    const uint32_t groupID = 0;

    const uint32_t surfaceID = 0;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(7u), "Unit size");
}

void
TestCluster::TestGetUnitSizeQuadEdgeCos()
{
    const uint8_t unitType = rw::collision::UNITTYPE_QUAD;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_EDGEANGLE;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 0;

    const uint32_t groupID = 0;

    const uint32_t surfaceID = 0;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(9u), "Unit size");
}

void
TestCluster::TestGetUnitSizeTriangleGroupID()
{
    const uint8_t unitType = rw::collision::UNITTYPE_TRIANGLE;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_GROUPID;
    unitParameters.groupIDSize = 1;
    unitParameters.surfaceIDSize = 0;

    const uint32_t groupID = 1;

    const uint32_t surfaceID = 0;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(5u), "Unit size");
}

void
TestCluster::TestGetUnitSizeTriangleSurfaceID()
{
    const uint8_t unitType = rw::collision::UNITTYPE_TRIANGLE;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_SURFACEID;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 1;

    const uint32_t groupID = 0;

    const uint32_t surfaceID = 1;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(5u), "Unit size");
}

void
TestCluster::TestGetUnitSizeTriangleBothIDs()
    {
    const uint8_t unitType = rw::collision::UNITTYPE_TRIANGLE;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_GROUPID | rw::collision::UNITFLAG_SURFACEID;
    unitParameters.groupIDSize = 1;
    unitParameters.surfaceIDSize = 1;

    const uint32_t groupID = 1;

    const uint32_t surfaceID = 1;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(6u), "Unit size");
}

void
TestCluster::TestGetUnitSizeQuadGroupID()
    {
    const uint8_t unitType = rw::collision::UNITTYPE_QUAD;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_GROUPID;
    unitParameters.groupIDSize = 2;
    unitParameters.surfaceIDSize = 0;

    const uint32_t groupID = 1;

    const uint32_t surfaceID = 0;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(7u), "Unit size");
    }

void
TestCluster::TestGetUnitSizeQuadSurfaceID()
    {
    const uint8_t unitType = rw::collision::UNITTYPE_QUAD;

    UnitParameters unitParameters;
    unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_SURFACEID;
    unitParameters.groupIDSize = 0;
    unitParameters.surfaceIDSize = 2;

    const uint32_t groupID = 0;

    const uint32_t surfaceID = 1;

    uint32_t size = rw::collision::ClusteredMeshCluster::GetUnitSize(
        unitType,
                        unitParameters,
        groupID,
        surfaceID);

    CheckValue(size, uint32_t(7u), "Unit size");
    }
#if !defined(EA_PLATFORM_PS3_SPU)
void
TestCluster::TestSetVertexOffset16BitCompression()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 1;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED;
    parameters.mTriangleUnitCount = 0;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        rw::collision::ClusteredMeshCluster::Vertex32 expectedOffset;
        expectedOffset.x = (uint32_t)1.0f;
        expectedOffset.y = (uint32_t)4.0f;
        expectedOffset.z = (uint32_t)9.0f;

        cluster->SetVertexOffset(expectedOffset);

        rw::collision::ClusteredMeshCluster::CompressedVertexDataUnion vdUnion;
        vdUnion.m_as_rwpmathVector3Ptr = cluster->vertexArray;
        const rw::collision::ClusteredMeshCluster::Vertex32 *actualOffset = vdUnion.m_asVertex32Ptr;

        CheckValue<const rw::collision::ClusteredMeshCluster::Vertex32&>(*actualOffset, expectedOffset, "Cluster vertex offset");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
    }


void
TestCluster::TestSetVertexOffset32BitCompression()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 1;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED;
    parameters.mTriangleUnitCount = 0;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        rw::collision::ClusteredMeshCluster::Vertex32 invalidOffset;
        invalidOffset.x = (uint32_t)1.0f;
        invalidOffset.y = (uint32_t)4.0f;
        invalidOffset.z = (uint32_t)9.0f;

        rw::collision::ClusteredMeshCluster::Vertex32 offset;
        offset.x = (uint32_t)0.0f;
        offset.y = (uint32_t)0.0f;
        offset.z = (uint32_t)0.0f;

        rw::collision::ClusteredMeshCluster::CompressedVertexDataUnion vdUnion;
        vdUnion.m_as_rwpmathVector3Ptr = cluster->vertexArray;
        rw::collision::ClusteredMeshCluster::Vertex32 *actualOffset = const_cast<rw::collision::ClusteredMeshCluster::Vertex32 *>(vdUnion.m_asVertex32Ptr);
        *actualOffset = offset;

        cluster->SetVertexOffset(invalidOffset);

        // Cluster offset should not have changed
        CheckValue<const rw::collision::ClusteredMeshCluster::Vertex32&>(*actualOffset, offset, "Cluster vertex offset");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
    }

void
TestCluster::TestSetVertexOffsetNoCompression()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 1;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 0;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        rw::collision::ClusteredMeshCluster::Vertex32 invalidOffset;
        invalidOffset.x = (uint32_t)1.0f;
        invalidOffset.y = (uint32_t)4.0f;
        invalidOffset.z = (uint32_t)9.0f;

        rw::collision::ClusteredMeshCluster::Vertex32 offset;
        offset.x = (uint32_t)0.0f;
        offset.y = (uint32_t)0.0f;
        offset.z = (uint32_t)0.0f;

        rw::collision::ClusteredMeshCluster::CompressedVertexDataUnion vdUnion;
        vdUnion.m_as_rwpmathVector3Ptr = cluster->vertexArray;
        rw::collision::ClusteredMeshCluster::Vertex32 *actualOffset = const_cast<rw::collision::ClusteredMeshCluster::Vertex32 *>(vdUnion.m_asVertex32Ptr);
        *actualOffset = offset;

        cluster->SetVertexOffset(invalidOffset);

        // Cluster offset should not have changed
        CheckValue<const rw::collision::ClusteredMeshCluster::Vertex32&>(*actualOffset, offset, "Cluster vertex offset");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
    }

void
TestCluster::TestSetVertex16BitCompression()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 1;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED;
    parameters.mTriangleUnitCount = 0;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        rw::collision::ClusteredMeshCluster::Vertex32 offset;
        offset.x = (uint32_t)0.0f;
        offset.y = (uint32_t)0.0f;
        offset.z = (uint32_t)0.0f;

        cluster->SetVertexOffset(offset);

        float compressionGranularity(1.0f);
        rwpmath::Vector3 expectedVertex(1.0f, 1.0f, 1.0f);

        cluster->SetVertex(expectedVertex, compressionGranularity);

        rwpmath::Vector3 actualVertex = cluster->GetVertex(0, compressionGranularity);

        CheckValue<rwpmath::Vector3::InParam>(actualVertex, expectedVertex, "Vertex 0");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
    }

void
TestCluster::TestSetVertex32BitCompression()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 1;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED;
    parameters.mTriangleUnitCount = 0;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        rw::collision::ClusteredMeshCluster::Vertex32 offset;
        offset.x = (uint32_t)0.0f;
        offset.y = (uint32_t)0.0f;
        offset.z = (uint32_t)0.0f;

        cluster->SetVertexOffset(offset);

        float compressionGranularity(1.0f);
        rwpmath::Vector3 expectedVertex(1.0f, 1.0f, 1.0f);

        cluster->SetVertex(expectedVertex, compressionGranularity);

        rwpmath::Vector3 actualVertex = cluster->GetVertex(0, compressionGranularity);

        CheckValue<rwpmath::Vector3::InParam>(actualVertex, expectedVertex, "Vertex 0");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
    }

void
TestCluster::TestSetVertexNoCompressionSingle()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 1;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 0;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        float compressionGranularity(1.0f);
        rwpmath::Vector3 expectedVertex(1.0f, 1.0f, 1.0f);

        cluster->SetVertex(expectedVertex, compressionGranularity);

        rwpmath::Vector3 actualVertex = cluster->GetVertex(0, compressionGranularity);

        CheckValue<rwpmath::Vector3::InParam>(actualVertex, expectedVertex, "Vertex 0");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
    }

void
TestCluster::TestSetVertexNoCompressionMultiple()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 100;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 0;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        float compressionGranularity(1.0f);

        for (uint32_t vertexIndex = 0 ; vertexIndex < parameters.mVertexCount ; ++vertexIndex)
        {
            rwpmath::Vector3 vertex(static_cast<float>(vertexIndex), -static_cast<float>(vertexIndex), static_cast<float>(vertexIndex * vertexIndex));
            cluster->SetVertex(vertex, compressionGranularity);
        }

        for (uint32_t vertexIndex = 0 ; vertexIndex < parameters.mVertexCount ; ++vertexIndex)
        {
            rwpmath::Vector3 expectedVertex(static_cast<float>(vertexIndex), -static_cast<float>(vertexIndex), static_cast<float>(vertexIndex * vertexIndex));
            rwpmath::Vector3 actualVertex = cluster->GetVertex(static_cast<uint8_t>(vertexIndex), compressionGranularity);
            char msg[128];
            sprintf(msg, "Vertex %u", vertexIndex);
            CheckValue<rwpmath::Vector3::InParam>(actualVertex, expectedVertex, msg);
        }
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
        }
    }

void
TestCluster::TestSetTriangle()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 3;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 1;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        UnitParameters unitParameters;
        unitParameters.unitFlagsDefault = 0;
        unitParameters.groupIDSize = 0;
        unitParameters.surfaceIDSize = 0;
        float vertexCompressionGranularity = 1.0f;

        rw::collision::ClusterParams clusterParameters;
        clusterParameters.mFlags = rw::collision::CMFLAG_ONESIDED;
        clusterParameters.mGroupIdSize = static_cast<uint8_t>(unitParameters.groupIDSize);
        clusterParameters.mSurfaceIdSize = static_cast<uint8_t>(unitParameters.surfaceIDSize);
        clusterParameters.mVertexCompressionGranularity = vertexCompressionGranularity;

        rwpmath::Vector3 v0(0.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v1(1.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v2(0.0f, 0.0f, 1.0f);

        cluster->SetVertex(v0, vertexCompressionGranularity);
        cluster->SetVertex(v1, vertexCompressionGranularity);
        cluster->SetVertex(v2, vertexCompressionGranularity);

        uint16_t groupID = 0;
        uint16_t surfaceID = 0;
        uint8_t v0Index = 0;
        uint8_t v1Index = 1;
        uint8_t v2Index = 2;
        uint8_t edgeCode0 = 0;
        uint8_t edgeCode1 = 0;
        uint8_t edgeCode2 = 0;

        cluster->SetTriangle(
            unitParameters,
            groupID,
            surfaceID,
            v0Index,
            v1Index,
            v2Index,
            edgeCode0,
            edgeCode1,
            edgeCode2);

        uint32_t unitOffset = 0;

        uint32_t expectedUnitSize = rw::collision::ClusteredMeshCluster::GetUnitSize(
                                        rw::collision::UNITTYPE_TRIANGLE,
                                        unitParameters,
                                        groupID,
                                        surfaceID);

        uint32_t expectedUnitType = rw::collision::UNITTYPE_TRIANGLE;
        uint8_t expectedV0 = 0;
        uint8_t expectedV1 = 1;
        uint8_t expectedV2 = 2;

        uint8_t * unitData = cluster->UnitData();
        uint32_t actualUnitSize = cluster->GetUnitSize(unitOffset, clusterParameters);
        uint32_t actualUnitType = cluster->GetUnitType(unitOffset);
        uint8_t actualV0 = unitData[1];
        uint8_t actualV1 = unitData[2];
        uint8_t actualV2 = unitData[3];

        CheckValue(actualUnitSize, expectedUnitSize, "Unit size");
        CheckValue(actualUnitType, expectedUnitType, "Unit type");
        CheckValue(actualV0, expectedV0, "Unit v0 index");
        CheckValue(actualV1, expectedV1, "Unit v1 index");
        CheckValue(actualV2, expectedV2, "Unit v2 index");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
    }

void
TestCluster::TestGetTriangleVolumeTriangle()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 4;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 2;
    parameters.mGroupIDSize = 1;
    parameters.mGroupIDCount = 2;
    parameters.mSurfaceIDSize = 2;
    parameters.mSurfaceIDCount = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        UnitParameters unitParameters;
        unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_GROUPID;
        unitParameters.groupIDSize = 1;
        unitParameters.surfaceIDSize = 2;
        float vertexCompressionGranularity = 1.0f;

        rw::collision::ClusterParams clusterParameters;
        clusterParameters.mFlags = rw::collision::CMFLAG_ONESIDED;
        clusterParameters.mGroupIdSize = static_cast<uint8_t>(unitParameters.groupIDSize);
        clusterParameters.mSurfaceIdSize = static_cast<uint8_t>(unitParameters.surfaceIDSize);
        clusterParameters.mVertexCompressionGranularity = vertexCompressionGranularity;

        rwpmath::Vector3 v0(0.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v1(0.0f, 0.0f, 1.0f);
        rwpmath::Vector3 v2(1.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v3(1.0f, 0.0f, 1.0f);

        cluster->SetVertex(v0, vertexCompressionGranularity);
        cluster->SetVertex(v1, vertexCompressionGranularity);
        cluster->SetVertex(v2, vertexCompressionGranularity);
        cluster->SetVertex(v3, vertexCompressionGranularity);

        uint16_t groupID = 123;
        uint16_t surfaceID = 2323;
        uint8_t v0Index = 0;
        uint8_t v1Index = 1;
        uint8_t v2Index = 2;
        uint8_t v3Index = 3;
        uint8_t edgeCode = 0;

        cluster->SetTriangle(
            unitParameters,
            groupID,
            surfaceID,
            v0Index,
            v1Index,
            v2Index,
            edgeCode,
            edgeCode,
            edgeCode);

        cluster->SetTriangle(
            unitParameters,
            groupID,
            surfaceID,
            v1Index,
            v3Index,
            v2Index,
            edgeCode,
            edgeCode,
            edgeCode);

        const uint32_t firstTriangleUnitOffset = 0;
        const uint32_t firstTriangleTriangleIndex = 0;
        const uint32_t secondTriangleUnitOffset = 7;
        const uint32_t secondTriangleTriangleIndex = 0;

        rw::collision::TriangleVolume * triangle = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>(*EA::Allocator::ICoreAllocator::GetDefaultAllocator()).New();

        if (NULL != triangle)
        {
            cluster->GetTriangleVolume(
                *triangle,
                firstTriangleUnitOffset,
                firstTriangleTriangleIndex,
                clusterParameters);

            rwpmath::Vector3 actualV0, actualV1, actualV2;
            triangle->GetPoints(actualV0, actualV1, actualV2);

            CheckValue<rwpmath::Vector3::InParam>(actualV0, v0, "Vertex 0");
            CheckValue<rwpmath::Vector3::InParam>(actualV1, v1, "Vertex 1");
            CheckValue<rwpmath::Vector3::InParam>(actualV2, v2, "Vertex 2");

            rwpmath::Vector3 expectedEdgeCosines(0.0f, 0.0f, 0.0f);

            rwpmath::Vector3 actualEdgeCosines;
            actualEdgeCosines = triangle->GetEdgeCosVector();

            CheckValue<rwpmath::Vector3::InParam>(actualEdgeCosines, expectedEdgeCosines, "Edge Cosines");

            CheckValue(triangle->GetGroup(), static_cast<uint32_t>(groupID), "GroupID");
            CheckValue(triangle->GetSurface(), static_cast<uint32_t>(surfaceID), "SurfaceID");

            cluster->GetTriangleVolume(
                *triangle,
                secondTriangleUnitOffset,
                secondTriangleTriangleIndex,
                clusterParameters);

            triangle->GetPoints(actualV0, actualV1, actualV2);

            CheckValue<rwpmath::Vector3::InParam>(actualV0, v1, "Vertex 0");
            CheckValue<rwpmath::Vector3::InParam>(actualV1, v3, "Vertex 1");
            CheckValue<rwpmath::Vector3::InParam>(actualV2, v2, "Vertex 2");

            expectedEdgeCosines = rwpmath::Vector3(0.0f, 0.0f, 0.0f);

            actualEdgeCosines = triangle->GetEdgeCosVector();

            CheckValue<rwpmath::Vector3::InParam>(actualEdgeCosines, expectedEdgeCosines, "Edge Cosines");

            CheckValue(triangle->GetGroup(), static_cast<uint32_t>(groupID), "GroupID");
            CheckValue(triangle->GetSurface(), static_cast<uint32_t>(surfaceID), "SurfaceID");

            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(triangle);
        }
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}


void
TestCluster::TestGetTriangleVolumeQuad()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 4;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mQuadUnitCount = 1;
    parameters.mGroupIDSize = 2;
    parameters.mGroupIDCount = 2;
    parameters.mSurfaceIDSize = 1;
    parameters.mSurfaceIDCount = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        UnitParameters unitParameters;
        unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_GROUPID;
        unitParameters.groupIDSize = 2;
        unitParameters.surfaceIDSize = 1;
        float vertexCompressionGranularity = 1.0f;

        rw::collision::ClusterParams clusterParameters;
        clusterParameters.mFlags = rw::collision::CMFLAG_ONESIDED;
        clusterParameters.mGroupIdSize = static_cast<uint8_t>(unitParameters.groupIDSize);
        clusterParameters.mSurfaceIdSize = static_cast<uint8_t>(unitParameters.surfaceIDSize);
        clusterParameters.mVertexCompressionGranularity = vertexCompressionGranularity;

        rwpmath::Vector3 v0(0.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v1(0.0f, 0.0f, 1.0f);
        rwpmath::Vector3 v2(1.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v3(1.0f, 0.0f, 1.0f);

        cluster->SetVertex(v0, vertexCompressionGranularity);
        cluster->SetVertex(v1, vertexCompressionGranularity);
        cluster->SetVertex(v2, vertexCompressionGranularity);
        cluster->SetVertex(v3, vertexCompressionGranularity);

        uint16_t groupID = 4321;
        uint16_t surfaceID = 123;
        uint8_t v0Index = 0;
        uint8_t v1Index = 1;
        uint8_t v2Index = 2;
        uint8_t v3Index = 3;
        uint8_t edgeCode = 0;

        cluster->SetQuad(
            unitParameters,
            groupID,
            surfaceID,
            v0Index,
            v1Index,
            v2Index,
            v3Index,
            edgeCode,
            edgeCode,
            edgeCode,
            edgeCode);

        const uint32_t firstTriangleUnitOffset = 0;
        const uint32_t firstTriangleTriangleIndex = 0;
        const uint32_t secondTriangleUnitOffset = 0;
        const uint32_t secondTriangleTriangleIndex = 1;

        rw::collision::TriangleVolume * triangle = EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>(*EA::Allocator::ICoreAllocator::GetDefaultAllocator()).New();

        if (NULL != triangle)
        {
            cluster->GetTriangleVolume(
                *triangle,
                firstTriangleUnitOffset,
                firstTriangleTriangleIndex,
                clusterParameters);

            rwpmath::Vector3 actualV0, actualV1, actualV2;
            triangle->GetPoints(actualV0, actualV1, actualV2);

            CheckValue<rwpmath::Vector3::InParam>(actualV0, v0, "Vertex 0");
            CheckValue<rwpmath::Vector3::InParam>(actualV1, v1, "Vertex 1");
            CheckValue<rwpmath::Vector3::InParam>(actualV2, v2, "Vertex 2");

            rwpmath::Vector3 expectedEdgeCosines(0.0f, 0.0f, 0.0f);

            rwpmath::Vector3 actualEdgeCosines;
            actualEdgeCosines = triangle->GetEdgeCosVector();

            CheckValue<rwpmath::Vector3::InParam>(actualEdgeCosines, expectedEdgeCosines, "Edge Cosines");

            CheckValue(triangle->GetGroup(), static_cast<uint32_t>(groupID), "GroupID");
            CheckValue(triangle->GetSurface(), static_cast<uint32_t>(surfaceID), "SurfaceID");

            cluster->GetTriangleVolume(
                *triangle,
                secondTriangleUnitOffset,
                secondTriangleTriangleIndex,
                clusterParameters);

            triangle->GetPoints(actualV0, actualV1, actualV2);

            CheckValue<rwpmath::Vector3::InParam>(actualV0, v3, "Vertex 0");
            CheckValue<rwpmath::Vector3::InParam>(actualV1, v2, "Vertex 1");
            CheckValue<rwpmath::Vector3::InParam>(actualV2, v1, "Vertex 2");

            expectedEdgeCosines = rwpmath::Vector3(0.0f, 0.0f, 0.0f);

            actualEdgeCosines = triangle->GetEdgeCosVector();

            CheckValue<rwpmath::Vector3::InParam>(actualEdgeCosines, expectedEdgeCosines, "Edge Cosines");

            CheckValue(triangle->GetGroup(), static_cast<uint32_t>(groupID), "GroupID");
            CheckValue(triangle->GetSurface(), static_cast<uint32_t>(surfaceID), "SurfaceID");

            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(triangle);
        }
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}

void
TestCluster::TestGetTriangleIndicesTriangle()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 4;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mTriangleUnitCount = 2;
    parameters.mGroupIDSize = 1;
    parameters.mGroupIDCount = 2;
    parameters.mSurfaceIDSize = 2;
    parameters.mSurfaceIDCount = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        UnitParameters unitParameters;
        unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_GROUPID;
        unitParameters.groupIDSize = 1;
        unitParameters.surfaceIDSize = 2;
        float vertexCompressionGranularity = 1.0f;

        rw::collision::ClusterParams clusterParameters;
        clusterParameters.mFlags = rw::collision::CMFLAG_ONESIDED;
        clusterParameters.mGroupIdSize = static_cast<uint8_t>(unitParameters.groupIDSize);
        clusterParameters.mSurfaceIdSize = static_cast<uint8_t>(unitParameters.surfaceIDSize);
        clusterParameters.mVertexCompressionGranularity = vertexCompressionGranularity;

        rwpmath::Vector3 v0(0.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v1(0.0f, 0.0f, 1.0f);
        rwpmath::Vector3 v2(1.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v3(1.0f, 0.0f, 1.0f);

        cluster->SetVertex(v0, vertexCompressionGranularity);
        cluster->SetVertex(v1, vertexCompressionGranularity);
        cluster->SetVertex(v2, vertexCompressionGranularity);
        cluster->SetVertex(v3, vertexCompressionGranularity);

        uint16_t groupID = 123;
        uint16_t surfaceID = 2323;
        uint8_t v0Index = 0;
        uint8_t v1Index = 1;
        uint8_t v2Index = 2;
        uint8_t v3Index = 3;
        uint8_t edgeCode = 0;

        cluster->SetTriangle(
            unitParameters,
            groupID,
            surfaceID,
            v0Index,
            v1Index,
            v2Index,
            edgeCode,
            edgeCode,
            edgeCode);

        cluster->SetTriangle(
            unitParameters,
            groupID,
            surfaceID,
            v1Index,
            v3Index,
            v2Index,
            edgeCode,
            edgeCode,
            edgeCode);

        const uint32_t firstTriangleUnitOffset = 0;
        const uint32_t firstTriangleTriangleIndex = 0;
        const uint32_t secondTriangleUnitOffset = 7;
        const uint32_t secondTriangleTriangleIndex = 0;

        uint8_t expectedV0 = 0u;
        uint8_t expectedV1 = 1u;
        uint8_t expectedV2 = 2u;

        uint8_t actualV0, actualV1, actualV2;

        cluster->GetTriangleVertexIndices(
            actualV0,
            actualV1,
            actualV2,
            firstTriangleUnitOffset,
            firstTriangleTriangleIndex,
            clusterParameters);

        CheckValue<uint8_t>(actualV0, expectedV0, "Vertex 0");
        CheckValue<uint8_t>(actualV1, expectedV1, "Vertex 1");
        CheckValue<uint8_t>(actualV2, expectedV2, "Vertex 2");

        uint8_t expectedV3 = 1u;
        uint8_t expectedV4 = 3u;
        uint8_t expectedV5 = 2u;

        uint8_t actualV3, actualV4, actualV5;

        cluster->GetTriangleVertexIndices(
            actualV3,
            actualV4,
            actualV5,
            secondTriangleUnitOffset,
            secondTriangleTriangleIndex,
            clusterParameters);

        CheckValue<uint8_t>(actualV3, expectedV3, "Vertex 3");
        CheckValue<uint8_t>(actualV4, expectedV4, "Vertex 4");
        CheckValue<uint8_t>(actualV5, expectedV5, "Vertex 5");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}

void
TestCluster::TestGetTriangleIndicesQuad()
{
    rw::collision::ClusterConstructionParameters parameters;

    parameters.mVertexCount = 4;
    parameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    parameters.mQuadUnitCount = 1;
    parameters.mGroupIDSize = 2;
    parameters.mGroupIDCount = 2;
    parameters.mSurfaceIDSize = 1;
    parameters.mSurfaceIDCount = 2;

    const uint16_t size = rw::collision::ClusteredMeshCluster::GetSize(parameters);

    void * buffer = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(size, NULL, 0, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    if (NULL != buffer)
    {
        rw::collision::ClusteredMeshCluster * cluster = rw::collision::ClusteredMeshCluster::Initialize(buffer, parameters);

        UnitParameters unitParameters;
        unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_SURFACEID | rw::collision::UNITFLAG_GROUPID;
        unitParameters.groupIDSize = 2;
        unitParameters.surfaceIDSize = 1;
        float vertexCompressionGranularity = 1.0f;

        rw::collision::ClusterParams clusterParameters;
        clusterParameters.mFlags = rw::collision::CMFLAG_ONESIDED;
        clusterParameters.mGroupIdSize = static_cast<uint8_t>(unitParameters.groupIDSize);
        clusterParameters.mSurfaceIdSize = static_cast<uint8_t>(unitParameters.surfaceIDSize);
        clusterParameters.mVertexCompressionGranularity = vertexCompressionGranularity;

        rwpmath::Vector3 v0(0.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v1(0.0f, 0.0f, 1.0f);
        rwpmath::Vector3 v2(1.0f, 0.0f, 0.0f);
        rwpmath::Vector3 v3(1.0f, 0.0f, 1.0f);

        cluster->SetVertex(v0, vertexCompressionGranularity);
        cluster->SetVertex(v1, vertexCompressionGranularity);
        cluster->SetVertex(v2, vertexCompressionGranularity);
        cluster->SetVertex(v3, vertexCompressionGranularity);

        uint16_t groupID = 4321;
        uint16_t surfaceID = 123;
        uint8_t v0Index = 0;
        uint8_t v1Index = 1;
        uint8_t v2Index = 2;
        uint8_t v3Index = 3;
        uint8_t edgeCode = 0;

        cluster->SetQuad(
            unitParameters,
            groupID,
            surfaceID,
            v0Index,
            v1Index,
            v2Index,
            v3Index,
            edgeCode,
            edgeCode,
            edgeCode,
            edgeCode);

        const uint32_t firstTriangleUnitOffset = 0;
        const uint32_t firstTriangleTriangleIndex = 0;
        const uint32_t secondTriangleUnitOffset = 0;
        const uint32_t secondTriangleTriangleIndex = 1;

        uint8_t expectedV0 = 0;
        uint8_t expectedV1 = 1;
        uint8_t expectedV2 = 2;

        uint8_t actualV0, actualV1, actualV2;

        cluster->GetTriangleVertexIndices(
            actualV0,
            actualV1,
            actualV2,
            firstTriangleUnitOffset,
            firstTriangleTriangleIndex,
            clusterParameters);

        CheckValue<uint8_t>(actualV0, expectedV0, "Vertex 0");
        CheckValue<uint8_t>(actualV1, expectedV1, "Vertex 1");
        CheckValue<uint8_t>(actualV2, expectedV2, "Vertex 2");

        uint8_t expectedV3 = 3;
        uint8_t expectedV4 = 2;
        uint8_t expectedV5 = 1;

        uint8_t actualV3, actualV4, actualV5;

        cluster->GetTriangleVertexIndices(
            actualV3,
            actualV4,
            actualV5,
            secondTriangleUnitOffset,
            secondTriangleTriangleIndex,
            clusterParameters);

        CheckValue<uint8_t>(actualV3, expectedV3, "Vertex 3");
        CheckValue<uint8_t>(actualV4, expectedV4, "Vertex 4");
        CheckValue<uint8_t>(actualV5, expectedV5, "Vertex 5");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer);
    }
}
#endif // !defined(EA_PLATFORM_PS3_SPU)

EA_ALIGNED(uint8_t, TestCluster::sResultsBuffer[TestCluster::RESULTS_SIZE], 16);


}   // namespace Tests
}   // namespace collision
}   // namespace EA
