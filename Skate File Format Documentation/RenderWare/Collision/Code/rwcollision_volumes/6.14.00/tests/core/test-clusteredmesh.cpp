// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/clusteredmesh.h>

#if !defined(EA_PLATFORM_PS3_SPU)
#include <rw/collision/clusteredmeshofflinebuilder.h>
#endif // !defined(EA_PLATFORM_PS3_SPU)

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rwpmath;
using namespace rw::collision;

// Unit tests for serializing ClusteredMeshes from archives.
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.
// The serialization tests do not check the values inside the clustered meshes other than
// relying on the asserted IsValid method called after serialization.

class TestClusteredMesh: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestClusteredMesh");

#define CLUSTERED_MESH_TEST(F, D) EATEST_REGISTER(#F, D, TestClusteredMesh, F)

        CLUSTERED_MESH_TEST(TestUncompressedClusteredMeshVersion1HLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh (class serialization version 1)");
        CLUSTERED_MESH_TEST(TestUncompressedClusteredMeshVersion2HLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh (class serialization version 2)");
        CLUSTERED_MESH_TEST(TestUncompressedClusteredMeshHLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh");

#if !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(TestUncompressedClusteredMeshLLVpuFileSerializationLoadOnly,
            "Low-level file serialization (loading only) of ClusteredMesh");
#endif // !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(TestUncompressedClusteredMeshLLFpuFileSerializationLoadOnly,
            "Low-level fpu-layout file Serialization (loading only) of ClusteredMesh");

        CLUSTERED_MESH_TEST(Test16BitCompressedClusteredMeshVersion1HLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh (class serialization version 1)");
        CLUSTERED_MESH_TEST(Test16BitCompressedClusteredMeshVersion2HLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh (class serialization version 2)");
        CLUSTERED_MESH_TEST(Test16BitCompressedClusteredMeshHLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh");
#if !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(Test16BitCompressedClusteredMeshLLVpuFileSerializationLoadOnly,
            "Low-level file serialization (loading only) of ClusteredMesh");
#endif // !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(Test16BitCompressedClusteredMeshLLFpuFileSerializationLoadOnly,
            "Low-level fpu-layout file Serialization (loading only) of ClusteredMesh");

        CLUSTERED_MESH_TEST(Test32BitCompressedClusteredMeshVersion1HLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh (class serialization version 1)");
        CLUSTERED_MESH_TEST(Test32BitCompressedClusteredMeshVersion2HLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh (class serialization version 2)");
        CLUSTERED_MESH_TEST(Test32BitCompressedClusteredMeshHLFileSerializationLoadOnly,
            "High-level file serialization (loading only) of ClusteredMesh");
#if !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(Test32BitCompressedClusteredMeshLLVpuFileSerializationLoadOnly,
            "Low-level file serialization (loading only) of ClusteredMesh");
#endif // !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(Test32BitCompressedClusteredMeshLLFpuFileSerializationLoadOnly,
            "Low-level fpu-layout file Serialization (loading only) of ClusteredMesh");

        CLUSTERED_MESH_TEST(TestGetCluster, "Test GetCluster accessor");
        CLUSTERED_MESH_TEST(TestKDSubTrees, "Test KDSubTree accessor");
        CLUSTERED_MESH_TEST(TestCreateWithoutKDSubTrees, "Test Creating mesh without KDSubTrees");
        CLUSTERED_MESH_TEST(TestCreateWithoutKDSubTreesFpu, "Test creating fpu mesh without KDSubTrees");
        CLUSTERED_MESH_TEST(TestKDSubTreeAssignment, "Test assigning KDSubTrees to a mesh");
        CLUSTERED_MESH_TEST(TestKDSubTreeCreation, "Test assinging KDSubTrees to a mesh");
        CLUSTERED_MESH_TEST(TestNoBranchNodes, "Test creation with no branch nodes");

#if !defined(EA_PLATFORM_PS3_SPU)
        CLUSTERED_MESH_TEST(TestGetVolumeFromChildIndexTriangleMesh, "Test GetVolumeFromChildIndex method with triangle mesh");
        CLUSTERED_MESH_TEST(TestGetVolumeFromChildIndexQuadMesh, "Test GetVolumeFromChildIndex method with quad mesh");
        CLUSTERED_MESH_TEST(TestGetClusterIndexFromChildIndex, "Test GetClusterIndexFromChildIndex method");
        CLUSTERED_MESH_TEST(TestGetUnitOffsetFromChildIndex, "Test GetUnitOffsetFromChildIndex method");
        CLUSTERED_MESH_TEST(TestGetTriangleIndexWithinUnitFromChildIndexTriangleMesh, "Test GetTriangleIndexWithinUnitFromChildIndex method with triangle mesh");
        CLUSTERED_MESH_TEST(TestGetTriangleIndexWithinUnitFromChildIndexQuadMesh, "Test GetTriangleIndexWithinUnitFromChildIndex method with quad mesh");
#endif // !defined(EA_PLATFORM_PS3_SPU)
#if !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(TestFpuConversionSize, "Test fpu de-serialization doesn't overflow allocated memory");
#endif // !defined(RWP_NO_VPU_MATH)

        CLUSTERED_MESH_TEST(TestHLLoadWithKDSubTrees, "Test High-level serialization (loading only) of Clustered mesh with KDSubTrees");
#if !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(TestLLVpuLoadWithKDSubTrees, "Test Low-level serialization (loading only) of Clustered mesh with KDSubTrees");
#endif // !defined(RWP_NO_VPU_MATH)
        CLUSTERED_MESH_TEST(TestLLFpuLoadWithKDSubTrees, "Test Low-level fpu-layout serialization (loading only) of Clustered mesh with KDSubTrees");

        CLUSTERED_MESH_TEST(TestGetSizeThis, "Tests the value of mSizeOfThis after serialization");
    }

#if !defined(EA_PLATFORM_PS3_SPU)
    rw::collision::ClusteredMesh * BuildClusteredMesh(
        const uint32_t triangleXCount,
        const uint32_t triangleZCount,
        const bool quads);
#endif //!defined(EA_PLATFORM_PS3_SPU)

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        // Initialise the collision system
        Volume::InitializeVTable();

#if !defined(EA_PLATFORM_PS3_SPU)
        // Build a quad mesh
        quadMesh = BuildClusteredMesh(4u,4u,true);
        // Build a triangle mesh
        triangleMesh = BuildClusteredMesh(4u, 4u, false);
#endif //!defined(EA_PLATFORM_PS3_SPU)
    }

    virtual void TeardownSuite()
    {
#if !defined(EA_PLATFORM_PS3_SPU)
        if (NULL != quadMesh)
        {
            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(quadMesh);
        }

        if (NULL != triangleMesh)
        {
            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(triangleMesh);
        }

        EA::Physics::UnitFramework::ResetAllocator();
#endif // !defined(EA_PLATFORM_PS3_SPU)
        tests::TestSuiteBase::TeardownSuite();
    }

private:
    void TestUncompressedClusteredMeshVersion1HLFileSerializationLoadOnly();
    void TestUncompressedClusteredMeshVersion2HLFileSerializationLoadOnly();
    void TestUncompressedClusteredMeshHLFileSerializationLoadOnly();
    void TestUncompressedClusteredMeshLLVpuFileSerializationLoadOnly();
    void TestUncompressedClusteredMeshLLFpuFileSerializationLoadOnly();

    void Test16BitCompressedClusteredMeshVersion1HLFileSerializationLoadOnly();
    void Test16BitCompressedClusteredMeshVersion2HLFileSerializationLoadOnly();
    void Test16BitCompressedClusteredMeshHLFileSerializationLoadOnly();
    void Test16BitCompressedClusteredMeshLLVpuFileSerializationLoadOnly();
    void Test16BitCompressedClusteredMeshLLFpuFileSerializationLoadOnly();

    void Test32BitCompressedClusteredMeshVersion1HLFileSerializationLoadOnly();
    void Test32BitCompressedClusteredMeshVersion2HLFileSerializationLoadOnly();
    void Test32BitCompressedClusteredMeshHLFileSerializationLoadOnly();
    void Test32BitCompressedClusteredMeshLLVpuFileSerializationLoadOnly();
    void Test32BitCompressedClusteredMeshLLFpuFileSerializationLoadOnly();

    void TestGetCluster();
    void TestKDSubTrees();

    void TestCreateWithoutKDSubTrees();
    void TestCreateWithoutKDSubTreesFpu();
    void TestKDSubTreeAssignment();
    void TestKDSubTreeCreation();
    void TestNoBranchNodes();
    void TestFpuConversionSize();

#if !defined(EA_PLATFORM_PS3_SPU)
    void TestGetVolumeFromChildIndexTriangleMesh();
    void TestGetVolumeFromChildIndexQuadMesh();
    void TestGetClusterIndexFromChildIndex();
    void TestGetUnitOffsetFromChildIndex();
    void TestGetTriangleIndexWithinUnitFromChildIndexTriangleMesh();
    void TestGetTriangleIndexWithinUnitFromChildIndexQuadMesh();
#endif // !defined(EA_PLATFORM_PS3_SPU)

    void TestHLLoadWithKDSubTrees();
#if !defined(RWP_NO_VPU_MATH)
    void TestLLVpuLoadWithKDSubTrees();
#endif // !defined(RWP_NO_VPU_MATH)
    void TestLLFpuLoadWithKDSubTrees();

    void TestGetSizeThis();

    void CheckHasNoClusterKDTrees(const rw::collision::ClusteredMesh & mesh);
    void CheckHasClusterKDTrees(const rw::collision::ClusteredMesh & mesh);

#if !defined(EA_PLATFORM_PS3_SPU)
    void AddInputToBuilder(
        rw::collision::ClusteredMeshOfflineBuilder & offlineBuilder,
        const uint32_t triangleXCount,
        const uint32_t triangleZCount);

    inline void AssertTrianglesTheSameExcludingFlags(const TriangleVolume * volumeA, const TriangleVolume * volumeB)
    {
        Vector3 triangleAVertices[3];
        volumeA->GetPoints(triangleAVertices[0], triangleAVertices[1], triangleAVertices[2], NULL);
        Vector3 triangleANormal;
        volumeA->GetNormal(triangleANormal, NULL);

        Vector3 triangleBVertices[3];
        volumeB->GetPoints(triangleBVertices[0], triangleBVertices[1], triangleBVertices[2], NULL);
        Vector3 triangleBNormal;
        volumeB->GetNormal(triangleBNormal, NULL);

        EATESTAssert(IsSimilar(triangleAVertices[0], triangleBVertices[0]), "Triangle vertices[0] do not match");
        EATESTAssert(IsSimilar(triangleAVertices[1], triangleBVertices[1]), "Triangle vertices[1] do not match");
        EATESTAssert(IsSimilar(triangleAVertices[2], triangleBVertices[2]), "Triangle vertices[2] do not match");
        EATESTAssert(IsSimilar(triangleANormal, triangleBNormal), "Triangle normals do not match");
        EATESTAssert(IsSimilar(volumeA->GetEdgeCosVector(), volumeB->GetEdgeCosVector()), "Edge cosine data does not match");
        EATESTAssert(IsSimilar(volumeA->GetRadius(), volumeB->GetRadius()), "Volume radius does not match");
        EATESTAssert(volumeA->GetGroup()   == volumeB->GetGroup(), "Volume group does not match");
        EATESTAssert(volumeA->GetSurface() == volumeB->GetSurface(), "Volume surface id does not match");
    }

    rw::collision::ClusteredMesh * triangleMesh;
    rw::collision::ClusteredMesh * quadMesh;
#endif // !defined(EA_PLATFORM_PS3_SPU)

} TestClusteredMeshSingleton;


void TestClusteredMesh::TestUncompressedClusteredMeshVersion1HLFileSerializationLoadOnly()
{
    // test that we can load a ClusteredMesh which has been serialized out with version 1 of the
    // ClusteredMesh serialization function.
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_raw_ver1");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::TestUncompressedClusteredMeshVersion2HLFileSerializationLoadOnly()
{
    // test that we can load a ClusteredMesh which has been serialized out with version 2 of the
    // ClusteredMesh serialization function.
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_raw_ver2");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}

void TestClusteredMesh::TestUncompressedClusteredMeshHLFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_raw");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


#if !defined(RWP_NO_VPU_MATH)
void TestClusteredMesh::TestUncompressedClusteredMeshLLVpuFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("clusteredmesh_raw");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_raw")), filename);

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed low level vpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level vpu file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}
#endif // !defined(RWP_NO_VPU_MATH)


void TestClusteredMesh::TestUncompressedClusteredMeshLLFpuFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("clusteredmesh_raw");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_raw")), filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_raw")), filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(loaded, "Failed low level fpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level fpu file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::Test16BitCompressedClusteredMeshVersion1HLFileSerializationLoadOnly()
{
    // test that we can load a ClusteredMesh which has been serialized out with version 1 of the
    // ClusteredMesh serialization function.
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_16bit_ver1");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::Test16BitCompressedClusteredMeshVersion2HLFileSerializationLoadOnly()
{
    // test that we can load a ClusteredMesh which has been serialized out with version 2 of the
    // ClusteredMesh serialization function.
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_16bit_ver2");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::Test16BitCompressedClusteredMeshHLFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_16bit");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


#if !defined(RWP_NO_VPU_MATH)
void TestClusteredMesh::Test16BitCompressedClusteredMeshLLVpuFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("clusteredmesh_16bit");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_16bit")), filename);

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<ClusteredMesh>(filename);
    EATESTAssert(loaded, "Failed low level vpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level vpu file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}
#endif // !defined(RWP_NO_VPU_MATH)


void TestClusteredMesh::Test16BitCompressedClusteredMeshLLFpuFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("clusteredmesh_16bit");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_16bit")), filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_16bit")), filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(loaded, "Failed low level fpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level fpu file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::Test32BitCompressedClusteredMeshVersion1HLFileSerializationLoadOnly()
{
    // test that we can load a ClusteredMesh which has been serialized out with version 1 of the
    // ClusteredMesh serialization function.
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit_ver1");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::Test32BitCompressedClusteredMeshVersion2HLFileSerializationLoadOnly()
{
    // test that we can load a ClusteredMesh which has been serialized out with version 2 of the
    // ClusteredMesh serialization function.
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit_ver2");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::Test32BitCompressedClusteredMeshHLFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}


#if !defined(RWP_NO_VPU_MATH)
void TestClusteredMesh::Test32BitCompressedClusteredMeshLLVpuFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("clusteredmesh_32bit");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit")), filename);

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<ClusteredMesh>(filename);
    EATESTAssert(loaded, "Failed low level vpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level vpu file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}
#endif // !defined(RWP_NO_VPU_MATH)


void TestClusteredMesh::Test32BitCompressedClusteredMeshLLFpuFileSerializationLoadOnly()
{
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("clusteredmesh_32bit");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit")), filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit")), filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(loaded, "Failed low level fpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level fpu file serialization (loading only).");
    CheckHasNoClusterKDTrees(*loaded);
}

void TestClusteredMesh::CheckHasNoClusterKDTrees(const rw::collision::ClusteredMesh & mesh)
{
    for (uint32_t c = 0; c < mesh.GetNumCluster(); ++c)
    {
        EATESTAssert(mesh.GetClusterKDTree(c) == NULL, "Should have no KDTree defined for any cluster");
    }
}
void TestClusteredMesh::CheckHasClusterKDTrees(const rw::collision::ClusteredMesh & mesh)
{
    for (uint32_t c = 0; c < mesh.GetNumCluster(); ++c)
    {
        EATESTAssert(mesh.GetClusterKDTree(c) != NULL, "Should have KDTree defined for each cluster");
    }
}

void TestClusteredMesh::TestKDSubTrees()
{
    // Load a mesh with no subtrees
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->GetKDTree() != NULL, "Should have a KDTree defined");
    EATESTAssert(loaded->GetKDTreeBase() == loaded->GetKDTree(), "Should have a KDTreeBase defined");
    EATESTAssert(loaded->GetNumCluster() > 0, "Should have at least one cluster");
    CheckHasNoClusterKDTrees(*loaded);
}


void TestClusteredMesh::TestGetCluster()
{
    // Load a mesh 
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");

    const ClusteredMesh * mesh = loaded;
    EATESTAssert(mesh->GetNumCluster() > 0, "Should have at least one cluster");
    uint32_t * clusterTable = mesh->GetClusterTableAddress();
    EATESTAssert(clusterTable, "Should have valid cluster table");
    uintptr_t clusterTableAddr = reinterpret_cast<uintptr_t>(clusterTable);
    for (uint32_t c = 0; c <mesh->GetNumCluster(); ++c)
    {
        const ClusteredMeshCluster * cluster = &mesh->GetCluster(c);
        uintptr_t clusterAddr = ClusteredMesh::GetClusterFromClusterTable(clusterTableAddr, clusterTable, c);
        const ClusteredMeshCluster * clusterPtr = reinterpret_cast<const ClusteredMeshCluster *>(clusterAddr);
        EATESTAssert(cluster, "Cluster should be valid");
        EATESTAssert(clusterPtr == cluster, "Cluster should be same from cluster table");
    }
}


void TestClusteredMesh::TestKDSubTreeAssignment()
{
    // Load a mesh with no subtrees
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");

    EATESTAssert(loaded->GetKDTree() != NULL, "Should have a KDTree defined");
    EATESTAssert(loaded->GetNumCluster() > 0, "Should have at least one cluster");
    CheckHasNoClusterKDTrees(*loaded);

    rw::collision::KDSubTree subtrees[4];
    loaded->SetClusterKDTrees(subtrees);
    CheckHasClusterKDTrees(*loaded);
    EATESTAssert(loaded->GetClusterKDTree(0) == &subtrees[0], "Should have first subtree set");
    rw::collision::KDSubTree moresubtrees[4];
    loaded->SetClusterKDTrees(moresubtrees);
    CheckHasClusterKDTrees(*loaded);
    EATESTAssert(loaded->GetClusterKDTree(0) == &moresubtrees[0], "Should have first subtree changed");
}


void TestClusteredMesh::TestKDSubTreeCreation()
{
    // Load a mesh with no subtrees
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_raw");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");

    EATESTAssert(loaded->GetKDTree() != NULL, "Should have a KDTree defined");
    EATESTAssert(loaded->GetNumCluster() > 0, "Should have at least one cluster");
    CheckHasNoClusterKDTrees(*loaded);
    EA::Physics::SizeAndAlignment sa = rw::collision::GetKDSubTreeWorkSpaceResourceDescriptor(*loaded);
    EA::Physics::MemoryPtr workspace = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sa.GetSize(), 0, 0, sa.GetAlignment());
    loaded->CreateClusterKDTrees(workspace);
    CheckHasNoClusterKDTrees(*loaded);  // no memory allocated

    EA_ASSERT(loaded->GetNumCluster() < 20);
    rw::collision::KDSubTree subtrees[20];
    rw::collision::CreateKDSubTreeArray(subtrees, workspace, *loaded);
    loaded->SetClusterKDTrees(subtrees);
    CheckHasClusterKDTrees(*loaded);
    EATESTAssert(loaded->IsValid(), "Loaded mesh should be valid");

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(workspace.GetMemory());
}

void TestClusteredMesh::TestNoBranchNodes()
{
    // Try and create a clustered mesh with one cluster but no branch nodes
    rw::collision::AABBox bbox(rwpmath::Vector3(-1.0f, -2.0f, -3.0f), rwpmath::Vector3(4.0f, 5.0f, 6.0f));
    uint32_t clusterSize = 16;
    rw::collision::ClusteredMesh * mesh = EA::Physics::UnitFramework::Creator<rw::collision::ClusteredMesh>().New(
        1u, clusterSize, 0u, 2u, bbox, 0.01f, uint32_t(sizeof(rw::collision::ClusteredMesh)), RwpBool(TRUE));

    EATESTAssert(mesh->GetKDTreeBase() != 0, "Should have a KDTree");
    EATESTAssert(mesh->GetKDTreeBase()->GetNumBranchNodes() == 0, "Should have no branch nodes");
    // Create a cluster - won't be accessing actual unit data
    rw::collision::ClusteredMeshCluster * cluster = mesh->AllocateNextCluster(clusterSize, 2);
    EATESTAssert(mesh->GetNumCluster() == 1, "Should have 1 cluster");
    EATESTAssert(cluster, "Should have allocated one cluster");

    // Can create KDSubTree with an empty KDTree
    EA::Physics::SizeAndAlignment workspaceRD = rw::collision::GetKDSubTreeWorkSpaceResourceDescriptor(*mesh);
    EATESTAssert(workspaceRD.GetSize() > 0, "Should always return non-zero resource descriptor");
    EA::Physics::MemoryPtr workspace = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(workspaceRD.GetSize(), 0, 0, workspaceRD.GetAlignment());
    mesh->CreateClusterKDTrees(workspace);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(workspace.GetMemory());

    const rw::collision::KDSubTree * t = mesh->GetClusterKDTree(0);
    EATESTAssert(t, "Should have a cluster KD tree");
    EATESTAssert(t->m_branchNodes == 0, "Branch nodes should be NULL");
    EATESTAssert(t->m_numBranchNodes == 0, "Num branch nodes should be 0");
    EATESTAssert(t->m_bbox.Min() == bbox.Min() && t->m_bbox.Max() == bbox.Max(), "BBox should be set");
    EATESTAssert(t->GetBranchNodeOffset() == 0, "Branch node offset should be zero");
    EATESTAssert(t->GetDefaultEntry() == 0, "Default entry should be zero");
    EATESTAssert(t->GetRootNode() == 0, "Root node should be zero");

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mesh);
}

void TestClusteredMesh::TestHLLoadWithKDSubTrees()
{
    // Load a mesh with subtrees
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmeshsubtrees");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");

    EATESTAssert(loaded->GetKDTree() != NULL, "Should have a KDTree defined");
    EATESTAssert(loaded->GetNumCluster() > 0, "Should have at least one cluster");
    CheckHasClusterKDTrees(*loaded);
}

#if !defined(EA_PLATFORM_PS3_SPU)

void TestClusteredMesh::AddInputToBuilder(
    rw::collision::ClusteredMeshOfflineBuilder & offlineBuilder,
    const uint32_t triangleXCount,
    const uint32_t triangleZCount)
{
    typedef rw::collision::meshbuilder::VectorType VectorType;

    uint32_t vertexIndex = 0;
    uint32_t triangleIndex = 0;

    for (uint32_t triangleXIndex = 0 ; triangleXIndex < triangleXCount ; ++triangleXIndex)
    {
        for (uint32_t triangleZIndex = 0 ; triangleZIndex < triangleZCount ; ++triangleZIndex)
        {
            VectorType v0(static_cast<float>(triangleXIndex) * 1.0f,
                0.0f,
                static_cast<float>(triangleZIndex) * 1.0f);
            VectorType v1(static_cast<float>(triangleXIndex) * 1.0f,
                0.0f,
                static_cast<float>(triangleZIndex + 1) * 1.0f);
            VectorType v2(static_cast<float>(triangleXIndex + 1) * 1.0f,
                0.0f,
                static_cast<float>(triangleZIndex) * 1.0f);
            VectorType v3(static_cast<float>(triangleXIndex + 1) * 1.0f,
                0.0f,
                static_cast<float>(triangleZIndex + 1) * 1.0f);

            offlineBuilder.SetVertex(vertexIndex, v0);
            offlineBuilder.SetVertex(vertexIndex + 1u, v1);
            offlineBuilder.SetVertex(vertexIndex + 2u, v2);

            offlineBuilder.SetTriangle(triangleIndex,
                vertexIndex,
                vertexIndex + 1u,
                vertexIndex + 2u);

            vertexIndex += 3;
            ++triangleIndex;

            offlineBuilder.SetVertex(vertexIndex, v1);
            offlineBuilder.SetVertex(vertexIndex + 1u, v3);
            offlineBuilder.SetVertex(vertexIndex + 2u, v2);

            offlineBuilder.SetTriangle(triangleIndex,
                vertexIndex,
                vertexIndex + 1u,
                vertexIndex + 2u);

            vertexIndex += 3;
            ++triangleIndex;
        }
    }
}


rw::collision::ClusteredMesh * TestClusteredMesh::BuildClusteredMesh(
    const uint32_t xCount,
    const uint32_t zCount,
    const bool quads)
{
    const uint32_t triangleCount = xCount * zCount * 2;
    const uint32_t vertexCount = triangleCount * 3;

    // Create mesh builder parameters
    rw::collision::ClusteredMeshOfflineBuilder::Parameters params;
    params.quads_Enable = quads;

    // Create mesh builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(
        triangleCount,
        vertexCount,
        0u,
        params,
        EA::Allocator::ICoreAllocator::GetDefaultAllocator());

    AddInputToBuilder(
        offlineBuilder,
        xCount,
        zCount);

    return offlineBuilder.BuildClusteredMesh();
}
#endif // !defined(EA_PLATFORM_PS3_SPU)

#if !defined(EA_PLATFORM_PS3_SPU)
void TestClusteredMesh::TestGetVolumeFromChildIndexTriangleMesh()
{
    rw::collision::TriangleVolume * actualTriangleVolume =
        EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero());

    rw::collision::TriangleVolume * expectedTriangleVolume =
        EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero());

    // Attempt to extract the second triangle from the mesh
    // ChildIndex = unit triangle index(0) and unit offset (7) and cluster index (0)
    const uint32_t childIndex = 0xE;

    // Get the triangle volume from the mesh
    triangleMesh->GetVolumeFromChildIndex(*actualTriangleVolume, childIndex);

    // Set the expected triangle volume
    expectedTriangleVolume->SetPoints(rwpmath::Vector3(0.0f, 0.0f, 1.0f),
                                      rwpmath::Vector3(1.0f, 0.0f, 1.0f),
                                      rwpmath::Vector3(1.0f, 0.0f, 0.0f));
    expectedTriangleVolume->SetEdgeCos(1.0f, 1.0f, 1.0f);

    // Check the triangles
    AssertTrianglesTheSameExcludingFlags(actualTriangleVolume, expectedTriangleVolume);

    // Check the flags
    uint32_t actualFlags = actualTriangleVolume->GetFlags();

    EATESTAssert(actualFlags & rw::collision::GPInstance::FLAG_TRIANGLEONESIDED, "Flag One-Sided should be set");

    EATESTAssert(actualFlags & rw::collision::GPInstance::TRIANGLE, "Flag Triangle should be set");

    // We don't test for the FLAG_TRIANGLEEDGEXCONVEX flags here since the test mesh
    // triangle are co-planar. These co-planar triangles are an edge case for the
    // convexity test, the result being undefined. In practice the edges are considered
    // convex on all platforms other than Wii.
}

void TestClusteredMesh::TestGetVolumeFromChildIndexQuadMesh()
{
    rw::collision::TriangleVolume * actualTriangleVolume =
        EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero());

    rw::collision::TriangleVolume * expectedTriangleVolume =
        EA::Physics::UnitFramework::Creator<rw::collision::TriangleVolume>().New(rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero(),
                                                                                 rwpmath::GetVector3_Zero());

    // Attempt to extract the first triangle from the mesh
    // ChildIndex = unit triangle index(1) and unit offset (9) and cluster index (0)
    const uint32_t childIndex = 0x212;

    // Get the triangle volume from the mesh
    quadMesh->GetVolumeFromChildIndex(*actualTriangleVolume, childIndex);

    // Set the expected triangle volume
    expectedTriangleVolume->SetPoints(rwpmath::Vector3(1.0f, 0.0f, 2.0f),
                                      rwpmath::Vector3(1.0f, 0.0f, 1.0f),
                                      rwpmath::Vector3(0.0f, 0.0f, 2.0f));
    expectedTriangleVolume->SetEdgeCos(1.0f, 1.0f, 1.0f);

    // Check the triangles, exluding the flags
    AssertTrianglesTheSameExcludingFlags(actualTriangleVolume, expectedTriangleVolume);

    // We don't test for the FLAG_TRIANGLEEDGEXCONVEX flags here since the test mesh
    // triangle are co-planar. These co-planar triangles are an edge case for the
    // convexity test, the result being undefined. In practice the edges are considered
    // convex on all platforms other than Wii.
}

void TestClusteredMesh::TestGetClusterIndexFromChildIndex()
{
    // ChildIndex = unit triangle index(0) and unit offset (14) and cluster index (0)
    const uint32_t childIndex = 0x1C;

    // Get the cluster index from the child index
    const uint32_t actualClusterIndex = triangleMesh->GetClusterIndexFromChildIndex(childIndex);

    const uint32_t expectedClusterIndex = 0;

    EATESTAssert(expectedClusterIndex == actualClusterIndex, "Cluster index should be zero");
}


void TestClusteredMesh::TestGetUnitOffsetFromChildIndex()
{
    // ChildIndex = unit triangle index(0) and unit offset (21) and cluster index (0)
    const uint32_t childIndex = 0x2A;

    // Get the cluster index from the child index
    const uint32_t actualUnitOffset = triangleMesh->GetUnitOffsetFromChildIndex(childIndex);

    const uint32_t expectedUnitOffset = 21;

    EATESTAssert(expectedUnitOffset == actualUnitOffset, "Unit offset should be zero");
}


void TestClusteredMesh::TestGetTriangleIndexWithinUnitFromChildIndexTriangleMesh()
{
    // ChildIndex = unit triangle index(0) and unit offset (21) and cluster index (0)
    const uint32_t childIndex = 0x2A;

    // Get the cluster index from the child index
    const uint32_t actualTriangleIndex = triangleMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex);

    const uint32_t expectedTriangleIndex = 0;

    EATESTAssert(expectedTriangleIndex == actualTriangleIndex, "Triangle index should be zero");
}

void TestClusteredMesh::TestGetTriangleIndexWithinUnitFromChildIndexQuadMesh()
{
    // ChildIndex = unit triangle index(1) and unit offset (36) and cluster index (0)
    const uint32_t childIndex = 0x244;

    // Get the cluster index from the child index
    const uint32_t actualTriangleIndex = quadMesh->GetTriangleIndexWithinUnitFromChildIndex(childIndex);

    const uint32_t expectedTriangleIndex = 1;

    EATESTAssert(expectedTriangleIndex == actualTriangleIndex, "Triangle index should be one");
}



#endif // !defined(EA_PLATFORM_PS3_SPU)

#if !defined(RWP_NO_VPU_MATH)
void TestClusteredMesh::TestFpuConversionSize()
{
    // Load a mesh with subtrees (mesh without subtrees seems to be OK)
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmeshsubtrees");

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    EATESTAssert(loaded, "Failed high level file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed high level file serialization (loading only).");

    // Convert from vpu to fpu by streaming into buffer and streaming out into fpu version of mesh
    rw::collision::ClusteredMesh & mesh = *loaded;
    rw::collision::detail::fpu::ClusteredMesh* originalFpu = EA::Physics::UnitFramework::CreateFpuObjectFromVpuObject<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(mesh);

    // Get address of last byte in last cluster
    rw::collision::detail::fpu::ClusteredMeshCluster & c = originalFpu->GetCluster(originalFpu->mNumClusters-1);
    uint8_t * u = reinterpret_cast<uint8_t*>(c.vertexArray) + c.unitDataStart * 16;
    uint8_t * m = (uint8_t *) originalFpu;
    // Compute minimum size needed for the mesh
    uint32_t s = (uint32_t) ((u + c.unitDataSize) - m);
    // Check the resource descriptor is this big
    EA::Physics::SizeAndAlignment rd = rw::collision::detail::fpu::ClusteredMesh::GetResourceDescriptor(originalFpu->GetObjectDescriptor());
    EATESTAssert(s <= rd.GetSize(), "Last cluster data should fit in allocated memory");
}
#endif // !defined(RWP_NO_VPU_MATH)


#if !defined(RWP_NO_VPU_MATH)
void TestClusteredMesh::TestLLVpuLoadWithKDSubTrees()
{
    const char* filename = UNITTEST_LL_SERIALIZED_DATA_FILE("clusteredmeshsubtrees");

    EA::Physics::UnitFramework::SaveLLVpuSerializationToFile(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmeshsubtrees")), filename);

    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLVpuSerializationFromFile<ClusteredMesh>(filename);
    EATESTAssert(loaded, "Failed low level vpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level vpu file serialization (loading only).");
    CheckHasClusterKDTrees(*loaded);
}
#endif // !defined(RWP_NO_VPU_MATH)


void TestClusteredMesh::TestLLFpuLoadWithKDSubTrees()
{
    const char* filename = UNITTEST_LL_FPU_SERIALIZED_DATA_FILE("clusteredmeshsubtrees");

#if !defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmeshsubtrees")), filename);
#else // if defined(RWP_NO_VPU_MATH)
    EA::Physics::UnitFramework::SaveLLFpuSerializationToFile<ClusteredMesh>(*EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmeshsubtrees")), filename);
#endif // defined(RWP_NO_VPU_MATH)

#if !defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh, rw::collision::detail::fpu::ClusteredMesh>(filename);
#else // if defined(RWP_NO_VPU_MATH)
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadLLFpuSerializationFromFile<ClusteredMesh>(filename);
#endif // defined(RWP_NO_VPU_MATH)

    EATESTAssert(loaded, "Failed low level fpu file serialization (loading only).");
    EATESTAssert(loaded->IsValid(), "Failed low level fpu file serialization (loading only).");
    CheckHasClusterKDTrees(*loaded);
}


void TestClusteredMesh::TestGetSizeThis()
{
    const char* filename = UNITTEST_HL_SERIALIZED_DATA_FILE("clusteredmesh_32bit");
    ClusteredMesh* loaded = EA::Physics::UnitFramework::LoadHLSerializationFromFile<ClusteredMesh>(filename);

    const ClusteredMesh::ObjectDescriptor objectDescriptor(loaded->GetObjectDescriptor());
    const EA::Physics::SizeAndAlignment sizeAndAlignment = ClusteredMesh::GetResourceDescriptor(objectDescriptor);

    EATESTAssert(sizeAndAlignment.GetSize() == loaded->GetSizeThis(), "GetSizeThis returns incorrect size.");
}


void TestClusteredMesh::TestCreateWithoutKDSubTrees()
{
    // Make these sizes as awkward as possible to stress the alignment code
    uint32_t maxClusters = 9;
    uint32_t clusterDataSize = 99;
    uint32_t numBranchNodes = 19;
    uint32_t maxUnits = 51;
    rw::collision::AABBox bbox(rwpmath::Vector3(-1.0f, -2.0f, -3.0f), rwpmath::Vector3(1.0f, 0.2f, -1.5f));

    rw::collision::ClusteredMesh::ObjectDescriptor withoutOD(maxClusters, clusterDataSize, numBranchNodes, maxUnits, bbox);
    EATESTAssert(!withoutOD.m_includeKDSubTrees, "Should default to not including subtrees");
    EA::Physics::SizeAndAlignment withoutRD = rw::collision::ClusteredMesh::GetResourceDescriptor(withoutOD);

    rw::collision::ClusteredMesh::ObjectDescriptor withOD(maxClusters, clusterDataSize, numBranchNodes, maxUnits, bbox, true);
    EATESTAssert(withOD.m_includeKDSubTrees, "Should now include subtrees");
    EA::Physics::SizeAndAlignment withRD = rw::collision::ClusteredMesh::GetResourceDescriptor(withOD);

    uint32_t withoutSize = withoutRD.GetSize();
    uint32_t withSize = withRD.GetSize();
    EATESTAssert(withSize > withoutSize, "Should be bigger with subtrees");
    EATESTAssert(withSize >= withoutSize + maxClusters*sizeof(rw::collision::KDSubTree), "Should be a lot bigger with subtrees");

    {
        rw::collision::ClusteredMesh * withoutMesh = EA::Physics::UnitFramework::Creator<ClusteredMesh>().New(withoutOD);
        withoutMesh->AllocateNextCluster(30, 4);
        CheckHasNoClusterKDTrees(*withoutMesh);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(withoutMesh);
    }

    {
        rw::collision::ClusteredMesh * withMesh = EA::Physics::UnitFramework::Creator<ClusteredMesh>().New(withOD);
        withMesh->AllocateNextCluster(30, 4);
        CheckHasClusterKDTrees(*withMesh);
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(withMesh);
    }
}

void TestClusteredMesh::TestCreateWithoutKDSubTreesFpu()
{
    // Make these sizes as awkward as possible to stress the alignment code
    uint32_t maxClusters = 9;
    uint32_t clusterDataSize = 99;
    uint32_t numBranchNodes = 19;
    uint32_t maxUnits = 51;
    rw::collision::detail::fpu::AABBox bbox;
    bbox.m_min = rw::math::fpu::Vector3(-1.0f, -2.0f, -3.0f);
    bbox.m_max = rw::math::fpu::Vector3(1.0f, 0.2f, -1.5f);

    rw::collision::detail::fpu::ClusteredMesh::ObjectDescriptor withoutOD(maxClusters, clusterDataSize, numBranchNodes, maxUnits, bbox);
    EATESTAssert(!withoutOD.m_includeKDSubTrees, "Should default to not including subtrees");
    EA::Physics::SizeAndAlignment withoutRD = rw::collision::detail::fpu::ClusteredMesh::GetResourceDescriptor(withoutOD);

    rw::collision::detail::fpu::ClusteredMesh::ObjectDescriptor withOD(maxClusters, clusterDataSize, numBranchNodes, maxUnits, bbox, true);
    EATESTAssert(withOD.m_includeKDSubTrees, "Should now include subtrees");
    EA::Physics::SizeAndAlignment withRD = rw::collision::detail::fpu::ClusteredMesh::GetResourceDescriptor(withOD);

    uint32_t withoutSize = withoutRD.GetSize();
    uint32_t withSize = withRD.GetSize();
    EATESTAssert(withSize > withoutSize, "Should be bigger with subtrees");
    EATESTAssert(withSize >= withoutSize + maxClusters*sizeof(rw::collision::detail::fpu::KDSubTree), "Should be a lot bigger with subtrees");
}
