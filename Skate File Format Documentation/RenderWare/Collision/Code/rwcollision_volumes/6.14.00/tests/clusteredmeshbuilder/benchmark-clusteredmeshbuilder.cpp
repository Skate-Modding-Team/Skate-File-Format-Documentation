// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <rw/collision/libcore.h>

#include <rw/collision/clusteredmeshruntimebuilder.h>
#include <rw/collision/clusteredmeshofflinebuilder.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilder.h>

#include <benchmarkenvironment/allocator.h>
#include <stdlib.h>

#include "unittest_datafile_utilities.hpp"
#include "eaphysics/unitframework/serialization_test_helpers.hpp"

#include "benchmark_timer.hpp"

#include "stdio.h"     // for sprintf()

#include "testsuitebase.h" // For TestSuiteBase

#include <eaphysics/hlserializable.h>

using namespace rw::collision;

// Unit tests for clustered mesh line queries
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.

#define USESIMPLETEST

class BenchmarkClusteredMeshBuilder: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("BenchmarkClusteredMeshBuilder");

// These tests are not included in the android test suite as they are too time consuming and we believe no android customer will be interested in the figures generated.
// However, just in case that assumption is wrong we allow the smallest data set benchmark to run, and we can enable the larger data sets should a customer request it.
#ifndef EA_PLATFORM_MOBILE
        EATEST_REGISTER("BenchmarkSmallInputSetBuild", "ClusteredMesh Build process using small input set", BenchmarkClusteredMeshBuilder, BenchmarkSmallInputSetBuild);
        EATEST_REGISTER("BenchmarkMediumInputSetBuild", "ClusteredMesh Build process using medium input set", BenchmarkClusteredMeshBuilder, BenchmarkMediumInputSetBuild);
        EATEST_REGISTER("BenchmarkLargeInputSetBuild", "ClusteredMesh Build process using large input set", BenchmarkClusteredMeshBuilder, BenchmarkLargeInputSetBuild);
        EATEST_REGISTER("BenchmarkExtraLargeInputSetBuild", "ClusteredMesh Build process using extra large input set", BenchmarkClusteredMeshBuilder, BenchmarkExtraLargeInputSetBuild);
#endif
    }

    virtual void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        rw::collision::InitializeVTables();
    }

private:

    static const size_t CLUSTEREDMESH_ALLOCATOR_DATA_BUFFER_SIZE;
    static const size_t CLUSTEREDMESH_ALLOCATOR_HEADER_BUFFER_SIZE;

    void BenchmarkGridInput(const uint32_t xCount,
                            const uint32_t yCount,
                            const uint32_t zCount,
                            const uint32_t bufferSize,
                            const char *text);

    void CreateBuilderInput(rw::collision::ClusteredMeshRuntimeBuilder &builder,
                            const uint32_t xCount,
                            const uint32_t yCount,
                            const uint32_t zCount);

    void BenchmarkSmallInputSetBuild();
    void BenchmarkMediumInputSetBuild();
    void BenchmarkLargeInputSetBuild();
    void BenchmarkExtraLargeInputSetBuild();

} BenchmarkKDTreeBuilderSingleton;


const size_t BenchmarkClusteredMeshBuilder::CLUSTEREDMESH_ALLOCATOR_DATA_BUFFER_SIZE = 1024 * 1024 *5;

const size_t BenchmarkClusteredMeshBuilder::CLUSTEREDMESH_ALLOCATOR_HEADER_BUFFER_SIZE = 1024 * 5;


void BenchmarkClusteredMeshBuilder::CreateBuilderInput(rw::collision::ClusteredMeshRuntimeBuilder &builder,
                                                       const uint32_t xCount,
                                                       const uint32_t yCount,
                                                       const uint32_t zCount)
{
    const rw::math::fpu::Vector3 quadVertices[4] =
    {
        rw::math::fpu::Vector3(0.0f, 0.0f, 0.0f), // Vertex 1
        rw::math::fpu::Vector3(1.0f, 0.0f, 0.0f), // Vertex 2
        rw::math::fpu::Vector3(0.0f, 0.0f, 1.0f), // Vertex 3
        rw::math::fpu::Vector3(1.0f, 0.0f, 1.0f), // Vertex 4
    };

    const uint32_t quadVertexIndices[6]=
    {
        0, 1, 2, // Triangle 1
        1, 3, 2, // Triangle 2
    };

    uint32_t triangleIndex = 0, vertexIndex = 0;
    rw::math::fpu::Vector3 offset(0.0f, 0.0f, 0.0f);

    for (uint32_t yIndex = 0 ; yIndex < yCount ; ++yIndex)
    {
        offset.SetY(1.0f * yIndex);
        for (uint32_t xIndex = 0 ; xIndex < xCount ; ++xIndex)
        {
            offset.SetX(1.0f * xIndex);
            for (uint32_t zIndex = 0 ; zIndex < zCount ; ++zIndex)
            {
                offset.SetZ(1.0f * zIndex);

                builder.SetTriangle(triangleIndex++, (vertexIndex), (vertexIndex + 1), (vertexIndex + 2));
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[0]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[1]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[2]] + offset);

                builder.SetTriangle(triangleIndex++, (vertexIndex), (vertexIndex + 1), (vertexIndex + 2));
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[3]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[4]] + offset);
                builder.SetVertex(vertexIndex++, quadVertices[quadVertexIndices[5]] + offset);
            }
        }
    }
}


void BenchmarkClusteredMeshBuilder::BenchmarkGridInput(const uint32_t xCount,
                                                       const uint32_t yCount,
                                                       const uint32_t zCount,
                                                       const uint32_t bufferSize,
                                                       const char *text)
{
    const uint32_t triangleCount = xCount * yCount * zCount * 2u;
    const uint32_t vertexCount = triangleCount * 3u;

    EA::Allocator::ICoreAllocator * allocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();
    const uint32_t NULL_ALLOCATION_FLAG = 0u;
    const uint32_t ALLOCATION_OFFSET = 0u;
    const uint32_t ALLOCATION_ALIGNMENT = 8u;

    // Allocate buffer for ClusteredMeshBuilder
    uint32_t sizeOfClusteredMeshBuilderAllocatorBuffer = static_cast<uint32_t>(1024*1024*bufferSize);
    void * clusteredMeshBuilderAllocatorBuffer = allocator->Alloc(sizeOfClusteredMeshBuilderAllocatorBuffer, NULL, NULL_ALLOCATION_FLAG, ALLOCATION_ALIGNMENT, ALLOCATION_OFFSET);
 
    // Allocate buffers for ClusteredMesh
    void * clusteredMeshAllocatorHeaderBuffer = allocator->Alloc(CLUSTEREDMESH_ALLOCATOR_HEADER_BUFFER_SIZE, NULL, NULL_ALLOCATION_FLAG, ALLOCATION_ALIGNMENT ,ALLOCATION_OFFSET);
    void * clusteredMeshAllocatorDataBuffer = allocator->Alloc(CLUSTEREDMESH_ALLOCATOR_DATA_BUFFER_SIZE, NULL, NULL_ALLOCATION_FLAG, ALLOCATION_ALIGNMENT);

    // Create allocator to allocate ClusteredMesh
    benchmarkenvironment::Allocator clusteredMeshAllocator(clusteredMeshAllocatorHeaderBuffer,
                                                           CLUSTEREDMESH_ALLOCATOR_HEADER_BUFFER_SIZE,
                                                           static_cast<benchmarkenvironment::Address>(clusteredMeshAllocatorDataBuffer),
                                                           CLUSTEREDMESH_ALLOCATOR_DATA_BUFFER_SIZE);

    // Use the default builder parameters
    rw::collision::ClusteredMeshRuntimeBuilder::Parameters builderParams;

    // Create the runtime builder
    rw::collision::ClusteredMeshRuntimeBuilder runtimeBuilder(triangleCount,
                                                              vertexCount,
                                                              0u,
                                                              builderParams,
                                                              static_cast<uint8_t*>(clusteredMeshBuilderAllocatorBuffer),
                                                              sizeOfClusteredMeshBuilderAllocatorBuffer,
                                                              &clusteredMeshAllocator);

    CreateBuilderInput(runtimeBuilder,
                       xCount,
                       yCount,
                       zCount);

    // Time mesh build process
    rw::collision::Tests::BenchmarkTimer timer;

    timer.Start();
    rw::collision::ClusteredMesh * clusteredMesh = runtimeBuilder.BuildClusteredMesh();
    timer.Stop();

    runtimeBuilder.Release();

    allocator->Free(clusteredMeshAllocatorDataBuffer);
    allocator->Free(clusteredMeshAllocatorHeaderBuffer);
    allocator->Free(clusteredMeshBuilderAllocatorBuffer);

    EATESTAssert(clusteredMesh != NULL, ("ClusteredMesh should not be NULL"));

    char buffer[256];
    sprintf(buffer,
            "suite:BenchmarkClusteredMeshBuilder,benchmark:GenerateClusteredMesh,method:BuildClusteredMesh,description:%s - Input %d - Split - %d - LargeItem - %f",
            text,
            triangleCount,
            builderParams.kdTreeBuilder_SplitThreshold,
            builderParams.kdTreeBuilder_LargeItemThreshold);

    EATESTSendBenchmark(buffer, timer.GetAverageDurationMilliseconds());
}


void BenchmarkClusteredMeshBuilder::BenchmarkSmallInputSetBuild()
{
    BenchmarkGridInput(10, 2, 10, 5, "Small Input Set");
}

void BenchmarkClusteredMeshBuilder::BenchmarkMediumInputSetBuild()
{
    BenchmarkGridInput(50, 2, 50, 5, "Medium Input Set");
}

void BenchmarkClusteredMeshBuilder::BenchmarkLargeInputSetBuild()
{
    BenchmarkGridInput(100, 5, 100, 20, "Large Input Set");
}

void BenchmarkClusteredMeshBuilder::BenchmarkExtraLargeInputSetBuild()
{
    BenchmarkGridInput(100, 10, 100, 28, "ExtraLarge Input Set");
}

