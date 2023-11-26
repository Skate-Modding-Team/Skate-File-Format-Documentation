// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/kdtreebuilder.h>

#include "benchmark_timer.hpp"
#include "random.hpp"

#include "stdio.h"     // for sprintf()

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;

// Unit tests for clustered mesh line queries
// This package is unable to easily create ClusteredMesh objects for testing so these
// tests rely on data files which have been created by the rwphysics_conditioning package.

class BenchmarkKDTreeBuilder: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("BenchmarkKDTreeBuilder");

// These tests are not included in the android test suite as they are too time consuming and we believe no android customer will be interested in the figures generated.
// However, just in case that assumption is wrong we allow the smallest data set benchmark to run, and we can enable the larger data sets should a customer request it.
#ifndef EA_PLATFORM_MOBILE
        EATEST_REGISTER("BenchmarkKDTreeSmallSetGrid", "KDTree build using small set of uniform grid input", BenchmarkKDTreeBuilder, BenchmarkKDTreeSmallSetGrid);
        EATEST_REGISTER("BenchmarkKDTreeSmallSetRandomEntries", "KDTree build using small set of random input", BenchmarkKDTreeBuilder, BenchmarkKDTreeSmallSetRandomEntries);
        EATEST_REGISTER("BenchmarkKDTreeMediumSetGrid", "KDTree build using medium set of uniform grid input", BenchmarkKDTreeBuilder, BenchmarkKDTreeMediumSetGrid);
        EATEST_REGISTER("BenchmarkKDTreeLargeSetGrid", "KDTree build using large set of uniform grid input", BenchmarkKDTreeBuilder, BenchmarkKDTreeLargeSetGrid);
        EATEST_REGISTER("BenchmarkKDTreeMediumSetRandomEntries", "KDTree build using medium set of random input", BenchmarkKDTreeBuilder, BenchmarkKDTreeMediumSetRandomEntries);
        EATEST_REGISTER("BenchmarkKDTreeLargeSetRandomEntries", "KDTree build using large set of random input", BenchmarkKDTreeBuilder, BenchmarkKDTreeLargeSetRandomEntries);
#endif
    }

private:

    // This Allocator is used to reduce costs of allocation as much as possible.
    // Since a fairly large number of allocation can take place during the build
    // process a slow allocator could distort the metrics, effectively shifting
    // the focus of the benchmarks onto the allocator rather than the KDTreeBuilder.
    class KDTreeAllocator : public EA::Allocator::ICoreAllocator
    {
    public:
        KDTreeAllocator()
            : m_base(NULL)
            , m_current(NULL)
            , m_size(0)
        {
        }

        void Initialize(uint8_t* base, uint32_t size)
        {
            m_base = base;
            m_current = m_base;
            m_size = size;
        }

        virtual void * Alloc(size_t size, const char * /*name*/, unsigned int /*flags*/)
        {
            return DoAllocate(size, 4);
        }

        virtual void * Alloc(size_t size, const char * /*name*/, unsigned int /*flags*/, unsigned int align, unsigned int alignOffset = 0)
        {
            (void)alignOffset;
            return DoAllocate(size, align);
        }

        virtual void Free(void *block, size_t size=0)
        {
            (void)block;
            (void)size;
        }

        void * DoAllocate(size_t size, unsigned int alignment)
        {
            uint8_t * ret = m_current;

            // Align the pointer
            ret = (uint8_t *)(((uintptr_t)ret + ((uintptr_t)alignment - 1)) & ~((uintptr_t)alignment - 1));
            
            if (ret + size <= m_base + m_size)
            {
                m_current = ret + size;
                return ret;
            }
            else
            {
                return NULL;
            }
        }

        void Reset()
        {
            m_current = m_base;
        }

        uint8_t * m_base;
        uint8_t * m_current;
        uint32_t m_size;
    };

    void BenchmarkKDTreeGeneration(const uint32_t numInputs,
                                   const rw::collision::AABBoxU* entryAABBoxes,
                                   uint32_t splitThreshold,
                                   float largeItemThreshold,
                                   const char *text);

    void BenchmarkAllThresholdVariations(uint32_t numInputs,
                                         rw::collision::AABBoxU* inputAABBoxes,
                                         const char *text);

    void BenchmarkKDTreeSmallSetGrid();
    void BenchmarkKDTreeMediumSetGrid();
    void BenchmarkKDTreeLargeSetGrid();
    void BenchmarkKDTreeSmallSetRandomEntries();
    void BenchmarkKDTreeMediumSetRandomEntries();
    void BenchmarkKDTreeLargeSetRandomEntries();

    KDTreeAllocator kdTreeAllocator;

} BenchmarkKDTreeBuilderSingleton;

void BenchmarkKDTreeBuilder::BenchmarkKDTreeGeneration(const uint32_t numInputs,
                                                       const rw::collision::AABBoxU* entryAABBoxes,
                                                       uint32_t splitThreshold,
                                                       float largeItemThreshold,
                                                       const char *text)
{
    rw::collision::Tests::BenchmarkTimer timer;
    rw::collision::KDTreeBuilder builder(kdTreeAllocator);

    // Time tree build process
    timer.Start();
    builder.BuildTree(numInputs, entryAABBoxes, splitThreshold, largeItemThreshold);
    timer.Stop();

    kdTreeAllocator.Reset();

    char buffer[256];
    sprintf(buffer, "suite:BenchmarkKDTreeBuilder,benchmark:GenerateKDTree,method:BuildTree,description:%s - Input %d - Split - %d - LargeItem - %f", text, numInputs, splitThreshold, largeItemThreshold);
    EATESTSendBenchmark(buffer, timer.GetAverageDurationMilliseconds());
}

void BenchmarkKDTreeBuilder::BenchmarkAllThresholdVariations(uint32_t numInputs,
                                                             rw::collision::AABBoxU* inputAABBoxes,
                                                             const char *text)
{
    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 8, 1.0f, text);
    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 4, 1.0f, text);

    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 8, 0.8f, text);
    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 4, 0.8f, text);
}

void BenchmarkKDTreeBuilder::BenchmarkKDTreeSmallSetGrid()
{
    // Generate the input data
    const float boxSize = 1.0f;
    const uint32_t xCount = 16u;
    const uint32_t yCount = 16u;
    const uint32_t zCount = 16u;
    const uint32_t numInputs = xCount * yCount * zCount;
    rw::collision::AABBoxU * inputAABBoxes = new rw::collision::AABBoxU[numInputs];

    for (uint32_t z = 0 ; z < xCount; ++z)
    {
        for (uint32_t y = 0 ; y < yCount; ++y)
        {
            for (uint32_t x = 0 ; x < zCount; ++x)
            {
                inputAABBoxes[x + (y * xCount) + (z * (xCount * yCount))].m_min = rw::collision::AABBoxU::Vector3Type(x * boxSize, y * boxSize, z * boxSize);
                inputAABBoxes[x + (y * xCount) + (z * (xCount * yCount))].m_max = rw::collision::AABBoxU::Vector3Type((x + 1) * boxSize, (y + 1) * boxSize, (z + 1) * boxSize);
            }
        }
    }

    // Size is determine by the number of inputs + the number of expected buildNodes.
    // At the time of writing this test the KDTreeBuilder created a maximum of 2047
    // BuildNodes for this group of tests. The  count of 2200 is used here to allow
    // some deviation.
    uint32_t kdTreeBufferSize = numInputs * 12 + 2200 * sizeof(KDTreeBuilder::BuildNode);
    uint8_t * kdTreeBuffer = new uint8_t[kdTreeBufferSize];
    kdTreeAllocator.Initialize(kdTreeBuffer, kdTreeBufferSize);

    BenchmarkAllThresholdVariations(numInputs, inputAABBoxes, "Uniform Entry - Uniform Distribution");

    delete [] kdTreeBuffer;
    delete [] inputAABBoxes;
}

void BenchmarkKDTreeBuilder::BenchmarkKDTreeMediumSetGrid()
{
    // Generate the input data
    const float boxSize = 1.0f;
    const uint32_t xCount = 32u;
    const uint32_t yCount = 32u;
    const uint32_t zCount = 32u;
    const uint32_t numInputs = xCount * yCount * zCount;
    rw::collision::AABBoxU * inputAABBoxes = new rw::collision::AABBoxU[numInputs];
    for (uint32_t z = 0 ; z < xCount; ++z)
    {
        for (uint32_t y = 0 ; y < yCount; ++y)
        {
            for (uint32_t x = 0 ; x < zCount; ++x)
            {
                inputAABBoxes[x + (y * xCount) + (z * (xCount * yCount))].m_min = rw::collision::AABBoxU::Vector3Type(x * boxSize, y * boxSize, z * boxSize);
                inputAABBoxes[x + (y * xCount) + (z * (xCount * yCount))].m_max = rw::collision::AABBoxU::Vector3Type((x + 1) * boxSize, (y + 1) * boxSize, (z + 1) * boxSize);
            }
        }
    }

    // Size is determine by the number of inputs + the number of expected buildNodes.
    // At the time of writing this test the KDTreeBuilder created a maximum of 16383
    // BuildNodes for this group of tests. The  count of 16800 is used here to allow
    // some deviation.
    uint32_t kdTreeBufferSize = numInputs * 12 + 16800  * sizeof(KDTreeBuilder::BuildNode);
    uint8_t * kdTreeBuffer = new uint8_t[kdTreeBufferSize];
    kdTreeAllocator.Initialize(kdTreeBuffer, kdTreeBufferSize);

    BenchmarkAllThresholdVariations(numInputs, inputAABBoxes, "Uniform Entry - Uniform Distribution");

    delete [] kdTreeBuffer;
    delete [] inputAABBoxes;
}

void BenchmarkKDTreeBuilder::BenchmarkKDTreeLargeSetGrid()
{
    // Generate the input data
    const float boxSize = 1.0f;
    const uint32_t xCount = 48u;
    const uint32_t yCount = 48u;
    const uint32_t zCount = 48u;
    const uint32_t numInputs = xCount * yCount * zCount;
    rw::collision::AABBoxU * inputAABBoxes = new rw::collision::AABBoxU[numInputs];
    for (uint32_t z = 0 ; z < xCount; ++z)
    {
        for (uint32_t y = 0 ; y < yCount; ++y)
        {
            for (uint32_t x = 0 ; x < zCount; ++x)
            {
                inputAABBoxes[x + (y * xCount) + (z * (xCount * yCount))].m_min = rw::collision::AABBoxU::Vector3Type(x * boxSize, y * boxSize, z * boxSize);
                inputAABBoxes[x + (y * xCount) + (z * (xCount * yCount))].m_max = rw::collision::AABBoxU::Vector3Type((x + 1) * boxSize, (y + 1) * boxSize, (z + 1) * boxSize);
            }
        }
    }

    // Size is determine by the number of inputs + the number of expected buildNodes.
    // At the time of writing this test the KDTreeBuilder created a maximum of 40959
    // BuildNodes for this group of tests. The  count of 41500 is used here to allow
    // some deviation.
    uint32_t kdTreeBufferSize = numInputs * 12 + 41500  * sizeof(KDTreeBuilder::BuildNode);
    uint8_t * kdTreeBuffer = new uint8_t[kdTreeBufferSize];
    kdTreeAllocator.Initialize(kdTreeBuffer, kdTreeBufferSize);

    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 8, 1.0f, "Uniform Entry - Uniform Distribution");
    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 8, 0.8f, "Uniform Entry - Uniform Distribution");

    delete [] kdTreeBuffer;
    delete [] inputAABBoxes;
}

void BenchmarkKDTreeBuilder::BenchmarkKDTreeSmallSetRandomEntries()
{
    rw::math::SeedRandom(9u);

    const float length = 1.0f;
    const uint32_t numInputs = 4096; // 16^3
    rw::collision::AABBoxU* inputAABBoxes = new rw::collision::AABBoxU[numInputs];

    for (uint32_t inputIndex = 0 ; inputIndex < numInputs ; ++inputIndex)
    {

        inputAABBoxes[inputIndex].m_min = rw::collision::AABBoxU::Vector3Type(RandomVector3(100.0f));
        inputAABBoxes[inputIndex].m_max = inputAABBoxes[inputIndex].m_min;

        inputAABBoxes[inputIndex].m_min -= rw::collision::AABBoxU::Vector3Type(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
        inputAABBoxes[inputIndex].m_max += rw::collision::AABBoxU::Vector3Type(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
    }

    // Size is determine by the number of inputs + the number of expected buildNodes.
    // At the time of writing this test the KDTreeBuilder created a maximum of 2843
    // BuildNodes for this group of tests. The  count of 3000 is used here to allow
    // some deviation.
    uint32_t kdTreeBufferSize = numInputs * 12 + 3000  * sizeof(KDTreeBuilder::BuildNode);
    uint8_t * kdTreeBuffer = new uint8_t[kdTreeBufferSize];
    kdTreeAllocator.Initialize(kdTreeBuffer, kdTreeBufferSize);

    BenchmarkAllThresholdVariations(numInputs, inputAABBoxes, "Random Entry - Random Distribution");

    delete [] kdTreeBuffer;
    delete [] inputAABBoxes;
}

void BenchmarkKDTreeBuilder::BenchmarkKDTreeMediumSetRandomEntries()
{
    rw::math::SeedRandom(9u);

    const float length = 1.0f;
    const uint32_t numInputs = 32768; // 32^3
    rw::collision::AABBoxU* inputAABBoxes = new rw::collision::AABBoxU[numInputs];

    for (uint32_t inputIndex = 0 ; inputIndex < numInputs ; ++inputIndex)
    {

        inputAABBoxes[inputIndex].m_min = rw::collision::AABBoxU::Vector3Type(RandomVector3(100.0f));
        inputAABBoxes[inputIndex].m_max = inputAABBoxes[inputIndex].m_min;

        inputAABBoxes[inputIndex].m_min -= rw::collision::AABBoxU::Vector3Type(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
        inputAABBoxes[inputIndex].m_max += rw::collision::AABBoxU::Vector3Type(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
    }

    // Size is determine by the number of inputs + the number of expected buildNodes.
    // At the time of writing this test the KDTreeBuilder created a maximum of 22571
    // BuildNodes for this group of tests. The  count of 23000 is used here to allow
    // some deviation.
    uint32_t kdTreeBufferSize = numInputs * 12 + 23000  * sizeof(KDTreeBuilder::BuildNode);
    uint8_t * kdTreeBuffer = new uint8_t[kdTreeBufferSize];
    kdTreeAllocator.Initialize(kdTreeBuffer, kdTreeBufferSize);

    BenchmarkAllThresholdVariations(numInputs, inputAABBoxes, "Random Entry - Random Distribution");

    delete [] kdTreeBuffer;
    delete [] inputAABBoxes;
}

void BenchmarkKDTreeBuilder::BenchmarkKDTreeLargeSetRandomEntries()
{
    rw::math::SeedRandom(9u);

    const float length = 1.0f;
    const uint32_t numInputs = 110592; // 48^3
    rw::collision::AABBoxU* inputAABBoxes = new rw::collision::AABBoxU[numInputs];

    for (uint32_t inputIndex = 0 ; inputIndex < numInputs ; ++inputIndex)
    {
        inputAABBoxes[inputIndex].m_min = rw::collision::AABBoxU::Vector3Type(RandomVector3(100.0f));
        inputAABBoxes[inputIndex].m_max = inputAABBoxes[inputIndex].m_min;

        inputAABBoxes[inputIndex].m_min -= rw::collision::AABBoxU::Vector3Type(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
        inputAABBoxes[inputIndex].m_max += rw::collision::AABBoxU::Vector3Type(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
    }

    // Size is determine by the number of inputs + the number of expected buildNodes.
    // At the time of writing this test the KDTreeBuilder created a maximum of 38789
    // BuildNodes for this group of tests. The  count of 23000 is used here to allow
    // some deviation.
    uint32_t kdTreeBufferSize = numInputs * 12 + 39500 * sizeof(KDTreeBuilder::BuildNode);
    uint8_t * kdTreeBuffer = new uint8_t[kdTreeBufferSize];
    kdTreeAllocator.Initialize(kdTreeBuffer, kdTreeBufferSize);

    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 8, 1.0f, "Random Entry - Random Distribution");
    BenchmarkKDTreeGeneration(numInputs, inputAABBoxes, 8, 0.8f, "Random Entry - Random Distribution");

    delete [] kdTreeBuffer;
    delete [] inputAABBoxes;
}


