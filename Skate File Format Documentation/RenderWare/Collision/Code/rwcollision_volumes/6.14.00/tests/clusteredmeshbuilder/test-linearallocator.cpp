// (c) Electronic Arts. All Rights Reserved.


#if defined _MSC_VER
#pragma warning(push)
#pragma warning(disable:4548)
#endif

#include <string.h> // For memset

#if defined _MSC_VER
#pragma warning(pop)
#endif

#include <unit/unit.h>

#include <rw/collision/meshbuilder/detail/linearallocator.h>

#include "testsuitebase.h" // For TestSuiteBase

// Unit tests for clustered mesh builder linear allocator class
class TestLinearAllocator : public rw::collision::tests::TestSuiteBase
{

public:

    virtual void Initialize()
    {
        SuiteName("TestLinearAllocator");

        EATEST_REGISTER("TestInstantiation00", "Test trivial instantiation", TestLinearAllocator, TestInstantiation00);

        EATEST_REGISTER("TestAlloc00", "Test allocation", TestLinearAllocator, TestAlloc00);
        EATEST_REGISTER("TestAlloc01", "Test allocation", TestLinearAllocator, TestAlloc01);
        EATEST_REGISTER("TestAlloc02", "Test allocation", TestLinearAllocator, TestAlloc02);
        EATEST_REGISTER("TestAlloc03", "Test allocation", TestLinearAllocator, TestAlloc03);
        EATEST_REGISTER("TestAlloc04", "Test allocation", TestLinearAllocator, TestAlloc04);
        EATEST_REGISTER("TestAlloc05", "Test allocation", TestLinearAllocator, TestAlloc05);

        EATEST_REGISTER("TestFree00", "Test free", TestLinearAllocator, TestFree00);

        EATEST_REGISTER("TestMarkRelease00", "Test mark and release", TestLinearAllocator, TestMarkRelease00);
        EATEST_REGISTER("TestMarkRelease01", "Test mark and release", TestLinearAllocator, TestMarkRelease01);
        EATEST_REGISTER("TestMarkRelease02", "Test mark and release", TestLinearAllocator, TestMarkRelease02);
        EATEST_REGISTER("TestMarkRelease03", "Test mark and release", TestLinearAllocator, TestMarkRelease03);
        EATEST_REGISTER("TestMarkRelease04", "Test mark and release", TestLinearAllocator, TestMarkRelease04);
        EATEST_REGISTER("TestMarkRelease05", "Test mark and release", TestLinearAllocator, TestMarkRelease05);
        EATEST_REGISTER("TestMarkRelease06", "Test mark and release", TestLinearAllocator, TestMarkRelease06);
        EATEST_REGISTER("TestMarkRelease07", "Test mark and release", TestLinearAllocator, TestMarkRelease07);
        EATEST_REGISTER("TestMarkRelease08", "Test mark and release", TestLinearAllocator, TestMarkRelease08);

        EATEST_REGISTER("TestAlignment00", "Test alignment", TestLinearAllocator, TestAlignment00);
        EATEST_REGISTER("TestAlignment01", "Test alignment", TestLinearAllocator, TestAlignment01);
        EATEST_REGISTER("TestAlignment02", "Test alignment", TestLinearAllocator, TestAlignment02);
        EATEST_REGISTER("TestAlignment03", "Test alignment", TestLinearAllocator, TestAlignment03);
        EATEST_REGISTER("TestAlignment04", "Test alignment", TestLinearAllocator, TestAlignment04);
        EATEST_REGISTER("TestAlignment05", "Test alignment", TestLinearAllocator, TestAlignment05);
        EATEST_REGISTER("TestAlignment06", "Test alignment", TestLinearAllocator, TestAlignment06);
        EATEST_REGISTER("TestAlignment07", "Test alignment", TestLinearAllocator, TestAlignment07);
        EATEST_REGISTER("TestAlignment08", "Test alignment", TestLinearAllocator, TestAlignment08);
        EATEST_REGISTER("TestAlignment09", "Test alignment", TestLinearAllocator, TestAlignment09);

        EATEST_REGISTER("TestTotal00", "Test total memory usage", TestLinearAllocator, TestTotal00);
        EATEST_REGISTER("TestTotal01", "Test total memory usage", TestLinearAllocator, TestTotal01);
        EATEST_REGISTER("TestTotal02", "Test total memory usage", TestLinearAllocator, TestTotal02);
        EATEST_REGISTER("TestTotal03", "Test total memory usage", TestLinearAllocator, TestTotal03);
        EATEST_REGISTER("TestTotal04", "Test total memory usage", TestLinearAllocator, TestTotal04);
        EATEST_REGISTER("TestTotal05", "Test total memory usage", TestLinearAllocator, TestTotal05);
        EATEST_REGISTER("TestTotal06", "Test total memory usage", TestLinearAllocator, TestTotal06);

        EATEST_REGISTER("TestPeak00", "Test total memory usage", TestLinearAllocator, TestPeak00);
        EATEST_REGISTER("TestPeak01", "Test total memory usage", TestLinearAllocator, TestPeak01);
        EATEST_REGISTER("TestPeak02", "Test total memory usage", TestLinearAllocator, TestPeak02);
        EATEST_REGISTER("TestPeak03", "Test total memory usage", TestLinearAllocator, TestPeak03);
        EATEST_REGISTER("TestPeak04", "Test total memory usage", TestLinearAllocator, TestPeak04);
        EATEST_REGISTER("TestPeak05", "Test total memory usage", TestLinearAllocator, TestPeak05);
        EATEST_REGISTER("TestPeak06", "Test total memory usage", TestLinearAllocator, TestPeak06);
    }

private:

    static const uint32_t BUFFER_SIZE = 1024;
    static const uint32_t GUARDBAND_SIZE = 32;
    static const uint8_t GUARDBAND_MARKER = 'X';

    uint8_t *AllocateBuffer(const uint32_t bufferSize);
    void FreeBuffer(uint8_t *const buffer, const uint32_t bufferSize);

    static bool SetMemoryBlock(uint8_t *const start, uint8_t *const end, const uint8_t val);
    static bool CheckMemoryBlock(const uint8_t *const start, const uint8_t *const end, const uint8_t val);

    void TestInstantiation00();

    void TestAlloc00();
    void TestAlloc01();
    void TestAlloc02();
    void TestAlloc03();
    void TestAlloc04();
    void TestAlloc05();

    void TestFree00();

    void TestMarkRelease00();
    void TestMarkRelease01();
    void TestMarkRelease02();
    void TestMarkRelease03();
    void TestMarkRelease04();
    void TestMarkRelease05();
    void TestMarkRelease06();
    void TestMarkRelease07();
    void TestMarkRelease08();

    void TestAlignment00();
    void TestAlignment01();
    void TestAlignment02();
    void TestAlignment03();
    void TestAlignment04();
    void TestAlignment05();
    void TestAlignment06();
    void TestAlignment07();
    void TestAlignment08();
    void TestAlignment09();

    void TestTotal00();
    void TestTotal01();
    void TestTotal02();
    void TestTotal03();
    void TestTotal04();
    void TestTotal05();
    void TestTotal06();

    void TestPeak00();
    void TestPeak01();
    void TestPeak02();
    void TestPeak03();
    void TestPeak04();
    void TestPeak05();
    void TestPeak06();

} TestLinearAllocatorSingleton;


/**
Allocates a memory buffer for use by the allocator.
*/
uint8_t *TestLinearAllocator::AllocateBuffer(const uint32_t bufferSize)
{
    // Pad the buffer with guardbands at the start and end
    static const uint32_t paddedSize(GUARDBAND_SIZE + bufferSize + GUARDBAND_SIZE);
	uint8_t *const paddedBuffer(static_cast<uint8_t *>(EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(paddedSize, "buffer", 0, 4)));

    // Write known marker values to the guardband regions
    SetMemoryBlock(paddedBuffer, paddedBuffer + GUARDBAND_SIZE, GUARDBAND_MARKER);
    SetMemoryBlock(paddedBuffer + GUARDBAND_SIZE + bufferSize, paddedBuffer + GUARDBAND_SIZE + bufferSize + GUARDBAND_SIZE, GUARDBAND_MARKER);

    // Return the address of the actual buffer within the padded buffer
    return paddedBuffer + GUARDBAND_SIZE;
}


/**
Frees the memory buffer used by the allocator.
*/
void TestLinearAllocator::FreeBuffer(uint8_t *const buffer, const uint32_t bufferSize)
{
    // Check the guardband regions haven't been overwritten
    EATESTAssert(CheckMemoryBlock(buffer - GUARDBAND_SIZE, buffer, GUARDBAND_MARKER), "Guardband 0 overwritten");
    EATESTAssert(CheckMemoryBlock(buffer + bufferSize, buffer + bufferSize + GUARDBAND_SIZE, GUARDBAND_MARKER), "Guardband 1 overwritten");

    // Free the padded buffer
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(buffer - GUARDBAND_SIZE, GUARDBAND_SIZE + bufferSize + GUARDBAND_SIZE);
}


/**
Sets all bytes in a given memory block to a given byte value.
*/
bool TestLinearAllocator::SetMemoryBlock(uint8_t *const start, uint8_t *const end, const uint8_t val)
{
    if (end <= start)
    {
        return false;
    }

    memset(start, val, static_cast<size_t>(end - start));
    return true;
}


/**
Checks that all bytes in a given memory block are set to a given byte value.
*/
bool TestLinearAllocator::CheckMemoryBlock(const uint8_t *const start, const uint8_t *const end, const uint8_t val)
{
    if (end <= start)
    {
        return false;
    }

    const uint8_t *p(start);
    while (p < end)
    {
        if (*p != val)
        {
            return false;
        }

        ++p;
    }

    return true;
}


/**
Tests trivial instantiation of the allocator class.
*/
void
TestLinearAllocator::TestInstantiation00()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests allocation.
*/
void
TestLinearAllocator::TestAlloc00()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something that fits easily
    const uint32_t objectSize(512);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "medium", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(object >= buffer, "Allocation outside buffer");
    EATESTAssert(object + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

#ifdef EA_DEBUG
    // In debug builds the block should have been initialized on allocation
    EATESTAssert(CheckMemoryBlock(object, object + objectSize, RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_CLEAR), "Allocated block not initialized");
#endif // EA_DEBUG

    // Write to the allocated memory
    EATESTAssert(SetMemoryBlock(object, object + objectSize, 0xa), "Write to allocated block failed");
    EATESTAssert(CheckMemoryBlock(object, object + objectSize, 0xa), "Write to allocated block not read back correctly");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests successive allocations.
*/
void
TestLinearAllocator::TestAlloc01()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate several things that should all fit
    const uint32_t objectSize(256);

    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "first", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

#ifdef EA_DEBUG
    // In debug builds the block should have been initialized on allocation
    EATESTAssert(CheckMemoryBlock(object0, object0 + objectSize, RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_CLEAR), "Allocated block not initialized");
#endif // EA_DEBUG

    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "second", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

#ifdef EA_DEBUG
    // In debug builds the block should have been initialized on allocation
    EATESTAssert(CheckMemoryBlock(object1, object1 + objectSize, RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_CLEAR), "Allocated block not initialized");
#endif // EA_DEBUG

    uint8_t *const object2(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "third", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object2, "Failed to allocate object");
    EATESTAssert(object2 >= buffer, "Allocation outside buffer");
    EATESTAssert(object2 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

#ifdef EA_DEBUG
    // In debug builds the block should have been initialized on allocation
    EATESTAssert(CheckMemoryBlock(object2, object2 + objectSize, RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_CLEAR), "Allocated block not initialized");
#endif // EA_DEBUG

    uint8_t *const object3(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "fourth", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object3, "Failed to allocate object");
    EATESTAssert(object3 >= buffer, "Allocation outside buffer");
    EATESTAssert(object3 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

#ifdef EA_DEBUG
    // In debug builds the block should have been initialized on allocation
    EATESTAssert(CheckMemoryBlock(object3, object3 + objectSize, RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_CLEAR), "Allocated block not initialized");
#endif // EA_DEBUG

    // Check the allocated objects don't overlap
    EATESTAssert(object0 + objectSize <= object1 || object1 + objectSize <= object0, "Allocated objects overlap");
    EATESTAssert(object0 + objectSize <= object2 || object2 + objectSize <= object0, "Allocated objects overlap");
    EATESTAssert(object0 + objectSize <= object3 || object3 + objectSize <= object0, "Allocated objects overlap");
    EATESTAssert(object1 + objectSize <= object2 || object2 + objectSize <= object1, "Allocated objects overlap");
    EATESTAssert(object1 + objectSize <= object3 || object3 + objectSize <= object1, "Allocated objects overlap");
    EATESTAssert(object2 + objectSize <= object3 || object3 + objectSize <= object2, "Allocated objects overlap");

    // Write to the allocated memory
    EATESTAssert(SetMemoryBlock(object0, object0 + objectSize, 0xa), "Write to allocated block failed");
    EATESTAssert(SetMemoryBlock(object1, object1 + objectSize, 0xb), "Write to allocated block failed");
    EATESTAssert(SetMemoryBlock(object2, object2 + objectSize, 0xc), "Write to allocated block failed");
    EATESTAssert(SetMemoryBlock(object3, object3 + objectSize, 0xd), "Write to allocated block failed");

    EATESTAssert(CheckMemoryBlock(object0, object0 + objectSize, 0xa), "Write to allocated block not read back correctly");
    EATESTAssert(CheckMemoryBlock(object1, object1 + objectSize, 0xb), "Write to allocated block not read back correctly");
    EATESTAssert(CheckMemoryBlock(object2, object2 + objectSize, 0xc), "Write to allocated block not read back correctly");
    EATESTAssert(CheckMemoryBlock(object3, object3 + objectSize, 0xd), "Write to allocated block not read back correctly");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests a single oversize allocation
*/
void
TestLinearAllocator::TestAlloc02()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Try to allocate something bigger than the buffer
    const uint32_t objectSize(1025);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "too large", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object == 0, "Allocated illegal object");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests a single small allocation that overfills the buffer
*/
void
TestLinearAllocator::TestAlloc03()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something large almost filling the buffer
    const uint32_t objectSize0(1020);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "large", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize0 <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Try to allocate something small that overfills the buffer
    const uint32_t objectSize1(8);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "small", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1 == 0, "Allocated illegal object");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests allocatons from both TEMP and PERM heaps
*/
void
TestLinearAllocator::TestAlloc04()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate from PERM heap
    const uint32_t objectSize0(256);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "perm", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize0 <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Allocate from TEMP heap
    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "temp", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize1 <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Check the allocated objects don't overlap
    EATESTAssert(object0 + objectSize0 <= object1 || object1 + objectSize1 <= object0, "Allocated objects overlap");

    // Write to the allocated memory
    EATESTAssert(SetMemoryBlock(object0, object0 + objectSize0, 0xa), "Write to allocated block failed");
    EATESTAssert(SetMemoryBlock(object1, object1 + objectSize1, 0xb), "Write to allocated block failed");
    EATESTAssert(CheckMemoryBlock(object0, object0 + objectSize0, 0xa), "Write to allocated block not read back correctly");
    EATESTAssert(CheckMemoryBlock(object1, object1 + objectSize1, 0xb), "Write to allocated block not read back correctly");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests that allocatons from both TEMP and PERM heaps don't confuse the overflow detection
*/
void
TestLinearAllocator::TestAlloc05()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate from PERM heap
    const uint32_t objectSize0(500);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "one", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize0 <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Allocate from TEMP heap
    const uint32_t objectSize1(500);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "two", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize1 <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Try to allocate something that won't fit
    const uint32_t objectSizeBad(28);
    uint8_t *const objectBad(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSizeBad, "bad", EA::Allocator::MEM_TEMP)));
    EATESTAssert(objectBad == 0, "Allocated illegal object");

    // Allocate something that should fit
    const uint32_t objectSize2(24);
    uint8_t *const object2(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize2, "three", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object2, "Failed to allocate object");
    EATESTAssert(object2 >= buffer, "Allocation outside buffer");
    EATESTAssert(object2 + objectSize2 <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Check the allocated objects don't overlap
    EATESTAssert(object0 + objectSize0 <= object1 || object1 + objectSize1 <= object0, "Allocated objects overlap");
    EATESTAssert(object0 + objectSize0 <= object2 || object2 + objectSize2 <= object0, "Allocated objects overlap");
    EATESTAssert(object1 + objectSize1 <= object2 || object2 + objectSize2 <= object1, "Allocated objects overlap");

    // Write to the allocated memory
    EATESTAssert(SetMemoryBlock(object0, object0 + objectSize0, 0xa), "Write to allocated block failed");
    EATESTAssert(SetMemoryBlock(object1, object1 + objectSize1, 0xb), "Write to allocated block failed");
    EATESTAssert(SetMemoryBlock(object2, object2 + objectSize2, 0xc), "Write to allocated block failed");

    EATESTAssert(CheckMemoryBlock(object0, object0 + objectSize0, 0xa), "Write to allocated block not read back correctly");
    EATESTAssert(CheckMemoryBlock(object1, object1 + objectSize1, 0xb), "Write to allocated block not read back correctly");
    EATESTAssert(CheckMemoryBlock(object2, object2 + objectSize2, 0xc), "Write to allocated block not read back correctly");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests freeing of allocations
*/
void
TestLinearAllocator::TestFree00()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate
    const uint32_t objectSize(100);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_PERM)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(object >= buffer, "Allocation outside buffer");
    EATESTAssert(object + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Free
    linearAllocator.Free(object, objectSize);

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests trivial mark and release
*/
void
TestLinearAllocator::TestMarkRelease00()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Create a mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Allocate
    const uint32_t objectSize(1000);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object0", EA::Allocator::MEM_PERM)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Free
    linearAllocator.Free(object0, objectSize);

    // Release to the mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");

    // Allocate
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object1", EA::Allocator::MEM_PERM)));

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests successive mark and release
*/
void
TestLinearAllocator::TestMarkRelease01()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Create a mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Allocate
    const uint32_t objectSize(1000);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object0", EA::Allocator::MEM_PERM)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Free
    linearAllocator.Free(object0, objectSize);

    // Release to the mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");

    // Create a mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Allocate
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object1", EA::Allocator::MEM_PERM)));

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Free
    linearAllocator.Free(object1, objectSize);

    // Release to the mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");

    // Allocate
    uint8_t *const object2(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object2", EA::Allocator::MEM_PERM)));

    EATESTAssert(object2, "Failed to allocate object");
    EATESTAssert(object2 >= buffer, "Allocation outside buffer");
    EATESTAssert(object2 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Write to the allocated memory
    EATESTAssert(SetMemoryBlock(object2, object2 + objectSize, 0xa), "Write to allocated block failed");
    EATESTAssert(CheckMemoryBlock(object2, object2 + objectSize, 0xa), "Write to allocated block not read back correctly");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests mark and release on separate heaps
*/
void
TestLinearAllocator::TestMarkRelease02()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Create mark points on PERM and TEMP
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_TEMP), "Mark returned false");

    // Allocate on both heaps
    const uint32_t objectSize(500);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object0", EA::Allocator::MEM_PERM)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object1", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Free on both
    linearAllocator.Free(object0);
    linearAllocator.Free(object1);

    // Release to the mark points
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_TEMP), "Release returned false");

    // Allocate again on both heaps
    uint8_t *const object2(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object2", EA::Allocator::MEM_PERM)));

    EATESTAssert(object2, "Failed to allocate object");
    EATESTAssert(object2 >= buffer, "Allocation outside buffer");
    EATESTAssert(object2 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    uint8_t *const object3(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object3", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object3, "Failed to allocate object");
    EATESTAssert(object3 >= buffer, "Allocation outside buffer");
    EATESTAssert(object3 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Check the allocated objects don't overlap
    EATESTAssert(object2 + objectSize <= object3 || object3 + objectSize <= object2, "Allocated objects overlap");

    // Write to the allocated memory
    EATESTAssert(SetMemoryBlock(object2, object2 + objectSize, 0xa), "Write to allocated block failed");
    EATESTAssert(SetMemoryBlock(object3, object3 + objectSize, 0xb), "Write to allocated block failed");

    EATESTAssert(CheckMemoryBlock(object2, object2 + objectSize, 0xa), "Write to allocated block not read back correctly");
    EATESTAssert(CheckMemoryBlock(object3, object3 + objectSize, 0xb), "Write to allocated block not read back correctly");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests that release returns to the same allocation point, for PERM
*/
void
TestLinearAllocator::TestMarkRelease03()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Create a mark point on PERM
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Allocate
    const uint32_t objectSize(500);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object0", EA::Allocator::MEM_PERM)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Release back to the mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");

    // Create another mark point (otherwise it's not fair)
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Allocate again
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object1", EA::Allocator::MEM_PERM)));

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Check the allocated objects are at the same location
    EATESTAssert(object0 == object1, "Allocated objects differ");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests that release returns to the same allocation point, for TEMP
*/
void
TestLinearAllocator::TestMarkRelease04()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Create a mark point on TEMP
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_TEMP), "Mark returned false");

    // Allocate
    const uint32_t objectSize(500);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object0", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(object0 >= buffer, "Allocation outside buffer");
    EATESTAssert(object0 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Release back to the mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_TEMP), "Release returned false");

    // Create another mark point (otherwise it's not fair)
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_TEMP), "Mark returned false");

    // Allocate again
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object1", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(object1 >= buffer, "Allocation outside buffer");
    EATESTAssert(object1 + objectSize <= buffer + BUFFER_SIZE, "Allocation outside buffer");

    // Check the allocated objects are at the same location
    EATESTAssert(object0 == object1, "Allocated objects differ");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests handling of release with no mark, on PERM
*/
void
TestLinearAllocator::TestMarkRelease05()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Release without marking, and again to be sure
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM) == false, "Release without mark returned true");
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM) == false, "Release without mark returned true");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests handling of release with no mark, on TEMP
*/
void
TestLinearAllocator::TestMarkRelease06()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Release without marking, and again to be sure
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_TEMP) == false, "Release without mark returned true");
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_TEMP) == false, "Release without mark returned true");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests handling of multiple non-trivial mark points, on PERM
*/
void
TestLinearAllocator::TestMarkRelease07()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Initial mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Several allocations
    const uint32_t objectSize(16);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object0", EA::Allocator::MEM_PERM)));
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object1", EA::Allocator::MEM_PERM)));

    // Second mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Several more allocations
    uint8_t *const object2(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object2", EA::Allocator::MEM_PERM)));
    uint8_t *const object3(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object3", EA::Allocator::MEM_PERM)));

    // Release second mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");

    // Re-mark second mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Several more allocations after the new second mark point
    uint8_t *const object4(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object4", EA::Allocator::MEM_PERM)));
    uint8_t *const object5(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object5", EA::Allocator::MEM_PERM)));

    // These check that we get the same allocations again after a release and re-mark
    EATESTAssert(object2 == object4, "Allocation placed incorrectly after mark and release");
    EATESTAssert(object3 == object5, "Allocation placed incorrectly after mark and release");

    // Release second mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");

    // Several more allocations on the new first mark point
    uint8_t *const object6(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object6", EA::Allocator::MEM_PERM)));
    uint8_t *const object7(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object7", EA::Allocator::MEM_PERM)));

    EATESTAssert(object6, "Allocation failed");
    EATESTAssert(object7, "Allocation failed");

    // Release first mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_PERM), "Release returned false");

    // Re-mark first mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_PERM), "Mark returned false");

    // Several more allocations after the new first mark point
    uint8_t *const object8(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object8", EA::Allocator::MEM_PERM)));
    uint8_t *const object9(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object9", EA::Allocator::MEM_PERM)));

    // These check that we get the same allocations again after a release and re-mark
    EATESTAssert(object0 == object8, "Allocation placed incorrectly after mark and release");
    EATESTAssert(object1 == object9, "Allocation placed incorrectly after mark and release");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests handling of multiple non-trivial mark points, on TEMP
*/
void
TestLinearAllocator::TestMarkRelease08()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Initial mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_TEMP), "Mark returned false");

    // Several allocations
    const uint32_t objectSize(16);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object0", EA::Allocator::MEM_TEMP)));
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object1", EA::Allocator::MEM_TEMP)));

    // Second mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_TEMP), "Mark returned false");

    // Several more allocations
    uint8_t *const object2(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object2", EA::Allocator::MEM_TEMP)));
    uint8_t *const object3(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object3", EA::Allocator::MEM_TEMP)));

    // Release second mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_TEMP), "Release returned false");

    // Re-mark second mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_TEMP), "Mark returned false");

    // Several more allocations after the new second mark point
    uint8_t *const object4(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object4", EA::Allocator::MEM_TEMP)));
    uint8_t *const object5(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object5", EA::Allocator::MEM_TEMP)));

    // These check that we get the same allocations again after a release and re-mark
    EATESTAssert(object2 == object4, "Allocation placed incorrectly after mark and release");
    EATESTAssert(object3 == object5, "Allocation placed incorrectly after mark and release");

    // Release second mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_TEMP), "Release returned false");

    // Several more allocations on the new first mark point
    uint8_t *const object6(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object6", EA::Allocator::MEM_TEMP)));
    uint8_t *const object7(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object7", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object6, "Allocation failed");
    EATESTAssert(object7, "Allocation failed");

    // Release first mark point
    EATESTAssert(linearAllocator.Release(EA::Allocator::MEM_TEMP), "Release returned false");

    // Re-mark first mark point
    EATESTAssert(linearAllocator.Mark(EA::Allocator::MEM_TEMP), "Mark returned false");

    // Several more allocations after the new first mark point
    uint8_t *const object8(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object8", EA::Allocator::MEM_TEMP)));
    uint8_t *const object9(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object9", EA::Allocator::MEM_TEMP)));

    // These check that we get the same allocations again after a release and re-mark
    EATESTAssert(object0 == object8, "Allocation placed incorrectly after mark and release");
    EATESTAssert(object1 == object9, "Allocation placed incorrectly after mark and release");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests default alignment of allocated objects, PERM heap
*/
void
TestLinearAllocator::TestAlignment00()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something with default 4-byte alignment
    const uint32_t objectSize(64);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_PERM)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object, 4), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests default alignment of allocated objects, TEMP heap
*/
void
TestLinearAllocator::TestAlignment01()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something with default 4-byte alignment
    const uint32_t objectSize(64);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object, 4), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests explicit trivial alignment of allocated objects, PERM heap
*/
void
TestLinearAllocator::TestAlignment02()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something with explicit 4-byte alignment
    const uint32_t objectSize(64);
    const uint32_t objectAlignment(4);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_PERM, objectAlignment)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object, objectAlignment), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests explicit trivial alignment of allocated objects, TEMP heap
*/
void
TestLinearAllocator::TestAlignment03()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something with explicit 4-byte alignment
    const uint32_t objectSize(64);
    const uint32_t objectAlignment(4);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_TEMP, objectAlignment)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object, objectAlignment), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests explicit non-trivial alignment of allocated objects, PERM heap
*/
void
TestLinearAllocator::TestAlignment04()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something with explicit non-default alignment
    const uint32_t objectSize(64);
    const uint32_t objectAlignment(8);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_PERM, objectAlignment)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object, objectAlignment), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests explicit non-trivial alignment of allocated objects, TEMP heap
*/
void
TestLinearAllocator::TestAlignment05()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something with explicit non-default alignment
    const uint32_t objectSize(64);
    const uint32_t objectAlignment(8);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_TEMP, objectAlignment)));

    EATESTAssert(object, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object, objectAlignment), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests explicit alignment of objects allocated after objects of inconvenient sizes, PERM heap
*/
void
TestLinearAllocator::TestAlignment06()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something inconveniently sized, with inconvenient explicit alignment
    const uint32_t objectSize0(4);
    const uint32_t objectAlignment0(16);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "shim", EA::Allocator::MEM_PERM, objectAlignment0)));

    // Allocate something inconveniently sized, with explicit alignment
    const uint32_t objectSize1(4);
    const uint32_t objectAlignment1(16);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_PERM, objectAlignment1)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object0, objectAlignment0), "Allocated object is not aligned");

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object1, objectAlignment1), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests explicit alignment of objects allocated after objects of inconvenient sizes, TEMP heap
*/
void
TestLinearAllocator::TestAlignment07()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something inconveniently sized, with inconvenient explicit alignment
    const uint32_t objectSize0(4);
    const uint32_t objectAlignment0(16);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "shim", EA::Allocator::MEM_TEMP, objectAlignment0)));

    // Allocate something inconveniently sized, with explicit alignment
    const uint32_t objectSize1(4);
    const uint32_t objectAlignment1(16);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP, objectAlignment1)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object0, objectAlignment0), "Allocated object is not aligned");

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object1, objectAlignment1), "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests default alignment of objects allocated after objects of inconvenient sizes, PERM heap
*/
void
TestLinearAllocator::TestAlignment08()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something inconveniently sized, with inconvenient explicit alignment
    const uint32_t objectSize0(4);
    const uint32_t objectAlignment0(16);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "shim", EA::Allocator::MEM_PERM, objectAlignment0)));

    // Allocate something inconveniently sized, with default alignment
    const uint32_t objectSize1(4);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_PERM)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object0, objectAlignment0), "Allocated object is not aligned");

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object1, 4), "Allocated object is not aligned");

    // This checks that the allocator hasn't needlessly padded to 16-byte alignment, wasting space
    EATESTAssert(linearAllocator.IsPointerAligned(object1, 16) == false, "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Tests default alignment of objects allocated after objects of inconvenient sizes, TEMP heap
*/
void
TestLinearAllocator::TestAlignment09()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Allocate something inconveniently sized, with inconvenient explicit alignment
    const uint32_t objectSize0(4);
    const uint32_t objectAlignment0(16);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "shim", EA::Allocator::MEM_TEMP, objectAlignment0)));

    // Allocate something inconveniently sized, with default alignment
    const uint32_t objectSize1(4);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP)));

    EATESTAssert(object0, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object0, objectAlignment0), "Allocated object is not aligned");

    EATESTAssert(object1, "Failed to allocate object");
    EATESTAssert(linearAllocator.IsPointerAligned(object1, 4), "Allocated object is not aligned");

    // This checks that the allocator hasn't needlessly padded to 16-byte alignment, wasting space
    EATESTAssert(linearAllocator.IsPointerAligned(object1, 16) == false, "Allocated object is not aligned");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Trivial test of total memory usage metric, temp heap
*/
void
TestLinearAllocator::TestTotal00()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    // Allocate something
    const uint32_t objectSize(512);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize + sizeof(uint8_t*), "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) == 0, "GetMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) == 0, "GetMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Trivial test of total memory usage metric, perm heap
*/
void
TestLinearAllocator::TestTotal01()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    // Allocate something
    const uint32_t objectSize(512);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize + sizeof(uint8_t*), "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) == 0, "GetMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) == 0, "GetMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Trivial test of total memory usage metric, both heaps at once
*/
void
TestLinearAllocator::TestTotal02()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    // Allocate things on both heaps
    const uint32_t objectSize0(256);
    const uint32_t objectSize1(128);

    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*), "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize1, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize1 + sizeof(uint8_t*), "GetMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object0);
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) == 0, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) == 0, "GetMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Total memory usage metric, multiple allocations, temp heap
*/
void
TestLinearAllocator::TestTotal03()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object0, "Failed to allocate object");

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0 + objectSize1, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + objectSize1 + sizeof(uint8_t*), "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) == 0, "GetMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object0);
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) == 0, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) == 0, "GetMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Total memory usage metric, multiple allocations, perm heap
*/
void
TestLinearAllocator::TestTotal04()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0 + objectSize1, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + objectSize1 + sizeof(uint8_t*), "GetMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object0);
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) == 0, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) == 0, "GetMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Total memory usage metric, multiple mark and release, temp heap
*/
void
TestLinearAllocator::TestTotal05()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // First mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object0, "Failed to allocate object");

    // Second mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0 + objectSize1, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetMemoryUsed incorrect");

    // Free and release second mark
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + sizeof(uint8_t*), "GetMemoryUsed incorrect");

    // Free and release first mark
    linearAllocator.Free(object0);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_TEMP) == 0, "GetMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Total memory usage metric, multiple mark and release, perm heap
*/
void
TestLinearAllocator::TestTotal06()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // First mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");

    // Second mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0 + objectSize1, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetMemoryUsed incorrect");

    // Free and release second mark
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0, "GetMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*), "GetMemoryUsed incorrect");

    // Free and release first mark
    linearAllocator.Free(object0);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetMemoryUsed(EA::Allocator::MEM_PERM) == 0, "GetMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Trivial test of total memory usage metric, temp heap
*/
void
TestLinearAllocator::TestPeak00()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    // Allocate something
    const uint32_t objectSize(512);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Trivial test of peak memory usage metric, perm heap
*/
void
TestLinearAllocator::TestPeak01()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    // Allocate something
    const uint32_t objectSize(512);
    uint8_t *const object(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Trivial test of total memory usage metric, both heaps at once
*/
void
TestLinearAllocator::TestPeak02()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    // Allocate something
    const uint32_t objectSize0(256);
    const uint32_t objectSize1(128);

    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object0);
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Peak memory usage metric, multiple allocations, temp heap
*/
void
TestLinearAllocator::TestPeak03()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object0, "Failed to allocate object");

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object0);
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Peak memory usage metric, multiple allocations, perm heap
*/
void
TestLinearAllocator::TestPeak04()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // Mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release
    linearAllocator.Free(object0);
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Peak memory usage metric, multiple mark and release, temp heap
*/
void
TestLinearAllocator::TestPeak05()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // First mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object0, "Failed to allocate object");

    // Second mark
    linearAllocator.Mark(EA::Allocator::MEM_TEMP);

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_TEMP)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release second mark
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release first mark
    linearAllocator.Free(object0);
    linearAllocator.Release(EA::Allocator::MEM_TEMP);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_TEMP) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}


/**
Peak memory usage metric, multiple mark and release, perm heap
*/
void
TestLinearAllocator::TestPeak06()
{
    // Allocate the buffer
    uint8_t *const buffer(AllocateBuffer(BUFFER_SIZE));
    EATESTAssert(buffer, "Failed to allocate test buffer");

    // Instantiate an allocator around the buffer
    rw::collision::meshbuilder::detail::LinearAllocator linearAllocator(buffer, BUFFER_SIZE);

    // First mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    // Allocate two things
    const uint32_t objectSize0(128);
    uint8_t *const object0(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize0, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object0, "Failed to allocate object");

    // Second mark
    linearAllocator.Mark(EA::Allocator::MEM_PERM);

    const uint32_t objectSize1(256);
    uint8_t *const object1(reinterpret_cast<uint8_t *>(linearAllocator.Alloc(objectSize1, "object", EA::Allocator::MEM_PERM)));
    EATESTAssert(object1, "Failed to allocate object");

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release second mark
    linearAllocator.Free(object1);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free and release first mark
    linearAllocator.Free(object0);
    linearAllocator.Release(EA::Allocator::MEM_PERM);

    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) >= objectSize0 + objectSize1, "GetPeakMemoryUsed incorrect");
    EATESTAssert(linearAllocator.GetPeakMemoryUsed(EA::Allocator::MEM_PERM) <= objectSize0 + sizeof(uint8_t*) + objectSize1 + sizeof(uint8_t*), "GetPeakMemoryUsed incorrect");

    // Free the buffer
    FreeBuffer(buffer, BUFFER_SIZE);
}





