// (c) Electronic Arts. All Rights Reserved.
#include <stdlib.h>

#include <EABase/eabase.h>
#include <coreallocator/icoreallocator_interface.h>
#include <PPMalloc/EAGeneralAllocatorDebug.h>


namespace EA
{
namespace Allocator
{

class ExampleAllocator : public EA::Allocator::ICoreAllocator
{
public:
    ExampleAllocator() : mHeap(mHeapBuffer, mHeapSize, false)
    {
    }

    virtual ~ExampleAllocator()
    {
    }

    virtual void* Alloc(size_t size, const char * /* name */, unsigned int  /* flags */, unsigned int align, unsigned int alignOffset)
    {
        return mHeap.MallocAligned(size, align, alignOffset);
    }

    virtual void* Alloc(size_t size, const char * /* name */, unsigned int  /* flags */)
    {
        return mHeap.Malloc(size);
    }

    virtual void Free(void *block, size_t)
    {
        mHeap.Free(block);
    }

private:

    static const int mHeapSize = 8 * 1024 * 1024;
    EA::Allocator::GeneralAllocatorDebug mHeap;
    EA_ALIGNED(uint8_t, mHeapBuffer[mHeapSize], 16);
};


// ***********************************************************************************************************
/// The user is responsible for providing an implementation of
/// EA::Allocator::ICoreAllocator::GetDefaultAllocator in their
/// applications' codebase.  The package contains
/// implementations of operator new and delete which call this
/// function.  Library authors should not call this function.
ICoreAllocator* ICoreAllocator::GetDefaultAllocator()
{
    static ExampleAllocator s_UnitTestAllocator;
    return &s_UnitTestAllocator;
}


} // namespace EA
} // namespace Allocator

// We need to provide definitions of these global new operators in order to be
// able to use EASTL without providing custom allocators
// NOTE: CodeWarrior requires all functions to be prototyped or static.
void* operator new[](size_t size, const char* /*pName*/, int /*flags*/, unsigned /*debugFlags*/, const char* /*file*/, int /*line*/);
void* operator new[](size_t size, const char* /*pName*/, int /*flags*/, unsigned /*debugFlags*/, const char* /*file*/, int /*line*/)
{
    // We'll just ignore the flags and alignment requests!
    return new uint8_t[size];
}


void* operator new[](size_t size, size_t /*alignment*/, size_t /*alignmentOffset*/, const char* /*pName*/, int /*flags*/, unsigned /*debugFlags*/, const char* /*file*/, int /*line*/);
void* operator new[](size_t size, size_t /*alignment*/, size_t /*alignmentOffset*/, const char* /*pName*/, int /*flags*/, unsigned /*debugFlags*/, const char* /*file*/, int /*line*/)
{
    // We'll just ignore the flags and alignment requests!
    return new uint8_t[size];
}
