// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_GENERALALLOCATOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_GENERALALLOCATOR_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/iallocator.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
General implementation of the IAllocator interface.
This implementation is a trivial wrapper around ICoreAllocator.
Calls to the Mark and Release methods do nothing and are ignored.
Instead, memory is freed one the fly by calls to Free in a general way.
*/
class GeneralAllocator : public IAllocator
{

public:

    typedef IAllocator Base;

    /**
    Default constructor.
    Constructs an uninitialized allocator.
    */
    GeneralAllocator() :
      m_allocator(0)

    {
    }

    /**
    Constructor.
    Constructs an initialized allocator around a provided ICoreAllocator.
    */
    explicit GeneralAllocator(EA::Allocator::ICoreAllocator * const allocator) :
      m_allocator(allocator)
    {
    }

    /**
    Destructor
    */
    virtual ~GeneralAllocator()
    {
        m_allocator = 0;
    }

    //
    // Mark/Release methods
    //

    /**
    Does nothing, in this implementation.
    */
    virtual bool Mark(const unsigned int /*flags*/)
    {
        // Do nothing
        return true;
    }

    /**
    Does nothing, in this implementation.
    */
    virtual bool Release(const unsigned int /*flags*/)
    {
        // Do nothing
        return true;
    }

    /**
    Returns a lower bound on the size of the biggest single contiguous block that can be allocated,
    given the alignment requirements of the block.

    \note Although this is a guaranteed lower bound for a single allocated block, multiple successive
    allocations may not be able to allocate this much memory in total. This is because a small
    but non-zero amount of memory is typically used internally for alignment and mark points.
    The amount of memory lost in this way will depend on the future usage of the allocator,
    which is generally not known at the time of this call.
    */
    uint32_t LargestAllocatableSize(const unsigned int /*flags*/, const uint32_t /*alignment*/) const
    {
        // TODO: We currently just return a large number here on the assumption that
        // in offline cases memory is effectively unbounded.
        return 64 * 1024 * 1024;
    }

    //
    // ICoreAllocator implementation
    //

    /**
    */
    virtual void *Alloc(size_t size, const char *name, unsigned int flags)
    {
        void *const memory(m_allocator->Alloc(size, name, flags));
        return memory;
    }

    /**
    */
    virtual void *Alloc(size_t size, const char *name, unsigned int flags, unsigned int align, unsigned int alignOffset = 0)
    {
        return m_allocator->Alloc(size, name, flags, align, alignOffset);
    }

    /**
    */
    virtual void Free(void *block, size_t size = 0)
    {
        m_allocator->Free(block, size);
    }

private:

    EA::Allocator::ICoreAllocator *m_allocator;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_GENERALALLOCATOR_H
