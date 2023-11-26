// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_IALLOCATOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_IALLOCATOR_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <coreallocator/icoreallocator_interface.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
Interface that extends ICoreAllocator with Mark and Release API.
The API notionally describes a linear allocator with mark-release semantics.
In this scenario the calls to Free typically do nothing and are ignored.
However implementations are free to simply provide general allocation
instead, in which case Mark and Free are trivial and Free works as normal.
*/
class IAllocator : public EA::Allocator::ICoreAllocator
{

public:

    typedef EA::Allocator::ICoreAllocator Base;

    /**
    Default constructor.
    */
    IAllocator()
    {
    }

    /**
    Destructor
    */
    virtual ~IAllocator()
    {
    }

    //
    // This interface adds these custom methods to ICoreAllocator
    //

    /**
    Creates a mark point at the current location, on the heap indicated by the flags.
    */
    virtual bool Mark(const unsigned int flags) = 0;

    /**
    Frees all allocations allocated subsequent to the most recently added mark point.
    In a linear allocator, the Free method does nothing and freeing is deferred until Release time.
    \note Free should still be called as normal, in case the allocator in use is a general allocator.
    */
    virtual bool Release(const unsigned int flags) = 0;

    /**
    Returns a lower bound on the size of the biggest single contiguous block that can be allocated,
    given the alignment requirements of the block.

    \note Although this is a guaranteed lower bound for a single allocated block, multiple successive
    allocations may not be able to allocate this much memory in total. This is because a small
    but non-zero amount of memory is typically used internally for alignment and mark points.
    The amount of memory lost in this way will depend on the future usage of the allocator,
    which is generally not known at the time of this call.
    */
    virtual uint32_t LargestAllocatableSize(const unsigned int flags, const uint32_t alignment = 4) const = 0;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_IALLOCATOR_H
