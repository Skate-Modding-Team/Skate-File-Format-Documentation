// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_LINEARALLOCATOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_LINEARALLOCATOR_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU


// XBOX headers generate warnings
#if defined _MSC_VER
#pragma warning(push, 0)
#endif

#include <new>
#include <string.h> // For memset

#if defined _MSC_VER
#pragma warning(pop)
#endif


#include <rw/collision/meshbuilder/detail/iallocator.h>


#define RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_CLEAR (0xc)
#define RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_FREE (0xe)


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
Linear implementation of the IAllocator interface.
This implementation is a linear allocator and supports the Mark/Release
semantics of the IAllocator interface non-trivially.

Calls to Free do nothing and are ignored. Instead, memory is freed by
calls to Release, which frees all memory allocated subquent to the most
recent call to Mark.

This implementation actually exposes two heaps, one allocated forward
from the bottom of the buffer and the other backward from the top.
The heap to be used is indicated on a per-call basis by flag values
defined in EA::Allocator::ICoreAllocator.
*/
class LinearAllocator : public IAllocator
{

private:

    struct MarkRecord
    {
        MarkRecord() : m_previousMark(0)
        {
        }

        MarkRecord *m_previousMark;
    };

public:

    typedef IAllocator Base;

    /**
    Returns true if the given pointer is aligned with the given alignment.
    */
    static inline bool IsPointerAligned(const void *const ptr, const uint32_t alignment)
    {
        return ((uintptr_t)ptr % (uintptr_t)alignment == 0);
    }

    /**
    Default constructor.
    Constructs an uninitialized allocator.
    */
    LinearAllocator() :
      m_start(0),
      m_end(0),
      m_lowPosition(0),
      m_highPosition(0),
      m_lowMark(0),
      m_highMark(0),
      m_lowPeak(0),
      m_highPeak(0),
      m_minFree(0)
    {
    }

    /**
    Constructor.
    Constructs an initialized allocator around a provided buffer.
    */
    LinearAllocator(void * const buffer, const uint32_t size) :
      m_start(reinterpret_cast<uint8_t *>(buffer)),
      m_end(m_start + size),
      m_lowPosition(m_start),
      m_highPosition(m_end),
      m_lowMark(0),
      m_highMark(0),
      m_lowPeak(m_lowPosition),
      m_highPeak(m_highPosition),
      m_minFree(static_cast<uint32_t>(m_highPosition - m_lowPosition))
    {
    }

    /**
    Destructor
    */
    virtual ~LinearAllocator()
    {
        m_start = 0;
        m_end = 0;
        m_lowPosition = 0;
        m_highPosition = 0;
        m_lowMark = 0;
        m_highMark = 0;
        m_lowPeak = 0;
        m_highPeak = 0;
        m_minFree = 0;
    }

    //
    // IAllocator methods
    //

    /**
    Creates a mark point at the current location.
    */
    virtual bool Mark(const unsigned int flags)
    {
        EA_ASSERT(m_lowMark == 0 || reinterpret_cast<uint8_t *>(m_lowMark) >= m_start);
        EA_ASSERT(m_lowMark == 0 || reinterpret_cast<uint8_t *>(m_lowMark) + sizeof(MarkRecord) <= m_end);
        EA_ASSERT(m_highMark == 0 || reinterpret_cast<uint8_t *>(m_highMark) >= m_start);
        EA_ASSERT(m_highMark == 0 || reinterpret_cast<uint8_t *>(m_highMark) + sizeof(MarkRecord) <= m_end);

        // Allocate a new mark record on the indicated heap (TEMP or PERM)
        void *const markRecordMemory = Alloc(sizeof(MarkRecord), "MarkRecord", flags);
        if (!markRecordMemory)
        {
            // Failed to allocate mark record
            return false;
        }

        // We're currently assuming that allocation sizes will naturally be multiples of four bytes
        // If that isn't true then we could force all allocations to at least four-byte alignment
        EA_ASSERT(IsPointerAligned(markRecordMemory, 4));
        MarkRecord *const record = new (markRecordMemory) MarkRecord;

        EA_ASSERT(record);
        EA_ASSERT(IsPointerAligned(record, 4));

        // Permanent allocations are allocated forwards from the low end of the heap,
        // while temporary allocations are allocated backwards from the high end
        if (flags & EA::Allocator::MEM_PERM)
        {
            // Point the new mark record at the previous one and remember the new one instead
            record->m_previousMark = m_highMark;
            m_highMark = record;
        }
        else
        {
            record->m_previousMark = m_lowMark;
            m_lowMark = record;
        }

        return true;
    }

    /**
    Frees all allocations made subsequent to the most recent call to \ref Mark.
    */
    virtual bool Release(const unsigned int flags)
    {
        if (flags & EA::Allocator::MEM_PERM)
        {
            if (m_highMark == 0)
            {
                // Call to Release with no corresponding call to Mark?
                return false;
            }

            EA_ASSERT(reinterpret_cast<uint8_t *>(m_highMark) >= m_start);
            EA_ASSERT(reinterpret_cast<uint8_t *>(m_highMark) + sizeof(MarkRecord) <= m_end);

            // Read this before we clear the block!
            MarkRecord *const newMark(m_highMark->m_previousMark);

#ifdef EA_DEBUG
            // Clear the released block to known marker values.
            ClearMemoryBlock(m_highPosition, reinterpret_cast<uint8_t *>(m_highMark) + sizeof(MarkRecord), RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_FREE);
#endif // EA_DEBUG

            // Set the high position to the most recent mark point and forget the mark
            m_highPosition = reinterpret_cast<uint8_t *>(m_highMark) + sizeof(MarkRecord);
            m_highMark = newMark;

            EA_ASSERT(m_highPosition >= m_start);
            EA_ASSERT(m_highPosition <= m_end);

            EA_ASSERT(m_highMark == 0 || reinterpret_cast<uint8_t *>(m_highMark) >= m_start);
            EA_ASSERT(m_highMark == 0 || reinterpret_cast<uint8_t *>(m_highMark) + sizeof(MarkRecord) <= m_end);
        }
        else
        {
            if (m_lowMark == 0)
            {
                // Call to Release with no corresponding call to Mark?
                return false;
            }

            EA_ASSERT(reinterpret_cast<uint8_t *>(m_lowMark) >= m_start);
            EA_ASSERT(reinterpret_cast<uint8_t *>(m_lowMark) + sizeof(MarkRecord) <= m_end);

            // Read this before we clear the block!
            MarkRecord *const newMark(m_lowMark->m_previousMark);

#ifdef EA_DEBUG
            // Clear the released block to known marker values.
            ClearMemoryBlock(reinterpret_cast<uint8_t *>(m_lowMark), m_lowPosition, RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_FREE);
#endif // EA_DEBUG

            // Set the low position to the most recent mark point and forget the mark
            m_lowPosition = reinterpret_cast<uint8_t *>(m_lowMark);
            m_lowMark = newMark;

            EA_ASSERT(m_lowPosition >= m_start);
            EA_ASSERT(m_lowPosition <= m_end);

            EA_ASSERT(m_lowMark == 0 || reinterpret_cast<uint8_t *>(m_lowMark) >= m_start);
            EA_ASSERT(m_lowMark == 0 || reinterpret_cast<uint8_t *>(m_lowMark) + sizeof(MarkRecord) <= m_end);
        }

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
    virtual uint32_t LargestAllocatableSize(const unsigned int /*flags*/, const uint32_t alignment) const
    {
        uint8_t *allocation(m_lowPosition);
        AlignPointerForward(allocation, alignment);

        return static_cast<uint32_t>(m_highPosition - allocation);
    }

    //
    // ICoreAllocator implementation
    //

    /**
    */
    virtual void *Alloc(size_t size, const char *name, unsigned int flags)
    {
        // Forward to the more general method, with a default minimum alignment of 4 bytes.
        return Alloc(size, name, flags, 4, 0);
    }

    /**
    \note alignOffset is currently ignored!
    */
    virtual void *Alloc(size_t size, const char * /*name*/, unsigned int flags, unsigned int align, unsigned int /*alignOffset*/ = 0)
    {
        uint8_t *allocation(0);

        EA_ASSERT(size < 0x0f0000000);
        EA_ASSERT(m_lowPosition >= m_start);
        EA_ASSERT(m_lowPosition <= m_end);
        EA_ASSERT(m_highPosition >= m_start);
        EA_ASSERT(m_highPosition <= m_end);

        if (flags & EA::Allocator::MEM_PERM)
        {
            allocation = m_highPosition - size;
            AlignPointerBackward(allocation, align);

            if (allocation < m_lowPosition)
            {
                // Out of memory in buffer
                return 0;
            }

            m_highPosition = allocation;

            // Update the high heap peak point
            if (m_highPosition < m_highPeak)
            {
                m_highPeak = m_highPosition;
            }
        }
        else
        {
            allocation = m_lowPosition;
            AlignPointerForward(allocation, align);

            if (allocation + size > m_highPosition)
            {
                // Out of memory in buffer
                return 0;
            }

            m_lowPosition = allocation + size;

            // Update the low heap peak point
            if (m_lowPosition > m_lowPeak)
            {
                m_lowPeak = m_lowPosition;
            }
        }

        EA_ASSERT(m_lowPosition >= m_start);
        EA_ASSERT(m_lowPosition <= m_end);
        EA_ASSERT(m_highPosition >= m_start);
        EA_ASSERT(m_highPosition <= m_end);
        EA_ASSERT(allocation >= m_start);
        EA_ASSERT(allocation <= m_end);
        EA_ASSERT(allocation + size >= m_start);
        EA_ASSERT(allocation + size <= m_end);

        // Update free memory low point
        const uint32_t freeSize(static_cast<uint32_t>(m_highPosition - m_lowPosition));
        if (freeSize < m_minFree)
        {
            m_minFree = freeSize;
        }

#ifdef EA_DEBUG
        // Clear the allocated block to known marker values.
        ClearMemoryBlock(allocation, allocation + size, RWCOLLISION_VOLUMES_LINEAR_ALLOCATOR_BYTE_CLEAR);
#endif // EA_DEBUG

        return allocation;
    }

    /**
    Does nothing, in this implementation.
    \note This method should still be called, in case the allocator in use is a general allocator.
    */
    virtual void Free(void * /*block*/, size_t /*size*/ = 0)
    {
        // Do nothing; frees are handled in blocks by Release
    }

    //
    // Local methods
    //

    /**
    Returns the current memory usage for the given heap.
    \note The memory usage includes any memory used internally for alignment padding and allocation tracking.
    */
    uint32_t GetMemoryUsed(const unsigned int flags) const
    {
        if (flags & EA::Allocator::MEM_PERM)
        {
            EA_ASSERT(m_highPosition <= m_end);
            return static_cast<uint32_t>(m_end - m_highPosition);
        }
        else
        {
            EA_ASSERT(m_lowPosition >= m_start);
            return static_cast<uint32_t>(m_lowPosition - m_start);
        }
    }

    /**
    Returns the current total memory usage for both heaps.
    \note The total memory usage includes any memory used internally for alignment padding and allocation tracking.
    */
    uint32_t GetTotalMemoryUsed() const
    {
        const uint32_t blockSize(static_cast<uint32_t>(m_end - m_start));
        const uint32_t freeSize(static_cast<uint32_t>(m_highPosition - m_lowPosition));
        const uint32_t totalUsedSize(blockSize - freeSize);
        return totalUsedSize;
    }

    /**
    Returns the maximum memory usage seen over the lifetime of the allocator, for the given heap.
    \note The peak memory usage may include memory used internally for alignment padding and allocation tracking.
    */
    uint32_t GetPeakMemoryUsed(const unsigned int flags) const
    {
        if (flags & EA::Allocator::MEM_PERM)
        {
            EA_ASSERT(m_highPeak <= m_end);
            return static_cast<uint32_t>(m_end - m_highPeak);
        }
        else
        {
            EA_ASSERT(m_lowPeak >= m_start);
            return static_cast<uint32_t>(m_lowPeak - m_start);
        }
    }

    /**
    Returns the maximum total memory usage seen over the lifetime of the allocator, for both heaps.
    \note The peak memory usage may include memory used internally for alignment padding and allocation tracking.
    */
    uint32_t GetPeakTotalMemoryUsed() const
    {
        const uint32_t blockSize(static_cast<uint32_t>(m_end - m_start));
        const uint32_t peakUsedSize(blockSize - m_minFree);
        return peakUsedSize;
    }

private:

    /**
    Aligns a pointer to a higher address.
    \param ptr pointer to align.
    \param alignment target alignment.
    */
    static inline void AlignPointerForward(uint8_t *&ptr, const uint32_t alignment)
    {
        // This assumes that the alignment is a power of two!
        EA_ASSERT((alignment & ~(alignment - 1)) == alignment);
        ptr = (uint8_t *)(((uintptr_t)ptr + ((uintptr_t)alignment - 1)) & ~((uintptr_t)alignment - 1));
        EA_ASSERT(IsPointerAligned(ptr, alignment));
    }

    /**
    Aligns a pointer to a lower address.
    \param ptr pointer to align.
    \param alignment target alignment.
    */
    static inline void AlignPointerBackward(uint8_t *&ptr, const uint32_t alignment)
    {
        // Just remove any bits less significant than the alignment value bit
        // This assumes that the alignment is a power of two!
        EA_ASSERT((alignment & ~(alignment - 1)) == alignment);
        ptr = (uint8_t *)((uintptr_t)ptr & ~((uintptr_t)alignment - 1));
        EA_ASSERT(IsPointerAligned(ptr, alignment));
    }

    /**
    Clears a block of memory to a given byte value.
    */
    static inline void ClearMemoryBlock(uint8_t *const start, uint8_t *const end, const uint8_t val)
    {
        const intptr_t size((intptr_t)end - (intptr_t)start);
        if (size > 0)
        {
            memset(start, val, static_cast<size_t>(size));
        }
    }

    uint8_t *m_start;
    uint8_t *m_end;
    uint8_t *m_lowPosition;
    uint8_t *m_highPosition;
    MarkRecord *m_lowMark;
    MarkRecord *m_highMark;
    uint8_t *m_lowPeak;
    uint8_t *m_highPeak;
    uint32_t m_minFree;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_LINEARALLOCATOR_H
