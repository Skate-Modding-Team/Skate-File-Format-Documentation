// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_EASTLBLOCKALLOCATOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_EASTLBLOCKALLOCATOR_H


/*************************************************************************************************************

File: eastlblockallocator.h

Purpose: EASTLBlockAllocator class.

A block allocator which implements the eastl container interface.

The allocator can be initialized with a block of memory or, given an IHeapAllocator it will attempt to
allocate memory using the given heap allocator.
*/


#include "rw/collision/common.h"


#if !defined EA_PLATFORM_PS3_SPU

#include "coreallocator/icoreallocator_interface.h"


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


class EASTLBlockAllocator
{
public:

    /**
    Default Constructor.
    */
    EASTLBlockAllocator(const char* /*pName*/)
        : m_base(NULL)
        , m_current(NULL)
        , m_limit(NULL)
        , m_peak(0)
        , m_allocator(NULL)
        , m_isValid(false)
    {
    }

    /**
    Destructor
    */
    ~EASTLBlockAllocator()
    {
        // reset all pointers
        m_limit = m_peak = m_base = NULL;
    }

    /**
    Attempts to initialize the allocator given a size and heap allocator

    \param numNodes number of container nodes required.
    \param sizeOfNode size of container nodes.
    \param alloc heap allocator used to allocate required memory.
    */
    bool Initialize(uint32_t numNodes, uint32_t sizeOfNode, EA::Allocator::ICoreAllocator * alloc)
    {
        m_allocator = alloc;

        // Mark the allocator and attempt to allocate the memory
        m_base = static_cast<uint8_t*>(m_allocator->Alloc(sizeOfNode * numNodes, NULL, 0, 4));

        // Check for a successful allocation.
        if (NULL != m_base)
        {
            m_current = m_base;
            m_limit = m_base + sizeOfNode * numNodes;
            m_isValid = true;
        }
        else
        {
            m_isValid = false;
        }
        return m_isValid;
    }

	/**
	Initialize the allocator with the given block of memory

	\param base base of memory block.
	\param size size of memory block.
	*/
	bool Initialize(uint8_t * base, uint32_t size)
	{
		m_base = base;
		m_limit = m_base + size;
		m_current = m_base;
		m_isValid = true;

		return true;
	}

    /**
    Allocates a block of memory of size a given size.

    \param size size of allocation.
    \param flags unused.

    \return a void pointer to the allocated memory.
    */
    void* allocate( size_t n, int /*flags*/ = 0)
    {
        // Check that enough space remains in the block to make the allocation
        EA_ASSERT_MSG(m_current + n <= m_limit,("EASTLBlockAllocator::allocate: attempted to allocate more memory that is available.\n"));
        if (m_current + n > m_limit)
        {
            return NULL;
        }

        // Increment the current pointer
        uint8_t * ret = m_current;
        m_current += n;

        // Increment the peak pointer
        if (m_current > m_peak)
            m_peak = m_current;

        return ret;
    }

    /**
    Not Implemented Yet. Expected to allocate a block of aligned memory.
    */
    void* allocate( size_t /*n*/, size_t /*alignment*/, size_t /*offset*/, int /*flag*/ = 0)
    {
        EA_FAIL_MSG(("EASTLBlockAllocator has no aligned allocation method.\n"));
        return NULL;
    }

    /**
    Deallocates the most recently allocated block of memory of a given size.

    \param p unused parameter.
    \param n size of block to deallocate.
    */
    void  deallocate( void* /*p*/, size_t n )
    {
        // Decrement current counter.
        m_current -= n;

        // Check for attempted allocations beyond start of block
        EA_ASSERT_MSG(m_base <= m_current,("EASTLBlockAllocator::deallocate: attempted to deallocate beyond start of block.\n"));
        if ( m_base > m_current)
        {
            m_current = m_base;
        }
    }

    /**
    If the allocator owns an internal allocator it calls release on that internal allocator.
    */
    void Release()
    {
        // Check for ownership of internal allocator
        if (NULL != m_allocator && m_isValid)
        {
            m_isValid = false;
            m_allocator->Free(m_base);

            // reset all pointers
            m_limit = m_peak = m_base = NULL;
        }
		else if (m_isValid)
		{
			m_limit = m_peak = m_base = NULL;
		}
    }

    /**
    Determines if the allocator is in a valid state.

    \return bool indicating validity of allocator.
    */
    bool IsValid()
    {
        return m_isValid;
    }

    /**
    Returns the entire size of the memory block.

    \return size of memory block.
    */
    uint32_t GetLimit()
    {
        return static_cast<uint32_t>(m_limit - m_base);
    }

private:

    /**
    Pointer to base of block of memory.
    */
    uint8_t * m_base;

    /**
    Pointer to start of allocated memory.
    */
    uint8_t * m_current;

    /**
    Pointer to end of allocated memory.
    */
    uint8_t * m_limit;

    /**
    Pointer to peak use of memory
    */
    uint8_t * m_peak;

    /**
    Pointer to internal allocator.
    */
    EA::Allocator::ICoreAllocator * m_allocator;

    /**
    Flag indicating validity of allocator.
    */
    bool m_isValid;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_EASTLBLOCKALLOCATOR_H
