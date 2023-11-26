// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_VECTOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_VECTOR_H


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
Simple implementation of a vector matching (partially) the interface of STL vectors,
but buffer-allocated using a simple GetSize/Initialize/Release API.
This implementation has the important feature of not performing any dynamic
allocations.
*/
template <class ItemType>
class Vector
{

public:

    typedef Vector<ItemType> this_type;
    typedef uint32_t size_type;
    typedef ItemType value_type;
    typedef ItemType &reference;
    typedef const ItemType &const_reference;

    struct Parameters
    {
        explicit Parameters(const uint32_t capacity) : m_capacity(capacity)
        {
        }

        uint32_t m_capacity;
    };

    /**
    Destructor
    */
    ~Vector()
    {
    }

    //
    // Convenience static allocation methods
    //

    /**
    \param allocator An allocator compatible with EA::Allocator::ICoreAllocator
    */
    static this_type *Allocate(EA::Allocator::ICoreAllocator *const allocator, const uint32_t capacity, const uint32_t flags = 0)
    {
        const Parameters params(capacity);
        const uint32_t bufferSize(GetSize(params));

        void *const buffer = allocator->Alloc(bufferSize, "vector", flags, 4);
        if (!buffer)
        {
            return 0;
        }

        return Initialize(params, buffer, bufferSize);
    }

    /**
    \param allocator An allocator compatible with EA::Allocator::ICoreAllocator
    */
    static void Free(EA::Allocator::ICoreAllocator *const allocator, this_type *&instance)
    {
        if (instance != 0)
        {
            instance->Release();
            allocator->Free(instance);
            instance = 0;
        }
    }

    //
    // Buffer-allocated initialization pattern
    //

    /**
    Returns the size of a memory buffer required to instantiate an instance with the given parameters.
    \note The size includes space for the class instance and also its dynamicly owned data.
    */
    static uint32_t GetSize(const Parameters &params)
    {
        // For now, alignment is not supported
        uint32_t size(sizeof(this_type));
        size += params.m_capacity * sizeof(value_type);
        return size;
    }

    /**
    Initializes an instance in the given memory buffer.
    */
    static this_type *Initialize(const Parameters &params, void *const buffer, const uint32_t bufferSize)
    {
        uint8_t *memory(reinterpret_cast<uint8_t *>(buffer));

        // Check the buffer is big enough
        EA_ASSERT(bufferSize >= GetSize(params));
        (void)(bufferSize);

        // Construct the instance at the start of the buffer
        this_type *const instance = new (memory) this_type();
        memory += sizeof(this_type);

        // Point the instance at the rest of the buffer
        instance->m_array = reinterpret_cast<value_type *>(memory);
        instance->m_capacity = params.m_capacity;
        instance->m_size = 0;

        return instance;
    }

    /**
    Releases any memory owned by the instance.
    */
    void Release()
    {
        m_array = 0;
        m_capacity = 0;
        m_size = 0;
    }

    //
    // Partial eastl::vector interface implementation
    //

    bool empty() const
    {
        return (m_size == 0);
    }

    size_type size() const
    {
        return m_size;
    }

    size_type capacity() const
    {
        return m_capacity;
    }

    void resize(const size_type n, const value_type &value)
    {
        EA_ASSERT(n <= m_capacity);

        for (uint32_t index(m_size); index < n; ++index)
        {
            m_array[index] = value;
        }

        m_size = n;
    }

    void resize(const size_type n)
    {
        EA_ASSERT(n <= m_capacity);

        // Construct any newly created elements
        for (uint32_t index(m_size); index < n; ++index)
        {
            m_array[index] = value_type();
        }

        m_size = n;
    }

    void reserve(const size_type n)
    {
        // Capacity is fixed at initialization time and can't be changed!
        EA_ASSERT(n == m_capacity);
        (void)n;
    }

    void set_capacity(const size_type n)
    {
        // Capacity is fixed at initialization time and can't be changed!
        EA_ASSERT(n == m_capacity);
        (void)n;
    }

    void push_back(const value_type &value)
    {
        EA_ASSERT(m_size < m_capacity);
        m_array[m_size++] = value;
    }

    void clear()
    {
        m_size = 0;
    }

    reference operator[](const size_type n)
    {
        EA_ASSERT(n < m_size);
        return m_array[n];
    }

    const_reference operator[](const size_type n) const
    {
        EA_ASSERT(n < m_size);
        return m_array[n];
    }

private:

    /**
    Default constructor.
    Vectors can't be constructed directly - use \ref Initialize instead.
    */
    Vector() : m_array(0), m_capacity(0), m_size(0)
    {
    }

    /**
    Copy constructor. Vectors can't be copied directly.
    */
    Vector(const Vector &other);

    /**
    Assignment operator. Vectors can't be assigned directly.
    */
    Vector &operator=(const Vector &other);

    value_type *m_array;
    uint32_t m_capacity;
    uint32_t m_size;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_VECTOR_H
