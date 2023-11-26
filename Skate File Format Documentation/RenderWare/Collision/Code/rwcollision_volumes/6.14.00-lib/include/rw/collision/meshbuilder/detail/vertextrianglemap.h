// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_VERTEXTRIANGLEMAP_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_VERTEXTRIANGLEMAP_H


/*************************************************************************************************************

File: vertextrianglemap.h

Purpose: VertexTriangleMap class.

A map used to associate vertices with triangles. The map is constructed in such a way as to allow fast random
access to the vertex-triangle associative data.

The fast random access is made possible by the use of an two containers. The first container consists of the
associative information about vertices and triangles, storing a sorted list of vertex and triangle index pairs.
The second container stores a list of indices into the first container, indicating the start point of a
group of entries.
*/


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <EASTL/sort.h>
#include <EASTL/vector.h>

#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/meshbuilder/detail/eastlblockallocator.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


class VertexTriangleMap
{
public:

    /**
    \class VertexIterator
    \brief Class which defines a vertex iterator.

    This iterator is designed to iterate over the entire list of merged vertices. The
    merged vertices are not known until after vertex merging takes place and therefore
    the iterator can not be used until after this step.

    Objects should be initialized using ClusteredMeshBuilder::VerticesBegin or
    ClusteredMeshBuilder::VerticesEnd.
    */

    class VertexIterator
    {
    public:

        VertexIterator(const VertexIterator & other)
            : m_vertexIndex(other.m_vertexIndex)
            , m_vertexTriangleMap(other.m_vertexTriangleMap)
        {
        }

        /**
        \brief Prefix increment operator overload
        \return An AjoiningTriangleIterator
        */
        VertexIterator & operator++()
        {
            m_vertexIndex = m_vertexTriangleMap.GetNextVertexIndex(m_vertexIndex);
            return *this;
        }

        /**
        \brief Indirection operator overload
        \return Index of current triangle or iterator::end value
        */
        uint32_t operator*()
        {
            return m_vertexIndex;
        }

        /**
        \brief Equality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are equal
        */
        bool operator==(const VertexIterator & other) const
        {
            return !(*this != other);
        }

        /**
        \brief Inequality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are not equal
        */
        bool operator!=(const VertexIterator & other) const
        {
            if(other.m_vertexIndex != m_vertexIndex)
            {
                return true;
            }

            return false;
        }

    private:

        /*
        VertexTriangleMap is a declared as a friend of this class to allow access to the private
        constructor.
        */
        friend class VertexTriangleMap;

        /*
        \brief Constructor
        \param vertexIndex Index of addressed vertex
        \param vertexTriangleMap related VertexTriangleMap
        */
        VertexIterator(const uint32_t vertexIndex, const VertexTriangleMap & vertexTriangleMap)
            : m_vertexIndex(vertexIndex)
            , m_vertexTriangleMap(vertexTriangleMap)
        {
        }

        /**
        \brief Assignment operator overload
        Operator is declared private as it is not intended for public use
        */
        VertexIterator & operator=(const VertexIterator &) { return *this; }

        void End()
        {
            m_vertexIndex = m_vertexTriangleMap.GetEndVertexIndex();
        }

        /// Index of vertex
        uint32_t m_vertexIndex;
        /// VertexTriangleMap holding vertex-triangle index information
        const VertexTriangleMap &m_vertexTriangleMap;
    };

    /**
    \class AdjoiningTriangleIterator
    \brief Class which defines an adjoining triangle iterator.

    Given a base vertex index this iterator will iterate through all adjoining triangles.
    These adjoining triangles are defined as the triangles which include the base vertex.

    Objects should be initialized using ClusteredMeshBuilder::AdjoiningTriangleBegin or
    ClusteredMeshBuilder::AdjoiningTriangleEnd.
    */
    class AdjoiningTriangleIterator
    {
    public:

        /**
        \brief Copy Constructor
        \param other AdjoiningTriangleIterator to copy
        */
        AdjoiningTriangleIterator(const AdjoiningTriangleIterator & other)
            : m_baseVertexIndex(other.m_baseVertexIndex)
            , m_mapIndex(other.m_mapIndex)
            , m_vertexTriangleMap(other.m_vertexTriangleMap)
        {
        }

        /**
        \brief Prefix increment operator overload
        \return An AjoiningTriangleIterator
        */
        AdjoiningTriangleIterator & operator++()
        {
            ++m_mapIndex;
            return *this;
        }

        /**
        \brief Indirection operator overload
        \return Index of current triangle or iterator::end value
        */
        uint32_t operator*()
        {
            if (m_vertexTriangleMap.NextTriangle(m_mapIndex, m_baseVertexIndex))
            {
                return m_vertexTriangleMap.GetTriangleIndex(m_mapIndex);
            }

            return 0xFFFFFFFF;
        }

        /**
        \brief Equality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are equal
        */
        bool operator==(const AdjoiningTriangleIterator & other) const
        {
            return !(*this != other);
        }

        /**
        \brief Inequality operator overload
        \param other AdjoiningTriangleIterator to compare to this object
        \return Flag indicating whether or not iterators are not equal
        */
        bool operator!=(const AdjoiningTriangleIterator & other) const
        {
            if(other.m_baseVertexIndex != m_baseVertexIndex ||
                other.m_mapIndex != m_mapIndex)
            {
                return true;
            }

            return false;
        }

    private:

        /*
        VertexTriangleMap is a declared as a friend of this class to allow access to the private
        constructor.
        */
        friend class VertexTriangleMap;

        /*
        \brief Constructor
        \param vertexIndex Index of base triangle
        \param mapIndex Index into vertexTriangleMap
        \param vertexTriangleMap Map of vertex and triangle index information
        */
        AdjoiningTriangleIterator(const uint32_t vertexIndex,
                                  const uint32_t mapIndex,
                                  const VertexTriangleMap & vertexTriangleMap)
            : m_baseVertexIndex(vertexIndex)
            , m_mapIndex(mapIndex)
            , m_vertexTriangleMap(vertexTriangleMap)
        {
        }

        /**
        \brief Assignment operator overload
        Operator is declared private as it is not intended for public use
        */
        AdjoiningTriangleIterator & operator=(const AdjoiningTriangleIterator &) { return *this; }

        /// Index of base vertex 
        uint32_t m_baseVertexIndex;
        /// Index into vertexTriangleMap
        uint32_t m_mapIndex;
        /// VertexTriangleMap holding vertex-triangle index information
        const VertexTriangleMap &m_vertexTriangleMap;
    }; // class AdjoiningTriangleIterator


private:

    struct VertexTrianglePair
    {
        VertexTrianglePair(uint32_t vertex,
                           uint32_t triangle)
            : vertexIndex(vertex)
            , triangleIndex(triangle)
        {
        }

        uint32_t vertexIndex;
        uint32_t triangleIndex;
    };

    typedef eastl::vector<VertexTrianglePair, detail::EASTLBlockAllocator> VertexTrianglePairVector;
    typedef eastl::vector<uint32_t, detail::EASTLBlockAllocator> VertexTrianglePairIndexVector;

public:

    VertexTriangleMap()
        : m_isSortedAndIndexed(false)
        , m_isValid(false)
    {
    }

    /*
    Attempts to initialize the map given a number of input triangles and an allocator
    */
    void Initialize(uint32_t numTri, EA::Allocator::ICoreAllocator* alloc)
    {
        // Setup the allocators for m_pairVector and m_indexVector
        m_pairVector.get_allocator().Initialize(numTri * 3, sizeof(VertexTrianglePairVector::value_type), alloc);
        if (m_pairVector.get_allocator().IsValid())
        {
            m_pairVector.reserve(numTri * 3);
        }
        else
        {
            m_isValid = false;
            return;
        }

        m_indexVector.get_allocator().Initialize(numTri * 3, sizeof(VertexTrianglePairIndexVector::value_type), alloc);
        if (m_indexVector.get_allocator().IsValid())
        {
            m_indexVector.resize(numTri * 3, 0xFFFFFFFF);
        }
        else
        {
            m_isValid = false;
            return;
        }

        m_isValid = true;
    }

    /*
    Releases the memory used by the internal containers
    */
    void Release()
    {
        if (m_isValid)
        {
            m_indexVector.get_allocator().Release();
            m_pairVector.get_allocator().Release();
        }
    }

    /*
    Inserts a pair into the map
    */
    EA_FORCE_INLINE void
    Insert(uint32_t vertexIndex, uint32_t triangleIndex)
    {
        VertexTrianglePair pair(vertexIndex, triangleIndex);
        m_pairVector.push_back(pair);
    }

    /*
    Sorts the map and generates the vector of indexes into the map.
    */
    EA_FORCE_INLINE void
    SortAndIndex()
    {
        // Sort the vector
        VertexTriangleEntryCompare compare;
        eastl::sort<VertexTrianglePairVector::iterator, VertexTriangleEntryCompare>(
            m_pairVector.begin(),
            m_pairVector.end(),
            compare);

        // Create the vector of indexes
        VertexTrianglePairVector::iterator it = m_pairVector.begin();
        VertexTrianglePairVector::const_iterator itEnd = m_pairVector.end();
        uint32_t currentVertexIndex = (*it).vertexIndex;
        uint32_t vectorIndex = 0;

        // Initialize the first index
        m_indexVector[currentVertexIndex] = vectorIndex;

        // Iterate through the sorted map and index each of the group of pairs
        while (it != itEnd)
        {
            while (it != itEnd && (*it).vertexIndex == currentVertexIndex)
            {
                ++vectorIndex;
                ++it;
            }

            if (it != itEnd)
            {
                m_indexVector[(*it).vertexIndex] = vectorIndex;
                currentVertexIndex = (*it).vertexIndex;
            }
        }

        m_isSortedAndIndexed = true;
    }

	/*
	Return returns the next vertex index given a vertex index.
	*/
	EA_FORCE_INLINE uint32_t
	GetNextVertexIndex(uint32_t vertexIndex) const
	{
		EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
		EA_ASSERT_MSG(vertexIndex < m_indexVector.size(), ("Attempted to access out of range element"));
		while (++vertexIndex < m_indexVector.size() && m_indexVector[vertexIndex] == 0xFFFFFFFF)
		{
		}

		return vertexIndex;
	}

	/*
	Return returns the index indicating the end of the vertex indices.
	*/
	EA_FORCE_INLINE uint32_t
	GetEndVertexIndex() const
	{
		EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
		return static_cast<uint32_t>(m_indexVector.size());
	}

    /*
    Returns an start index into the map given a vertexIndex. This is the first index of the group of
    pairs with the given vertexIndex (only applies to a sorted and indexed map)
    */
    EA_FORCE_INLINE uint32_t
    GetStartMapIndex(uint32_t vertexIndex) const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        EA_ASSERT_MSG(vertexIndex < m_indexVector.size(), ("Attempted to access out of range element"));
        return m_indexVector[vertexIndex];
    }

    /*
    Returns an end index into the map given a vertexIndex. This is the index of the start of the 
    following group of pairs(only applies to a sorted and indexed map)
    */
    EA_FORCE_INLINE uint32_t
    GetEndMapIndex(uint32_t vertexIndex) const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        EA_ASSERT_MSG(vertexIndex < m_indexVector.size(), ("Attempted to access out of range element"));
        uint32_t mapIndex = m_indexVector[vertexIndex];
        while(m_pairVector.size() > mapIndex && m_pairVector[mapIndex].vertexIndex == vertexIndex)
        {
            ++mapIndex;
        }

        return mapIndex;
    }

    /*
    Returns a Triangle index given a map index.
    */
    EA_FORCE_INLINE uint32_t
    GetTriangleIndex(uint32_t mapIndex) const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        EA_ASSERT_MSG(mapIndex < m_pairVector.size(), ("Attempted to access out of range element"));
        return m_pairVector[mapIndex].triangleIndex;
    }

    /*
    Returns a bool indicating whether or not the triangle corresponding to the mapIndex,
    is paired to the given vertexIndex, eg whether or not the triangle shares the vertex.
    */
    EA_FORCE_INLINE bool
    NextTriangle(uint32_t mapIndex, uint32_t vertexIndex) const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        return (mapIndex < m_pairVector.size() && m_pairVector[mapIndex].vertexIndex == vertexIndex);
    }

    /*
    Returns the memory consumption
    */
    EA_FORCE_INLINE uint32_t
    GetMemUsed()
    {
        return (m_pairVector.get_allocator().GetLimit() + m_indexVector.get_allocator().GetLimit());
    }

    /*
    Returns a bool indicating whether or not this VertexTriangleMap is valid
    */
    EA_FORCE_INLINE bool
    IsValid() const
    {
        return m_isValid;
    }

    /**
    brief Returns an AdjoiningTriangleIterator addressing the first element in
    the group of triangles base on a given vertex index.

    param vertexIndex Index of vertex.
    return AdjoiningTriangleIterator base on a given vertex index.
    */
    EA_FORCE_INLINE
    AdjoiningTriangleIterator AdjoiningTriangleBegin(uint32_t vertexIndex) const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        return AdjoiningTriangleIterator(vertexIndex, GetStartMapIndex(vertexIndex), *this);
    }

    /**
    brief Returns an AdjoiningTriangleIterator addressing the first element beyond
    the group of triangles base on a given vertex index.

    \param vertexIndex index of vertex.
    \return AdjoiningTriangleIterator base on a given vertex index.
    */
    EA_FORCE_INLINE
    AdjoiningTriangleIterator AdjoiningTriangleEnd(uint32_t vertexIndex) const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        return AdjoiningTriangleIterator(vertexIndex, GetEndMapIndex(vertexIndex), *this);
    }

    /**
    brief Returns a VertexIterator addressing the first vertex in
    the merged vertex collection.

    return VertexIterator addressing the first vertex in the merged vertex group
    */
    EA_FORCE_INLINE
    VertexIterator VerticesBegin() const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        return VertexIterator(m_pairVector[0].vertexIndex, *this);
    }

    /**
    brief Returns a VertexIterator addressing a point beyond the last vertex in
    the merged vertex collection.

    return VertexIterator addressing a point beyond the last vertex in the
    merged vertex group
    */
    EA_FORCE_INLINE
    VertexIterator VerticesEnd() const
    {
        EA_ASSERT_MSG(m_isSortedAndIndexed, ("VertexTriangleMap::SortAndIndex must be called before attempting to access elements"));
        VertexIterator ret(0, *this);
        ret.End();
        return ret;
    }

private:

    // Structure used to define the sorting query for the eastl sort operations
    struct VertexTriangleEntryCompare
    {
        bool operator()(const VertexTrianglePair &left, const VertexTrianglePair &right) const
        {
            // sorts on key/vertexIndex <
            return ((left.vertexIndex < right.vertexIndex) ||
                    ((left.vertexIndex == right.vertexIndex) && (left.triangleIndex < right.triangleIndex)));
        }
    };

    VertexTrianglePairVector m_pairVector;
    VertexTrianglePairIndexVector m_indexVector;

    bool m_isSortedAndIndexed;

    bool m_isValid;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_VERTEXTRIANGLEMAP_H
