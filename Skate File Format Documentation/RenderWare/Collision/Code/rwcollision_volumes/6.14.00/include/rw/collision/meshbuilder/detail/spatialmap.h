// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_SPATIALMAP_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_SPATIALMAP_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <EASTL/map.h>
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


/**
brief A spatial map used to partition the point cloud, allowing similar points to be collapsed into single points.

The map consists of a vector of MapEntry. Each MapEntry consists of a cellIndex (or key) and an entry (or value).
The Sort method is used to order the map entries, which allows iteration through groups of entries (grouped by cellIndex).
*/
class SpatialMap
{
public:

    typedef eastl::map<uint32_t, uint32_t, eastl::less<uint32_t>, detail::EASTLBlockAllocator> CellIndexMap;
    typedef CellIndexMap::const_iterator CellIterator;

    /**
    Constructor
    */
    SpatialMap(
        const uint32_t numMaxEntries,
        const uint32_t x,
        const uint32_t y,
        EA::Allocator::ICoreAllocator *const alloc)
        : m_allocator(alloc)
        , m_entryCountLimit(numMaxEntries)
        , m_xbits(x)
        , m_ybits(y)
        , m_isClosed(false)
        , m_isValid(false)
    {
        // Attempt to Initialize the EntryVector
        m_spatialMapEntries.get_allocator().Initialize(m_entryCountLimit, sizeof(EntryVector::value_type), m_allocator);
        if (m_spatialMapEntries.get_allocator().IsValid())
        {
            // Reserve the resources required for the maximum number of entries
            m_spatialMapEntries.reserve(m_entryCountLimit);
            m_isValid = true;
        }
    }

    /**
    Releases allocated resources
    */
    void Release()
    {
        if (m_isValid)
        {
            // The map has to be cleared before its resources are released as it
            // will attempt to traverse its internal structure to deallocate each
            // node.
            m_spatialMapIndex.clear();
            m_spatialMapIndex.get_allocator().Release();
            m_spatialMapEntries.get_allocator().Release();
        }
    }

    /**
    Returns a flag indicating whether or not the SpatialMap is valid.
    */
    bool IsValid()
    {
        return m_isValid;
    }

    /**
    Returns the memory consumption.
    */
    uint32_t GetMemUsed() const
    {
        size_t ret = m_spatialMapEntries.size() * sizeof(EntryVector::value_type);
        ret += m_spatialMapIndex.size() * sizeof(CellIndexMap::node_type);
        return static_cast<uint32_t>(ret);
    }

    /**
    Inserts a given entry into the map, using the x,y and z values as a key.
    */
    void Insert(const uint32_t x, const uint32_t y, const uint32_t z, const uint32_t entry)
    {
        EA_ASSERT_MSG(m_spatialMapEntries.size() < m_entryCountLimit,("Attempted to add more entries than entry count limit"));
        MapEntry me;
        me.cellId = Combine(x, y, z);
        me.entry = entry;
        m_spatialMapEntries.push_back(me);
    }

    /**
    Returns an entry given an index into the EntryVector.
    */
    EA_FORCE_INLINE
    uint32_t GetEntry(const uint32_t index) const
    {
        EA_ASSERT_MSG(m_isClosed, ("SpatialMap::Close method must be called before attempting to access elements"));
        EA_ASSERT_MSG(index < m_spatialMapEntries.size(),("Attempted to access out of range element"));
        return m_spatialMapEntries[index].entry;
    }

    /**
    Sorts the EntryVector and Creates the index vector
    */
    EA_FORCE_INLINE
    void Close()
    {
        // Sort the EntryVector using an EntryCompare Struct, which sorts the entries by cellId.
        EntryCompare compare;
        eastl::sort<EntryVector::iterator, EntryCompare>(m_spatialMapEntries.begin(),
            m_spatialMapEntries.end(),
            compare);

        const uint32_t totalEntryCount = GetNumEntries();

        if (totalEntryCount > 0)
        {
            // Determine the number of cells
            uint32_t cellCount = 1;
            uint32_t currentCellId = m_spatialMapEntries[0].cellId;

            // Cycle through the entire EntryVector to determine how many cells contain
            // entries.
            for (uint32_t entryIndex = 1; entryIndex < totalEntryCount ; ++entryIndex)
            {
                if (currentCellId != m_spatialMapEntries[entryIndex].cellId)
                {
                    ++cellCount;
                    currentCellId = m_spatialMapEntries[entryIndex].cellId;
                }
            }

            // Attempt to initialize the CellIndexMap
            m_spatialMapIndex.get_allocator().Initialize(cellCount, sizeof(CellIndexMap::node_type), m_allocator);
            if (!m_spatialMapIndex.get_allocator().IsValid())
            {
                m_isValid = false;
                return;
            }

            // Initialize the first CellIndexMap entry
            currentCellId = m_spatialMapEntries[0].cellId;
            m_spatialMapIndex[currentCellId] = 0;

            // Cycle through the entire EntryVector to determine the starting index of
            // each cell in the EntryVector. As a new cell is found add its ID and start
            // entry index into the CellIndexMap.
            for (uint32_t entryIndex = 1 ; entryIndex < totalEntryCount ; ++entryIndex)
            {
                if (currentCellId != m_spatialMapEntries[entryIndex].cellId)
                {
                    currentCellId = m_spatialMapEntries[entryIndex].cellId;
                    m_spatialMapIndex[currentCellId] = entryIndex;
                }
            }

            m_isClosed = true;
        }
    }

    /**
    Given a cellId returns the index of the first entry relating to that cellId
    */
    EA_FORCE_INLINE
    uint32_t GetFirstEntryInCell(const uint32_t x, const uint32_t y, const uint32_t z) const
    {
        return GetFirstEntryInCell(Combine(x, y, z));
    }

    /**
    Given a cellId returns the index of the first entry relating to that cellId.
    If the cell does not exist then return the entry count, indicating failure.
    */
    EA_FORCE_INLINE
    uint32_t GetFirstEntryInCell(const uint32_t cellId) const
    {
        EA_ASSERT_MSG(m_isClosed, ("SpatialMap::Close method must be called before attempting to access elements"));

        const CellIterator it = m_spatialMapIndex.find(cellId);
        if (it != m_spatialMapIndex.end())
        {
            return (*it).second;
        }
        else
        {
            return static_cast<uint32_t>(m_spatialMapEntries.size());
        }
    }

    /**
    Given a cellId returns the index of the start of the following cell.
    */
    EA_FORCE_INLINE
    uint32_t GetFirstEntryInNextCell(const uint32_t x, const uint32_t y, const uint32_t z) const
    {
        return GetFirstEntryInNextCell(Combine(x, y, z));
    }

    /**
    Given a cellId returns the index of the start of the following cell.
    If the cell does not exist then return the entry count, indicating failure.
    */
    EA_FORCE_INLINE
    uint32_t GetFirstEntryInNextCell(const uint32_t cellId) const
    {
        EA_ASSERT_MSG(m_isClosed, ("SpatialMap::Close method must be called before attempting to access elements"));

        // Obtain an iterator referencing the given cell. Increment the iterator to reference
        // the following cell and return the corresponding entry index. This is possible since
        // both the EntryVector and CellIndexMap are sorted by CellId.
        CellIterator it = m_spatialMapIndex.find(cellId);

        // Check if there is an entry for this cell/the cell exists
        if(it != m_spatialMapIndex.end())
        {
            // Advance the iterator
            ++it;

            // Check this next cell has an entry/the cell exists
            if (it != m_spatialMapIndex.end())
            {
                // Return the start entry index of this next cell
                return it->second;
            }
        }

        // The cell was either the last cell or it does not exist.
        return static_cast<uint32_t>(m_spatialMapEntries.size());
    }

    /**
    Finds the first spatialMap entry, in a given cell, which has a higher vertex index
    than the given index.
    */
    uint32_t FindHigherIndexEntry(
        const uint32_t x,
        const uint32_t y,
        const uint32_t z,
        const uint32_t comparisonEntry) const
    {
        uint32_t cellId = Combine(x, y, z);

        const uint32_t startEntry = GetFirstEntryInCell(cellId);
        const uint32_t endEntry = GetFirstEntryInNextCell(cellId);

        // Check for an empty cell
        if (startEntry == endEntry)
        {
            return static_cast<uint32_t>(m_spatialMapEntries.size());
        }

        const uint32_t comparisonVertexIndex = GetEntry(comparisonEntry);

        uint32_t higherEntry = startEntry;
        while ( (higherEntry < endEntry) && ( GetEntry(higherEntry) < comparisonVertexIndex) )
        {
            // Advance the lower entry
            ++higherEntry;
        }
        return higherEntry;
    }

    /**
    Returns an iterator addressing the first element of the CellIndexMap.
    */
    EA_FORCE_INLINE
    CellIterator Begin() const
    {
        EA_ASSERT_MSG(m_isClosed, ("SpatialMap::Close method must be called before attempting to access elements"));
        return m_spatialMapIndex.begin();
    }

    /**
    Returns an iterator addressing the location succeeding the last element of the CellIndexMap.
    */
    EA_FORCE_INLINE
    CellIterator End() const
    {
        EA_ASSERT_MSG(m_isClosed, ("SpatialMap::Close method must be called before attempting to access elements"));
        return m_spatialMapIndex.end();
    }

private:

    /**
    Combines three values to produce a cellId.
    */
    EA_FORCE_INLINE
    uint32_t Combine(uint32_t x, uint32_t y, uint32_t z) const
    {
        return ((x) + ((y) << m_xbits) + ((z) << (m_xbits + m_ybits)));
    }

    /**
    Returns the number of entries in the map
    */
    EA_FORCE_INLINE uint32_t GetNumEntries() const
    {
        return static_cast<uint32_t>(m_spatialMapEntries.size());
    }

    /**
    MapEntry structure used to hold a cellId(key) and entry(value) pair.
    */
    struct MapEntry
    {
        MapEntry()
            : cellId(0)
            , entry(0)
        {
        }

        MapEntry(const MapEntry& other)
        {
            cellId = other.cellId;
            entry = other.entry;
        }

        uint32_t cellId;
        uint32_t entry;
    };

    /**
    EntryCompare structure used to define sorting query for eastl sort operation
    */
    struct EntryCompare
    {
        bool operator()(const MapEntry &left, const MapEntry &right) const
        {
            // Sorts on cellId < and then entry <
            return (left.cellId < right.cellId || (left.cellId == right.cellId && left.entry < right.entry) );
        }
    };

    typedef eastl::vector<MapEntry, detail::EASTLBlockAllocator> EntryVector;

    // Vector used to store the entries
    EntryVector        m_spatialMapEntries;
    // Index into EntryVector
    CellIndexMap       m_spatialMapIndex;

    // Allocator
    EA::Allocator::ICoreAllocator *m_allocator;

    // Maximum number of entries
    uint32_t m_entryCountLimit;

    // Used to determine how many bits used to store each coordinate component
    uint32_t m_xbits;
    uint32_t m_ybits;

    // Flag indicating whether or not collection has been closed.
    bool m_isClosed;

    // Used to flag memory requirement issues.
    bool m_isValid;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // #define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_SPATIALMAP_H
