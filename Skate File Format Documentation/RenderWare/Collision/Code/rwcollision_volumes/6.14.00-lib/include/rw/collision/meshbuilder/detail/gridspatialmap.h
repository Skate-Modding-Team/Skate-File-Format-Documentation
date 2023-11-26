// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_GRIDSPATIALMAP_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_GRIDSPATIALMAP_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <EASTL/vector.h>
#include <EASTL/sort.h>

#include <rw/collision/meshbuilder/detail/eastlblockallocator.h>

namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{

#if (1 == RWPMATH_IS_VPU)
#define RW_COLLISION_GRIDSPATIALMAP_ALIGNMENT 16
#else
#define RW_COLLISION_GRIDSPATIALMAP_ALIGNMENT 4
#endif

/**
A grid-based spatial map.

The spatial map provides fast spatial searches and queries. It values stored in the map
are the the indices of triangles.
*/
class EA_PREFIX_ALIGN(RW_COLLISION_GRIDSPATIALMAP_ALIGNMENT) GridSpatialMap
{
public:

	class BoxEntryIterator
	{
	public:

		BoxEntryIterator(const BoxEntryIterator & other)
			: m_entryIndex(other.m_entryIndex)
			, m_spatialMap(other.m_spatialMap)
		{
		}

		/**
		\brief Prefix increment operator overload
		\return A CellEntryIterator
		*/
		BoxEntryIterator & operator++()
		{
			++m_entryIndex;
			return *this;
		}

		/**
		\brief Indirection operator overload
		\return Index of current triangle or iterator::end value
		*/
		uint32_t operator*() const
		{
			return m_spatialMap.GetEntryViaIndex(m_entryIndex);
		}

		/**
		\brief Equality operator overload
		\param other CellEntryIterator to compare to this object
		\return Flag indicating whether or not iterators are equal
		*/
		bool operator==(const BoxEntryIterator & other) const
		{
			return !(*this != other);
		}

		/**
		\brief Inequality operator overload
		\param other CellEntryIterator to compare to this object
		\return Flag indicating whether or not iterators are not equal
		*/
		bool operator!=(const BoxEntryIterator & other) const
		{
			return (other.m_entryIndex != m_entryIndex);
		}

	private:

		friend class GridSpatialMap;

		BoxEntryIterator(const uint32_t entryIndex, const GridSpatialMap &spatialMap)
			: m_entryIndex(entryIndex)
			, m_spatialMap(spatialMap)
		{
		}

		BoxEntryIterator & operator=(const BoxEntryIterator &) { return *this; }

		/// CellIndex
		uint32_t m_entryIndex;

		// Spatial map
		const GridSpatialMap &m_spatialMap;
	};

	typedef rwpmath::Vector3 VectorType;

	/// Constructor
    inline GridSpatialMap(EA::Allocator::ICoreAllocator * alloc);

	inline ~GridSpatialMap();

	inline void Release();

	// Static methods
	static uint32_t MaxNumInputs( const uint32_t memoryBufferSize, const uint32_t gridResolution );

	/// \param minPoint The minimum coordinates of the axis-aligned bounding box of the contained data
	/// \param maxPoint The maximum coordinates of the axis-aligned bounding box of the contained data
	/// \param resolution The number of boxes on each side of the 3D grid
	/// \param numItems The maximum number of items to be inserted
	inline bool Initialize(
		const VectorType &minPoint,
		const VectorType &maxPoint,
		const uint32_t resolution,
		const uint32_t numInputs);

	inline void BeginInsertion(uint32_t &entryIndex);

	/// Inserts a handle into the spatial map, using the provided axis-aligned bounding box of the item
	/// referenced by the handle.
	inline bool Insert(
		const VectorType &minPoint,
		const VectorType &maxPoint,
		const uint32_t triangleIndex,
		uint32_t &entryIndex);

	inline void EndInsertion(const uint32_t entryIndex);

	inline BoxEntryIterator BoxEntryIteratorBegin(
		const uint32_t x,
		const uint32_t y,
		const uint32_t z) const;

	inline BoxEntryIterator BoxEntryIteratorEnd(
		const uint32_t x,
		const uint32_t y,
		const uint32_t z) const;

	inline void CalculateTightGridBox(
		const VectorType &minPoint,
		const VectorType &maxPoint,
		VectorType &minBoxCoords,
		VectorType &maxBoxCoords) const;

private:

	typedef rwpmath::VecFloat FloatType;

	struct Box
	{
		Box() : m_firstEntryIndex(0)
		{
		}

		uint32_t m_firstEntryIndex;
	};

	struct BoxEntry
	{
		uint32_t m_triangleIndex;
		uint32_t m_boxIndex;
	};

	struct EntryCompare
	{
		bool operator()(const BoxEntry &left, const BoxEntry &right) const
		{
			return (left.m_boxIndex < right.m_boxIndex);
		}
	};

	static inline uint32_t GetBoxCount(uint32_t resolution);

	inline bool AddEntryToBox(
		const uint32_t triangleIndex,
		const uint32_t boxIndex,
		uint32_t &entryIndex);

	inline uint32_t ComputeBoxIndex(const uint32_t x, const uint32_t y, const uint32_t z) const;

	inline uint32_t GetEntryViaIndex(const uint32_t index) const
	{
		return m_entries[index].m_triangleIndex;
	}

	VectorType m_minPoint;
	VectorType m_resolution;
	VectorType m_boxSize;

	uint32_t m_resX;
	uint32_t m_resY;
	uint32_t m_resZ;
	uint32_t m_maxEntries;

	eastl::vector<Box, detail::EASTLBlockAllocator> m_boxes;
	eastl::vector<BoxEntry, detail::EASTLBlockAllocator> m_entries;
	uint8_t * m_memoryBuffer;

    EA::Allocator::ICoreAllocator * m_allocator;
} EA_POSTFIX_ALIGN(RW_COLLISION_GRIDSPATIALMAP_ALIGNMENT);

inline GridSpatialMap::GridSpatialMap(EA::Allocator::ICoreAllocator * alloc) :
	m_minPoint(),
	m_resolution(),
	m_boxSize(),
	m_resX(0),
	m_resY(0),
	m_resZ(0),
	m_maxEntries(0),
	m_boxes(),
	m_entries(),
	m_memoryBuffer(NULL),
	m_allocator(alloc)
{
}


inline GridSpatialMap::~GridSpatialMap()
{
	Release();
}

inline void GridSpatialMap::Release()
{
    if (NULL != m_memoryBuffer)
    {
        m_allocator->Free(m_memoryBuffer);
        m_memoryBuffer = NULL;
    }
}

inline uint32_t GridSpatialMap::MaxNumInputs(
	const uint32_t memoryBufferSize,
	const uint32_t gridResolution)
{
	// Determine the space required for the boxes given the resolution
	uint32_t boxesRequirement = GetBoxCount(gridResolution) * sizeof(Box);

	if (boxesRequirement > memoryBufferSize)
		return 0;

	// Determine the maximum number of entries we can store in the
	// remaining memory
	uint32_t remainingMemory = memoryBufferSize - boxesRequirement;
	return remainingMemory / sizeof(BoxEntry);
}

inline bool GridSpatialMap::Initialize(
	const VectorType &minPoint,
	const VectorType &maxPoint,
	const uint32_t resolution,
	const uint32_t numInputs)
{
	// Don't allow the resolution to be zero in any dimension
	m_resolution = rwpmath::Vector3(static_cast<float>(resolution), static_cast<float>(resolution), static_cast<float>(resolution));
	m_minPoint = minPoint;
	m_boxSize = (maxPoint - minPoint) / m_resolution;

	m_resX = resolution;
	m_resY = resolution;
	m_resZ = resolution;

	// We don't currently hash the box index - instead we store every box explicitly

	uint32_t boxCount = GetBoxCount(m_resX);
	uint32_t boxMemoryRequirement = boxCount * sizeof(Box);
	uint32_t entryMemoryRequirement = numInputs * sizeof(BoxEntry);

	// Attempt to allocate memory for the containers
	m_memoryBuffer = static_cast<uint8_t*>(m_allocator->Alloc(boxMemoryRequirement + entryMemoryRequirement, NULL, 0, 4u));
	if (NULL == m_memoryBuffer)
	{
		return false;
	}

	// Initialize the box container
	m_boxes.get_allocator().Initialize( m_memoryBuffer, boxMemoryRequirement );
	m_boxes.resize(GetBoxCount(m_resX));

	// Initialize the box entry container
	m_entries.get_allocator().Initialize(m_memoryBuffer + boxMemoryRequirement, entryMemoryRequirement);
	m_entries.resize(numInputs);

	m_maxEntries = numInputs;

	return true;
}


inline void GridSpatialMap::BeginInsertion(uint32_t &entryIndex)
{
	entryIndex = 0;
}


inline bool GridSpatialMap::Insert(
	const VectorType &minPoint,
	const VectorType &maxPoint,
	const uint32_t triangleIndex,
	uint32_t &entryIndex)
{
	// Find the grid-snapped box that tightly contains the query shape
	// The maximum range values are exclusive
	VectorType minBoxCoords;
	VectorType maxBoxCoords;
	CalculateTightGridBox(
		minPoint,
		maxPoint,
		minBoxCoords,
		maxBoxCoords);

	// Potential load-hit-stores here
	const uint32_t minX = static_cast<uint32_t>(static_cast<float>(minBoxCoords.GetX()));
	const uint32_t minY = static_cast<uint32_t>(static_cast<float>(minBoxCoords.GetY()));
	const uint32_t minZ = static_cast<uint32_t>(static_cast<float>(minBoxCoords.GetZ()));

	const uint32_t maxX = static_cast<uint32_t>(static_cast<float>(maxBoxCoords.GetX()));
	const uint32_t maxY = static_cast<uint32_t>(static_cast<float>(maxBoxCoords.GetY()));
	const uint32_t maxZ = static_cast<uint32_t>(static_cast<float>(maxBoxCoords.GetZ()));

	for (uint32_t x = minX; x <= maxX; ++x)
	{
		for (uint32_t y = minY; y <= maxY; ++y)
		{
			for (uint32_t z = minZ; z <= maxZ; ++z)
			{
				const uint32_t boxIndex = ComputeBoxIndex(x, y, z);
				if (!AddEntryToBox(triangleIndex, boxIndex, entryIndex))
				{
					return false;
				}
			}
		}
	}
	return true;
}


inline void GridSpatialMap::EndInsertion(const uint32_t entryIndex)
{
	// Truncate the triangle list to its final size
	m_maxEntries = entryIndex;
	m_entries.resize(entryIndex);

	//
	// Sort the entries by ascending box index, so that all the entries for a single box are adjacent
	//

	EntryCompare compare;
	eastl::sort<eastl::vector<BoxEntry>::iterator, EntryCompare>(
		m_entries.begin(),
		m_entries.end(),
		compare);

	//
	// Review the triangle list and record the start of each box's entries
	// Some boxes have no triangles in them so we reset their first triangle indices to invalid
	//

	uint32_t currentBoxIndex = 0xFFFFFFFF;
	for (uint32_t i = 0; i < entryIndex; ++i)
	{
		const uint32_t boxIndex(m_entries[i].m_boxIndex);
		if (boxIndex != currentBoxIndex)
		{
			EA_ASSERT(currentBoxIndex == 0xFFFFFFFF || boxIndex > currentBoxIndex);

			// Invalidate any boxes we skipped; they contain no triangles at all
			for (uint32_t j = currentBoxIndex + 1; j < boxIndex; ++j)
			{
				m_boxes[j].m_firstEntryIndex = i;
			}

			m_boxes[boxIndex].m_firstEntryIndex = i;
			currentBoxIndex = boxIndex;
		}
	}

	for (uint32_t boxIndex = currentBoxIndex + 1 ; boxIndex < m_boxes.size() ; ++boxIndex)
	{
		m_boxes[boxIndex].m_firstEntryIndex = m_maxEntries;
	}
}


inline uint32_t GridSpatialMap::GetBoxCount(uint32_t resolution)
{
	return ((resolution * resolution * resolution) + 1);
}


inline bool GridSpatialMap::AddEntryToBox(
	const uint32_t triangleIndex,
	const uint32_t boxIndex,
	uint32_t &entryIndex)
{
	// Avoid overflow
	if (entryIndex >= m_maxEntries)
	{
		return false;
	}

	// Add this entry to the list of entries
	m_entries[entryIndex].m_triangleIndex = triangleIndex;
	m_entries[entryIndex].m_boxIndex = boxIndex;

	++entryIndex;

	return true;
}


inline void GridSpatialMap::CalculateTightGridBox(
	const VectorType &minPoint,
	const VectorType &maxPoint,
	VectorType &minBoxCoords,
	VectorType &maxBoxCoords) const
{
	// Find the grid-snapped box that tightly contains the query shape
	const VectorType minOffset(minPoint - m_minPoint);
	const VectorType maxOffset(maxPoint - m_minPoint);

	const VectorType minMultiple(minOffset / m_boxSize);
	const VectorType maxMultiple(maxOffset / m_boxSize);

	VectorType minCoords(
		rwpmath::Floor(minMultiple.GetX()),
		rwpmath::Floor(minMultiple.GetY()),
		rwpmath::Floor(minMultiple.GetZ()));

	VectorType maxCoords(
		rwpmath::Floor(maxMultiple.GetX()),
		rwpmath::Floor(maxMultiple.GetY()),
		rwpmath::Floor(maxMultiple.GetZ()));

	// Nudge maximum bounds in slightly to flip them into the last valid index
	const VectorType maxBounds(m_resolution - rwpmath::GetVecFloat_One());

	// Clamp at the integer index boundaries
	minCoords = rwpmath::Max(minCoords, rwpmath::GetVector3_Zero());
	minCoords = rwpmath::Min(minCoords, maxBounds);

	maxCoords = rwpmath::Max(maxCoords, rwpmath::GetVector3_Zero());
	maxCoords = rwpmath::Min(maxCoords, maxBounds);

	minBoxCoords = minCoords;
	maxBoxCoords = maxCoords;
}


inline uint32_t GridSpatialMap::ComputeBoxIndex(const uint32_t x, const uint32_t y, const uint32_t z) const
{
	const uint32_t boxIndex = (x * m_resY * m_resZ) + (y * m_resZ) + z;

	EA_ASSERT(boxIndex < m_boxes.size());
	return boxIndex;
}

inline GridSpatialMap::BoxEntryIterator GridSpatialMap::BoxEntryIteratorBegin(const uint32_t x, const uint32_t y, const uint32_t z) const
{
	const uint32_t boxIndex = ComputeBoxIndex(x, y, z);
	return BoxEntryIterator(m_boxes[boxIndex].m_firstEntryIndex, *this);
}

inline GridSpatialMap::BoxEntryIterator GridSpatialMap::BoxEntryIteratorEnd(const uint32_t x, const uint32_t y, const uint32_t z) const
{
	const uint32_t boxIndex = ComputeBoxIndex(x, y, z) + 1;
	return BoxEntryIterator(m_boxes[boxIndex].m_firstEntryIndex, *this);
}


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_GRIDSPATIALMAP_H

