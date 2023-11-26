// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_REGULARGRID_HPP
#define PUBLIC_RW_REGULARGRID_HPP

/*************************************************************************************************************

 File: regulargrid.hpp

 Purpose: Regular grid based spatial map.
 */

#include "rw/collision/common.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/aalineclipper.h"

namespace rw
{
namespace collision
{

/**
\internal
\brief
Maximum number of entries in a RegularGrid
Note: 0xffff is not a valid entry index (it's used for end of list etc)
*/
#define rwREGULARGRID_MAX_ENTRIES       -1

/**
\internal
\brief
Maximum number of cells in RegularGrid
*/
#define rwREGULARGRID_MAX_CELLS         -1

/**
\internal
\brief
Alignment of RegularGrid
*/
#if defined(RWCROSS) && defined(RWP_NO_VPU_MATH)
// workaround a bug in rwmath where rw::math::fpu::Vector3::Alignment incorrectly has a value of 1 in cross builds
#define rwREGULARGRID_ALIGNMENT         4
#else // if !defined(RWCROSS) || !defined(RWP_NO_VPU_MATH)
#define rwREGULARGRID_ALIGNMENT         rwpmath::Vector3::Alignment
#endif // !defined(RWCROSS) || !defined(RWP_NO_VPU_MATH)

/**
\internal
\brief
Alignment of cells within RegularGrid
*/
#define rwREGULARGRID_CELL_ALIGNMENT     4

/**
\internal
\brief
Alignment of entries within RegularGrid
*/
#define rwREGULARGRID_ENTRY_ALIGNMENT    4

/**
\internal
\brief
Alignment of Bounding Boxes within RegularGrid
*/
#if defined(RWCROSS) && defined(RWP_NO_VPU_MATH)
// workaround a bug in rwmath where rw::math::fpu::Vector3::Alignment incorrectly has a value of 1 in cross builds
#define rwREGULARGRID_BBOX_ALIGNMENT    4
#else // if !defined(RWCROSS) || !defined(RWP_NO_VPU_MATH)
#define rwREGULARGRID_BBOX_ALIGNMENT    rwpmath::Vector3::Alignment
#endif // !defined(RWCROSS) || !defined(RWP_NO_VPU_MATH)


// ***********************************************************************************************************
//                                               GridIterator CLASS
// ***********************************************************************************************************
/**
\internal
\brief
Iterator class for RegularGrid
*/
class GridIterator
{
private:

    struct LoopCounter
    {
        // Sets min, max and current values for LoopCounter    
        EA_FORCE_INLINE void Set(int32_t min, int32_t max, int32_t cur)
        {
            m_min = min;
            m_max = max;
            m_cur = cur;
        }

        // Increments LoopCounter
        EA_FORCE_INLINE void Increment()
        {
            ++m_cur;
        }

        //Resets LoopCounter
        EA_FORCE_INLINE void Reset()
        {
            m_cur = m_min;
        }

        // Check if current value of LoopCounter is less than max 
        EA_FORCE_INLINE bool LessThanMax() const
        {
            return m_cur < m_max;
        }

        // Returns minimum value
        EA_FORCE_INLINE int32_t Min() const
        {
            return m_min;
        }

        // Returns maximum value
        EA_FORCE_INLINE int32_t Max() const
        {
            return m_max;
        }

        // Returns current value
        EA_FORCE_INLINE int32_t Cur() const
        {
            return m_cur;
        }

        int32_t m_min;
        int32_t m_max;
        int32_t m_cur;
    };

    LoopCounter m_xAxisCounter;
    LoopCounter m_yAxisCounter;
    LoopCounter m_zAxisCounter;

public:
    /**
    \internal    
    \brief
    Initializes GridIterator to iterate through the range minX-maxX, minY-maxY, minZ-maxZ

    \param minX    Minimum inclusive X value
    \param minY    Minimum inclusive Y value
    \param minZ    Minimum inclusive Z value
    \param maxX    Maximum inclusive X value
    \param maxY    Maximum inclusive Y value
    \param maxZ    Maximum inclusive Z value
    
    */

    EA_FORCE_INLINE void Init(int32_t minX, int32_t maxX, int32_t minY, int32_t maxY, int32_t minZ, int32_t maxZ)
    {
        // Start one back on X. This is to make sure single cell and first cell, get processed.
        // See GetNextCellIndices for counter usage.
        m_xAxisCounter.Set(minX, maxX, minX - 1);
        m_yAxisCounter.Set(minY, maxY, minY);
        m_zAxisCounter.Set(minZ, maxZ, minZ);
    }


    /**
    \internal    
    \brief
    Updates x, y, z references to the indices of the next grid cell.
    
    \param x    Output reference to x value
    \param y    Output reference to y value
    \param z    Output reference to z value

    \return     true unless already at the end.
    
    */
    bool GetNextCellIndices(int32_t & x, int32_t & y, int32_t & z)
    {
        if (m_xAxisCounter.LessThanMax())
        {
            m_xAxisCounter.Increment();
        }
        else
        {
            m_xAxisCounter.Reset();

            if (m_yAxisCounter.LessThanMax())
            {
                m_yAxisCounter.Increment();
            }
            else
            {
                m_yAxisCounter.Reset();

                if (m_zAxisCounter.LessThanMax())
                {
                    m_zAxisCounter.Increment();
                }
                else
                {
                    return false;
                }
            }
        }

        x = m_xAxisCounter.Cur();
        y = m_yAxisCounter.Cur();
        z = m_zAxisCounter.Cur();

        return true;
    };
};


/**
\brief
Regular grid based spatial map.

\importlib rwccore
*/
template<class ENTRY_TYPE, class CELL_TYPE>
class TRegularGrid
{
public:
    typedef ENTRY_TYPE entry_type;
    typedef CELL_TYPE  cell_type;


    // *******************************************************************************************************
    //                                          TRegularGrid::Entry CLASS
    // *******************************************************************************************************

    /**
    \internal
    \brief TRegularGrid entry.

    \importlib rwccore
    */
    struct Entry
    {
#if !defined(REGULARGRID_NO_GROUP_SUPPORT)
        // Group this entry belongs to
        int32_t m_group;
#endif // !defined(REGULARGRID_NO_GROUP_SUPPORT)

        // Index of regular grid cell we belong to
        cell_type m_cell;

        // Next entry in list
        entry_type m_next;

        /**
        \internal
        \brief
        Initializes the Entry with indexes for the cell it belongs to, and the next entry in the cell.
        
        \param cell     Index of cell the entry belongs too
        \param next     Index of next entry in list
        */
        EA_FORCE_INLINE void Init(uint32_t cell, uint32_t next)
        {
#if !defined(REGULARGRID_NO_GROUP_SUPPORT)
            m_group = 0u;
#endif // !defined(REGULARGRID_NO_GROUP_SUPPORT)
            m_cell = (cell_type)cell;
            m_next = (entry_type)next;
        }

        /**
        \internal 
        \brief
        Returns the index of the cell the entry is in.

        \return     Index of cell to which entry belongs
        */
        EA_FORCE_INLINE uint32_t GetCellIndex() const
        {
            return (uint32_t)m_cell;
        }

        /**
        \internal
        \brief
        Sets the index of the next entry in the linked list.

        \param next     Index of next entry in list
        */
        EA_FORCE_INLINE void SetNext(const uint32_t next)
        { 
            m_next = (entry_type)next;
        }

        /**
        \internal
        \brief
        Gets the index of the next entry

        \return     Index of next entry in list
        */
        EA_FORCE_INLINE uint32_t GetNext() const 
        { 
            return (uint32_t)m_next;
        }

#if !defined(REGULARGRID_NO_GROUP_SUPPORT)
        /**
        \internal
        \brief
        Sets the group id of the entry

        \param  group   Group ID entry belongs too
        */
        EA_FORCE_INLINE void SetGroup(const int32_t group)
        { 
            m_group = group;
        }

        /**
        \internal        
        \brief
        Gets the group id of the entry
        
        \return     Group ID entry belongs too
        
        */
        EA_FORCE_INLINE int32_t GetGroup() const 
        { 
            return m_group;
        }
#endif // !defined(REGULARGRID_NO_GROUP_SUPPORT)
    };


    // *******************************************************************************************************
    //                                          TRegularGrid::Cell CLASS
    // *******************************************************************************************************

    /**
    \internal
    \brief
    TRegularGrid cell.

    \importlib rwccore
    */
    struct Cell
    {
        entry_type m_entry;

        /**
        \internal
        \brief
        Initializes the cell
        */
        EA_FORCE_INLINE void Init()
        {
            m_entry = (entry_type)rwREGULARGRID_MAX_ENTRIES;
        }


        /**
        \internal
        \brief
        Sets the Entry Index

        \param entry    The index of the head entry in the cells entry list.
        */
        EA_FORCE_INLINE void SetEntryIndex(uint32_t entry)
        {
            m_entry = (entry_type)entry;
        }


        /**
        \internal
        \brief
        Gets the Entry Index

        \return     The index of the head entry in the cells entry list.
        */
        EA_FORCE_INLINE uint32_t GetEntryIndex() const
        {
            return (uint32_t)m_entry;
        }
        

        /**
        \internal
        \brief
        Check if cell is empty

        \return     true if cell is empty, else false. 
        
        */
        EA_FORCE_INLINE bool IsEmpty() const
        {
            return m_entry != (entry_type)rwREGULARGRID_MAX_ENTRIES ? false : true;
        }
    };


    // *******************************************************************************************************
    //          TRegularGrid CLASS members
    // *******************************************************************************************************

    // Outer extent
    AABBox              m_extent;

    // Cell size
    rwpmath::Vector3    m_cellSize;
    rwpmath::Vector3    m_recipCellSize;

    // Max number of entries
    uint32_t            m_maxEntries;

    // Number of cells in each axis
    int32_t             m_xCells;
    int32_t             m_yCells;
    int32_t             m_zCells;

    // Entry bbox array
    AABBox             *m_bboxes;

    // Entry array
    Entry              *m_entries;

    // Branch nodes
    Cell               *m_cells;


    /**
    \internal    
    \brief
    Check to see if bounding box contains a point

    \param bbox     Reference to bounding box to be tested
    \param point    Point to be tested
    
    \return     true if point is within bounding box, else false.    
    */

    EA_FORCE_INLINE bool ContainsPoint(const AABBox &bbox, rwpmath::Vector3::InParam point) const
    {
        bool result;

        if (point.GetX() >= bbox.Max().GetX() || point.GetX() < bbox.Min().GetX() ||
            point.GetY() >= bbox.Max().GetY() || point.GetY() < bbox.Min().GetY() ||
            point.GetZ() >= bbox.Max().GetZ() || point.GetZ() < bbox.Min().GetZ())
        {
            result = false;
        }
        else
        {
            result = true;
        }

        return result;
    }


    /**
    \internal    
    \brief
    Calculates the index of the cell from provided x,y,z indices.

    \param x    x value in grid to lookup
    \param y    y value in grid to lookup
    \param z    z value in grid to lookup
    \return     Index of cell
    */
    EA_FORCE_INLINE uint32_t GetCellIndex(int32_t x, int32_t y, int32_t z) const
    {
        uint32_t index(((m_xCells * m_zCells) * y) + (m_xCells * z) + x + 1u);
        return index;
    }


    /**
    \internal    
    \brief
    Sets the referenced x,y and z values to the indices of the grid cell containing the referenced point.

    \param point    Input reference to point to be located in grid
    \param x        Output reference to x value in grid
    \param y        Output reference to y value in grid
    \param z        Output reference to z value in grid
    */
    EA_FORCE_INLINE void GetCellIndices(const rwpmath::Vector3 & point, int32_t & x, int32_t & y, int32_t & z) const
    {
        // Calc the cell index
        rwpmath::Vector3 offset(point - m_extent.Min());
        rwpmath::Vector3 cellEntry(offset * m_recipCellSize);

        // TODO: RG Get math to implement Vector IntFloor?
        x = rwpmath::IntFloor(cellEntry.GetX());
        y = rwpmath::IntFloor(cellEntry.GetY());
        z = rwpmath::IntFloor(cellEntry.GetZ());
    }


    /**
    \internal    
    \brief
    Returns the cell index that the min of the box occupies.

    If the min of the box is outside of the grid extent then 0 is returned. If the box is larger in any axis
    than the cell 0 is returned.

    Cell zero is used to store out of extent and larger than cell boxes.

    \param bbox     Reference to the bounding box.

    \return         Index of cell the minimum extent of the bounding box occupies.
    */
    EA_FORCE_INLINE uint32_t GetCellIndex(const AABBox & bbox) const
    {
        uint32_t cellIndex(0);

        // Use min point for insertion
        if (ContainsPoint(m_extent, bbox.Min()))
        {
            rwpmath::Vector3 bboxExtent(bbox.Max() - bbox.Min());

            if (m_cellSize.GetX() > bboxExtent.GetX() &&
                m_cellSize.GetY() > bboxExtent.GetY() &&
                m_cellSize.GetZ() > bboxExtent.GetZ())
            {
                int32_t x, y, z;
                GetCellIndices(bbox.Min(), x, y, z);

                cellIndex = GetCellIndex(x, y, z);
            }
        }

        return cellIndex;
    }


    /**
    \brief
    Returns the cell index that the min of the bbox occupies.

    This function does no checking on the bbox to make sure it is with in the regular grids extent. The min
    of the bounding box must be inside the grids extent to use this function.
    
    \param bbox     Reference to the bounding box.

    \return         Index of the cell the minimum extent of the bounding box occupies.
    
    */
    EA_FORCE_INLINE uint32_t GetCellIndexFast(const AABBox & bbox) const
    {
        EA_ASSERT(ContainsPoint(m_extent, bbox.Min()) && 
            m_cellSize.GetX() > (bbox.Max() - bbox.Min()).GetX() &&
            m_cellSize.GetY() > (bbox.Max() - bbox.Min()).GetY() &&
            m_cellSize.GetZ() > (bbox.Max() - bbox.Min()).GetZ());
        // Use min point for insertion
        int32_t x, y, z;
        GetCellIndices(bbox.Min(), x, y, z);

        uint32_t cellIndex(GetCellIndex(x, y, z));

        return cellIndex;
    }


    /**
    \internal    
    \brief
    Adds the indexed entry to the list associated with the indexed cell.
    
    \param iEntry   Index of entry to be added.
    \param iCell    Index of cell entry is to be added to.  
    */
    EA_FORCE_INLINE void
    AddEntryToCell(uint32_t iEntry, uint32_t iCell)
    {
        Cell  &cell  = m_cells[iCell];
        Entry &entry = m_entries[iEntry];

        // Wire together the entry into the cells single indexed list.
        uint32_t nextCellEntry(cell.GetEntryIndex());
        entry.Init(iCell, nextCellEntry);
        cell.SetEntryIndex(iEntry);
    }


    /**
    \internal
    \brief
    Removes the indexed entry from grid
    \param iEntry   Index of entry to be removed
    */
    EA_FORCE_INLINE void
    RemovedEntryFromCell(uint32_t iEntry)
    {
        Entry &entry = m_entries[iEntry];

        uint32_t iCell(entry.GetCellIndex());
        Cell &cell = m_cells[iCell];

        // Find entry in the list
        entry_type * entryIndexPtr(&cell.m_entry);

        while (*entryIndexPtr != (entry_type)iEntry)
        {
            EA_ASSERT(*entryIndexPtr != (entry_type)rwREGULARGRID_MAX_ENTRIES);
            entryIndexPtr = &m_entries[*entryIndexPtr].m_next;
        }

        *entryIndexPtr = m_entries[*entryIndexPtr].m_next;
    }


    /**
    \internal    
    \brief
    Initializes a GridIterator with the supplied min and max values.
    \param gridIterator     Reference to GridIterator to be initialized
    \param xMin             Minimum of x range to initialize GridIterator to.
    \param yMin             Minimum of y range to initialize GridIterator to.
    \param zMin             Minimum of z range to initialize GridIterator to.
    \param xMax             Maximum of x range to initialize GridIterator to.
    \param yMax             Maximum of y range to initialize GridIterator to.
    \param zMax             Maximum of z range to initialize GridIterator to.
    */
    EA_FORCE_INLINE void InitializeGridIterator(GridIterator & gridIterator,
                                       int32_t xMin, int32_t xMax,
                                       int32_t yMin, int32_t yMax,
                                       int32_t zMin, int32_t zMax) const
    {
        // Clamp to the grid size in all directions.
        xMin = xMin < 0 ? 0 : xMin;
        xMax = xMax < m_xCells ? xMax : (m_xCells - 1);

        yMin = yMin < 0 ? 0 : yMin;
        yMax = yMax < m_yCells ? yMax : (m_yCells - 1);

        zMin = zMin < 0 ? 0 : zMin;
        zMax = zMax < m_zCells ? zMax : (m_zCells - 1);

        if (xMin > xMax || yMin > yMax || zMin > zMax)
        {
            gridIterator.Init(1, 0, 1, 0, 1, 0);
        }
        else
        {
            gridIterator.Init(xMin, xMax, yMin, yMax, zMin, zMax);
        }
    }


private:
    /**    
    \internal
    \brief
    Initialize the regular grid data structure.

    \param maxEntries   Maximum number of entries that can be stored in the regular grid. This value must be
                        the same as that passed to the GetResourceDescriptor function.
    \param xcells       The number of cells in the X axis.
    \param ycells       The number of cells in the Y axis.
    \param zcells       The number of cells in the Z axis.
    \param extent       The volume of space covered by the far field.
    */
    TRegularGrid(uint32_t maxEntries,
                 uint32_t xcells,
                 uint32_t ycells,
                 uint32_t zcells,
                 const AABBox & extent)
    {
        EA_ASSERT(maxEntries <= (entry_type)rwREGULARGRID_MAX_ENTRIES);

        m_extent = extent;

        // Calc Cell size
        rwpmath::Vector3 extentDiag(extent.Max() - extent.Min());
        rwpmath::Vector3 vecCells((rwpmath::Vector3::FloatType)xcells,
                                  (rwpmath::Vector3::FloatType)ycells,
                                  (rwpmath::Vector3::FloatType)zcells);

        m_cellSize = extentDiag / vecCells;
#if RW_MATH_VERSION >= RW_MATH_CREATE_VERSION_NUMBER(1, 5, 0)
        m_recipCellSize = rwpmath::Reciprocal(m_cellSize);
#else
        m_recipCellSize = rwpmath::Vector3(1.0f / m_cellSize.X(), 1.0f / m_cellSize.Y(), 1.0f / m_cellSize.Z());
#endif
        m_maxEntries = maxEntries;

        m_xCells = (int32_t)xcells;
        m_yCells = (int32_t)ycells;
        m_zCells = (int32_t)zcells;

        uint32_t numCells((xcells * ycells * zcells) + 1);

        // Setup pointers for entry bboxes, nodes and entry data
        uintptr_t addr = (uintptr_t)this;

        addr += sizeof(TRegularGrid);

        addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwREGULARGRID_BBOX_ALIGNMENT);
        m_bboxes = reinterpret_cast<AABBox *>(addr);
        addr += m_maxEntries * sizeof(AABBox);

        addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwREGULARGRID_ENTRY_ALIGNMENT);
        m_entries = reinterpret_cast<Entry *>(addr);
        addr += m_maxEntries * sizeof(Entry);

        addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwREGULARGRID_CELL_ALIGNMENT);
        m_cells = reinterpret_cast<Cell *>(addr);

        // Initialize cells
        for (uint32_t i(0); i < numCells; ++i)
        {
            m_cells[i].Init();
        }
    }

public:

    /**
    \brief
    Return the memory requirements of an regular grid.

    \param maxEntries   Maximum number of entries that can be stored in the regular grid.
    \param xcells       The number of cells in the X axis.
    \param ycells       The number of cells in the Y axis.
    \param zcells       The number of cells in the Z axis.
    \param extent       The volume of space covered by the far field.

    \return the EA::Physics::SizeAndAlignment
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t maxEntries,
                          uint32_t xcells,
                          uint32_t ycells,
                          uint32_t zcells,
                          const AABBox & /*extent*/)
    {
        EA_ASSERT(maxEntries <= (entry_type)rwREGULARGRID_MAX_ENTRIES);
        EA_ASSERT(EA::Physics::SizeAlign<uint32_t>(sizeof(AABBox), rwREGULARGRID_BBOX_ALIGNMENT) == sizeof(AABBox));

        uint32_t size = 0;

        // Base struct
        size += sizeof(TRegularGrid);

        // Entry bboxes
        size = EA::Physics::SizeAlign<uint32_t>(size, rwREGULARGRID_BBOX_ALIGNMENT);
        size += maxEntries * sizeof(AABBox);

        // Entries
        size = EA::Physics::SizeAlign<uint32_t>(size, rwREGULARGRID_ENTRY_ALIGNMENT);
        size += maxEntries * sizeof(Entry);

        // Cells
        size = EA::Physics::SizeAlign<uint32_t>(size, rwREGULARGRID_CELL_ALIGNMENT);
        uint32_t numCells((xcells * ycells * zcells) + 1u);
        size += numCells * sizeof(Cell);

        return EA::Physics::SizeAndAlignment(size, rwREGULARGRID_ALIGNMENT);
    }


    /**
    \brief
    Initialize the regular grid.

    \param resource     The EA::Physics::MemoryPtr that the RegularGrid is initialized into.
    \param maxEntries   Maximum number of entries that can be stored in the regular grid. This value must be
                        the same as that passed to the GetResourceDescriptor function.
    \param xcells       The number of cells in the X axis.
    \param ycells       The number of cells in the Y axis.
    \param zcells       The number of cells in the Z axis.
    \param extent       The volume of space covered by the far field.
    */
    EA_FORCE_INLINE static TRegularGrid *
    Initialize(const EA::Physics::MemoryPtr& resource,
               uint32_t maxEntries, 
               uint32_t xcells,
               uint32_t ycells,
               uint32_t zcells,
               const AABBox &extent)
    {
        TRegularGrid* regularGrid = new (resource.memory) TRegularGrid(maxEntries, xcells, ycells, zcells, extent);
        return regularGrid;
    }

    /**
    \brief
    Destruct the regular grid.
    */
    EA_FORCE_INLINE void Release()
    {
    }


    /**
    \brief
    Insert an entry into the regular grid with a particular index. The index must not already be in use.
    It is up to the caller to manage which indices are free.

    \param iEntry   Index of entry to be inserted.
    \param bbox     The bounding box of the entry.
    */
    EA_FORCE_INLINE void Insert(uint32_t iEntry, const AABBox &bbox)
    {
        EA_ASSERT(iEntry < m_maxEntries);

        // Set bbox
        m_bboxes[iEntry] = bbox;

        // Get the cell index for the box
        uint32_t iCell(GetCellIndex(bbox));

        // Add the box to the cell
        AddEntryToCell(iEntry, iCell);

#ifdef EA_DEBUG
        if (0 == iCell)
        {
            static uint32_t mc(0);

            if (++mc < 20)
            {
                EAPHYSICS_MESSAGE("Performance Warning: Object %u is outside of the regular grids bounding box.", iEntry);
            }
        }
#endif
    }


    /**
    \brief
    Insert an entry into the regular grid with a particular index. The index must not already be in use.
    It is up to the caller to manage which indices are free.

    This function does no checking on the bbox to make sure it is with in the regular grids extent. The min
    of the bounding box must be inside the grids extent to use this function.

    \param iEntry   Index of entry to be inserted.
    \param bbox     The bounding box of the entry.
    */
    EA_FORCE_INLINE void InsertFast(uint32_t iEntry, const AABBox &bbox)
    {
        EA_ASSERT(iEntry < m_maxEntries);

        // Set bbox
        m_bboxes[iEntry] = bbox;

        // Get the cell index for the box
        uint32_t iCell(GetCellIndexFast(bbox));

        // Add the box to the cell
        AddEntryToCell(iEntry, iCell);
    }


    /**
    \brief
    Update an regular grids entry's bounding box.

    \param iEntry  Index of entry.
    \param bbox    New bounding box.
    */
    EA_FORCE_INLINE void Update(uint32_t iEntry, const AABBox &bbox)
    {
        // Set bbox
        m_bboxes[iEntry] = bbox;

        // Get the cell index for the box
        uint32_t iCell(GetCellIndex(bbox));

        // If the cell index is not the same as the current then move the entry to the new cell.
        if (iCell != m_entries[iEntry].GetCellIndex())
        {
            // Add the box to the cell
            RemovedEntryFromCell(iEntry);
            AddEntryToCell(iEntry, iCell);
        }

#ifdef EA_DEBUG
        if (0 == iCell)
        {
            static uint32_t mc(0);

            if (++mc < 20)
            {
                EAPHYSICS_MESSAGE("Performance Warning: Object %u is outside of the regular grids bounding box.", iEntry);
            }
        }
#endif
    }


    /**
    \brief
    Update a regular grids entry's bounding box.

    This function does no checking on the bbox to make sure it is with in the regular grids extent. The min
    of the bounding box must be inside the grids extent to use this function.

    \param iEntry  Index of entry.
    \param bbox    New bounding box.
    */
    EA_FORCE_INLINE void UpdateFast(uint32_t iEntry, const AABBox &bbox)
    {
        // Set bbox
        m_bboxes[iEntry] = bbox;

        // Get the cell index for the box
        uint32_t iCell(GetCellIndexFast(bbox));

        // If the cell index is not the same as the current then move the entry to the new cell.
        if (iCell != m_entries[iEntry].GetCellIndex())
        {
            // Add the box to the cell
            RemovedEntryFromCell(iEntry);
            AddEntryToCell(iEntry, iCell);
        }

#ifdef EA_DEBUG
        if (0 == iCell)
        {
            static uint32_t mc(0);

            if (++mc < 20)
            {
                EAPHYSICS_MESSAGE("Performance Warning: Object %u is outside of the regular grids bounding box.", iEntry);
            }
        }
#endif
    }


    /**
    \brief
    Remove an entry from the regular grid.

    \param iEntry  Index of entry to be removed.
    */
    EA_FORCE_INLINE void Remove(uint32_t iEntry)
    {
        EA_ASSERT(iEntry < m_maxEntries);
        RemovedEntryFromCell(iEntry);
    }


    /**
    \brief
    Retrieve an entry's bounding box.

    \param index    Index of entry.
    \return Pointer to the bounding box. This cannot be modified.
    */
    EA_FORCE_INLINE const AABBox * GetEntryBBox(uint32_t index) const
    {
        return &m_bboxes[index];
    }


    // *******************************************************************************************************
    //                                      TRegularGrid::BBoxQuery CLASS
    // *******************************************************************************************************

    /**
    \brief
    Query object that may be used to find all entries whose bounding boxes overlap a given query box.

    \importlib rwccore
    */
    class BBoxQuery
    {
    private:
        // Query BBox 
        AABBox              m_bbox;

        // Cell range to process
        GridIterator        m_gridIterator;

        // Current cell being processed
        uint32_t            m_currentCell;

        // Current entry in cell being processes
        uint32_t            m_nextEntry;

    public:
        // RegularGrid we're querying
        const TRegularGrid * m_regularGrid;

        /**
        \brief
        Initialize a regular grid bounding box query. This will return all entries that overlap
        the bounding box. Use the GetNext method to iterate through the results.

        \param regularGrid   Pointer to the regular grid.
        \param bbox     Bounding box region to be queried.
        */
        inline BBoxQuery(const TRegularGrid * regularGrid, const AABBox & bbox)
            :   m_bbox(bbox),
                m_regularGrid(regularGrid)
        {
            // Pad the bounding box in the min by a cell size.
            AABBox paddedBBox(bbox.Min() - m_regularGrid->m_cellSize, bbox.Max());

            // Find the start and end cells of the min and max of the box in cell index space.
            int32_t xMin, yMin, zMin;
            m_regularGrid->GetCellIndices(paddedBBox.Min(), xMin, yMin, zMin);

            int32_t xMax, yMax, zMax;
            m_regularGrid->GetCellIndices(paddedBBox.Max(), xMax, yMax, zMax);

            // Initialize the grid iterator this will clamp and handle out of range cell sets.
            regularGrid->InitializeGridIterator(m_gridIterator, xMin, xMax, yMin, yMax, zMin, zMax);

            // Always have to start with cell zero so any large or outside the grid extent boxes also get considered.
            m_currentCell = 0;
            m_nextEntry = (entry_type)rwREGULARGRID_MAX_ENTRIES;
        }


        /**
        \internal
        \brief
        Gets the next entry from the cells containing the bounding box

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        inline bool GetNextEntry(uint32_t & entry)
        {
            EA_ASSERT((cell_type)rwREGULARGRID_MAX_CELLS != m_currentCell);

            if ((entry_type)rwREGULARGRID_MAX_ENTRIES == m_nextEntry)
            {
                m_nextEntry = m_regularGrid->m_cells[m_currentCell].GetEntryIndex();
            }
            else
            {
                m_nextEntry = m_regularGrid->m_entries[m_nextEntry].GetNext();
            }

            while ((entry_type)rwREGULARGRID_MAX_ENTRIES == m_nextEntry)
            {
                // No more entries in the current cell to process. Find the next cell
                int32_t x = -1, y = -1, z = -1;
                if (m_gridIterator.GetNextCellIndices(x, y, z))
                {
                    m_currentCell = m_regularGrid->GetCellIndex(x, y, z);
                    EA_ASSERT(m_currentCell < ((m_regularGrid->m_xCells * m_regularGrid->m_yCells * m_regularGrid->m_zCells) + 1u));

                    // There is another cell to process get the first entry. 
                    m_nextEntry = m_regularGrid->m_cells[m_currentCell].GetEntryIndex();

                    // If the cell is empty then we will loop back around and find the next cell and
                    // keep looking until there are no more cells to process
                }
                else
                {
                    return false;
                }
            }

            entry = m_nextEntry;

            return true;
        }


        /**
        

        \brief
        Gets the next entry whose bounding box overlaps the query box.

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        EA_FORCE_INLINE bool GetNext(uint32_t & entry)
        {
            while (GetNextEntry(entry))
            {
                if (m_bbox.Overlaps(m_regularGrid->m_bboxes[entry]))
                {
                    return true;
                }
            }

            return false;
        }
    };

    // *******************************************************************************************************
    //                                      TRegularGrid::LineQuery CLASS
    // *******************************************************************************************************

    /**
    \brief
    Query object to find all entries in a regular grid whose bounding box intersects a line.

    \importlib rwccore
    */
    class LineQuery
    {
    private:
        // Line clipper object
        AALineClipper       m_aaLineClipper;

        rwpmath::Vector3    m_td;

        float             m_tx;
        float             m_ty;
        float             m_tz;

        // Cell range to process
        GridIterator        m_gridIterator;

        // Current cell being processed
        uint32_t            m_currentCell;

        // Current entry in cell being processes
        uint32_t            m_nextEntry;

        int32_t             m_curCellX;
        int32_t             m_curCellY;
        int32_t             m_curCellZ;

        int32_t             m_endCellX;
        int32_t             m_endCellY;
        int32_t             m_endCellZ;

        int32_t             m_sizeCellX;
        int32_t             m_sizeCellY;
        int32_t             m_sizeCellZ;

        int32_t             m_leadingEdgeCellX;
        int32_t             m_leadingEdgeCellY;
        int32_t             m_leadingEdgeCellZ;

        int8_t              m_cellDisplacementX;
        int8_t              m_cellDisplacementY;
        int8_t              m_cellDisplacementZ;

        int8_t              m_cellDisplacementPad;

        // RegularGrid we're querying
        const TRegularGrid * m_regularGrid;

    public:
        /**
        \brief
        Initialize an regular grid line query. This can be used to find all entries whose bounding box
        intersect the line. Use the GetNext method to find the next result.

        \param regularGrid Pointer to the regular grid.
        \param start       Position of the start of the line.
        \param end         Position of the end of the line.
        \param fatness     Optional fatness of the line (equivalent to swept box).
         */
        LineQuery(const TRegularGrid *regularGrid,
                  rwpmath::Vector3::InParam start,
                  rwpmath::Vector3::InParam end,
                  const float fatness = 0.0f)
            : m_aaLineClipper(start, end, rwpmath::Vector3(fatness, fatness, fatness), regularGrid->m_extent),
              m_regularGrid(regularGrid)
        {
            EA_ASSERT(fatness >= 0.0f);

            // Find the sub grids min cell indices, this is also the starting grid cell.
            m_regularGrid->GetCellIndices(start - (m_regularGrid->m_cellSize + fatness), m_curCellX, m_curCellY, m_curCellZ);

            // Find the sub grids max cell indices.
            int32_t maxCellX, maxCellY, maxCellZ;
            m_regularGrid->GetCellIndices(start + fatness, maxCellX, maxCellY, maxCellZ);

            // Initialize the grid iterator, this will clamp and handle out of range cell sets.
            regularGrid->InitializeGridIterator(m_gridIterator,
                                                m_curCellX, maxCellX,
                                                m_curCellY, maxCellY,
                                                m_curCellZ, maxCellZ);

            // Calc the sub grids size.
            m_sizeCellX = maxCellX - m_curCellX;
            m_sizeCellY = maxCellY - m_curCellY;
            m_sizeCellZ = maxCellZ - m_curCellZ;

            // Calc the end cell grid indices.
            m_regularGrid->GetCellIndices(end - (m_regularGrid->m_cellSize + fatness), m_endCellX, m_endCellY, m_endCellZ);

#if RW_MATH_VERSION >= RW_MATH_CREATE_VERSION_NUMBER(1, 5, 0)
            rwpmath::Vector3 recipLineDirection(rwpmath::Reciprocal(rwpmath::Abs(end - start)));
#else
            rwpmath::Vector3 recipLineDirection(rwpmath::Abs(end - start));
            recipLineDirection.SetX(1.0f / recipLineDirection.GetX());
            recipLineDirection.SetY(1.0f / recipLineDirection.GetY());
            recipLineDirection.SetZ(1.0f / recipLineDirection.GetZ());
#endif
            // Calc td
            m_td = m_regularGrid->m_cellSize * recipLineDirection;

            // Clac cell min
            rwpmath::Vector3 cellOffset((start - m_regularGrid->m_extent.Min()) * m_regularGrid->m_recipCellSize);
            rwpmath::Vector3 cellFloor(rwpmath::Floor(cellOffset.GetX()),
                                       rwpmath::Floor(cellOffset.GetY()),
                                       rwpmath::Floor(cellOffset.GetZ()));
            rwpmath::Vector3 cellMin(m_regularGrid->m_extent.Min() + (cellFloor * m_regularGrid->m_cellSize));

            // X
            if (start.GetX() < end.GetX())
            {
                // Going left to right
                m_cellDisplacementX = 1;
                m_leadingEdgeCellX = maxCellX;

                float cellMax(cellMin.GetX() + m_regularGrid->m_cellSize.GetX());
                m_tx = (cellMax - start.GetX()) * recipLineDirection.GetX();
            }
            else if (start.GetX() > end.GetX())
            {
                // Going right to left
                m_cellDisplacementX = -1;
                m_leadingEdgeCellX = m_curCellX;

                m_tx = (start.GetX() - cellMin.GetX()) * recipLineDirection.GetX();
            }
            else
            {
                // Perpendicular to X
                m_cellDisplacementX = 0;
                m_leadingEdgeCellX = -1;

                m_tx = rwpmath::GetVecFloat_MaxValue();
            }

            // Y
            if (start.GetY() < end.GetY())
            {
                // Going left to right
                m_cellDisplacementY = 1;
                m_leadingEdgeCellY = maxCellY;

                float cellMax(cellMin.GetY() + m_regularGrid->m_cellSize.GetY());
                m_ty = (cellMax - start.GetY()) * recipLineDirection.GetY();
            }
            else if (start.GetY() > end.GetY())
            {
                // Going right to left
                m_cellDisplacementY = -1;
                m_leadingEdgeCellY = m_curCellY;

                m_ty = (start.GetY() - cellMin.GetY()) * recipLineDirection.GetY();
            }
            else
            {
                // Perpendicular to Y
                m_cellDisplacementY = 0;
                m_leadingEdgeCellY = -1;

                m_ty = rwpmath::GetVecFloat_MaxValue();
            }

            // Z
            if (start.GetZ() < end.GetZ())
            {
                // Going left to right
                m_cellDisplacementZ = 1;
                m_leadingEdgeCellZ = maxCellZ;

                float cellMax(cellMin.GetZ() + m_regularGrid->m_cellSize.GetZ());
                m_tz = (cellMax - start.GetZ()) * recipLineDirection.GetZ();
            }
            else if (start.GetZ() > end.GetZ())
            {
                // Going right to left
                m_cellDisplacementZ = -1;
                m_leadingEdgeCellZ = m_curCellZ;

                m_tz = (start.GetZ() - cellMin.GetZ()) * recipLineDirection.GetZ();
            }
            else
            {
                // Perpendicular to Y
                m_cellDisplacementZ = 0;
                m_leadingEdgeCellZ = -1;

                m_tz = rwpmath::GetVecFloat_MaxValue();
            }

            // Always have to start with cell zero so any large or outside the grid extent boxes also get considered.
            m_currentCell = 0;
            m_nextEntry = (entry_type)rwREGULARGRID_MAX_ENTRIES;
        }


        /**
        
        \internal
        \brief
        Gets the next entry from the cells containing the line

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        bool GetNextEntry(uint32_t & entry)
        {
            EA_ASSERT((cell_type)rwREGULARGRID_MAX_CELLS != m_currentCell);

            if ((entry_type)rwREGULARGRID_MAX_ENTRIES == m_nextEntry)
            {
                m_nextEntry = m_regularGrid->m_cells[m_currentCell].GetEntryIndex();
            }
            else
            {
                m_nextEntry = m_regularGrid->m_entries[m_nextEntry].GetNext();
            }

            while ((entry_type)rwREGULARGRID_MAX_ENTRIES == m_nextEntry)
            {
                // No more entries in the current cell to process. Find the next cell

                int32_t x = -1, y = -1, z = -1;
                bool nextCellfound(m_gridIterator.GetNextCellIndices(x, y, z));

                if (nextCellfound)
                {
                    m_currentCell = m_regularGrid->GetCellIndex(x, y, z);
                    EA_ASSERT(m_currentCell < ((m_regularGrid->m_xCells * m_regularGrid->m_yCells * m_regularGrid->m_zCells) + 1u));

                    // There is another cell to process get the first entry. 
                    m_nextEntry = m_regularGrid->m_cells[m_currentCell].GetEntryIndex();

                    // If the cell is empty then we will loop back around and find the next cell and
                    // keep looking until there are no more cells to process
                }
                else
                {
                    // Walk the line
                    if (m_tx <= m_ty && m_tx <= m_tz)
                    {
                        if (m_curCellX == m_endCellX)
                        {
                            return false;
                        }

                        m_tx += m_td.GetX();
                        m_curCellX += m_cellDisplacementX;
                        m_leadingEdgeCellX += m_cellDisplacementX;

                        m_regularGrid->InitializeGridIterator(m_gridIterator,
                                                              m_leadingEdgeCellX, m_leadingEdgeCellX,
                                                              m_curCellY, m_curCellY + m_sizeCellY,
                                                              m_curCellZ, m_curCellZ + m_sizeCellZ);
                    }
                    else if (m_ty <= m_tx && m_ty <= m_tz)
                    {
                        if (m_curCellY == m_endCellY)
                        {
                            return false;
                        }

                        m_ty += m_td.GetY();
                        m_curCellY += m_cellDisplacementY;
                        m_leadingEdgeCellY += m_cellDisplacementY;

                        m_regularGrid->InitializeGridIterator(m_gridIterator,
                                                              m_curCellX, m_curCellX + m_sizeCellX,
                                                              m_leadingEdgeCellY, m_leadingEdgeCellY,
                                                              m_curCellZ, m_curCellZ + m_sizeCellZ);
                    }
                    else
                    {
                        if (m_curCellZ == m_endCellZ)
                        {
                            return false;
                        }

                        m_tz += m_td.GetZ();
                        m_curCellZ += m_cellDisplacementZ;
                        m_leadingEdgeCellZ += m_cellDisplacementZ;

                        m_regularGrid->InitializeGridIterator(m_gridIterator,
                                                              m_curCellX, m_curCellX + m_sizeCellX,
                                                              m_curCellY, m_curCellY + m_sizeCellY,
                                                              m_leadingEdgeCellZ, m_leadingEdgeCellZ);
                    }
                }
            }

            entry = m_nextEntry;

            return true;
        }

        /**
        \brief
        Gets the next entry whose bounding box overlaps the line.

        \param  entry  Reference to variable that will receive the next entry index.
        \return FALSE if there are no more results
        */
        EA_FORCE_INLINE bool GetNext(uint32_t & entry)
        {
            while (GetNextEntry(entry))
            {
                float pa(0.0f);
                float pb(1.0f);

                if (m_aaLineClipper.ClipToAABBox(pa, pb, m_regularGrid->m_bboxes[entry]))
                {
                    return true;
                }
            }
            return false;
        }


        /**
        \brief
        Modifies the end clip point during iteration over results of an line query. This will eliminate, from
        the iteration process, any cells  that lie further along the line than the given point.

        \param endVal  End clip parameter (should lie between 0 and 1).
        */
        EA_FORCE_INLINE void ClipEnd(float /* endVal */)
        {
            // TODO: RG
        }
    };
};

typedef TRegularGrid<uint16_t, uint32_t>  RegularGrid;


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_REGULARGRID_HPP
