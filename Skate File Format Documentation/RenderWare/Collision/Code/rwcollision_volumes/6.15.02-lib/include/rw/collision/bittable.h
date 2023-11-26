// (c) Electronic Arts. All Rights Reserved.

/**
 \file rw/collision/bittable.h

 \brief Declaration of a simple bit table class
*/

#ifndef RW_COLLISION_BITTABLE_H
#define RW_COLLISION_BITTABLE_H


// ***********************************************************************************************************
// Includes

#include "rw/collision/common.h"

namespace rw
{
namespace collision
{
// ***********************************************************************************************************
// Forward Declarations
struct BitTable;

}}

// Specify name for serializing the class into textual archives - must be called from global namespace
// and CodeWarrior complains if it is moved below the class/structure declaration.
EA_SERIALIZATION_CLASS_NAME(struct rw::collision::BitTable, "rw::BitTable")

namespace rw
{
namespace collision
{

// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts

#define RWBITTABLEARENAOBJECT_LINKNAME ("binarybittablelink")

// ***********************************************************************************************************
// Global Variables

// ***********************************************************************************************************
// External Functions Prototypes


// ***********************************************************************************************************
// Structs + Unions + Classes
/**
    A BitTable is a 2-dimensional table of bits in which bits can be set and
    reset in (row,column) coordinates. Run-on array allocation is used to place the
    bit table at the end of the struct.
\importlib rwbittable
*/
struct BitTable
{
    uint32_t m_rows;        ///< Number of rows in the table.
    uint32_t m_columns;     ///< Number of columns in the table.
    uint32_t m_arraySize;   ///< Number of 32-bit words in the table.
    uint32_t m_array[1];    ///< First word of the table.

    /**
    Construct a Bit Table - Applications should create bit tables by allocating memory and
    using BitTable::Initialize() rather than calling the constructor directly.

    \param rows Number of rows in Bit Table
    \param columns Number of columns in Bit Table
    */
    BitTable(uint32_t rows, uint32_t columns)
        : m_rows(rows),
          m_columns(columns)
    {
        m_arraySize = ((rows * columns)+31)>>5;

    }

    /**
    Get the resource descriptor describing the requirements of an instance of the class.
    \return The resource requirements descriptor.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t numRows, uint32_t numCols)
    {
        uint32_t size = (3 + (((numRows * numCols)+31)>>5)) * sizeof(uint32_t);
        return EA::Physics::SizeAndAlignment(size, EA_ALIGN_OF(uint32_t));
    }

    //See .cpp file for documentation
    static BitTable *
    Initialize(const EA::Physics::MemoryPtr& resource, uint32_t numRows, uint32_t numCols);

    /**
    Releases a bit table object.

    \note It does not free the memory that the bit table was initialized with.
    */
    static void
    Release(BitTable *bitTable)
    {
        bitTable->Release();
    }


    /**
    Releases a bit table object.
    This function is required for BitTables to be compatible with rw::SharedResource.
    It does not free the memory that the bit table was initialized with, therefore
    there is nothing to do.
    */
    void
    Release()
    {
    }


    /**
    Set the identified Bit to 1.
    \param row Bit row location
    \param column Bit column location
    */
    void
    SetBit(uint32_t row, uint32_t column)
    {
        EA_ASSERT_MSG(row < m_rows && column < m_columns, ("Index out of range."));
        ((void)(m_array[((row)*m_columns+(column))>>5] |=
             (1<<(((row)*m_columns+(column))&31))));
    }

    /**
    Set the identified Bit to 0.
    \param row Bit row location
    \param column Bit column location
    */
    void
    ClearBit(uint32_t row, uint32_t column)
    {
        EA_ASSERT_MSG(row < m_rows && column < m_columns, ("Index out of range."));
        ((void)(m_array[((row)*m_columns+(column))>>5] &=
                         ~(1<<(((row)*m_columns+(column))&31))));
    }

    /**
    Set the identified bit to the given value and optionally change the symmetric bit also.
    \param row Bit row location
    \param column Bit column location
    \param value new value of the bit, 0 or 1
    \param symmetric change the (column, row) also, to maintain a symmetric table.
    */
    void
    SetBitValue(uint32_t row, uint32_t column, RwpBool value, RwpBool symmetric = FALSE)
    {
        if (value)
        {
            SetBit(row, column);
            if (symmetric && row != column)
            {
                SetBit(column, row);
            }
        }
        else
        {
            ClearBit(row, column);
            if (symmetric && row != column)
            {
                ClearBit(column, row);
            }
        }
    }

    /**
    Set all the Bits in the table to 0.
    */
    void
    ClearTable()
    {
        for(uint32_t i = 0; i < m_arraySize; i++)
        {
            m_array[i] = 0;
        }
    }

    /**
    Set all the Bits in the table to 1.
    */
    void
    FillTable()
    {
        for(uint32_t i = 0; i < m_arraySize; i++)
        {
            m_array[i] = ~0U;
        }
    }

    /**
    Get the value of the identified Bit.
    \param row Bit row location.
    \param column Bit column location.
    \return 0 or non-zero
    */
    uint32_t
    GetBit(uint32_t row, uint32_t column) const
    {
        EA_ASSERT_MSG(row < m_rows && column < m_columns, ("Index out of range."));
        return  (m_array[((row)*m_columns+(column))>>5] &
                  (1<<(((row)*m_columns+(column))&31))) ;
    }

    /**
    Get number of rows in the table.
    \return number of rows.
    */
    uint32_t
    GetRowCount() const
    {
        return m_rows;
    }

    /**
    Get number of columns in the table.
    \return number of columns.
    */
    uint32_t
    GetColumnCount() const
    {
        return m_columns;
    }



    /**
    The BitTable::ObjectDescriptor struct is required to allow BitTables to be
    serialized using high level versionable serialization. This object
    encapsulates the parameters that are required to calculate the memory
    requirements and to initialize a BitTable object. This object is used in
    conjuction with high level serialization and BitTable::GetResourceDescriptor
    and BitTable::Initialize overloads which accept an ObjectDescriptor.
    */
    struct ObjectDescriptor
    {
        ObjectDescriptor()
        {
        }

        ObjectDescriptor(uint32_t numRows, uint32_t numCols)
          : m_numRows(numRows),
            m_numCols(numCols)
        {
        }

        template <class Archive>
        void Serialize(Archive & ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_numRows);
            ar & EA_SERIALIZATION_NAMED_VALUE(m_numCols);
        }

        uint32_t m_numRows;
        uint32_t m_numCols;
    };

    /**
    Gets an ObjectDescriptor from an existing BitTable
    object which can be used to create a copy of this table.
     */
    const ObjectDescriptor GetObjectDescriptor() const
    {
        return ObjectDescriptor(m_rows, m_columns);
    }

    /**
    Calculate the memory requirements of a BitTable object given an ObjectDescriptor.
     */
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor& objDesc)
    {
        return BitTable::GetResourceDescriptor(objDesc.m_numRows, objDesc.m_numCols);
    }

    /**
    Initialize a BitTable object from a EA::Physics::MemoryPtr and ObjectDescriptor.
     */
    static BitTable* Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor& objDesc)
    {
        return BitTable::Initialize(resource, objDesc.m_numRows, objDesc.m_numCols);
    }


    template <class Archive>
    void Serialize(Archive& ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_rows);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_columns);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_arraySize);

        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_array, m_arraySize);
    }

};

}} //namespace rw::collision

#endif // RW_COLLISION_BITTABLE_H
