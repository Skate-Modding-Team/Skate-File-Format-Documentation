// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwbittable.cpp

 Purpose: Implementation of a simple bit table class

 */

// ***********************************************************************************************************
// Includes
#include <new>
#include <rw/collision/common.h>
#include "rw/collision/bittable.h"


namespace rw
{
namespace collision
{

// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts


// ***********************************************************************************************************
// Global Variables


// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions


// ***********************************************************************************************************
// Structs + Unions + Classes


// ***********************************************************************************************************
// Static Functions

// ***********************************************************************************************************
// External Functions

// ***********************************************************************************************************
// Class Member Functions

// ***********************************************************************************************************
//                                                Rw... CLASS
// ***********************************************************************************************************

/**
Initializes a memory block as a bit table object with the specified number
of rows and columns with all the bits set to zero.

\param numRows Number of rows in Bit Table
\param numCols Number of columns in Bit Table

\return Ptr to the initialized bit table object.
 */
BitTable *
BitTable::Initialize(const EA::Physics::MemoryPtr& resource, uint32_t numRows, uint32_t numCols)
{
    void * memPtr = resource.GetMemory();
    EA_ASSERT(EA::Physics::IsMemAligned(memPtr, EA_ALIGN_OF(uint32_t)));
    BitTable *bt = new (memPtr) BitTable(numRows, numCols);
    bt->ClearTable();
    return bt;
}

}} //namespace rw::collision
