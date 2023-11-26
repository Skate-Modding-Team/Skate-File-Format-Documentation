// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcinitialize.cpp

 Purpose: Definitions of functions required to initialize the collision library arena read and write
          callbacks and volume vTable entries.
 */

// ***********************************************************************************************************
// Includes

#include "rw/collision/volume.h"
#include "rw/collision/initialize.h"

// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts


// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions


// ***********************************************************************************************************
// Structs + Unions + Classes


// ***********************************************************************************************************
// Static Functions


// ***********************************************************************************************************
// External Functions
namespace rw
{
namespace collision
{


/**
Initializes all Virtual Table entries for collision library core objects.
\return true on success, false on failure.
*/
bool
InitializeVTables()
{
    Volume::InitializeVTable();

    return true;
}

} // namespace collision
} // namespace rw

// ***********************************************************************************************************
// Class Member Functions



