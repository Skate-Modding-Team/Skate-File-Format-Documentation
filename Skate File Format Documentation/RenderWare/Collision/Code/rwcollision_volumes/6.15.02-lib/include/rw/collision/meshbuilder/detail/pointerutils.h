// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_POINTERUTILS_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_POINTERUTILS_H


/*************************************************************************************************************

File: allocatorutils.h

Purpose: Contains a number of pointer alignment utility functions.
*/

#include "rw/collision/common.h"

#if !defined EA_PLATFORM_PS3_SPU

#include "eaphysics/sizeandalignment.h"

namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{

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
    EA_ASSERT(EA::Physics::IsMemAligned(ptr, alignment));
}

} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_POINTERUTILS_H
