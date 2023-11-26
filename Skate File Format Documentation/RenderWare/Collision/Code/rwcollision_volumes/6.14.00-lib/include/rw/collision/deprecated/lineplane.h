// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINEPLANE_H
#define RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINEPLANE_H

#include "rw/collision/volume.h"

namespace rw
{
namespace collision
{
int32_t rwcPlaneLineSegIntersect(Fraction *dist, float orig_i, float seg_i, float sign, float disp);
}
}
#endif //!RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINEPLANE_H
