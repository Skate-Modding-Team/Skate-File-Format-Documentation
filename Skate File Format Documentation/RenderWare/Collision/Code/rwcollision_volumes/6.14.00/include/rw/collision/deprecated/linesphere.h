// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINESPHERE_H
#define RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINESPHERE_H

#include "rw/collision/volume.h"

namespace rw
{
namespace collision
{
int32_t rwcSphereLineSegIntersect(Fraction *dist, 
                                  const rwpmath::Vector3 & orig, 
                                  const rwpmath::Vector3 & seg, 
                                  const rwpmath::Vector3 & center, 
                                  const float radius);
}
}

#endif //!RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINESPHERE_H
