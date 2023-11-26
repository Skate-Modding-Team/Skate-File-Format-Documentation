// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINETORUS_H
#define RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINETORUS_H

#include "rw/collision/volume.h"

namespace rw
{
    namespace collision
{
    RwpBool SolveQuarticRoots(float coefficients[5], float & root);
    int32_t rwcTorusLineSegIntersect(float & dist, 
                                    rwpmath::Vector3::InParam orig, 
                                    rwpmath::Vector3::InParam dir, 
                                    float majorRadius, 
                                    float minorRadius);
}
}
#endif //!RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINETORUS_H
