// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINECYLINDER_H
#define RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINECYLINDER_H

#include "rw/collision/volume.h"
#include "rw/collision/cylinder.h"
#include "rw/collision/aalineclipper.h"
#include "rw/collision/deprecated/linetorus.h"


namespace rw
{
namespace collision
{

int32_t rwcCylinderLineSegIntersect(Fraction * dist,
                                    rwpmath::Vector3::InParam orig,
                                    rwpmath::Vector3::InParam seg,
                                    rwpmath::Vector3::InParam center,
                                    rwpmath::Vector3::InParam axis,
                                    float axisLengthSq,
                                    float radius,
                                    RwpBool invert,
                                    RwpBool ignoreInside);
}
}

#endif //!RWCOLLISION_VOLUMES_RW_COLLISION_DEPRECATED_LINECYLINDER_H
