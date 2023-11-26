// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLENORMAL_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLENORMAL_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


class TriangleNormal
{

public:

    static rwpmath::Vector3 ComputeTriangleNormalFast(
        rwpmath::Vector3::InParam p0,
        rwpmath::Vector3::InParam p1,
        rwpmath::Vector3::InParam p2);
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLENORMAL_H
