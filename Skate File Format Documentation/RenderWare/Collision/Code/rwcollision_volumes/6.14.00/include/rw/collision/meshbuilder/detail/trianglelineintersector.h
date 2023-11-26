// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLELINEINTERSECTOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLELINEINTERSECTOR_H


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


class TriangleLineIntersector
{

public:

    static bool IntersectLineWithTriangle2D(
        rwpmath::Vector2::InParam trianglePoint0,
        rwpmath::Vector2::InParam trianglePoint1,
        rwpmath::Vector2::InParam trianglePoint2,
        rwpmath::Vector2::InParam linePoint0,
        rwpmath::Vector2::InParam linePoint1);

private:

    static void BestSeparationLineTriangle2D(
        rwpmath::VecFloat & bestSeparation,
        rwpmath::Vector2::InParam trianglePoint0,
        rwpmath::Vector2::InParam trianglePoint1,
        rwpmath::Vector2::InParam trianglePoint2,
        rwpmath::Vector2::InParam linePoint0,
        rwpmath::Vector2::InParam linePoint1,
        rwpmath::Vector2::InParam candidateDirection);
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_TRIANGLELINEINTERSECTOR_H
