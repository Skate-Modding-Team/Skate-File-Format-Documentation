// (c) Electronic Arts. All Rights Reserved.


#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDERUTILS_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDERUTILS_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/common.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


class ClusteredMeshBuilderUtils
{

public:

    static uint8_t EdgeCosineToAngleByte(
        const rwpmath::VecFloat & edgeCosine);

    static bool EdgeProducesFeaturelessPlane(
        rwpmath::Vector3::InOutParam edgeA,
        rwpmath::Vector3::InOutParam edgeB,
        rwpmath::Vector3::InParam edgeC,
        const rwpmath::VecFloat & cosineTolerance);

    static bool EdgeDisablesVertex(
        rwpmath::Vector3::InOutParam edgeA,
        rwpmath::Vector3::InOutParam edgeB,
        rwpmath::Vector3::InOutParam edgeC,
        rwpmath::Vector3::InParam planeNormal,
        const rwpmath::VecFloat & coplanarCosineTolerance,
        const rwpmath::VecFloat & cosineTolerance,
        const rwpmath::VecFloat & concaveCosineTolerance);
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDERUTILS_H
