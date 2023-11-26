// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTERPARAMETERSBUILDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTERPARAMETERSBUILDER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
*/
class ClusterParametersBuilder
{

public:

    typedef detail::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef detail::TriangleGroupIDList TriangleGroupIDList;
    typedef detail::UnitList UnitList;

    /**
    \brief Initializes a ClusterConstructionParameter struct

    \param parameters           ClusterConstructionParameter struct.
    \param unitCluster          UnitCluster used to initialize the parameters.
    \param triangleSurfaceIDs   Collection of triangle surface IDs.
    \param triangleGroupIDs     Collection of triangle group IDs.
    \param units                Collection of Units.
    \param unitParameters       The default unit parameters.
    */
    static void InitializeClusterParameters(
        rw::collision::ClusterConstructionParameters & parameters,
        const UnitCluster & unitCluster,
        const TriangleSurfaceIDList & triangleSurfaceIDs,
        const TriangleGroupIDList & triangleGroupIDs,
        const UnitList & units,
        const UnitParameters & unitParameters);

    /**
    \brief sums each of the individual unit counts.

    These counts are used during ClusteredMeshCluster initialization.

    \param parameters ClusteredMeshCluster construction parameters.
    \param unitType the unit type.
    \param flagsDefault default unit flags.
    \param groupID the unit group ID.
    \param surfaceID the unit surface ID.
    */
    static void SumUnitComponentCounts(
        rw::collision::ClusterConstructionParameters & parameters,
        const uint32_t unitType,
        const uint32_t flagsDefault,
        const TriangleGroupID groupID,
        const TriangleSurfaceID surfaceID);
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTERPARAMETERSBUILDER_H
