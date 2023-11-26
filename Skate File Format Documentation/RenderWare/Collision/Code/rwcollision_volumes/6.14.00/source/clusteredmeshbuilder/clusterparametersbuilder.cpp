// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/clusterparametersbuilder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


void ClusterParametersBuilder::InitializeClusterParameters(
    rw::collision::ClusterConstructionParameters & parameters,
    const UnitCluster & unitCluster,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitList & units,
    const UnitParameters & unitParameters)
{
    // Set the per-cluster parameters.
    parameters.mVertexCompressionMode = unitCluster.compressionMode;
    parameters.mVertexCount = static_cast<uint8_t>(unitCluster.numVertices);
    parameters.mSurfaceIDSize = static_cast<uint16_t>(unitParameters.surfaceIDSize);
    parameters.mGroupIDSize = static_cast<uint16_t>(unitParameters.groupIDSize);

    // Set the Per Unit parameters.
    for (uint32_t unitIndex = 0 ; unitIndex < unitCluster.numUnits ; ++unitIndex)
    {
        const Unit & unit = units[unitCluster.unitIDs[unitIndex]];

        SumUnitComponentCounts(
            parameters,
            unit.type,
            unitParameters.unitFlagsDefault,
            triangleGroupIDs[unit.tri0],
            triangleSurfaceIDs[unit.tri0]);
    }
}


void ClusterParametersBuilder::SumUnitComponentCounts(
    rw::collision::ClusterConstructionParameters & parameters,
    const uint32_t unitType,
    const uint32_t flagsDefault,
    const TriangleGroupID groupID,
    const TriangleSurfaceID surfaceID)
{
    uint16_t numEdgeCosines = 0;

    if (Unit::TYPE_QUAD == unitType)
    {
        ++parameters.mQuadUnitCount;
        numEdgeCosines = 4;
    }
    else
    {
        ++parameters.mTriangleUnitCount;
        numEdgeCosines = 3;
    }

    if (flagsDefault & rw::collision::UNITFLAG_EDGEANGLE)
    {
        // NOTE: Can't use += due to bizarre int to uint16_t conversion warning in VS2005?
        parameters.mEdgeCosineCount = (uint16_t) (numEdgeCosines + parameters.mEdgeCosineCount);
    }

    // If the unit contains a group ID
    if ((flagsDefault & rw::collision::UNITFLAG_GROUPID) && (groupID != rw::collision::ClusteredMeshCluster::DEFAULT_GROUPID))
    {
        ++parameters.mGroupIDCount;
    }

    // If the unit contains a surface ID
    if ((flagsDefault & rw::collision::UNITFLAG_SURFACEID) && (surfaceID != rw::collision::ClusteredMeshCluster::DEFAULT_SURFACEID))
    {
        ++parameters.mSurfaceIDCount;
    }
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

