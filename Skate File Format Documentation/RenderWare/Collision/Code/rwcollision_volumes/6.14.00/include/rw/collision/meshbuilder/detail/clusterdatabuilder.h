// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTERDATABUILDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTERDATABUILDER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/common.h>

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
class ClusterDataBuilder
{

public:

    typedef detail::VertexList VertexList;
    typedef detail::TriangleList TriangleList;
    typedef detail::TriangleEdgeCodesList TriangleEdgeCodesList;
    typedef detail::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef detail::TriangleGroupIDList TriangleGroupIDList;
    typedef detail::UnitList UnitList;

    /**
    \brief Writes vertex data and unit data to a ClusteredMeshCluster with a UnitCluster.

    \param cluster ClusteredMeshCluster.
    \param unitCluster unit cluster.
    \param vertices collection of vertices.
    \param triangles collection of triangles.
    \param triangleEdgeCodes collection of edge codes.
    \param triangleSurfaceIDs collection of triangle surface IDs.
    \param triangleGroupIDs collection of triangle group IDs.
    \param units collection of Units.
    \param unitParameters the unit parameters
    \param vertexCompressionGranularity vertex compression granularity.
    */
    static void Build(
        rw::collision::ClusteredMeshCluster & cluster,
        const UnitCluster & unitCluster,
        const VertexList & vertices,
        const TriangleList & triangles,
        const TriangleEdgeCodesList & triangleEdgeCodes,
        const TriangleSurfaceIDList & triangleSurfaceIDs,
        const TriangleGroupIDList & triangleGroupIDs,
        const UnitList & units,
        const UnitParameters & unitParameters,
        const float vertexCompressionGranularity);

private:

    typedef meshbuilder::VectorType VectorType;

    /**
    \brief Writes vertex data to a ClusteredMeshCluster.

    \param cluster destination cluster.
    \param vertexIDs collection of vertex IDs.
    \param vertices collection of vertices.
    \param clusterOffset the offset of the clusters vertices, used by clusters with 16bit vertex compression.
    \param vertexCompressionGranularity vertex compression granularity.
    */
    static void WriteVertexDataToCluster(
        rw::collision::ClusteredMeshCluster & cluster,
        const UnitCluster::VertexSet & vertexIDs,
        const uint32_t vertexCount,
        const VertexList & vertices,
        const rw::collision::ClusteredMeshCluster::Vertex32 & clusterOffset,
        const rwpmath::VecFloat & vertexCompressionGranularity);

    /**
    \brief Writes unit data to a ClusteredMeshCluster.

    \param cluster ClusteredMeshCluster.
    \param triangles collection of triangles.
    \param triangleEdgeCodes collection of edge codes.
    \param triangleSurfaceIDs collection of triangle surface IDs.
    \param triangleGroupIDs collection of triangle group IDs.
    \param units collection of Units.
    \param unitCluster unit cluster.
    \param unitParameters unit parameters.
    */
    static void WriteUnitDataToCluster(
        rw::collision::ClusteredMeshCluster & cluster,
        const TriangleList & triangles,
        const TriangleEdgeCodesList & triangleEdgeCodes,
        const TriangleSurfaceIDList & triangleSurfaceIDs,
        const TriangleGroupIDList & triangleGroupIDs,
        const UnitList & units,
        const UnitCluster &unitCluster,
        const UnitParameters & unitParameters);
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTERDATABUILDER_H
