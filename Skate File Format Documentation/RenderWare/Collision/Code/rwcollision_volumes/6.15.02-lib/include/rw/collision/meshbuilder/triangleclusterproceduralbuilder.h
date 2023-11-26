// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_TRIANGLECLUSTERPROCEDURALBUILDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_TRIANGLECLUSTERPROCEDURALBUILDER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/clusteredmeshcluster.h>
#include <rw/collision/triangleclusterprocedural.h>

#include <rw/collision/meshbuilder/common.h>

#include <rw/collision/meshbuilder/detail/containers.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
\brief Static helper class that can be used to build a TriangleClusterProcedural.

A TriangleClusterProcedural is a procedural aggregate wrapper around a single cluster.
It accepts line queries and bounding box queries, and enables a standalone cluster to
be used as an aggregate volume for collision.

\note Users wishing to build single clusters directly with no aggregate volume
should see \ref ClusterBuilder instead.

\see ClusterBuilder
\see TriangleClusterProcedural
*/
class TriangleClusterProceduralBuilder
{

public:

    typedef detail::VertexList VertexList;
    typedef detail::TriangleList TriangleList;
    typedef detail::TriangleEdgeCodesList TriangleEdgeCodesList;
    typedef detail::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef detail::TriangleGroupIDList TriangleGroupIDList;
    typedef detail::UnitList UnitList;

    /**
    \brief Parameters used to control the building of the cluster and wrapping procedural.
    */
    struct BuildParameters
    {
        BuildParameters() :
          compressVertices(true),
          vertexCompressionGranularity(1.0f)
        {
            unitParameters.unitFlagsDefault = 0u;
            unitParameters.groupIDSize = 0u;
            unitParameters.surfaceIDSize = 0u;
        }

        rw::collision::UnitParameters unitParameters;       ///< Unit description.
        bool compressVertices;                              ///< Enables use of lossy vertex compression.
        float vertexCompressionGranularity;                 ///< Controls severity of vertex compression.
    };

    /**
    \brief Allocates and builds a cluster and a wrapping TriangleClusterProcedural around it.

    \note The cluster and procedural aggregate are allocated by the method using the provided allocator.
    It is the caller's responsibility to free the procedural when finished with it. The cluster is freed
    as part of the freeing of the procedural.

    \param triangleClusterProceduralAllocator   An allocator used to allocate the TriangleClusterProcedural.
    \param workspaceAllocator                   An allocator used to allocate temporary data required by the builder during the build process.
    \param vertices                             A collection of vertices referenced by the triangles in the cluster.
    \param triangles                            A collection of triangles referenced by the units in the cluster.
    \param units                                A collection of units (triangles and triangle pairs) comprising the cluster.
    \param triangleEdgeCodes                    A collection of per-triangle edge code triples for the triangles in the cluster.
    \param triangleSurfaceIDs                   A collection of per-triangle surface IDs for the triangles in the cluster.
    \param triangleGroupIDs                     A collection of per-triangle group IDs for the triangles in the cluster.

    \return The built TriangleClusterProcedural.
    */
    static TriangleClusterProcedural *Build(
        EA::Allocator::ICoreAllocator &triangleClusterProceduralAllocator,
        EA::Allocator::ICoreAllocator &workspaceAllocator,
        const BuildParameters &buildParameters,
        const VertexList &vertices,
        const TriangleList &triangles,
        const UnitList &units,
        const TriangleEdgeCodesList &triangleEdgeCodes,
        const TriangleSurfaceIDList &triangleSurfaceIDs,
        const TriangleGroupIDList &triangleGroupIDs);

private:

    typedef meshbuilder::VectorType VectorType;

    // Not copyable
    TriangleClusterProceduralBuilder(const TriangleClusterProceduralBuilder &other);
    TriangleClusterProceduralBuilder &operator=(const TriangleClusterProceduralBuilder &other);

    /**
    \internal Determines the cluster vertex compression mode using the specified
    vertex compression granularity.
    */
    static void DetermineVertexCompressionMode(
        const BuildParameters &buildParameters,
        uint8_t &compressionMode,
        rw::collision::ClusteredMeshCluster::Vertex32 &clusterOffset,
        const VertexList &vertices);

    /**
    \internal Initializes the TriangleClusterProcedural. This involves allocating the TriangleClusterProcedural
    and Initializing its state.

    \param constructionParameters The construction parameters required to allocate the TriangleClusterProcedural
    resource.
    \return a TriangleClusterProcedural
    */
    static TriangleClusterProcedural *InitializeTriangleClusterProcedural(
        EA::Allocator::ICoreAllocator &triangleClusterProceduralAllocator,
        const ClusterConstructionParameters &constructionParameters);

    /**
    \internal Finalizes the TriangleClusterProcedural. This involves filling in the
    ClusteredMeshCluster object with the vertex and triangle\quad data.

    \param triangleClusterProcedural The TriangleClusterProcedural to finalize.
    */
    static void FinalizeTriangleClusterProcedural(
        rw::collision::TriangleClusterProcedural &triangleClusterProcedural,
        EA::Allocator::ICoreAllocator &workspaceAllocator,
        const BuildParameters &buildParameters,
        const VertexList &vertices,
        const TriangleList &triangles,
        const UnitList &units,
        const TriangleEdgeCodesList &triangleEdgeCodes,
        const TriangleSurfaceIDList &triangleSurfaceIDs,
        const TriangleGroupIDList &triangleGroupIDs,
        const uint8_t compressionMode,
        const rw::collision::ClusteredMeshCluster::Vertex32 &clusterOffset);
};


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_TRIANGLECLUSTERPROCEDURALBUILDER_H
