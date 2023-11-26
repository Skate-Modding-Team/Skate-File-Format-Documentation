// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_CLUSTERBUILDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_CLUSTERBUILDER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/detail/containers.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
Static helper class that can be used to build a single standalone cluster.

Clusters are the building blocks of clustered meshes. A typical clustered mesh consists
of a large number of clusters. Each cluster contains a spatially coherent set of "units",
which are either triangles or pairs of adjacent triangles (misleadingly called "quads").
The number of units that a cluster can contain is limited.

Spatial queries on the clustered mesh result in hits on zero or more clusters. Each
returned cluster is then typically tested exhaustively, with each unit in the cluster
being considered.

Since clusters are spatially coherent, the idea is that all the units in the cluster
are likely to be worth testing. The acceleration provided by the clustered mesh is in
the rapid elimination of clusters for testing, using a hierarchical tree structure.

Clusters also provide some benefit in their own right. Specifically they are a
compressed format for a collection of triangles, and provide a compact form in which
a collection of spatially coherent triangles can be stored, uploaded and tested.
The vertices of a cluster can be compressed. In a compressed cluster the vertices
are typically represented relative to a single base offset global to the cluster,
with the result that the individual vertex component values tend to have small
absolute values and a smaller range. This range is quantized and mapped to integer
values.

This class provides a mechanism by which individual clusters can be built standalone
without requiring the overhead of building an entire clustered mesh. The intention
is that building of individual clusters can be done rapidly at runtime, providing
a solution for teams that want to build meshes dynamically (for example, from
procedurally generated terrain).

Note that the resulting clusters are not associated with a clustered mesh and would
likely require some other spatial acceleration structure to avoid exhaustive testing
of large numbers of clusters.

\see TriangleClusterProcedural
*/
class ClusterBuilder
{

public:

    typedef detail::VertexList VertexList;
    typedef detail::TriangleList TriangleList;
    typedef detail::TriangleEdgeCodesList TriangleEdgeCodesList;
    typedef detail::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef detail::TriangleGroupIDList TriangleGroupIDList;
    typedef detail::UnitList UnitList;

    /**
    \brief Parameters that control the building of the cluster.
    */
    struct BuildParameters
    {
        BuildParameters() : vertexCompressionGranularity(1.0f)
        {
            unitParameters.unitFlagsDefault = 0u;
            unitParameters.groupIDSize = 0u;
            unitParameters.surfaceIDSize = 0u;
        }

        UnitParameters unitParameters;          ///< Unit description
        float vertexCompressionGranularity;     ///< Controls severity of vertex compression
    };

    /**
    \brief Fills a provided ClusterConstructionParameters struct with parameters describing a cluster.
    
    The ClusterConstructionParameters instance describes the memory requirements of the cluster
    and is required to allocate a cluster.

    \param parameters           The ClusterConstructionParameter struct to be filled.
    \param numVerticesInCluster The number of vertices in the cluster.
    \param numUnitsInCluster    The number of units (triangles or triangle pairs) in the cluster.
    \param triangleSurfaceIDs   Collection of per-triangle surface IDs.
    \param triangleGroupIDs     Collection of per-triangle group IDs.
    \param units                Collection of units comprising the cluster (triangles or triangle pairs).
    \param unitParameters       The default unit parameters.
    \param compressionMode      The compression mode to be used for the cluster vertices.

    \see VertexCompression
    */
    static void InitializeClusterParameters(
        rw::collision::ClusterConstructionParameters &parameters,
        const uint32_t numVerticesInCluster,
        const uint32_t numUnitsInCluster,
        const TriangleSurfaceIDList &triangleSurfaceIDs,
        const TriangleGroupIDList &triangleGroupIDs,
        const UnitList &units,
        const UnitParameters &unitParameters,
        const uint8_t compressionMode);

    /**
    \brief Builds a cluster in a previously allocated cluster instance.

    \note The allocator provided to this method is used for temporary internal allocations,
    not to allocate the cluster itself. All allocations performed by the call are freed before
    return.

    \param cluster              The cluster to be built, allocated previously by the caller.
    \param allocator            An allocator to be used for temporary internal allocations.
    \param buildParameters      Parameters controlling the building of the cluster.
    \param vertices             A collection of vertices referenced by the cluster triangles.
    \param triangles            A collection of triangles referenced by the cluster units.
    \param units                A collection of units comprising the cluster.
    \param triangleEdgeCodes    A collection of per-triangle edge code triples.
    \param triangleSurfaceIDs   A collection of per-triangle surface IDs.
    \param triangleGroupIDs     A collection of per-triangle group IDs.
    \param compressionMode      The compression mode to be used for the cluster vertices.
    \param clusterOffset        The base translation offset of the cluster for vertex compression.

    \return True, if the cluster was built successfully.
    */
    static bool Build(
        rw::collision::ClusteredMeshCluster &cluster,
        EA::Allocator::ICoreAllocator &allocator,
        const BuildParameters &buildParameters,
        const VertexList &vertices,
        const TriangleList &triangles,
        const UnitList &units,
        const TriangleEdgeCodesList &triangleEdgeCodes,
        const TriangleSurfaceIDList &triangleSurfaceIDs,
        const TriangleGroupIDList &triangleGroupIDs,
        const uint8_t compressionMode,
        const rw::collision::ClusteredMeshCluster::Vertex32 &clusterOffset);

private:

    typedef detail::Unit Unit;

    // Not copyable
    ClusterBuilder(const ClusterBuilder &other);
    ClusterBuilder &operator=(const ClusterBuilder &other);
};


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_CLUSTERBUILDER_H
