// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/clusterdatabuilder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


void ClusterDataBuilder::Build(
    rw::collision::ClusteredMeshCluster & cluster,
    const UnitCluster & unitCluster,
    const VertexList & vertices,
    const TriangleList & triangles,
    const TriangleEdgeCodesList & triangleEdgeCodes,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitList & units,
    const UnitParameters & unitParameters,
    const float vertexCompressionGranularity)
{
    WriteVertexDataToCluster(
        cluster,
        unitCluster.vertexIDs,
        unitCluster.numVertices,
        vertices,
        unitCluster.clusterOffset,
        vertexCompressionGranularity);

    WriteUnitDataToCluster(
        cluster,
        triangles,
        triangleEdgeCodes,
        triangleSurfaceIDs,
        triangleGroupIDs,
        units,
        unitCluster,
        unitParameters);
}


void ClusterDataBuilder::WriteVertexDataToCluster(
    rw::collision::ClusteredMeshCluster & cluster,
    const UnitCluster::VertexSet & vertexIDs,
    const uint32_t vertexCount,
    const VertexList & vertices,
    const rw::collision::ClusteredMeshCluster::Vertex32 & clusterOffset,
    const rwpmath::VecFloat & vertexCompressionGranularity)
{
    // Write the vertex offset to the cluster
    cluster.SetVertexOffset(clusterOffset);

    // Write the UnitCluster's vertex collection to the cluster
    for (uint32_t vertexIndex = 0 ; vertexIndex < vertexCount ; ++vertexIndex)
    {
        const VectorType & v = vertices[vertexIDs[vertexIndex]];
        cluster.SetVertex(
            rwpmath::Vector3(v.GetX(), v.GetY(), v.GetZ()),
            vertexCompressionGranularity);
    }
}


void ClusterDataBuilder::WriteUnitDataToCluster(
    rw::collision::ClusteredMeshCluster & cluster,
    const TriangleList & triangles,
    const TriangleEdgeCodesList & triangleEdgeCodes,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitList & units,
    const UnitCluster & unitCluster,
    const UnitParameters & unitParameters)
{
    const uint32_t numUnit = unitCluster.numUnits;

    for (uint32_t unitIndex = 0; unitIndex < numUnit; ++unitIndex)
    {
        uint32_t unitId = unitCluster.unitIDs[unitIndex];

        const Unit &unit = units[unitId];

        const TriangleEdgeCodes & triangleEdgeCodes0 = triangleEdgeCodes[unit.tri0];
        const Triangle & triangle0 = triangles[unit.tri0];

        if (unit.type == Unit::TYPE_QUAD)
        {
            const TriangleEdgeCodes & triangleEdgeCodes1 = triangleEdgeCodes[unit.tri1];
            const Triangle & triangle1 = triangles[unit.tri1];

            cluster.SetQuad(
                unitParameters,
                triangleGroupIDs[unit.tri0],
                triangleSurfaceIDs[unit.tri0],
                unitCluster.GetVertexCode(triangle0.vertices[(unit.edgeOpposingExtraVertex + 2) % 3]),
                unitCluster.GetVertexCode(triangle0.vertices[unit.edgeOpposingExtraVertex]),
                unitCluster.GetVertexCode(triangle0.vertices[(unit.edgeOpposingExtraVertex + 1) % 3]),
                unitCluster.GetVertexCode(triangle1.vertices[unit.extraVertex]),
                triangleEdgeCodes0.encodedEdgeCos[(unit.edgeOpposingExtraVertex + 2) % 3],
                triangleEdgeCodes1.encodedEdgeCos[(unit.longestEdgeOnTri1 + 1) % 3],
                triangleEdgeCodes0.encodedEdgeCos[(unit.edgeOpposingExtraVertex + 1) % 3],
                triangleEdgeCodes1.encodedEdgeCos[(unit.longestEdgeOnTri1 + 2) % 3]);
        }
        else
        {
            cluster.SetTriangle(
                unitParameters,
                triangleGroupIDs[unit.tri0],
                triangleSurfaceIDs[unit.tri0],
                unitCluster.GetVertexCode(triangle0.vertices[0]),
                unitCluster.GetVertexCode(triangle0.vertices[1]),
                unitCluster.GetVertexCode(triangle0.vertices[2]),
                triangleEdgeCodes0.encodedEdgeCos[0],
                triangleEdgeCodes0.encodedEdgeCos[1],
                triangleEdgeCodes0.encodedEdgeCos[2]);
        }
    }
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

