// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDERMETHODS_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDERMETHODS_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/meshbuilder/common.h>

#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/vertextrianglemap.h>


namespace rw
{
    namespace collision
    {
        class ClusteredMeshCluster;

        namespace meshbuilder
        {
            class UnitClusterStack;

            namespace detail
            {
                class GridSpatialMap;
                class VertexTriangleMap;
            }
        }
    }
}


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


class ClusteredMeshBuilderMethods
{

public:

    typedef meshbuilder::VectorType VectorType;
    typedef meshbuilder::AABBoxType AABBoxType;

    typedef detail::VertexList VertexList;
    typedef detail::TriangleList TriangleList;
    typedef detail::TriangleEdgeCosinesList TriangleEdgeCosinesList;
    typedef detail::TriangleEdgeCodesList TriangleEdgeCodesList;
    typedef detail::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef detail::TriangleGroupIDList TriangleGroupIDList;
    typedef detail::TriangleFlagsList TriangleFlagsList;
    typedef detail::TriangleNeighborsList TriangleNeighborsList;
    typedef detail::UnitList UnitList;

    enum ClusterGenerationFailures{
        CLUSTER_GENERATION_FAILURE_NO_FAILURES = 0x00,
        CLUSTER_GENERATION_FAILURE_OUT_OF_MEMORY = 0x01,
        CLUSTER_GENERATION_FAILURE_MULTI_LEAF_CLUSTER = 0x02
    };

    static void CalculateAverageAndMinimumEdgeLength(
        rwpmath::VecFloat & averageEdgeLength,
        rwpmath::VecFloat & minimumEdgeLength,
        const TriangleList & triangles,
        const VertexList & vertices);

    static void AdjustVertexMergeDistanceTolerance(
        rwpmath::VecFloat & tolerance,
        const rwpmath::VecFloat & averageEdgeLength,
        const rwpmath::VecFloat & minimumEdgeLength);

    static uint32_t ValidateTriangles(
        TriangleFlagsList & triangleFlags,
        const TriangleList & triangles,
        const VertexList & vertices);

    static void DisableInternalTriangles(
        TriangleFlagsList & triangleFlags,
        const TriangleList & triangles,
        const TriangleGroupIDList & triangleGroupIDs,
        const VertexList & vertices,
        const VertexTriangleMap & vertexTriangleMap);

    static void MergeWithPlanes(
        TriangleEdgeCosinesList & triangleEdgeCosines,
        TriangleNeighborsList & triangleNeighbors,
        const TriangleList & triangles,
        const TriangleFlagsList & triangleFlags,
        const VertexList & vertices,
        const rwpmath::Vector3 * const planeNormals,
        const rwpmath::VecFloat * const planeDistances,
        const uint32_t planeCount,
        const rwpmath::VecFloat & coplanarCosineTolerance,
        const rwpmath::VecFloat & coplanarHeightTolerance,
        const rwpmath::VecFloat & maximumEdgeCosineMergeTolerance);

    static void MergeTriangleWithPlane(
        float * const planarTriangleEdgeCosines,
        TriangleEdgeCosinesList & triangleEdgeCosines,
        const TriangleNeighborsList & triangleNeighbors,
        const TriangleList & triangles,
        const VertexList & vertices,
        const uint32_t planarTriangleIndex,
        const uint32_t * const planarTriangleVertexIndices,
        const uint32_t edgeIndex,
        const uint32_t neighborTriangleIndex,
        rwpmath::Vector3::InParam planeNormal);

    static void FixUnmatchedEdges(
        GridSpatialMap & spatialMap,
        const TriangleGroupIDList & triangleGroupIDs,
        TriangleEdgeCosinesList & triangleEdgeCosines,
        TriangleNeighborsList & triangleNeighbors,
        const VertexList & vertices,
        const TriangleList & triangles,
        const TriangleFlagsList & triangleFlags,
        const rwpmath::VecFloat & coplanarCosineTolerance,
        const rwpmath::VecFloat & coplanarHeightTolerance,
        const rwpmath::VecFloat & maximumEdgeCosineMergeTolerance);

    static void SmoothVertices(
        const VertexTriangleMap & vertexTriangleMap,
        const TriangleList & triangles,
        TriangleEdgeCodesList & triangleEdgeCodes,
        const TriangleFlagsList & triangleFlags,
        const VertexList & vertices,
        const rwpmath::VecFloat & coplanarCosineTolerance,
        const rwpmath::VecFloat & cosineTolerance,
        const rwpmath::VecFloat & concaveCosineTolerance);

    static void BuildUnitAABBoxesList(
        AABBoxType * const unitAABBoxList,
        const UnitList & unitList,
        const TriangleList & triangles,
        const VertexList & vertices);

    static void InitializeUnitClustersUsingKDTree(
        LeafMap & leafMap,
        UnitClusterStack & UnitClusterStack,
        const TriangleList & triangles,
        uint32_t * const mergedVertices,
        uint32_t & failureFlags,
        const UnitList & unitList,
        const VertexList & vertices,
        const rw::collision::KDTreeBuilder & kdtreeBuilder);

    static uint32_t AddOrderedUnitsToUnitCluster(
        UnitCluster::VertexSet & clusterVertexIDs,
        uint32_t & clusterVertexCount,
        UnitCluster::UnitID clusterUnitIDs [],
        uint32_t & clusterUnitCount,
        const uint32_t unitIDs [],
        const uint32_t startUnitIndex,
        const uint32_t numUnitsToAdd,
        const TriangleList & triangles,
        const UnitList & unitList,
        const uint32_t maxVerticesPerUnit);

    static void AdjustKDTreeNodeEntriesForClusterCollection(
        UnitClusterStack & unitClusterStack,
        const LeafMap & leafMap,
        const UnitList & unitList,
        const TriangleSurfaceIDList & triangleSurfaceIDs,
        const TriangleGroupIDList & triangleGroupIDs,
        const UnitParameters & unitParameters);

    static void AdjustKDTreeNodeEntriesForCluster(
        const UnitCluster & unitCluster,
        const LeafMap & leafMap,
        const UnitList & unitList,
        const TriangleSurfaceIDList & triangleSurfaceIDs,
        const TriangleGroupIDList & triangleGroupIDs,
        const UnitParameters & unitParameters,
        const uint32_t unitClusterID,
        const uint32_t unitClusterIDShift);

private:

    static void FindQuadVertex(
        uint32_t &triangle2Index,
        uint32_t &quadVertexIndex,
        const uint32_t triangle1Index,
        const uint32_t edgeVertexIndex,
        const uint32_t edgeVertexNextIndex,
        const TriangleList & triangles,
        const TriangleGroupIDList & triangleGroupIDs,
        const TriangleFlagsList & triangleFlags,
        const VertexTriangleMap & vertexTriangleMap);

    static uint32_t FindEdgeByNeighbor(const uint32_t *const neighbors, const uint32_t n);

    static uint32_t FillGridSpatialMap(
        GridSpatialMap & spatialMap,
        uint32_t & triangleIndex,
        const VertexList & vertices,
        const TriangleList & triangles,
        const TriangleFlagsList & triangleFlags,
        const rwpmath::VecFloat & coplanarCosineTolerance);

    static void MergeWithHorizontalTriangles(
        const GridSpatialMap &spatialMap,
        const TriangleGroupIDList & triangleGroupIDs,
        TriangleEdgeCosinesList & triangleEdgeCosines,
        TriangleNeighborsList & triangleNeighbors,
        const VertexList & vertices,
        const TriangleList & triangles,
        const TriangleFlagsList & triangleFlags,
        const rwpmath::VecFloat & coplanarCosineTolerance,
        const rwpmath::VecFloat & coplanarHeightTolerance,
        const rwpmath::VecFloat & maximumEdgeCosineMergeTolerance);

    static bool DoesEdgeLieInAnyTriangle(
        const VertexList & vertices,
        const TriangleList & triangles,
        const TriangleGroupIDList & triangleGroupIDs,
        const uint32_t edgeTriangleIndex,
        rwpmath::Vector3::InParam edgeVertex0,
        rwpmath::Vector3::InParam edgeVertex1,
        const GridSpatialMap &spatialMap,
        const rwpmath::VecFloat & coplanarHeightTolerance);

    static bool DoesEdgeLieInTriangle(
        const VertexList & vertices,
        const TriangleList & triangles,
        const TriangleGroupIDList & triangleGroupIDs,
        const uint32_t edgeTriangleIndex,
        rwpmath::Vector3::InParam edgeVertex0,
        rwpmath::Vector3::InParam edgeVertex1,
        const uint32_t triangleIndex,
        const float height,
        const rwpmath::VecFloat & coplanarHeightTolerance);

    static bool AllCoplanarTriangles(
        VertexTriangleMap::AdjoiningTriangleIterator triangleIterator,
        const VertexTriangleMap::AdjoiningTriangleIterator triangleIteratorEnd,
        const TriangleList & triangles,
        const TriangleFlagsList & triangleFlags,
        const VertexList & vertices,
        const rwpmath::VecFloat & coplanarCosineTolerance);

    static void FindNextEnabledTriangle(
        VertexTriangleMap::AdjoiningTriangleIterator & triangleIterator,
        const VertexTriangleMap::AdjoiningTriangleIterator & triangleIteratorEnd,
        const TriangleFlagsList & triangleFlags);

    static bool VertexIsNonFeature(
        const uint32_t vertexIndex,
        rwpmath::Vector3::InParam vertexPosition,
        VertexTriangleMap::AdjoiningTriangleIterator triangleIterator,
        const VertexTriangleMap::AdjoiningTriangleIterator triangleIteratorEnd,
        const TriangleList & triangles,
        const TriangleFlagsList & triangleFlags,
        const VertexList & vertices,
        const rwpmath::VecFloat & coplanarCosineTolerance,
        const rwpmath::VecFloat & cosineTolerance,
        const rwpmath::VecFloat & concaveCosineTolerance);

    static void GetOppositeVertices(
        rwpmath::Vector3::InOutParam vertexA,
        rwpmath::Vector3::InOutParam vertexB,
        const uint32_t vertexIndex,
        const uint32_t * triangleVertexIndices,
        const VertexList & vertices);

    static void DisableVertex(
        const uint32_t vertexIndex,
        const VertexTriangleMap & vertexTriangleMap,
        const TriangleList & triangles,
        TriangleEdgeCodesList & triangleEdgeCodes,
        const TriangleFlagsList & triangleFlags);

    static void EncodeTriangleVertexData(
        TriangleEdgeCodesList & triangleEdgeCodes,
        const uint32_t triangleIndex,
        const uint32_t vertexIndex,
        const uint8_t vertexFlag);

    static void MeasureEdge(
        rwpmath::Vector3::InParam u,
        rwpmath::Vector3::InParam v,
        rwpmath::VecFloat & minedge,
        rwpmath::VecFloat & totedge);

    static uint32_t WalkBranch(
        rw::collision::KDTreeBuilder::BuildNode * buildNode,
        LeafMap & leafMap,
        UnitClusterStack & UnitClusterStack,
        const TriangleList & triangles,
        uint32_t * const mergedVertices,
        uint32_t & failureFlags,
        const UnitList & unitList,
        const uint32_t * sortedObjects,
        const VertexList & vertices);

    static bool MergeLastTwoClusters(
        UnitClusterStack & UnitClusterStack,
        uint32_t * mergedVertices);

};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDERMETHODS_H
