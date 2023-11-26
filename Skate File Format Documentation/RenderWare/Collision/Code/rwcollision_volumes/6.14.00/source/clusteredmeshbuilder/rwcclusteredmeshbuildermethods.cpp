// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>

#include <rw/collision/meshbuilder/edgecosines.h>

#include <rw/collision/meshbuilder/detail/trianglenormal.h>
#include <rw/collision/meshbuilder/detail/trianglevalidator.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/unitclusterbuilder.h>
#include <rw/collision/meshbuilder/detail/unitclusterstack.h>
#include <rw/collision/meshbuilder/detail/clusteredmeshbuildermethods.h>
#include <rw/collision/meshbuilder/detail/clusteredmeshbuilderutils.h>
#include <rw/collision/meshbuilder/detail/gridspatialmap.h>
#include <rw/collision/meshbuilder/detail/trianglelineintersector.h>


using namespace rw::collision;


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/*
\brief Calculates the average and minimum edge lengths of a collection of triangles.

\param averageEdgeLength average edge length.
\param minumumEdgeLength minimum edge length.
\param triangles collection of triangles.
\param vertices collection of vertices.
**/
void
ClusteredMeshBuilderMethods::CalculateAverageAndMinimumEdgeLength(
    rwpmath::VecFloat & averageEdgeLength,
    rwpmath::VecFloat & minimumEdgeLength,
    const TriangleList & triangles,
    const VertexList & vertices)
{
    averageEdgeLength = rwpmath::GetVecFloat_Zero();
    minimumEdgeLength = rwpmath::MAX_FLOAT;

    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        const uint32_t *k = triangles[triangleIndex].vertices;

        const uint32_t index0(k[0]);
        const uint32_t index1(k[1]);
        const uint32_t index2(k[2]);

        rwpmath::Vector3 v0(vertices[index0]);
        rwpmath::Vector3 v1(vertices[index1]);
        rwpmath::Vector3 v2(vertices[index2]);

        MeasureEdge(v0, v1, minimumEdgeLength, averageEdgeLength);
        MeasureEdge(v1, v2, minimumEdgeLength, averageEdgeLength);
        MeasureEdge(v2, v0, minimumEdgeLength, averageEdgeLength);
    }

    averageEdgeLength /= (3.0f * numTriangles);
}


/**
\brief Adjusts the tolerance used to control vertex merging.
The adjustment factor is determined by the larger of the minimum edge length and 100th of
the average edge length. The vertex merge distance tolerance is then multiplied by the adjustment factor.

\param tolerance tolerance to be adjusted by edge length metrics
\param averageEdgeLength average edge length
\param minimumEdgeLength minimum edge length
*/
void
ClusteredMeshBuilderMethods::AdjustVertexMergeDistanceTolerance(
    rwpmath::VecFloat & tolerance,
    const rwpmath::VecFloat & averageEdgeLength,
    const rwpmath::VecFloat & minimumEdgeLength)
{
    tolerance *= rwpmath::Max(minimumEdgeLength, averageEdgeLength * 0.01f);
}


/**
\brief Build the collection of builder triangles

\param triangleFlags collection of triangle flags
\param triangles collection of triangles
\param vertices collection of vertices
\return number of valid triangles
*/
uint32_t
ClusteredMeshBuilderMethods::ValidateTriangles(
    TriangleFlagsList & triangleFlags,
    const TriangleList & triangles,
    const VertexList & vertices)
{
    // This method validates the triangle data, marking degenerates as invalid
    const uint32_t numTriangles(triangles.size());
    uint32_t numDiscardedTriangles(0);

    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        const Triangle &triangle = triangles[triangleIndex];

        const uint32_t v0(triangle.vertices[0]);
        const uint32_t v1(triangle.vertices[1]);
        const uint32_t v2(triangle.vertices[2]);

        const rwpmath::Vector3 p0(vertices[v0]);
        const rwpmath::Vector3 p1(vertices[v1]);
        const rwpmath::Vector3 p2(vertices[v2]);

        // Mark triangles with bad normals/zero areas as invalid
        if (TriangleValidator::IsTriangleValid(p0, p1, p2))
        {
            // Mark the triangle as enabled
            triangleFlags[triangleIndex].enabled = true;
        }
        else
        {
            // Mark the triangle as disabled
            triangleFlags[triangleIndex].enabled = false;
            ++numDiscardedTriangles;
        }
    }

    if (numDiscardedTriangles > 0)
    {
        EAPHYSICS_MESSAGE("Discarding %u of %u triangles because they have negligible area.", numDiscardedTriangles, numTriangles);
    }

    return (numTriangles - numDiscardedTriangles);
}


/**
\brief Finds internal triangles and quads, and disables them.

Internal triangles are pairs of triangle which share all three vertices with each other and have
different groupIDs. Both of the triangle in the pair are disabled.

Internal quads, like the internal triangle, are quads which completely overlap each other
and have different groupIDs. Here any two triangle which share an edge and have a similar groupID
are considered a quad.

\param triangleFlags collection of triangle flags
\param triangles collection of triangles
\param triangleGroupIDs collection of group IDs
\param vertices collection of vertices
\param vertexTriangleMap map from vertex indices to triangle indices
*/
void
ClusteredMeshBuilderMethods::DisableInternalTriangles(
    TriangleFlagsList & triangleFlags,
    const TriangleList & triangles,
    const TriangleGroupIDList & triangleGroupIDs,
    const VertexList & vertices,
    const VertexTriangleMap & vertexTriangleMap)
{
    EA_ASSERT_MSG(triangles.size() != 0, "triangles count should not be zero");
    EA_ASSERT_MSG(triangleGroupIDs.size() != 0, "triangleGroupIDs count should not be zero");
    EA_ASSERT_MSG(triangleFlags.size() != 0, "triangleFlags count should not be zero");
    EA_ASSERT_MSG(vertices.size() != 0, "vert count should not be zero");
    EA_ASSERT_MSG(vertexTriangleMap.IsValid() == true, "vertMap should be valid");

    // For each triangle in the collection
    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangle1Index = 0; triangle1Index < numTriangles; ++triangle1Index)
    {
        // If the triangle has been disabled it can be ignored
        if (triangleFlags[triangle1Index].enabled == false)
        {
            continue;
        }

        // For each edge of the current triangle
        for (uint32_t edge1Index = 0; edge1Index < 3; ++edge1Index)
        {
            // Get the next edge vertex index
            const uint32_t edge1NextIndex = (edge1Index < 2) ? (edge1Index + 1) : 0;

            // Get the current triangle vertex indices
            const uint32_t *const triangle1VertexIndices = triangles[triangle1Index].vertices;

            // Get the triangle normal
            const rwpmath::Vector3 t1Normal(TriangleNormal::ComputeTriangleNormalFast(
                rwpmath::Vector3(vertices[triangle1VertexIndices[0]]),
                rwpmath::Vector3(vertices[triangle1VertexIndices[1]]),
                rwpmath::Vector3(vertices[triangle1VertexIndices[2]])));

            // Get the edge vector
            const rwpmath::Vector3 tri1EdgeVector(vertices[triangle1VertexIndices[edge1NextIndex]] - vertices[triangle1VertexIndices[edge1Index]]);

            // Get the adjoining triangle iterators for the current vertex index
            VertexTriangleMap::AdjoiningTriangleIterator adIt = vertexTriangleMap.AdjoiningTriangleBegin(triangle1VertexIndices[edge1Index]);
            VertexTriangleMap::AdjoiningTriangleIterator adItEnd = vertexTriangleMap.AdjoiningTriangleEnd(triangle1VertexIndices[edge1Index]);

            // Iterate through the surrounding triangles
            while (adIt != adItEnd)
            {
                // Get the adjoining triangle index
                const uint32_t triangle2Index = *adIt;

                // If the triangle is a valid candidate for removal
                if (triangle1Index < triangle2Index &&
                    triangleFlags[triangle2Index].enabled)
                {
                    // Get the adjoining triangle vertex indices
                    const uint32_t *const triangle2VertexIndices = triangles[triangle2Index].vertices;

                    // Get the adjoining triangle normal
                    rwpmath::Vector3 t2Normal(TriangleNormal::ComputeTriangleNormalFast(
                        rwpmath::Vector3(vertices[triangle2VertexIndices[0]]),
                        rwpmath::Vector3(vertices[triangle2VertexIndices[1]]),
                        rwpmath::Vector3(vertices[triangle2VertexIndices[2]])));

                    // For each edge of the adjoining triangle
                    for (uint32_t edge2Index = 2, edge2NextIndex = 0; edge2NextIndex < 3; edge2Index = edge2NextIndex++)
                    {
                        // If we have a matching edge
                        if (triangle1VertexIndices[edge1Index] == triangle2VertexIndices[edge2NextIndex] &&
                            triangle2VertexIndices[edge2Index] == triangle1VertexIndices[edge1NextIndex])
                        {
                            // Determine the edge cosine between the two triangles
                            rwpmath::VecFloat edgeCosine(EdgeCosines::ComputeExtendedEdgeCosine(
                                t1Normal,
                                t2Normal,
                                tri1EdgeVector));

                            // If the edge cosine indicates the triangles are within the coplanar tolerance
                            if (edgeCosine > rwpmath::VecFloat(2.99f) || edgeCosine < rwpmath::VecFloat(-0.99f))
                            {
                                // If the current triangle and the adjacent triangle share 3 vertices
                                if (triangle1VertexIndices[(edge1NextIndex < 2) ? (edge1NextIndex + 1) : 0] == triangle2VertexIndices[(edge2NextIndex < 2) ? (edge2NextIndex + 1) : 0])
                                {
                                    // Disabled both triangles
                                    triangleFlags[triangle1Index].enabled = false;
                                    triangleFlags[triangle2Index].enabled = false;
                                }
                                else
                                {
                                    // Determine if we have a coplanar quad
                                    uint32_t quad1ExtraVertexIndex = 0;
                                    uint32_t quad1ExtraTriangleIndex = 0;
                                    uint32_t quad2ExtraVertexIndex = 0;
                                    uint32_t quad2ExtraTriangleIndex = 0;

                                    const uint32_t triangle1OppositeVertexIndex = triangle1VertexIndices[(edge1NextIndex < 2) ? (edge1NextIndex + 1) : 0];

                                    // Find the index of the triangle which shares an edge with triangle 1 and its quad vertex
                                    FindQuadVertex(
                                        quad1ExtraTriangleIndex,
                                        quad1ExtraVertexIndex,
                                        triangle1Index,
                                        triangle1VertexIndices[edge1NextIndex],
                                        triangle1OppositeVertexIndex,
                                        triangles,
                                        triangleGroupIDs,
                                        triangleFlags,
                                        vertexTriangleMap);

                                    const uint32_t triangle2OppositeVertexIndex = triangle2VertexIndices[(edge2NextIndex < 2) ? (edge2NextIndex + 1) : 0];

                                    // Find the index of the triangle which shares an edge with triangle 2 and its quad vertex
                                    FindQuadVertex(
                                        quad2ExtraTriangleIndex,
                                        quad2ExtraVertexIndex,
                                        triangle2Index,
                                        triangle2VertexIndices[edge2NextIndex],
                                        triangle2OppositeVertexIndex,
                                        triangles,
                                        triangleGroupIDs,
                                        triangleFlags,
                                        vertexTriangleMap);

                                    // If the 4 triangles create an overlaying quad
                                    if (quad1ExtraVertexIndex == triangle2OppositeVertexIndex &&
                                        quad2ExtraVertexIndex == triangle1OppositeVertexIndex)
                                    {
                                        // Disable all of the triangles
                                        triangleFlags[triangle1Index].enabled = false;
                                        triangleFlags[quad1ExtraTriangleIndex].enabled = false;
                                        triangleFlags[triangle2Index].enabled = false;
                                        triangleFlags[quad2ExtraTriangleIndex].enabled = false;
                                    }
                                }
                            }
                        }
                    }
                }

                ++adIt;
            }
        }
    }
}


/**
\brief Merges the collection of triangles with a collection of planes.

\param triangleEdgeCodes collection of triangle edge cosine codes
\param triangles collection of triangles
\param triangleFlags collection of triangle flags
\param vertices collection of vertices
\param planeNormals array of merge plane normals
\param planeDistances array of merge plane distances
\param planeCount count of merge planes
\param coplanarCosineTolernance tolerance used to determine angle similarity
\param coplanarHeightTolerance tolerance used to determine height similarity
\param maximumEdgeCosineMergeTolerance limiting tolerance used to determine when to merge triangles.
*/
void
ClusteredMeshBuilderMethods::MergeWithPlanes(
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
    const rwpmath::VecFloat & maximumEdgeCosineMergeTolerance)
{
    EA_ASSERT_MSG(triangles.size() != 0, "triangles count should not be zero");
    EA_ASSERT_MSG(triangleEdgeCosines.size() != 0, "triangleEdgeCosines count should not be zero");
    EA_ASSERT_MSG(triangleNeighbors.size() != 0, "triangleNeighbors count should not be zero");
    EA_ASSERT_MSG(triangleFlags.size() != 0, "triangleFlags count should not be zero");
    EA_ASSERT_MSG(vertices.size() != 0, "vert count should not be zero");

    // Cycle through all of the planes
    for (uint32_t planeIndex = 0; planeIndex < planeCount; ++planeIndex)
    {
        // Cycle through all triangles
        const uint32_t numTriangles(triangles.size());
        for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
        {
            // Ignore triangle which have been disabled
            if (triangleFlags[triangleIndex].enabled == false)
            {
                continue;
            }

            const rwpmath::Vector3 & currentPlaneNormal = planeNormals[planeIndex];
            const float & currentPlaneDistance = planeDistances[planeIndex];

            const uint32_t * const triangleVertexIndices = triangles[triangleIndex].vertices;
            const uint32_t * const triangleNeighborIndices = triangleNeighbors[triangleIndex].neighbor;

            rwpmath::Vector3 triangleNormal(TriangleNormal::ComputeTriangleNormalFast(
                rwpmath::Vector3(vertices[triangleVertexIndices[0]]),
                rwpmath::Vector3(vertices[triangleVertexIndices[1]]),
                rwpmath::Vector3(vertices[triangleVertexIndices[2]])));

            // for each triangle which lies in the plane
            const bool parallelWithPlane(rwpmath::IsSimilar(-triangleNormal, currentPlaneNormal, coplanarCosineTolerance));
            if (parallelWithPlane)
            {
                const rwpmath::Vector3 trianglePoint(vertices[triangleVertexIndices[1]]);
                const rwpmath::VecFloat distanceFromPlane(rwpmath::Dot(trianglePoint, currentPlaneNormal));
                const bool inPlane(rwpmath::IsSimilar(distanceFromPlane, currentPlaneDistance, coplanarHeightTolerance));

                if (inPlane)
                {
                    // Get the triangle edge cosines
                    float * edgeCosines = triangleEdgeCosines[triangleIndex].edgeCos;

                    // for each edge of the triangle
                    for (uint32_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
                    {
                        // Get the neighboring triangle index
                        uint32_t neighborTriangleIndex = triangleNeighborIndices[edgeIndex];

                        // if the edge cosine is within the target range
                        if ( neighborTriangleIndex != CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH &&
                             edgeCosines[edgeIndex] < maximumEdgeCosineMergeTolerance )
                        {
                            MergeTriangleWithPlane(
                                edgeCosines,
                                triangleEdgeCosines,
                                triangleNeighbors,
                                triangles,
                                vertices,
                                triangleIndex,
                                triangleVertexIndices,
                                edgeIndex,
                                neighborTriangleIndex,
                                currentPlaneNormal);
                        }
                    }
                }                
            }
        }
    }
}


/**
\brief Utility method merges a triangle with a plane along a single edge.

The specified triangle edge cosine is adjusted so that the triangle merges with the plane,
as though that edge was shared with a triangle which lay in the plane.

\param planarTriangleEdgeCosines the edge cosines of the 1st triangle
\param triangleEdgeCodes collection of triangle edge cosine codes
\param triangles collection of triangles
\param vertices collection of vertices
\param planarTriangleIndex index of triangle which lies in the plane
\param planarTriangleVertexIndices the indices of the 1st triangles vertices
\param edgeIndex the index of the edge to be merged
\param neighborTriangleIndex the index of the neighboring triangle
\param planeNormal normal of the merge plane
*/
void
ClusteredMeshBuilderMethods::MergeTriangleWithPlane(
    float * const planarTriangleEdgeCosines,
    TriangleEdgeCosinesList & triangleEdgeCosines,
    const TriangleNeighborsList & triangleNeighbors,
    const TriangleList & triangles,
    const VertexList & vertices,
    const uint32_t planarTriangleIndex,
    const uint32_t * const planarTriangleVertexIndices,
    const uint32_t edgeIndex,
    const uint32_t neighborTriangleIndex,
    rwpmath::Vector3::InParam planeNormal)
{
    // Neighboring triangle vertex indices
    const uint32_t * const neighborTriangleVertexIndices = triangles[neighborTriangleIndex].vertices;
    const uint32_t * const neighborTriangleNeighborIndices = triangleNeighbors[neighborTriangleIndex].neighbor;
    float * const neighborTriangleEdgeCosines = triangleEdgeCosines[neighborTriangleIndex].edgeCos;

    // Triangle2 normal
    rwpmath::Vector3 neighborTriangleNormal(TriangleNormal::ComputeTriangleNormalFast(
        rwpmath::Vector3(vertices[neighborTriangleVertexIndices[0]]),
        rwpmath::Vector3(vertices[neighborTriangleVertexIndices[1]]),
        rwpmath::Vector3(vertices[neighborTriangleVertexIndices[2]])));

    const uint32_t edgeNextIndex = (edgeIndex < 2) ? (edgeIndex + 1) : 0;

    // Determine the new edge cosine
    rwpmath::VecFloat edgeCosine = EdgeCosines::ComputeExtendedEdgeCosine(
        planeNormal,
        neighborTriangleNormal,
        rwpmath::Vector3(vertices[planarTriangleVertexIndices[edgeNextIndex]] - vertices[planarTriangleVertexIndices[edgeIndex]]));

    // Determine the neighboring triangle index
    uint32_t neighborEdgeIndex = FindEdgeByNeighbor(neighborTriangleNeighborIndices, planarTriangleIndex);

    // Set the edge cosines
    neighborTriangleEdgeCosines[neighborEdgeIndex] = edgeCosine;

    // This value should be 1 since the planar triangle is coplanar with the merge plane.
    planarTriangleEdgeCosines[edgeIndex] += edgeCosine;
}


/**
\brief Fixes edge cosines of edges which have not been neighbored correctly

An edge can be considered to not have neighbored properly if it intersects a
triangle which is not its neighbor, and the edge lies in the plane of the intersected triangle.
This method corrects the edge cosines of edges which lie horizontally and are shared between
a downward facing and upward facing triangle.

A GridSpatialMap is used to reduce the complexity of the issue. The current
implementation of this spatial map makes it difficult to predict the amount of
memory it requires without having added all of the entries.

The current pattern of use consists of setting a maximum number of entries.
The map is filled to this limit and then used. After use the map is filled again
with the remaining entries and used again. This repeats until all entries have
been dealt with.

\param spatialMap spatial map used to reduce complexity of problem
\param triangleGroupIDs collection of group IDs
\param triangleEdgeCosines collection of edge cosines
\param triangleNeighbors collection of triangle neighbor indices
\param vertices collection of vertices
\param triangles collection of triangles
\param triangleFlags collection of triangle flags
\param coplanarCosineTolerance tolerance used to determine whether two cosine values are similar.
\param coplanarHeightTolerance tolerance used to determine whether two height values are similar.
\param maximumEdgeCosineMergeTolerance limiting tolerance used to determine when to merge triangles.
*/
void
ClusteredMeshBuilderMethods::FixUnmatchedEdges(
    GridSpatialMap & spatialMap,
    const TriangleGroupIDList & triangleGroupIDs,
    TriangleEdgeCosinesList & triangleEdgeCosines,
    TriangleNeighborsList & triangleNeighbors,
    const VertexList & vertices,
    const TriangleList & triangles,
    const TriangleFlagsList & triangleFlags,
    const rwpmath::VecFloat & coplanarCosineTolerance,
    const rwpmath::VecFloat & coplanarHeightTolerance,
    const rwpmath::VecFloat & maximumEdgeCosineMergeTolerance)
{
    // The index of the last entry into the spatial map
    uint32_t lastEntry = 0;

    const uint32_t numTriangles(triangles.size());
    while (lastEntry < numTriangles)
    {
        // Attempt to fill the spatial map
        lastEntry = FillGridSpatialMap(
            spatialMap,
            lastEntry,
            vertices,
            triangles,
            triangleFlags,
            coplanarCosineTolerance);

        // Merge the horizontal triangle with the map entries
        MergeWithHorizontalTriangles(
            spatialMap,
            triangleGroupIDs,
            triangleEdgeCosines,
            triangleNeighbors,
            vertices,
            triangles,
            triangleFlags,
            coplanarCosineTolerance,
            coplanarHeightTolerance,
            maximumEdgeCosineMergeTolerance);
    }
}


/**
\brief Disables vertices in a collection, which are considered non-feature vertices.

\param vertexTriangleMap mapping from vertex indices to triangle indices
\param triangles collection of triangles.
\param triangleFlags collection of triangle flags
\param vertices collection of vertices
\param coplanarCosineTolerance coplanar tolerance
\param cosineTolerance Tolerance used to determine when an edge, which is assumed to be coplanar to a
pair of edges lies between those two edges.
*/
void
ClusteredMeshBuilderMethods::SmoothVertices(
    const VertexTriangleMap & vertexTriangleMap,
    const TriangleList & triangles,
    TriangleEdgeCodesList & triangleEdgeCodes,
    const TriangleFlagsList & triangleFlags,
    const VertexList & vertices,
    const rwpmath::VecFloat & coplanarCosineTolerance,
    const rwpmath::VecFloat & cosineTolerance,
    const rwpmath::VecFloat & concaveCosineTolerance)
{
    EA_ASSERT_MSG(triangles.size() != 0, "triangles count should not be zero");
    EA_ASSERT_MSG(triangleEdgeCodes.size() != 0, "triangleEdgeCodes count should not be zero");
    EA_ASSERT_MSG(triangleFlags.size() != 0, "triangleFlags count should not be zero");
    EA_ASSERT_MSG(vertices.size() != 0, "vert count should not be zero");

    VertexTriangleMap::VertexIterator vIt = vertexTriangleMap.VerticesBegin();
    VertexTriangleMap::VertexIterator vItEnd = vertexTriangleMap.VerticesEnd();

    // Iterate through all vertices
    while (vIt != vItEnd)
    {
        // Get current vertex hub index
        const uint32_t vertexHubIndex = *vIt;

        // Get vertex hub position
        rwpmath::Vector3 vertexHub(vertices[vertexHubIndex]);

        VertexTriangleMap::AdjoiningTriangleIterator atIt = vertexTriangleMap.AdjoiningTriangleBegin(vertexHubIndex);
        VertexTriangleMap::AdjoiningTriangleIterator atItEnd = vertexTriangleMap.AdjoiningTriangleEnd(vertexHubIndex);

        // If all triangle are coplanar or a featureless plane can be found
        if (atIt != atItEnd)
        {
            bool disableVertex = AllCoplanarTriangles(
                atIt,
                atItEnd,
                triangles,
                triangleFlags,
                vertices,
                coplanarCosineTolerance);

            disableVertex = disableVertex || VertexIsNonFeature(
                vertexHubIndex,
                vertexHub,
                atIt,
                atItEnd,
                triangles,
                triangleFlags,
                vertices,
                coplanarCosineTolerance,
                cosineTolerance,
                concaveCosineTolerance);

            if (disableVertex)
            {
                // Disable this vertex
                DisableVertex(
                    vertexHubIndex,
                    vertexTriangleMap,
                    triangles,
                    triangleEdgeCodes,
                    triangleFlags);
            }
        }

        // Move the iterator forward
        ++vIt;
    }
}


/**
\brief Builds the AABBoxes of the units.

\param unitAABBoxList List of unit AABBoxes.
\param unitList collection of units
\param triangles collection of triangles.
\param vertices collection of vertices.
*/
void
ClusteredMeshBuilderMethods::BuildUnitAABBoxesList(
    AABBoxType * const unitAABBoxList,
    const UnitList & unitList,
    const TriangleList & triangles,
    const VertexList & vertices)
{
    const uint32_t numUnits(unitList.size());
    for (uint32_t unitIndex = 0; unitIndex < numUnits; ++unitIndex)
    {
        const Unit & unit = unitList[unitIndex];
        const Triangle & triangle = triangles[unit.tri0];

        rwpmath::Vector3 v0(vertices[triangle.vertices[0]]);
        rwpmath::Vector3 v1(vertices[triangle.vertices[1]]);
        rwpmath::Vector3 v2(vertices[triangle.vertices[2]]);

        rwpmath::Vector3 min(rwpmath::Min(rwpmath::Min(v0, v1), v2));
        rwpmath::Vector3 max(rwpmath::Max(rwpmath::Max(v0, v1), v2));

        if (Unit::TYPE_QUAD == unit.type)
        {
            rwpmath::Vector3 v3(vertices[triangles[unit.tri1].vertices[unit.extraVertex]]);
            min = rwpmath::Min(min, v3);
            max = rwpmath::Max(max, v3);
        }

        AABBoxType::Vector3Type aabboxMin(min);
        AABBoxType::Vector3Type aabboxMax(max);
        unitAABBoxList[unitIndex] = AABBoxType(aabboxMin, aabboxMax);
    }
}


/**
\brief Initializes the UnitClusters using the KDTree::BuildNode structure.

\param leafMap maps ID of first unit in each KDTree::BuildNode leaf node to the leaf node ID.
\param unitClusterStack unit cluster allocator.
\param triangles collection of triangles.
\param mergedVertices vertex index array used used during cluster merging.
\param failureFlags a collection of flags used to indicate any occurrence of any ClusterGenerationFailures.
\param unitList collection of units.
\param vertices collection of vertices.
\param kdtreeBuilder KDTreeBuilder object containing BuildNode structure and sorted unit IDs.
*/
void
ClusteredMeshBuilderMethods::InitializeUnitClustersUsingKDTree(
    LeafMap & leafMap,
    UnitClusterStack & unitClusterStack,
    const TriangleList & triangles,
    uint32_t * const mergedVertices,
    uint32_t & failureFlags,
    const UnitList & unitList,
    const VertexList & vertices,
    const rw::collision::KDTreeBuilder & kdtreeBuilder)
{
    // Recursively walk the kdtree buildnode structure and create clusters
    WalkBranch(
        kdtreeBuilder.GetRootNode(),
        leafMap,
        unitClusterStack,
        triangles,
        mergedVertices,
        failureFlags,
        unitList,
        kdtreeBuilder.GetSortedEntryIndices(),
        vertices);
}




// Private Methods ------------------------------------------------------------------------------------


/**
\brief Utility method which finds the quad vertex of a given triangle and edge.

The quad vertex can be found by searching for the triangle which shares the
specified edge. The quad vertex is the vertex which is opposite the shared edge on
the neighboring triangle.

\param triangle2Index index of 2nd triangle in quad
\param quadVertexIndex index of extra quad vertex
\param triangle1Index index of 1st triangle in quad
\param edgeVertexIndex index of shared edge start vertex on 1st triangle
\param edgeVertexNextIndex index of shared edge end vertex on 1st triangle
\param triangles collection of triangles
\param triangleGroupIDs collection of group IDs
\param triangleFlags collection of triangle flags
\param vertexTriangleMap maps vertex indices to triangle indices
*/
void
ClusteredMeshBuilderMethods::FindQuadVertex(
    uint32_t &triangle2Index,
    uint32_t &quadVertexIndex,
    const uint32_t triangle1Index,
    const uint32_t edgeVertexIndex,
    const uint32_t edgeVertexNextIndex,
    const TriangleList & triangles,
    const TriangleGroupIDList & triangleGroupIDs,
    const TriangleFlagsList & triangleFlags,
    const VertexTriangleMap & vertexTriangleMap)
{
    VertexTriangleMap::AdjoiningTriangleIterator adIt = vertexTriangleMap.AdjoiningTriangleBegin(edgeVertexIndex);
    VertexTriangleMap::AdjoiningTriangleIterator adItEnd = vertexTriangleMap.AdjoiningTriangleEnd(edgeVertexIndex);

    // Iterate through the triangles surrounding the edge vertex
    while (adIt != adItEnd)
    {
        // Get the candidate triangle index
        triangle2Index = *adIt;

        // If the candidate is valid
        if (triangle1Index != triangle2Index &&
            triangleFlags[triangle2Index].enabled &&
            triangleGroupIDs[triangle1Index] == triangleGroupIDs[triangle2Index])
        {
            // Get the candidate triangle vertex indices
            const uint32_t *triangle2VertexIndices = triangles[triangle2Index].vertices;

            // Search through the candidate edges
            for (uint32_t triangle2EdgeVertex = 2, triangle2EdgeNextVertex = 0; triangle2EdgeNextVertex < 3; triangle2EdgeVertex = triangle2EdgeNextVertex++ )
            {
                // If the candidate edge matches the first edge
                if ( edgeVertexIndex == triangle2VertexIndices[triangle2EdgeNextVertex] &&
                    edgeVertexNextIndex == triangle2VertexIndices[triangle2EdgeVertex] )
                {
                    // Set the quad vertex index
                    quadVertexIndex = triangle2VertexIndices[(triangle2EdgeNextVertex < 2) ? (triangle2EdgeNextVertex + 1) : 0];
                    return;
                }
            }
        }
        // Advance the iterator
        ++adIt;
    }
}


/**
\brief Finds an edge index given a two triangle indices.

\param t TriangleDataEx holding edge information.
\param n Index of 2nd triangle.

\return Index of shared edge.
*/
uint32_t
ClusteredMeshBuilderMethods::FindEdgeByNeighbor(const uint32_t *const neighbors, const uint32_t n)
{
    uint32_t i;
    for (i=0; i < 3; ++i)
    {
        if (neighbors[i] == n) break;
    }

    EA_ASSERT(i < 3);
    return i;
}


/**
\brief Utility method fills a gridspatialmap with triangles.

The specified triangle edge cosine is adjusted so that the triangle merges with the plane,
as though that edge was shared with a triangle which lay in the plane.

\param spatialMap the spatial map to fill
\param triangleIndex index of 1st triangle to insert into map
\param vertices collection of vertices
\param triangles collection of triangles
\param triangleFlags collection of triangle flags
\param coplanarCosineTolerance tolerance used to determine angle similarity

\return index of last triangle inserted into map
*/
uint32_t
ClusteredMeshBuilderMethods::FillGridSpatialMap(
    GridSpatialMap & spatialMap,
    uint32_t & triangleIndex,
    const VertexList & vertices,
    const TriangleList & triangles,
    const TriangleFlagsList & triangleFlags,
    const rwpmath::VecFloat & coplanarCosineTolerance)
{
    uint32_t spatialMapEntryCount = 0;

    // Begin map insertion
    spatialMap.BeginInsertion(spatialMapEntryCount);

    // Iterate through each of the triangles
    for (; triangleIndex < triangles.size(); ++triangleIndex )
    {
        // Ignore disabled triangles
        if (triangleFlags[triangleIndex].enabled == false)
        {
            continue;
        }

        // Get the current triangle vertex indices
        const uint32_t* triangleVertexIndices = triangles[triangleIndex].vertices;

        // Get the current triangle vertex positions
        rwpmath::Vector3 vec0(vertices[triangleVertexIndices[0]]);
        rwpmath::Vector3 vec1(vertices[triangleVertexIndices[1]]);
        rwpmath::Vector3 vec2(vertices[triangleVertexIndices[2]]);

        // Calculate the normal
        rwpmath::Vector3 triangleNormal(TriangleNormal::ComputeTriangleNormalFast(
            vec0,
            vec1,
            vec2));

        // Determine if the triangle is horizontal AND facing up
        if (rwpmath::IsSimilar(rwpmath::VecFloat(1.0f), triangleNormal.GetY(), coplanarCosineTolerance))
        {
            // We rasterize the bounding box of the triangle rather than the triangle itself
            rwpmath::Vector3 triMin(rwpmath::Min(vec0, vec1));
            rwpmath::Vector3 triMax(rwpmath::Max(vec0, vec1));
            triMin = rwpmath::Min(triMin, vec2);
            triMax = rwpmath::Max(triMax, vec2);

            if(!spatialMap.Insert(
                triMin,
                triMax,
                triangleIndex,
                spatialMapEntryCount))
            {
                // Run out of space
                break;
            }
        }
    }

    // End map insertion
    spatialMap.EndInsertion(spatialMapEntryCount);

    // Return success
    return triangleIndex;
}


/**
\brief Merges triangle with those inserted into the spatial map.

\param spatialMap spatial map containing upfacing triangles
\param triangleGroupIDs collection of group IDs
\param triangleEdgeCodes collection of triangle edge cosine codes
\param vertices collection of vertices
\param triangles collection of triangles
\param triangleFlags collection of triangle flags
\param coplanarCosineTolerance tolerance used to determine angle similarity
\param coplanarHeightTolerance tolerance used to determine height similarity
\param maximumEdgeCosineMergeTolerance limiting tolerance used to determine when to merge triangles.
*/
void
ClusteredMeshBuilderMethods::MergeWithHorizontalTriangles(
    const GridSpatialMap &spatialMap,
    const TriangleGroupIDList & triangleGroupIDs,
    TriangleEdgeCosinesList & triangleEdgeCosines,
    TriangleNeighborsList & triangleNeighbors,
    const VertexList & vertices,
    const TriangleList & triangles,
    const TriangleFlagsList & triangleFlags,
    const rwpmath::VecFloat & coplanarCosineTolerance,
    const rwpmath::VecFloat & coplanarHeightTolerance,
    const rwpmath::VecFloat & maximumEdgeCosineMergeTolerance)
{
    const rwpmath::Vector3 mergeNormal(0.0f, 1.0f, 0.0f);

    // Iterate over the entire triangle collection to find the horizontal downward facing
    // triangles.
    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
    {
        // Avoid disabled triangles
        if (triangleFlags[triangleIndex].enabled == false)
        {
            continue;
        }

        // Get the triangle vertex indices
        const uint32_t* triangleVertexIndices = triangles[triangleIndex].vertices;

        // Get the triangle normal
        rwpmath::Vector3 triangleNormal(TriangleNormal::ComputeTriangleNormalFast(
            rwpmath::Vector3(vertices[triangleVertexIndices[0]]),
            rwpmath::Vector3(vertices[triangleVertexIndices[1]]),
            rwpmath::Vector3(vertices[triangleVertexIndices[2]])));

        // If the triangle is horizontal AND facing down
        if ( rwpmath::IsSimilar(-mergeNormal.GetY(), triangleNormal.GetY(), coplanarCosineTolerance) )
        {
            // Get the triangle neighbor indices
            uint32_t * triangleNeighborIndices = triangleNeighbors[triangleIndex].neighbor;

            // Get the triangle edge cosines
            float * edgeCosines = triangleEdgeCosines[triangleIndex].edgeCos;

            // Check each edgecos of the triangle to determine if it needs to be corrected
            for (uint32_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
            {
                // Index of triangle sharing current edge
                uint32_t oppositeTriangleIndex = triangleNeighborIndices[edgeIndex];

                // If the edge is shared and its edgecos is less than the tolerance
                if ( oppositeTriangleIndex != CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH &&
                     edgeCosines[edgeIndex] < maximumEdgeCosineMergeTolerance )
                {
                    // Determine if edge sits in any horizontal up facing triangle
                    rwpmath::Vector3 edgeVertex0(vertices[triangleVertexIndices[edgeIndex]]);
                    rwpmath::Vector3 edgeVertex1(vertices[triangleVertexIndices[(edgeIndex < 2) ? (edgeIndex + 1) : 0]]);

                    if (DoesEdgeLieInAnyTriangle(
                        vertices,
                        triangles,
                        triangleGroupIDs,
                        triangleIndex,
                        edgeVertex0,
                        edgeVertex1,
                        spatialMap,
                        coplanarHeightTolerance))
                    {
                        MergeTriangleWithPlane(
                            edgeCosines,
                            triangleEdgeCosines,
                            triangleNeighbors,
                            triangles,
                            vertices,
                            triangleIndex,
                            triangleVertexIndices,
                            edgeIndex,
                            oppositeTriangleIndex,
                            mergeNormal);
                    }
                }
            }
        }
    }
}


/**
\brief Determines if a edge intersects, and is in the plane of, any triangles in
a given collection

\param vertices collection of vertices
\param triangles collection of triangles
\param triangleGroupID collection of group ID
\param triangleEdgeCodes collection of triangle edge cosine codes
\param edgeTriangleIndex index of the edge triangle
\param edgeVertex0 first vertex of the edge
\param edgeVertex1 second vertex of the edge
\param spatialMap spatial map containing triangles to test the edge against
\param coplanarHeightTolerance Tolerance used to determine height similarity

\return true if intersection is found, false otherwise.
*/
bool
ClusteredMeshBuilderMethods::DoesEdgeLieInAnyTriangle(
    const VertexList & vertices,
    const TriangleList & triangles,
    const TriangleGroupIDList & triangleGroupIDs,
    const uint32_t edgeTriangleIndex,
    rwpmath::Vector3::InParam edgeVertex0,
    rwpmath::Vector3::InParam edgeVertex1,
    const GridSpatialMap &spatialMap,
    const rwpmath::VecFloat & coplanarHeightTolerance)
{
    // Determine the extents of the edges AABBox
    rwpmath::Vector3 vec0(edgeVertex0);
    rwpmath::Vector3 vec1(edgeVertex1);

    rwpmath::Vector3 edgeMin = rwpmath::Min(vec0, vec1);
    rwpmath::Vector3 edgeMax = rwpmath::Max(vec0, vec1);

    // Pad the min and max values by the tolerance
    edgeMin.SetY(edgeMin.GetY() - coplanarHeightTolerance);
    edgeMax.SetY(edgeMax.GetY() + coplanarHeightTolerance);

    // Calculate the spatialMap Grid Box
    rwpmath::Vector3 minBox, maxBox;
    spatialMap.CalculateTightGridBox(
        edgeMin,
        edgeMax,
        minBox,
        maxBox);

    // Potential load-hit-stores here
    const uint32_t minX = static_cast<uint32_t>(static_cast<float>(minBox.GetX()));
    const uint32_t minY = static_cast<uint32_t>(static_cast<float>(minBox.GetY()));
    const uint32_t minZ = static_cast<uint32_t>(static_cast<float>(minBox.GetZ()));

    const uint32_t maxX = static_cast<uint32_t>(static_cast<float>(maxBox.GetX()));
    const uint32_t maxY = static_cast<uint32_t>(static_cast<float>(maxBox.GetY()));
    const uint32_t maxZ = static_cast<uint32_t>(static_cast<float>(maxBox.GetZ()));

    // For each box the triangle intersects
    for (uint32_t x = minX; x <= maxX; ++x)
    {
        for (uint32_t y = minY; y <= maxY; ++y)
        {
            for (uint32_t z = minZ; z <= maxZ; ++z)
            {
                GridSpatialMap::BoxEntryIterator it(spatialMap.BoxEntryIteratorBegin(x, y, z));
                GridSpatialMap::BoxEntryIterator itEnd(spatialMap.BoxEntryIteratorEnd(x, y, z));

                // Iterate through all entries of this box
                while (it != itEnd)
                {
                    // Check if edge intersects triangle
                    if (DoesEdgeLieInTriangle(
                        vertices,
                        triangles,
                        triangleGroupIDs,
                        edgeTriangleIndex,
                        edgeVertex0, 
                        edgeVertex1,
                        *it,
                        edgeVertex0.GetY(),
                        coplanarHeightTolerance))
                    {
                        // Return Success. Intersection found.
                        return true;
                    }

                    // Advance the iterator
                    ++it;
                }
            }
        }
    }
    // Return failure. No intersections found
    return false;
}


/**
\brief Determines if a edge intersects, and is in the plane of, a given triangle

\param vertices collection of vertices
\param triangles collection of triangles
\param triangleGroupIDs collection of group IDs
\param triangleEdgeCodes collection of triangle edge cosine codes
\param edgeTriangleIndex index of the edge triangle
\param edgeVertex0 first vertex of the edge
\param edgeVertex1 second vertex of the edge
\param triangleIndex index of the triangle
\param height height of triangle
\param coplanarHeightTolerance tolerance used to determine height similarity

\return true if intersection is found, false otherwise.
*/
bool
ClusteredMeshBuilderMethods::DoesEdgeLieInTriangle(
    const VertexList & vertices,
    const TriangleList & triangles,
    const TriangleGroupIDList & triangleGroupIDs,
    const uint32_t edgeTriangleIndex,
    rwpmath::Vector3::InParam edgeVertex0,
    rwpmath::Vector3::InParam edgeVertex1,
    const uint32_t triangleIndex,
    const float height,
    const rwpmath::VecFloat & coplanarHeightTolerance)
{
    // If the two triangle are not in the same group
    if (triangleGroupIDs[edgeTriangleIndex] != triangleGroupIDs[triangleIndex])
    {
        // Get the triangle indices
        const uint32_t * triangleVertexIndices = triangles[triangleIndex].vertices;

        VectorType vec0(vertices[triangleVertexIndices[0]]);

        // Determine if the triangles are at the "same" height
        if ( rwpmath::IsSimilar(vec0.GetY(), height, coplanarHeightTolerance) )
        {
            VectorType vec1(vertices[triangleVertexIndices[1]]);
            VectorType vec2(vertices[triangleVertexIndices[2]]);

            rwpmath::Vector2 v0(vec0.GetX(), vec0.GetZ());
            rwpmath::Vector2 v1(vec1.GetX(), vec1.GetZ());
            rwpmath::Vector2 v2(vec2.GetX(), vec2.GetZ());

            rwpmath::Vector2 edgeV0(edgeVertex0.GetX(), edgeVertex0.GetZ());
            rwpmath::Vector2 edgeV1(edgeVertex1.GetX(), edgeVertex1.GetZ());

            // Check for intersection
            if (TriangleLineIntersector::IntersectLineWithTriangle2D(
                v0,
                v1,
                v2,
                edgeV0,
                edgeV1))
            {
                // Return success. Intersection found.
                return true;
            }
        }
    }
    // Return failure. No intersection found.
    return false;
}


/**
\brief Determines if a collection of triangles are all coplanar

\param triangleIterator iterator referencing start of triangle collection
\param triangleIteratorEnd iterator referencing the end of the triangle collection
\param triangles collection of triangles
\param triangleFlags collection of triangle flags
\param vertices collection of vertices
\param coplanarCosineTolerance coplanar tolerance

\return true if all triangles in the collection are coplanar, false otherwise.
*/
bool
ClusteredMeshBuilderMethods::AllCoplanarTriangles(
    VertexTriangleMap::AdjoiningTriangleIterator triangleIterator,
    const VertexTriangleMap::AdjoiningTriangleIterator triangleIteratorEnd,
    const TriangleList & triangles,
    const TriangleFlagsList & triangleFlags,
    const VertexList & vertices,
    const rwpmath::VecFloat & coplanarCosineTolerance)
{
    FindNextEnabledTriangle(
        triangleIterator,
        triangleIteratorEnd,
        triangleFlags);

    if (triangleIterator == triangleIteratorEnd)
    {
        return false;
    }

    // Get current triangle index
    uint32_t triangleIndex = *triangleIterator;
    // Get current triangle indices
    const uint32_t * triangleVertexIndices = triangles[triangleIndex].vertices;
    // Determine the candidate plane normal
    rwpmath::Vector3 planeNormal(TriangleNormal::ComputeTriangleNormalFast(
        rwpmath::Vector3(vertices[triangleVertexIndices[0]]),
        rwpmath::Vector3(vertices[triangleVertexIndices[1]]),
        rwpmath::Vector3(vertices[triangleVertexIndices[2]])));

    // Advance the iterator
    ++triangleIterator;

    // Iterate through all of the triangles
    while(triangleIterator != triangleIteratorEnd)
    {
        // Get the current triangle index
        triangleIndex = *triangleIterator;

        // If the current triangle is not disabled
        if (triangleFlags[triangleIndex].enabled)
        {
            // Get the current triangle vertex indices
            triangleVertexIndices = triangles[triangleIndex].vertices;
            // Get the current triangle normal
            rwpmath::Vector3 triangleNormal(TriangleNormal::ComputeTriangleNormalFast(
                rwpmath::Vector3(vertices[triangleVertexIndices[0]]),
                rwpmath::Vector3(vertices[triangleVertexIndices[1]]),
                rwpmath::Vector3(vertices[triangleVertexIndices[2]])));

            // If the current triangle normal and the candidate plane normal are not within the tolerance of each other
            if (!rwpmath::IsSimilar(rwpmath::Dot(triangleNormal, planeNormal), 1.0f, coplanarCosineTolerance))
            {
                // The vertex hub is not surrounded by coplanar triangles
                return false;
            }
        }
        // Advance the iterator
        ++triangleIterator;
    }

    // The vertex hub is surrounded by coplanar triangles
    return true;
}


/**
\brief Advances a triangle iterator to the next enabled triangle in a collection.

\param triangleIterator iterator referencing start of triangle collection
\param triangleIteratorEnd iterator referencing the end of the triangle collection
\param triangleFlags triangle flags collection.
*/
void
ClusteredMeshBuilderMethods::FindNextEnabledTriangle(
    VertexTriangleMap::AdjoiningTriangleIterator & triangleIterator,
    const VertexTriangleMap::AdjoiningTriangleIterator & triangleIteratorEnd,
    const TriangleFlagsList & triangleFlags)
{
    while(triangleIterator != triangleIteratorEnd)
    {
        // If this triangle is not disabled
        if (triangleFlags[*triangleIterator].enabled)
        {
            break;
        }

        // Advance the iterator
        ++triangleIterator;
    }
}


/**
\brief Determines whether the specified vertex is a feature vertex.

The two reasons a vertex will be considered to be a non-vertex feature are:
A) It is surrounded by a featureless plane.
B) It resides in a concave region.

A featureless plane is a single plane which passes through a given vertex, which
all surround features of that vertexHub lay below and which can only be rotated around
a single axis while all surrounding features remain underneath the plane.

\param vertexIndex index of vertex
\param vertexPosition position of vertex
\param triangleIterator iterator addressing the first triangle.
\param triangleIteratorEnd iterator addressing the end of a collection of triangles
\param triangles collection of triangles
\param triangleFlags collection of triangle flags
\param vertices collection of vertices
\param coplanarCosineTolerance determines when an edge is coplanar to the candidate plane. The
tolerance is applied to the cosine-angle between and edge and the candidate plane normal.
\param cosineTolerance determines when an edge which is assumed to be coplanar to a pair of edges
lies between those two edges.
\param concaveCosineTolerance determines when the angle between an edge and the candidate
plane can be considered concave, therefore disabling the vertex. The tolerance is the minimum
cosine-angle between the plane normal and edge direction.

\return true is a vertex can be disabled.
*/
bool
ClusteredMeshBuilderMethods::VertexIsNonFeature(
    const uint32_t vertexIndex,
    rwpmath::Vector3::InParam vertexPosition,
    VertexTriangleMap::AdjoiningTriangleIterator triangleIterator,
    const VertexTriangleMap::AdjoiningTriangleIterator triangleIteratorEnd,
    const TriangleList & triangles,
    const TriangleFlagsList & triangleFlags,
    const VertexList & vertices,
    const rwpmath::VecFloat & coplanarCosineTolerance,
    const rwpmath::VecFloat & cosineTolerance,
    const rwpmath::VecFloat & concaveCosineTolerance)
{
    // Edges which describe advancing front of feature plane
    rwpmath::Vector3 edgeA;
    rwpmath::Vector3 edgeB;

    // Candidate edge for advancing front of feature plane
    rwpmath::Vector3 edgeC;

    // Vertices of candidate edges
    rwpmath::Vector3 vertA;
    rwpmath::Vector3 vertB;

    FindNextEnabledTriangle(
        triangleIterator,
        triangleIteratorEnd,
        triangleFlags);

    if (triangleIterator == triangleIteratorEnd)
    {
        return false;
    }

    // Index of current triangle
    uint32_t triangleIndex = *triangleIterator;
    // Vertex indices of current triangles
    const uint32_t * triangleVertexIndices = triangles[triangleIndex].vertices;
    // Feature plane normal
    rwpmath::Vector3 planeNormal(TriangleNormal::ComputeTriangleNormalFast(
        rwpmath::Vector3(vertices[triangleVertexIndices[0]]),
        rwpmath::Vector3(vertices[triangleVertexIndices[1]]),
        rwpmath::Vector3(vertices[triangleVertexIndices[2]])));

    // Get vertices of edges
    GetOppositeVertices(
        vertA,
        vertB,
        vertexIndex,
        triangleVertexIndices,
        vertices);

    // Initialize edges of advancing feature plane
    edgeA = rwpmath::NormalizeFast(vertexPosition - vertA);
    edgeB = rwpmath::NormalizeFast(vertexPosition - vertB);

    // Select the next triangle
    ++triangleIterator;

    // Tests all vertex hub triangles
    while (triangleIterator != triangleIteratorEnd)
    {
        // Get current triangle index
        triangleIndex = *triangleIterator;

        // If the current triangle is not disabled
        if (triangleFlags[triangleIndex].enabled)
        {
            // Get the current triangle opposite vertices
            const uint32_t * triangleVertexIndices = triangles[triangleIndex].vertices;

            GetOppositeVertices(
                vertA,
                vertB,
                vertexIndex,
                triangleVertexIndices,
                vertices);

            // Set candidate edge
            edgeC = rwpmath::NormalizeFast(vertexPosition - vertA);

            if (ClusteredMeshBuilderUtils::EdgeDisablesVertex(
                edgeA,
                edgeB,
                edgeC,
                planeNormal,
                coplanarCosineTolerance,
                cosineTolerance,
                concaveCosineTolerance))
            {
                return true;
            }

            // Set candidate edge
            edgeC = rwpmath::NormalizeFast(vertexPosition - vertB);

            if (ClusteredMeshBuilderUtils::EdgeDisablesVertex(
                edgeA,
                edgeB,
                edgeC,
                planeNormal,
                coplanarCosineTolerance,
                cosineTolerance,
                concaveCosineTolerance))
            {
                return true;
            }
        }

        // Advance the triangle iterator
        ++triangleIterator;
    }

    // Edge does not disable vertex
    return false;
}


/**
\brief Finds the two opposite vertex positions, given a triangle and vertex index.

\param vertexA position of first opposite vertex
\param vertexB position of second opposite vertex
\param vertexIndex index of vertex.
\param triangleVertexIndices triangle vertex indices.
\param vertices collection of vertices
*/
void
ClusteredMeshBuilderMethods::GetOppositeVertices(
    rwpmath::Vector3::InOutParam vertexA,
    rwpmath::Vector3::InOutParam vertexB,
    const uint32_t vertexIndex,
    const uint32_t * triangleVertexIndices,
    const VertexList & vertices)
{
    if (triangleVertexIndices[0] == vertexIndex)
    {
        vertexA = rwpmath::Vector3(vertices[triangleVertexIndices[1]]);
        vertexB = rwpmath::Vector3(vertices[triangleVertexIndices[2]]);
    }
    else if(triangleVertexIndices[1] == vertexIndex)
    {
        vertexA = rwpmath::Vector3(vertices[triangleVertexIndices[2]]);
        vertexB = rwpmath::Vector3(vertices[triangleVertexIndices[0]]);
    }
    else
    {
        vertexA = rwpmath::Vector3(vertices[triangleVertexIndices[0]]);
        vertexB = rwpmath::Vector3(vertices[triangleVertexIndices[1]]);
    }
}



/**
\brief Disables the vertex indicated by the given index.

\param vertexIndex index of the vertex
\param vertexTriangleMap mapping from vertex indices to triangle indices
\param triangles collection of triangles
\param triangleFlags collection of triangle flags
*/
void
ClusteredMeshBuilderMethods::DisableVertex(
    const uint32_t vertexIndex,
    const VertexTriangleMap & vertexTriangleMap,
    const TriangleList & triangles,
    TriangleEdgeCodesList & triangleEdgeCodes,
    const TriangleFlagsList & triangleFlags)
{
    VertexTriangleMap::AdjoiningTriangleIterator triIt = vertexTriangleMap.AdjoiningTriangleBegin(vertexIndex);
    VertexTriangleMap::AdjoiningTriangleIterator triItEnd = vertexTriangleMap.AdjoiningTriangleEnd(vertexIndex);

    // Iterate through all of the triangles
    while (triIt != triItEnd)
    {
        // Get the current triangle index
        uint32_t triangleIndex = *triIt;

        // If the current triangle isn't disabled
        if (triangleFlags[triangleIndex].enabled)
        {
            // Get the current triangle vertex indices
            const uint32_t* triangleVertexIndices = triangles[triangleIndex].vertices;

            // Determine the triangle-local-index of the vertex and disable it.
            if (triangleVertexIndices[0] == vertexIndex)
            {
                EncodeTriangleVertexData(triangleEdgeCodes, triangleIndex, 0, rw::collision::EDGEFLAG_VERTEXDISABLE);
            }
            else if (triangleVertexIndices[1] == vertexIndex)
            {
                EncodeTriangleVertexData(triangleEdgeCodes, triangleIndex, 1, rw::collision::EDGEFLAG_VERTEXDISABLE);
            }
            else
            {
                EncodeTriangleVertexData(triangleEdgeCodes, triangleIndex, 2, rw::collision::EDGEFLAG_VERTEXDISABLE);
            }
        }

        // Advance the iterator
        ++triIt;
    }
}


/**
\brief Encodes the triangle vertex data.

\param triangles collection of triangles.
\param triangleIndex index of triangle.
\param vertex index local index of vertex.
\param vertexFlag vertex flag.
*/
void
ClusteredMeshBuilderMethods::EncodeTriangleVertexData(TriangleEdgeCodesList & triangleEdgeCodes,
                                                      const uint32_t triangleIndex,
                                                      const uint32_t vertexIndex,
                                                      const uint8_t vertexFlag)
{
    EA_ASSERT(triangleIndex < triangleEdgeCodes.size());
    EA_ASSERT(vertexIndex < 3);

    triangleEdgeCodes[triangleIndex].encodedEdgeCos[vertexIndex] =
        static_cast<uint8_t>(triangleEdgeCodes[triangleIndex].encodedEdgeCos[vertexIndex] | vertexFlag);
}


/**
\brief Gather statistics for one edge, adding its length to a sum and maintaining a smallest edge value

\param u Edge start.
\param v Edge end.
\param minedge A minimum edge length.
\param totedge A sum of edge lengths.
*/
void
ClusteredMeshBuilderMethods::MeasureEdge(rwpmath::Vector3::InParam u,
                                         rwpmath::Vector3::InParam v,
                                         rwpmath::VecFloat & minedge,
                                         rwpmath::VecFloat & totedge)
{
    rwpmath::VecFloat len = rwpmath::Magnitude(u - v);
    if (len < minedge)
    {
        minedge = len;
    }
    totedge += len;
}

/*
\brief Adds an ordered collection of units to a cluster.

A collection of units, indicated by a collection of ordered unit IDs, are added to the cluster.
A start index, into the ordered collection, determines the first unit to add to the cluster.
A number of units to add, counting from the start index, determine the number of units to add to the
cluster.
Units are added to the cluster until all units are added or the vertex count limit is reached.

\param clusterVertexIDs the collection of cluster vertex IDs.
\param clusterVertexCount the count of cluster vertices.
\param clusterUnitIDs the collection of cluster unit IDs.
\param clusterUnitCount the count of cluster units.
\param orderedUnitIDs an ordered collection of units IDs, identifying the units to add to the cluster.
\param startUnitIndex the index, into the unitID collection, of the first unit to add to the cluster.
\param numUnitsToAdd the number of units to add to the cluster.
\param triangles collection of triangles.
\param vertices collection of vertices.
\param unitList collection of units.
\param maxVerticesPerUnit the maximum number of vertices a single unit can have.
**/
uint32_t
ClusteredMeshBuilderMethods::AddOrderedUnitsToUnitCluster(
    UnitCluster::VertexSet & clusterVertexIDs,
    uint32_t & clusterVertexCount,
    UnitCluster::UnitID clusterUnitIDs [],
    uint32_t & clusterUnitCount,
    const uint32_t orderedUnitIDs [],
    const uint32_t startUnitIndex,
    const uint32_t numUnitsToAdd,
    const TriangleList & triangles,
    const UnitList & unitList,
    const uint32_t maxVerticesPerUnit)
{
    // For each unit to add to the UnitCluster
    uint32_t unitIndex = 0;
    for ( ; unitIndex < numUnitsToAdd ; ++unitIndex)
    {
        // Attempt to add the unit to the cluster
        const bool added = UnitClusterBuilder::AddUnitToCluster(
                               clusterVertexIDs,
                               clusterVertexCount,
                               clusterUnitIDs,
                               clusterUnitCount,
                               orderedUnitIDs[startUnitIndex + unitIndex],
                               triangles,
                               unitList,
                               maxVerticesPerUnit);

        // If  the unit was not added to the cluster
        if (!added)
        {
            // return the number of units added to the cluster
            return unitIndex;
        }
    }

    // Sort and compress the cluster vertex set
    UnitCluster::SortAndCompressVertexSet(clusterVertexIDs,
                                          clusterVertexCount);

    // Return the number of units which have been added to the cluster
    return unitIndex;
}





/**
\brief Recursive call which converts kdTree branches to clusters.

As the branch is traversed, the clusters are appended to the clusterList.

\param buildNode KDTreeBuilder build node under inspection.
\param leafMap maps ID of first object in each leaf node to the leaf node ID.
\param unitClusterStack unit cluster allocator.
\param triangles collection of triangles.
\param mergedVertices vertex index array used used during cluster merging.
\param failureFlags a collection of flags used to indicate any occurrence of ClusterGenerationFailures.
\param unitList collection of units.
\param sortedObj Array to convert kdtree indices into unitList indices.
\param vertices collection of vertices.

\return the number of vertices in the branch.
Specifically: 0                   if the branch is empty, NO CLUSTERS
              1..vertexCountLimit if the branch generated ONE cluster, which is unitClusters.GetLastCluster()
              >vertexCountLimit   if the branch generated MORE THAN ONE cluster.
*/
uint32_t
ClusteredMeshBuilderMethods::WalkBranch(
    rw::collision::KDTreeBuilder::BuildNode * buildNode,
    LeafMap & leafMap,
    UnitClusterStack & unitClusterStack,
    const TriangleList & triangles,
    uint32_t * const mergedVertices,
    uint32_t & failureFlags,
    const UnitList & unitList,
    const uint32_t * sortedObjects,
    const VertexList & vertices)
{
    uint32_t maxVerticesPerUnit = 4;
    uint32_t vcount0 = 0, vcount1 = 0;

    // Check for any failures
    if (CLUSTER_GENERATION_FAILURE_NO_FAILURES != failureFlags)
    {
        return 0;
    }

    // If the node is a leaf node
    if (buildNode->m_left == NULL)
    {
        // Get the start index and count of units in this leaf node
        const uint32_t start = buildNode->m_firstEntry;
        const uint32_t totalNumUnitsToAdd = buildNode->m_numEntries;

        // If the leaf node is empty ignore it
        if (0 == totalNumUnitsToAdd)
        {
            return vcount0 + vcount1;
        }

        // Add the unitID to the leaf map so we can fix the leaf "start" during finalization.
        leafMap[sortedObjects[start]] = buildNode;

        // Get a new UnitCluster
        UnitCluster *cluster = unitClusterStack.GetUnitCluster();
        // Check if we have valid cluster
        if (NULL == cluster)
        {
            // Flag the failure and return
            failureFlags |= CLUSTER_GENERATION_FAILURE_OUT_OF_MEMORY;
            return 0;
        }

            // Add the units to the cluster
        const uint32_t numUnitsAdded = AddOrderedUnitsToUnitCluster(
                                 cluster->vertexIDs,
                                 cluster->numVertices,
                                 cluster->unitIDs,
                                 cluster->numUnits,
                                 sortedObjects,
                                           start,
                                           totalNumUnitsToAdd,
                                 triangles,
                                 unitList,
                                 maxVerticesPerUnit);

        // If there are remaining units to add then the current UnitCluster must be full.
            if (numUnitsAdded < totalNumUnitsToAdd)
            {
            // Mark the failure
            failureFlags |= CLUSTER_GENERATION_FAILURE_MULTI_LEAF_CLUSTER;
            }

        // Set vcount0 to the count of the last cluster
        vcount0 = cluster->numVertices;

        // Check that the last cluster has more than 0 vertices
        EA_ASSERT_MSG(vcount0 > 0, ("Attempting to add a cluster with no vertices."));
    }
    else     // buildNode is not a leaf
    {
        vcount0 = WalkBranch(
            buildNode->m_left,
            leafMap,
            unitClusterStack,
            triangles,
            mergedVertices,
            failureFlags,
            unitList,
            sortedObjects,
            vertices);

        vcount1 = WalkBranch(
            buildNode->m_right,
            leafMap,
            unitClusterStack,
            triangles,
            mergedVertices,
            failureFlags,
            unitList,
            sortedObjects,
            vertices);

        // Check for any failures
        if (CLUSTER_GENERATION_FAILURE_NO_FAILURES != failureFlags)
        {
            return 0;
        }

        //  If both children are small, and not empty, then try to merge them.
        if (vcount0 > 0 && vcount0 <= ClusteredMeshCluster::MAX_VERTEX_COUNT && vcount1 > 0 && vcount1 <= ClusteredMeshCluster::MAX_VERTEX_COUNT)
        {
            // assert that the child clusters MUST be the last two on the clusterList.
            // Last Cluster
            EA_ASSERT((*(unitClusterStack.RBegin()))->numVertices == vcount1);
            // Penultimate Cluster
            EA_ASSERT((*(++unitClusterStack.RBegin()))->numVertices == vcount0);

            if (MergeLastTwoClusters(
                    unitClusterStack,
                    mergedVertices))
            {
                UnitClusterStack::ReverseClusterIterator rIt = unitClusterStack.RBegin();
                vcount0 = (*rIt)->numVertices;
                vcount1 = 0;
            }
            else
            {
                // An assert failure implies that there are too many vertices to merge,
                // assert that this is definitely the case.
                EA_ASSERT(vcount0 + vcount1 > ClusteredMeshCluster::MAX_VERTEX_COUNT);
            }
        }
    }
    return vcount0 + vcount1;
}


/**
\brief Attempt to merge the last two clusters in the clusterList.
Two clusters can be merged if the total unique vertex count of the vertices of
both clusters is less than the given limit.

\param lastCluster the last cluster (to have been created).
\param penultimate the penultimate cluster.
\param mergedVertices array used to merge vertices.

\return False if unable to merge because size limit was exceeded, else true.
*/
bool
ClusteredMeshBuilderMethods::MergeLastTwoClusters(
    UnitClusterStack & unitClusterStack,
    uint32_t * mergedVertices)
{
    UnitClusterStack::ReverseClusterIterator rIt = unitClusterStack.RBegin();
    UnitCluster * lastCluster = *rIt;
    ++rIt;
    UnitCluster * penultimateCluster = *rIt;

    uint32_t mergedVertexCount = 0;

    uint32_t penultimateCounter = 0;
    uint32_t lastCounter = 0;

    // Iterate over all vertices in each cluster until all vertices have been iterated
    // or the max vertex count, ClusteredMeshCluster::MAX_VERTEX_COUNT, has been reached
    while(penultimateCounter < penultimateCluster->numVertices && lastCounter < lastCluster->numVertices && mergedVertexCount < ClusteredMeshCluster::MAX_VERTEX_COUNT)
    {
        if (penultimateCluster->vertexIDs[penultimateCounter] == lastCluster->vertexIDs[lastCounter])
        {
            ++lastCounter;
            mergedVertices[mergedVertexCount++] = penultimateCluster->vertexIDs[penultimateCounter++];
        }
        else if (penultimateCluster->vertexIDs[penultimateCounter] < lastCluster->vertexIDs[lastCounter])
        {
            mergedVertices[mergedVertexCount++] = penultimateCluster->vertexIDs[penultimateCounter++];
        }
        else
        {
            mergedVertices[mergedVertexCount++] = lastCluster->vertexIDs[lastCounter++];
        }
    }

    // Attempt to add the remaining entries in cluster g0
    while (penultimateCounter < penultimateCluster->numVertices && mergedVertexCount < ClusteredMeshCluster::MAX_VERTEX_COUNT)
    {
        mergedVertices[mergedVertexCount++] = penultimateCluster->vertexIDs[penultimateCounter++];
    }

    // Attempt to add the remaining entries in cluster g1
    while (lastCounter < lastCluster->numVertices && mergedVertexCount < ClusteredMeshCluster::MAX_VERTEX_COUNT)
    {
        mergedVertices[mergedVertexCount++] = lastCluster->vertexIDs[lastCounter++];
    }

    // If the combined vertex count is less than the limit merge the two clusters
    if (penultimateCounter == penultimateCluster->numVertices && lastCounter == lastCluster->numVertices && mergedVertexCount <= ClusteredMeshCluster::MAX_VERTEX_COUNT)
    {
        // Copy the merged vertex set into the penultimate cluster.
        for(uint32_t i = 0; i < mergedVertexCount; ++i)
        {
            penultimateCluster->vertexIDs[i] = mergedVertices[i];
        }

        penultimateCluster->numVertices = mergedVertexCount;

        // Merge the last two clusters
        unitClusterStack.MergeLastTwoClusters();

        return TRUE;
    }

    // return failure
    return FALSE;
}


/*
\brief Updates KDTreeBuilder::BuildNode leaf node entries using UnitClusters

\param unitClusterStack     Collection of UnitClusters
\param leafMap              Mapping of unitIDs to KDTreeBuilder::BuildNode leaf nodes
\param unitList             Collection of units.
\param triangleSurfaceIDs   Collection of triangle surface IDs.
\param triangleGroupIDs     Collection of triangle group IDs.
\param UnitParameters       Unit parameters
**/

void
ClusteredMeshBuilderMethods::AdjustKDTreeNodeEntriesForClusterCollection(
    UnitClusterStack & unitClusterStack,
    const LeafMap & leafMap,
    const UnitList & unitList,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitParameters & unitParameters)
{
    // Determine the number of UnitClusters
    uint32_t unitClusterCount = unitClusterStack.Size();

    // Determine unit cluster ID shift
    const uint32_t unitClusterIDShift = (unitClusterCount > 65536) ? 20U : 16U;

    UnitClusterStack::ClusterIterator it = unitClusterStack.Begin();
    const UnitClusterStack::ClusterIterator itEnd = unitClusterStack.End();

    while (it != itEnd)
    {
        const UnitCluster & unitCluster = *(*it);
        AdjustKDTreeNodeEntriesForCluster(unitCluster,
                                          leafMap,
                                          unitList,
                                          triangleSurfaceIDs,
                                          triangleGroupIDs,
                                          unitParameters,
                                          unitCluster.clusterID,
                                          unitClusterIDShift);

        ++it;
    }
}


/*
\brief Updates KDTreeBuilder::BuildNode leaf node entries using UnitClusters

\param unitCluster          UnitCluster
\param leafMap              Mapping of unitIDs to KDTreeBuilder::BuildNode leaf nodes
\param unitList             Collection of units.
\param triangleSurfaceIDs   Collection of triangle surface IDs.
\param triangleGroupIDs     Collection of triangle group IDs.
\param unitParameters       Unit parameters.
\param unitClusterID        ID of UnitCluster.
\param unitClusterIDShift   The UnitCluster ID shift.
**/
void
ClusteredMeshBuilderMethods::AdjustKDTreeNodeEntriesForCluster(
    const UnitCluster & unitCluster,
    const LeafMap & leafMap,
    const UnitList & unitList,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const UnitParameters & unitParameters,
    const uint32_t unitClusterID,
    const uint32_t unitClusterIDShift)
{
    const uint32_t numUnits = unitCluster.numUnits;
    const uint32_t shiftedClusterId = unitClusterID << unitClusterIDShift;

    EA_ASSERT((shiftedClusterId >> unitClusterIDShift) == unitClusterID);
    EA_ASSERT(numUnits <= (1u << unitClusterIDShift));

    uint32_t sizeofUnitData = 0;

    for (uint32_t unitIndex = 0 ; unitIndex < numUnits ; ++unitIndex)
    {
        uint32_t unitID = unitCluster.unitIDs[unitIndex];
        LeafMap::const_iterator it = leafMap.find(unitID);

        if(leafMap.end() != it)
        {
            rw::collision::KDTreeBuilder::BuildNode * buildNode = it->second;
            EA_ASSERT(NULL == buildNode->m_left);

            uint32_t reformattedStartIndex = shiftedClusterId + sizeofUnitData;
            buildNode->m_firstEntry = reformattedStartIndex;

            // If this node has a parent, i.e. it is not the root node of a 1 node KDTree.
            if (NULL != buildNode->m_parent)
            {
                // Check the other child of this parent isn't empty. If it is then set it's start index
                // to be the same as this one otherwise it can cause problems during the queries.

                // Get the parent node.
                rw::collision::KDTreeBuilder::BuildNode * parent = buildNode->m_parent;

                // Is this the right child
                if (parent->m_right == buildNode)
                {
                    // Is the left child empty
                    if(parent->m_left->m_numEntries == 0)
                    {
                        //Set the start for the left child to be the same as the right
                        parent->m_left->m_firstEntry = reformattedStartIndex;
                    }
                }
                else // must be the left child so check right
                {
                    // Is the right child empty
                    if (parent->m_right->m_numEntries == 0)
                    {
                        //Set the start for the right child to be the same as the right
                        parent->m_right->m_firstEntry = reformattedStartIndex;
                    }
                }
            }
        }

        const Unit &unit = unitList[unitID];

        sizeofUnitData += ClusteredMeshCluster::GetUnitSize(static_cast<uint8_t>(unit.type),
                                                            unitParameters,
                                                            triangleGroupIDs[unit.tri0],
                                                            triangleSurfaceIDs[unit.tri0]);
    }
}


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw
