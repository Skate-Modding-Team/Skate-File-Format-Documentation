// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <common/common.h>

#include <rw/collision/clusteredmeshcluster.h>
#include <rw/collision/meshbuilder/detail/clusterparametersbuilder.h>
#include <rw/collision/meshbuilder/edgecodegenerator.h>

#include "quadheightfield.h"

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This example demonstrates how to optimally build a TriangleClusterProcedural without using the
TriangleClusterProceduralBuilder. Its assumes the units are of a uniform type, e.g. all quads
with edge cosines, surface IDs and group IDs. The approach demonstrated here uses minimal memory
resources and removes all extraneous work carried out by the builder for generic input.

The process is broken down into 5 Steps

Step 1 - Create an rw::collision::ClusterConstructionParameters
This structure describes the resource requirements of a TriangleClusterProcedural. At this point of the process the
description of each unit needs to be known.

Step 2 - Create a TriangleClusterProcedural using the ClusterConstructionParameters
Using the GetSizeAndAlignment and Initialize API, a TriangleClusterProcedural can be created in an Initialized state.
In this initialized state the TriangleClusterProcedural contains no vertices or units, but has the resources required
to store the vertices and units described by the ClusterConstructionParameters.

Step 3 - Add the Vertices
Using the Cluster's AddVertex API each of the vertices are added to the Cluster. The order in which the vertices are added
to the cluster determines their indices.

Step 4 - Add the Units
Using the Cluster's AddQuad/Triangle API each of the units are added to the Cluster. This requires knowledge of the units vertex
indices and the units encoded edge codes.

Step 5 - Finalize the TriangleClusterProcedural
Using the TriangleClusterProcedural's UpdateThis() API the TriangleClusterProcedural is put into a state at which it is ready for
runtime use.
**/

// Helper methods ---

/*
Calculates a triangle normal using the fast normalize methods of rw::math

\param p0 first vertex of triangle
\param p1 second vertex of triangle
\param p2 third vertex of triangle
**/
static rwpmath::Vector3 ComputeTriangleNormalFast(
    rwpmath::Vector3::InParam p0,
    rwpmath::Vector3::InParam p1,
    rwpmath::Vector3::InParam p2)
{
    const rwpmath::Vector3 p0p1(rwpmath::NormalizeFast(p1 - p0));
    const rwpmath::Vector3 p0p2(rwpmath::NormalizeFast(p2 - p0));

    return rwpmath::NormalizeFast(rwpmath::Cross(p0p1, p0p2));
}


/*
Calculates the extended edge cosine of an edge.

\param edgeStart the start of the edge.
\param edgeEnd the end of the edge.
\param triangleNormal the normal of the triangle owning the edge.
\param neighboringTriangleVertex the vertex of the neighboring triangle
       which shares the edge
\return the extended edge cosine
**/
static rwpmath::VecFloat CalculateExtendedEdgeCosine(
    rwpmath::Vector3::InParam edgeStart,
    rwpmath::Vector3::InParam edgeEnd,
    rwpmath::Vector3::InParam triangleNormal,
    const rwpmath::Vector3 * neighboringTriangleVertex)
{
    rwpmath::VecFloat edgeCosine(0.0f);

    // If the triangle has a neighbor along this edge, i.e whether 
    // the edge is shared between two triangles.
    if (NULL != neighboringTriangleVertex)
    {
        // Calculate the normal of the neighboring triangle
        const rwpmath::Vector3 neighborTriangleNormal = ComputeTriangleNormalFast(
            edgeEnd,
            edgeStart,
            *neighboringTriangleVertex);

        // Calculate the edge direction
        rwpmath::Vector3 edgeVector = edgeEnd - edgeStart;

        // Calculate the edge cosine
        edgeCosine = rw::collision::meshbuilder::EdgeCosines::ComputeExtendedEdgeCosine(
            triangleNormal,
            neighborTriangleNormal,
            edgeVector);
    }
    else
    {
        // Calculate the edge cosine and flags of the edge which
        // is not shared between two triangle.
        edgeCosine = CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE;
    }

    return edgeCosine;
}

/*
Calculates the encoded edge cosine of an edge.

\param edgeStart the start of the edge.
\param edgeEnd the end of the edge.
\param triangleNormal the normal of the triangle owning the edge.
\param neighboringTriangleVertex the vertex of the neighboring triangle
       which shares the edge
\param edgeMatched flag indicating whether or not this edge is matched/shared
\param edgeCosineConcaveAngleTolerance tolerance used to determine which range
       of concave edges are disabled.
\return the encoded edge cosine
**/
static uint8_t CalculateEncodedEdgeCosine(
    rwpmath::Vector3::InParam edgeStart,
    rwpmath::Vector3::InParam edgeEnd,
    rwpmath::Vector3::InParam triangleNormal,
    const rwpmath::Vector3 * neighboringTriangleVertex,
    const bool edgeMatched,
    const rwpmath::VecFloat & edgeCosineConcaveAngleTolerance)
{
    rwpmath::VecFloat extendedEdgeCosine = CalculateExtendedEdgeCosine(
                                               edgeStart,
                                               edgeEnd,
                                               triangleNormal,
                                               neighboringTriangleVertex);

    return rw::collision::meshbuilder::EdgeCodeGenerator::GenerateEdgeCode(
        extendedEdgeCosine,
        edgeCosineConcaveAngleTolerance,
        edgeMatched);
}

// main ---

int EAMain(int /*argc*/, char ** /*argv*/)
{
    // Create a heightfield
    const uint32_t xCount = 15;
    const uint32_t zCount = 15;
    QuadHeightField heightField(xCount, zCount);

    // The quad and vertex counts
    const uint32_t numQuads = ((xCount - 1) * (zCount - 1));
    const uint32_t numVertices = xCount * zCount;

    // The unit IDs
    uint16_t unitGroupID[numQuads];
    uint16_t unitSurfaceID[numQuads];
    for (uint32_t i = 0 ; i < numQuads ; ++i)
    {
        unitGroupID[i] = static_cast<uint16_t>(i + 1u);
        unitSurfaceID[i] = static_cast<uint16_t>(numQuads - i);
    }

    // STEP 1 ---
    // The default flag collection describing the uniform unit format we will use.
    uint32_t unitFlagDefault = 0u;
    // Each unit will store edge cosines
    unitFlagDefault |= rw::collision::UNITFLAG_EDGEANGLE;
    // Each unit will store a group ID
    unitFlagDefault |= rw::collision::UNITFLAG_GROUPID;
    // Each unit will store a surface ID
    unitFlagDefault |= rw::collision::UNITFLAG_SURFACEID;

    // Create the Cluster Construction Parameters
    rw::collision::ClusterConstructionParameters clusterConstructionParameters;
    {
        // Initialize the known parameters
        // Each group ID is 2 bytes
        clusterConstructionParameters.mGroupIDSize = 2u;
        // Each surface ID is 2 bytes
        clusterConstructionParameters.mSurfaceIDSize = 2u;
        // The cluster will store X vertices
        clusterConstructionParameters.mVertexCount = numVertices;
        // The vertices will be uncompressed
        clusterConstructionParameters.mVertexCompressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;

        // Sum the unit components
        for (uint32_t quadIndex = 0 ; quadIndex < numQuads ; ++quadIndex)
        {
            rw::collision::meshbuilder::detail::ClusterParametersBuilder::SumUnitComponentCounts(
                clusterConstructionParameters,
                rw::collision::UNITTYPE_QUAD,
                unitFlagDefault,
                unitGroupID[quadIndex],
                unitSurfaceID[quadIndex]);
        }
    }

    // STEP 2 ---
    // Initialize the TriangleClusterProcedural
    rw::collision::TriangleClusterProcedural * triangleClusterProcedural = NULL;
    {
        // Get the resource descriptor required by the TriangleClusterProcedural
        EA::Physics::SizeAndAlignment triangleClusterResourceDescriptor = rw::collision::TriangleClusterProcedural::GetResourceDescriptor(
            clusterConstructionParameters);

        // Allocate the TriangleClusterProcedural
        EA::Physics::MemoryPtr triangleClusterProceduralResource = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(
            static_cast<size_t>(triangleClusterResourceDescriptor.GetSize()),
            NULL,
            0u,
            static_cast<unsigned int>(triangleClusterResourceDescriptor.GetAlignment()));

        // Initialize the TriangleClusterProcedural
        triangleClusterProcedural = rw::collision::TriangleClusterProcedural::Initialize(
            triangleClusterProceduralResource,
            clusterConstructionParameters);

        // Set the ID sizes
        triangleClusterProcedural->SetGroupIdSize(static_cast<uint8_t>(clusterConstructionParameters.mGroupIDSize));
        triangleClusterProcedural->SetSurfaceIdSize(static_cast<uint8_t>(clusterConstructionParameters.mSurfaceIDSize));
    }

    rw::collision::ClusteredMeshCluster & cluster = triangleClusterProcedural->GetCluster();

    // STEP 3 ---
    // Write the vertex collection to the cluster
    for (uint32_t zIndex = 0 ; zIndex < zCount ; ++zIndex)
    {
        for (uint32_t xIndex = 0 ; xIndex < xCount ; ++xIndex)
        {
            rwpmath::Vector3 vec;
            heightField.GetVertex(vec, xIndex, zIndex);
            cluster.SetVertex(
                vec,
                triangleClusterProcedural->GetClusterParams().mVertexCompressionGranularity);
        }
    }

    // STEP 4 ---
    // Write the unit data to the cluster
    rw::collision::UnitParameters unitParameters;
    unitParameters.groupIDSize = static_cast<uint8_t>(clusterConstructionParameters.mGroupIDSize);
    unitParameters.surfaceIDSize = static_cast<uint8_t>(clusterConstructionParameters.mSurfaceIDSize);
    unitParameters.unitFlagsDefault = static_cast<uint8_t>(unitFlagDefault);

    // Set the edge cosine concave angle tolerance to -1.0f to indicate that
    // no concave edges will be disabled.
    const rwpmath::VecFloat edgeCosineConcaveAngleTolerance = -1.0f;

    uint32_t unitIndex = 0u;
    for (uint32_t zIndex = 0 ; zIndex < (zCount - 1) ; ++zIndex)
    {
        for (uint32_t xIndex = 0 ; xIndex < (xCount - 1) ; ++xIndex)
        {
            // The Quad Vertices
            rwpmath::Vector3 quadVertex0;
            rwpmath::Vector3 quadVertex1;
            rwpmath::Vector3 quadVertex2;
            rwpmath::Vector3 quadVertex3;

            heightField.GetQuadVertices(
                quadVertex0,
                quadVertex1,
                quadVertex2,
                quadVertex3,
                xIndex,
                zIndex);

            // The vertices of the triangle surrounding the Quad
            rwpmath::Vector3 * neighboringVertex0;
            rwpmath::Vector3 * neighboringVertex1;
            rwpmath::Vector3 * neighboringVertex2;
            rwpmath::Vector3 * neighboringVertex3;

            heightField.GetAdjacentVertices(
                &neighboringVertex0,
                &neighboringVertex1,
                &neighboringVertex2,
                &neighboringVertex3,
                xIndex,
                zIndex);

            // Calculate the triangle normals of the quad
            const rwpmath::Vector3 triangleNormalA = ComputeTriangleNormalFast(
                quadVertex0,
                quadVertex1,
                quadVertex2);

            const rwpmath::Vector3 triangleNormalB = ComputeTriangleNormalFast(
                quadVertex1,
                quadVertex3,
                quadVertex2);

            const bool edge0Matched = (xIndex > 0 ? true : false);
            const bool edge1Matched = (zIndex < (zCount - 2) ? true : false);
            const bool edge2Matched = (zIndex > 0 ? true : false);
            const bool edge3Matched = (xIndex < (xCount - 2) ? true : false);

            // Generate the edge codes using the extended edge cosines
            const uint8_t encodedEdgeCosine0 = CalculateEncodedEdgeCosine(
                quadVertex0,
                quadVertex1,
                triangleNormalA,
                neighboringVertex0,
                edge0Matched,
                edgeCosineConcaveAngleTolerance);

            const uint8_t encodedEdgeCosine1 = CalculateEncodedEdgeCosine(
                quadVertex1,
                quadVertex3,
                triangleNormalB,
                neighboringVertex1,
                edge1Matched,
                edgeCosineConcaveAngleTolerance);

            const uint8_t encodedEdgeCosine2 = CalculateEncodedEdgeCosine(
                quadVertex2,
                quadVertex0,
                triangleNormalA,
                neighboringVertex2,
                edge2Matched,
                edgeCosineConcaveAngleTolerance);

            const uint8_t encodedEdgeCosine3 = CalculateEncodedEdgeCosine(
                quadVertex3,
                quadVertex2,
                triangleNormalB,
                neighboringVertex3,
                edge3Matched,
                edgeCosineConcaveAngleTolerance);

            // Get the vertex indices
            uint32_t vertexIndex0;
            uint32_t vertexIndex1;
            uint32_t vertexIndex2;
            uint32_t vertexIndex3;

            heightField.GetQuadVertexIndices(
                vertexIndex0,
                vertexIndex1,
                vertexIndex2,
                vertexIndex3,
                xIndex,
                zIndex);

            // Add the quad to the cluster
           cluster.SetQuad(
                unitParameters,
                unitGroupID[unitIndex],
                unitSurfaceID[unitIndex],
                static_cast<uint8_t>(vertexIndex0),
                static_cast<uint8_t>(vertexIndex1),
                static_cast<uint8_t>(vertexIndex2),
                static_cast<uint8_t>(vertexIndex3),
                static_cast<uint8_t>(encodedEdgeCosine0),
                static_cast<uint8_t>(encodedEdgeCosine1),
                static_cast<uint8_t>(encodedEdgeCosine2),
                static_cast<uint8_t>(encodedEdgeCosine3));

            ++unitIndex;
        }
    }

    // STEP 5 ---
    // Finalize the TriangleClusterProcedural.
    triangleClusterProcedural->UpdateThis();

    common::DescribeTriangleClusterProcedural(*triangleClusterProcedural);

    // Release resources
    if (NULL != triangleClusterProcedural)
        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(triangleClusterProcedural);

    return 0;
}

#endif // #if !defined EA_PLATFORM_PS3_SPU

