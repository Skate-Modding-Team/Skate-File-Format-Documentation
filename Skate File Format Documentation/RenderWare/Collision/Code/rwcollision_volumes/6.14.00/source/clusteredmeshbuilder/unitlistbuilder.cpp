// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/unitlistbuilder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


uint32_t UnitListBuilder::BuildUnitListWithQuads(
    UnitList & unitList,
    IDList & compressedUnitIndex,
    const TriangleList & triangles,
    const TriangleSurfaceIDList & triangleSurfaceIDs,
    const TriangleGroupIDList & triangleGroupIDs,
    const TriangleNeighborsList & triangleNeighbors,
    const TriangleFlagsList & triangleFlags,
    const VertexList & vertices,
    const uint32_t surfaceIDSize,
    const uint32_t groupIDSize)
{
    // Each valid input triangle is converted into a Unit and added to the Unit collection. As
    // a new unit is created (UnitA) a search takes place to find another Unit (UnitB) which shares
    // the longest edge of UnitA. If UnitB is found and its type is TRIANGLE then UnitB is converted
    // into a Unit of type QUAD using the extra data from UnitA, after which UnitA is deleted.
    // However, if UnitB is not found, or UnitB is of type QUAD then UnitA is added to the Unit collection.

    Unit unit;
    uint32_t numTrisHandled = 0, numUnitsAdded = 0;

    // The compressedUnitIndex is used to map triangle indices to unit indices.
    // For example: if triangles with indices A and B are merged to form a quad then
    // compressedUnitIndex[A] = compressedUnitIndex[B] = A (the indices merge to the
    // lower of the two)

    // Iterate over all of the triangles
    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        // Ignore disabled triangles
        if (triangleFlags[triangleIndex].enabled)
        {
            // see if any of this triangle's neighbors are already in the unit list

            // if any are suitable, append this one and mark the result as a quad
            const Triangle &triangle = triangles[triangleIndex];
            const TriangleSurfaceID &tSurfaceID = triangleSurfaceIDs[triangleIndex];
            const TriangleGroupID &tGroupID = triangleGroupIDs[triangleIndex];
            const TriangleNeighbors &tNeighbors = triangleNeighbors[triangleIndex];

            // find longest edge on the current triangle, as we only attempt to merge
            // on the longest edge
            rwpmath::Vector3 v0(vertices[triangles[triangleIndex].vertices[0]]);
            rwpmath::Vector3 v1(vertices[triangles[triangleIndex].vertices[1]]);
            rwpmath::Vector3 v2(vertices[triangles[triangleIndex].vertices[2]]);

            uint32_t longestEdge = 0;
            rwpmath::VecFloat edgeLen = rwpmath::Magnitude(v1 - v0);
            rwpmath::VecFloat biggestEdgeLen = edgeLen;

            edgeLen = rwpmath::Magnitude(v2-v1);
            if ( edgeLen > biggestEdgeLen )
            {
                biggestEdgeLen = edgeLen;
                longestEdge = 1;
            }

            edgeLen = rwpmath::Magnitude(v0-v2);
            if ( edgeLen > biggestEdgeLen )
            {
                biggestEdgeLen = edgeLen;
                longestEdge = 2;
            }

            bool thisTriAppended = false;

            uint32_t neighboringTriangle = tNeighbors.neighbor[longestEdge];

            // Check if the neighboring triangle is not disabled and has already been converted into a Unit
            if ( neighboringTriangle < numTrisHandled && triangleFlags[neighboringTriangle].enabled)
            {
                // Obtain the Unit related to the neighboring triangle
                Unit &neighborUnit = unitList[compressedUnitIndex[neighboringTriangle]];

                // If the neighboring triangle has not yet been merged into a quad
                if ( neighborUnit.type == Unit::TYPE_TRIANGLE )
                {
                    // Obtain the neighboring triangle data
                    const Triangle &nt = triangles[neighboringTriangle];
                    const TriangleSurfaceID &ntSurfaceID = triangleSurfaceIDs[neighboringTriangle];
                    const TriangleGroupID &ntGroupID = triangleGroupIDs[neighboringTriangle];
                    const TriangleNeighbors &ntNeighbors = triangleNeighbors[neighboringTriangle];

                    // Check if both the group IDs and surface IDs match, if appropriate
                    if ((surfaceIDSize == 0 || ntSurfaceID == tSurfaceID) &&
                        (groupIDSize == 0 || ntGroupID == tGroupID))
                    {
                        // Convert the unit into a quad
                        neighborUnit.type = Unit::TYPE_QUAD;
                        neighborUnit.tri1 = triangleIndex;
                        neighborUnit.extraVertex = (longestEdge + 2) % 3;
                        neighborUnit.longestEdgeOnTri1 = longestEdge;

                        if (ntNeighbors.neighbor[0] == triangleIndex && nt.vertices[1] == triangle.vertices[longestEdge])
                        {
                            neighborUnit.edgeOpposingExtraVertex = 0;
                        }
                        else if (ntNeighbors.neighbor[1] == triangleIndex && nt.vertices[2] == triangle.vertices[longestEdge])
                        {
                            neighborUnit.edgeOpposingExtraVertex = 1;
                        }
                        else if (ntNeighbors.neighbor[2] == triangleIndex && nt.vertices[0] == triangle.vertices[longestEdge])
                        {
                            neighborUnit.edgeOpposingExtraVertex = 2;
                        }
                        else
                        {
                            // this should never happen, one-way neighboring?!?
                            EA_FAIL_MSG("Triangle neighboring information is invalid." );
                        }

                        // Update the compressedUnitIndex list
                        compressedUnitIndex[triangleIndex] = compressedUnitIndex[neighboringTriangle];
                        thisTriAppended = true;
                    }
                }
            }

            // If this triangle has not been merged into a quad
            if ( !thisTriAppended )
            {
                // still need to add it normally
                unit.tri0 = triangleIndex;
                unit.tri1 = 0;
                unit.type = Unit::TYPE_TRIANGLE;
                unit.extraVertex = 0;
                unit.edgeOpposingExtraVertex = 0;
                unit.longestEdgeOnTri1 = 0;

                unitList.push_back(unit);

                compressedUnitIndex[triangleIndex] = numUnitsAdded;
                numUnitsAdded++;
            }
        }

        numTrisHandled++;
    }

    EAPHYSICS_MESSAGE("%f%% triangles converted into quads (%d out of %d)",
        100.0f*(numTrisHandled - numUnitsAdded)*2.0f/numTrisHandled,
        (numTrisHandled - numUnitsAdded)*2, numTrisHandled );

    return unitList.size();
}


uint32_t UnitListBuilder::BuildUnitListWithTriangles(
    UnitList & unitList,
    const TriangleList & triangles,
    const TriangleFlagsList & triangleFlags)
{
    Unit unit;

    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex)
    {
        if (triangleFlags[triangleIndex].enabled)
        {
            unit.tri0 = triangleIndex;
            unit.tri1 = 0;
            unit.type = Unit::TYPE_TRIANGLE;
            unit.extraVertex = 0;
            unit.edgeOpposingExtraVertex = 0;
            unit.longestEdgeOnTri1 = 0;

            unitList.push_back(unit);
        }
    }

    return unitList.size();
}


} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

