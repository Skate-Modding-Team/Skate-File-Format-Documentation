// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilderutils.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
\brief Converts the edge cosine real number [-1...3] into the angleByte which is -Log(A/pi)/Log(sqrt(2)).

The factor sqrt(2) is used for the log base because the reversal function (computing edge cosine from B)
works out nicely that way.  B=0 means fully convex, B=26 means the two triangles are coplanar.
The range of the result is 0...26 which is 5 bits.  B is clamped to 26 because larger values cause
division by zero in the DecodeEdgeCos function.

\param edgeCosine A edge cosine value in the range of -1.0 to 3.0.
\return The byte encoding data for this edge.
*/
uint8_t
ClusteredMeshBuilderUtils::EdgeCosineToAngleByte(const rwpmath::VecFloat & edgeCosine)
{
    const rwpmath::VecFloat min_angle(6.6e-5f);    // this is PI * sqrt(2) ^ (-31)

    rwpmath::VecFloat angle;

    if ( edgeCosine > rwpmath::VecFloat(1.0f) )
    {
        angle = rwpmath::ACos(rwpmath::VecFloat(2.0f) - edgeCosine);
    }
    else
    {
        angle = rwpmath::ACos(edgeCosine);
    }

    EA_ASSERT_FORMATTED(angle >= rwpmath::VecFloat(0.0f) && angle <= rwpmath::PI, ("Bad angle"));// %g, edgeCos %g.",angle, edgeCosine));

    angle = rwpmath::Max(angle, min_angle);

    int result = static_cast<int>( rwpmath::VecFloat(-2.0f) * rwpmath::Log(angle / rwpmath::PI) / rwpmath::Log(2.0f));

    // Clamp the result to the range of 0 - 26
    if ( result < 0 )
    {
        return static_cast<uint8_t>(0u);
    }
    else if ( result > 26 )
    {
        return static_cast<uint8_t>(26u);
    }

    return static_cast<uint8_t>(result);
}


/**
\brief Determines if an edge produces a featureless plane when applied to 
an existing pair of edges.

Given a vertex hub and its surrounding features a featureless plane is defined
as a plane passing though the vertex which may only rotate around one axis,
at most, while all feature surrounding the hub remain on one side of the plane.
The candidate axis around which this plane can rotate are defined by the
edges features of the vertex hub.

The cosine tolerance is used when determining whether or not edgeC lies between
edgeA and edgeB.

\param edgeA first edge of plane
\param edgeB second plane of edge
\param edgeC new candidate edge
\param cosineTolerance cosine tolerance

\return true if the edge produces a featureless plane, false otherwise.
*/
bool
ClusteredMeshBuilderUtils::EdgeProducesFeaturelessPlane(rwpmath::Vector3::InOutParam edgeA,
                                                        rwpmath::Vector3::InOutParam edgeB,
                                                        rwpmath::Vector3::InParam edgeC,
                                                        const rwpmath::VecFloat & cosineTolerance)
{
    // Tolerance used to determine whether or not an edge lies inbetween two given edges.
    // The tolerance is a cosine-angle.

    rwpmath::VecFloat AdotB(rwpmath::Dot(edgeA, edgeB));

    // Determine the halfspace defined by the combination of the feature edge
    rwpmath::Vector3 halfSpace(edgeA + edgeB);
    // If the candidate edge lies on the negative halfspace
    if (rwpmath::Dot(halfSpace, -edgeC) >= rwpmath::GetVecFloat_Zero())
    {
        // Check if the negated candidate edge lies between the feature edges
        if ( (rwpmath::Dot(-edgeC, edgeA) >= (AdotB - cosineTolerance)) &&
            (rwpmath::Dot(-edgeC, edgeB) >= (AdotB - cosineTolerance)) )
        {
            // Point lies in no-tilt-zone, therefore vertex can be disabled.
            return true;
        }
    }

    rwpmath::VecFloat AdotC(rwpmath::Dot(edgeA, edgeC));
    rwpmath::VecFloat BdotC(rwpmath::Dot(edgeB, edgeC));

    // Determine if the candidate edge lies outside of edge B
    if ( AdotC < BdotC && AdotC < AdotB)
    {
        // Update edge B, expanding the feature edges
        edgeB = edgeC;
    }
    // Determine if the candidate edge lies outside of edge A
    else if (BdotC < AdotB)
    {
        // Update edge A, expanding the feature edges.
        edgeA = edgeC;
    }

    // The candidate edge does not contribute to the feature edges, it 
    // lies between the feature edges.
    return false;
}



/**
\brief Determines if an edge disables a vertex.

This method determines if adding the edge to the current feature plane disables the edge,
firstly by checking if the new edge creates a featureless plane and secondly by
checking if the edge causes the vertex to be situated in a concave region.

\param edgeA
\param mergeQuads Unused.

\return Number of units constructed.
*/

bool
ClusteredMeshBuilderUtils::EdgeDisablesVertex(rwpmath::Vector3::InOutParam edgeA,
                                              rwpmath::Vector3::InOutParam edgeB,
                                              rwpmath::Vector3::InOutParam edgeC,
                                              rwpmath::Vector3::InParam planeNormal,
                                              const rwpmath::VecFloat & coplanarCosineTolerance,
                                              const rwpmath::VecFloat & cosineTolerance,
                                              const rwpmath::VecFloat & concaveCosineTolerance)
{
    rwpmath::VecFloat planeEdgeCDot(rwpmath::Dot(edgeC,planeNormal));

    // If the edge is coplanar
    if (rwpmath::IsSimilar(planeEdgeCDot, 0.0f, coplanarCosineTolerance))
    {
        // Check if the edge disabled the vertex, or advances the feature plane edges
        if (EdgeProducesFeaturelessPlane(edgeA, edgeB, edgeC, cosineTolerance))
        {
            // Edge disabled vertex
            return true;
        }
    }
    // If the dot product of the edge and the plane normal is less than
    // cosineConcaveTolerance then the vertex hub is concave and therefore
    // this edge disables the vertex hub. The less than operator is used as
    // the edge points towards the vertex hub.
    else if (planeEdgeCDot < concaveCosineTolerance)
    {
        // Edge disabled vertex
        return true;
    }

    return false;
}


} // namespace detail
} // namespace meshbuilder
} // namespace rw
} // namespace collision

