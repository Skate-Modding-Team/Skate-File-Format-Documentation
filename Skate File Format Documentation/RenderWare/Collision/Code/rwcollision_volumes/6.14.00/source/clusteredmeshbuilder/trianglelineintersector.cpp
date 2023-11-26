// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/trianglelineintersector.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
\brief Intersection test between a line and a triangle in 2D

\param trianglePoint0 first vertex of triangle
\param trianglePoint1 second vertex of triangle
\param trianglePoint2 third vertex of triangle
\param linePoint0 first vertex of line
\param linePoint1 second vertex of line

\return true if intersection is found, false otherwise.
*/
bool
TriangleLineIntersector::IntersectLineWithTriangle2D(
    rwpmath::Vector2::InParam trianglePoint0,
    rwpmath::Vector2::InParam trianglePoint1,
    rwpmath::Vector2::InParam trianglePoint2,
    rwpmath::Vector2::InParam linePoint0,
    rwpmath::Vector2::InParam linePoint1)
{
    typedef rwpmath::VecFloat FloatType;
    typedef rwpmath::Vector2 VectorType;
    typedef rwpmath::MaskScalar MaskScalarType;

    const VectorType triangleEdgeDirection01(trianglePoint1 - trianglePoint0);
    const VectorType triangleEdgeDirection12(trianglePoint2 - trianglePoint1);
    const VectorType triangleEdgeDirection20(trianglePoint0 - trianglePoint2);

    const VectorType edgeDirection(linePoint1 - linePoint0);

    // Check for degenerate pairs of points; we discout the results
    const FloatType triangleEdgeLength01(rwpmath::MagnitudeFast(triangleEdgeDirection01));
    const FloatType triangleEdgeLength12(rwpmath::MagnitudeFast(triangleEdgeDirection12));
    const FloatType triangleEdgeLength20(rwpmath::MagnitudeFast(triangleEdgeDirection20));
    const FloatType edgeLength(rwpmath::MagnitudeFast(edgeDirection));

    // Normalize the edge directions
    const VectorType triangleEdgeNormal01(triangleEdgeDirection01 / triangleEdgeLength01);
    const VectorType triangleEdgeNormal12(triangleEdgeDirection12 / triangleEdgeLength12);
    const VectorType triangleEdgeNormal20(triangleEdgeDirection20 / triangleEdgeLength20);
    const VectorType edgeNormal(edgeDirection / edgeLength);

    // It's sufficient to test just the edge-vertex candidate feature pairs
    // The vertex-vertex pairs are more specialized and may produce better separations,
    // but they're dominated in the sense that, in every case where a vertex-vertex case
    // produces a positive separation, an edge-vertex case produces one too.
    // Futhermore there are cases (vertex right next to middle of an edge) where the edge-vertex
    // case is separated but none of the vertex-vertex cases are.
    FloatType separation(-rwpmath::MAX_FLOAT);

    // Triangle edge 01
    BestSeparationLineTriangle2D(
        separation,
        trianglePoint0,
        trianglePoint1,
        trianglePoint2,
        linePoint0,
        linePoint1,
        triangleEdgeNormal01);

    // Triangle edge 12
    BestSeparationLineTriangle2D(
        separation,
        trianglePoint0,
        trianglePoint1,
        trianglePoint2,
        linePoint0,
        linePoint1,
        triangleEdgeNormal12);

    // Triangle edge 20
    BestSeparationLineTriangle2D(
        separation,
        trianglePoint0,
        trianglePoint1,
        trianglePoint2,
        linePoint0,
        linePoint1,
        triangleEdgeNormal20);

    // Edge direction
    BestSeparationLineTriangle2D(
        separation,
        trianglePoint0,
        trianglePoint1,
        trianglePoint2,
        linePoint0,
        linePoint1,
        edgeNormal);

    // Set the result once
    return (!(rwpmath::CompGreaterThan(separation, rwpmath::GetVecFloat_Zero()).GetBool()));
}


/**
\brief Best separation test between a line and a triangle

\param trianglePoint0 first vertex of triangle
\param trianglePoint1 second vertex of triangle
\param trianglePoint2 third vertex of triangle
\param edgePoint0 first vertex of edge
\param edgePoint1 second vertex of edge
\param edgeDirection direction of edge
\param bestSeparation best separation
*/
void
TriangleLineIntersector::BestSeparationLineTriangle2D(
    rwpmath::VecFloat & bestSeparation,
    rwpmath::Vector2::InParam trianglePoint0,
    rwpmath::Vector2::InParam trianglePoint1,
    rwpmath::Vector2::InParam trianglePoint2,
    rwpmath::Vector2::InParam linePoint0,
    rwpmath::Vector2::InParam linePoint1,
    rwpmath::Vector2::InParam candidateDirection)
{
    typedef rwpmath::VecFloat FloatType;
    typedef rwpmath::Vector2 VectorType;
    typedef rwpmath::MaskScalar MaskScalarType;

    // Generate a candidate separating direction perpendicular to the edge direction
    // This gives us one of two possible, opposite orientations
    const VectorType candidateNormal(candidateDirection.GetY(), -candidateDirection.GetX());

    // Compute the maximum and minimum dots of the triangle in the candidate direction
    const FloatType triangleDot0(rwpmath::Dot(trianglePoint0, candidateNormal));
    const FloatType triangleDot1(rwpmath::Dot(trianglePoint1, candidateNormal));
    const FloatType triangleDot2(rwpmath::Dot(trianglePoint2, candidateNormal));

    const FloatType minTriangleDot01(rwpmath::Min(triangleDot0, triangleDot1));
    const FloatType maxTriangleDot01(rwpmath::Max(triangleDot0, triangleDot1));
    const FloatType minTriangleDot(rwpmath::Min(minTriangleDot01, triangleDot2));
    const FloatType maxTriangleDot(rwpmath::Max(maxTriangleDot01, triangleDot2));

    // Compute the maximum and minimum dots of the edge in the candidate direction
    const FloatType lineDot0(rwpmath::Dot(linePoint0, candidateNormal));
    const FloatType lineDot1(rwpmath::Dot(linePoint1, candidateNormal));

    const FloatType minLineDot(rwpmath::Min(lineDot0, lineDot1));
    const FloatType maxLineDot(rwpmath::Max(lineDot0, lineDot1));

    // Candidate separations in the forward and backward candidate direction
    const FloatType separationForward(minTriangleDot - maxLineDot);
    const FloatType separationBackward(minLineDot - maxTriangleDot);
    const FloatType candidateSeparation(rwpmath::Max(separationForward, separationBackward));

    // Update if better than current
    const MaskScalarType improvesSeparation(rwpmath::CompGreaterThan(candidateSeparation, bestSeparation));
    bestSeparation = rwpmath::Select(improvesSeparation, candidateSeparation, bestSeparation);
}


} // namespace detail
} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

