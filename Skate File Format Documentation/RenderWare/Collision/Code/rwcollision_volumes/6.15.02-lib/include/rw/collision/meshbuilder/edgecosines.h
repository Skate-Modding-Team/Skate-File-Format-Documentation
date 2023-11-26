// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_EDGECOSINES_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_EDGECOSINES_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
Static helper class that computes edge cosine values for paired triangle edges.
*/
class EdgeCosines
{

public:

    /**
    \brief Computes the edge cosine of a given triangle edge, and a flag indicating its convexity.

    The computed edge cosine value has range [-1, +1] and is effectively the cosine of the angle
    between the normal vectors of the two triangles incident to the edge. Note that edge cosines
    are only properly defined for matched edges with two incident triangles.

    Additionally a Boolean flag value is returned indicating whether the edge is convex or concave.
    A convex edge is one where the normals of the incident triangles point away from each other.

    This convexity information can't be represented by a simple edge cosine, since positive and
    negative edge angles produce identical edge cosines. Together, the edge cosine value and the
    flag serve to completely characterize the geometry of the edge for collision.

    \param edgeCosine                               Returned cosine of the angle between the triangle normals.
    \param convex                                   Returned MaskScalar flag indicating whether the edge is convex.
    \param triangleOneNormal                        Normalized direction vector of the first triangle incident to the edge.
    \param triangleTwoNormal                        Normalized direction vector of the second triangle incident to the edge.
    \param normalizedEdgeDirectionInTriangleOne     Normalized direction vector of the edge, as it occurs in the first incident triangle.

    \note By convention the direction of the edge is specified with the orientation that the edge
    takes in the winding order of the first incident triangle.

    \see ComputeExtendedEdgeCosine
    \see DecodeExtendedEdgeCosine
    */
    static void ComputeEdgeCosine(
        rwpmath::VecFloat &edgeCosine,
        rwpmath::MaskScalar &convex,
        rwpmath::Vector3::InParam triangleOneNormal,
        rwpmath::Vector3::InParam triangleTwoNormal,
        rwpmath::Vector3::InParam normalizedEdgeDirectionInTriangleOne);

    /**
    \brief Computes an "extended" edge cosine value characterizing a given triangle edge.

    The edge cosine of an edge is effectively the cosine of the angle between the normal vectors of
    the two triangles incident to the edge. Note that edge cosines are only properly defined for
    matched edges with two incident triangles.

    Unlike a conventional edge cosine, the "extended" edge cosine has range [-1, +3]. The range [-1, +1]
    is used to denote the edge cosines of convex edges, and the range [+1, +3] is used to denote the
    edge cosines of concave or reflex edges. The [+1, +3] range provides valid edge cosine values
    in the case where the run-time code wants to do collision with the convex back side of the concave
    edge.

    A convex edge is one where the normals of the incident triangles point away from each other.
    This convexity information can't be represented by a simple edge cosine, since positive and
    negative edge angles produce identical edge cosines. Instead, this method returns an extended
    value with values for convex and concave edges mapped into different halves of the range.

    The extended edge cosine of a concave or reflex edge is simply represented as the conventional
    edge cosine of the edge plus two. Edges cosines of convex edges are stored as normal. In this
    way an extended edge cosine serves to completely characterize the geometry of the edge for collision.

    \param triangleOneNormal                        Normalized direction vector of the first triangle incident to the edge.
    \param triangleTwoNormal                        Normalized direction vector of the second triangle incident to the edge.
    \param normalizedEdgeDirectionInTriangleOne     Normalized direction vector of the edge, as it occurs in the first incident triangle.

    \return The "extended" cosine of the angle between the triangle normals.

    \note By convention the direction of the edge is specified with the orientation that the edge
    takes in the winding order of the first incident triangle.

    \see DecodeExtendedEdgeCosine
    */
    static rwpmath::VecFloat ComputeExtendedEdgeCosine(
        rwpmath::Vector3::InParam triangleOneNormal,
        rwpmath::Vector3::InParam triangleTwoNormal,
        rwpmath::Vector3::InParam normalizedEdgeDirectionInTriangleOne);

    /**
    \brief Decodes an "extended" edge cosine value to produce an edge cosine and a convexity flag.

    The extended edge cosine of a concave or reflex edge is simply represented as the conventional
    edge cosine of the edge plus two. Edges cosines of convex edges are stored as normal. In this
    way an extended edge cosine serves to completely characterize the geometry of the edge for collision.

    This method maps an extended edge cosine with range [-1, +3] to a conventional edge cosine with range
    [-1, +1] and an additional MaskScalar flag indicating edge convexity.

    \param edgeCosine           Returned cosine of the angle between the triangle normals.
    \param convex               Returned MaskScalar flag indicating whether the edge is convex.
    \param extendedEdgeCosine   Extended range edge cosine of the triangle edge.
    
    \see ComputeExtendedEdgeCosine
    */
    static void DecodeExtendedEdgeCosine(
        rwpmath::VecFloat &edgeCosine,
        rwpmath::MaskScalar &convex,
        const rwpmath::VecFloat &extendedEdgeCosine);
};


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_EDGECOSINES_H
