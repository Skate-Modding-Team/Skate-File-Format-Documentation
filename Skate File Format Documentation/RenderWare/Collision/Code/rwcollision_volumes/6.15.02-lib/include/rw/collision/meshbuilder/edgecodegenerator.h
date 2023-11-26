// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_EDGECODEGENERATOR_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_EDGECODEGENERATOR_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/detail/containers.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
Static helper class that generates byte-encoded edge cosine values.
*/
class EdgeCodeGenerator
{

public:

    typedef detail::TriangleEdgeCodesList TriangleEdgeCodesList;
    typedef detail::TriangleEdgeCosinesList TriangleEdgeCosinesList;
    typedef detail::TriangleNeighborsList TriangleNeighborsList;

    /**
    \brief Initializes a provided collection of triangle edge codes before use.

    Resets the edge codes of all triangle edges to zero. It is important that the
    edge codes are reset because GenerateTriangleEdgeCodes and other similar methods
    OR in the edge codes that they compute. Nevertheless calling this method is typically
    non-essential since the constructor of the TriangleEdgeCodes struct resets
    the values on construction. However it is provided in case users wish to explicitly
    initialize the codes for some reason, for example if the edge code collection was
    initialized without construction (via reinterpret_cast).

    \param triangleEdgeCodes The triangle edge cosine codes container to be initialized.
    */
    static void InitializeTriangleEdgeCodes(TriangleEdgeCodesList &triangleEdgeCodes);

    /**
    \brief Computes edge cosine codes for the edges of a collection of triangles.

    This method computes encoded representations of the edge cosine values and per-edge
    adjacency information for a set of triangles and stores them in a provided container
    of per-triangle-edge edge codes.

    The classification of edges as convex or concave from their extended edge cosines
    is controlled by a caller-supplied tolerance parameter that allows user to broadly
    control the classification of edges. This parameter has legal range [-1, +1], and
    represents (the edge cosine of) the angle of the most concave edge permitted (not
    disabled). A value of +1, corresponding to a planar edge (with parallel triangle
    normals pointing in the same direction), dictates that any even slightly concave
    edge will be disabled. A value of -1, corresponding to a "closed" edge with no
    inner region on the concave side, dictates that even such edges are permitted and
    are not disabled.

    \param triangleEdgeCodes        Collection of triangle edge cosine codes to be filled.
    \param triangleEdgeCosines      Collection of per-triangle edge cosine triples.
    \param triangleNeighbors        Collection of per-triangle edge neighbor triples.
    \param minConcaveEdgeCosine     Threshold edge cosine below which concave edges are disabled.
    */
    static void GenerateTriangleEdgeCodes(
        TriangleEdgeCodesList &triangleEdgeCodes,
        const TriangleEdgeCosinesList &triangleEdgeCosines,
        const TriangleNeighborsList &triangleNeighbors,
        const rwpmath::VecFloat &minConcaveEdgeCosine);

    /**
    \brief Encodes an "extended" edge cosine value and "matched" flag of a triangle edge into a single byte.

    If the angle is convex, the flag EDGEFLAG_EDGECONVEX is added to the result.

    If the edge is unmatched, the flags EDGEFLAG_EDGEUNMATCHED and EDGEFLAG_ANGLEZERO
    are added to the result.

    If the edge is more concave than the limit defined by the minConcaveEdgeCosine tolerance
    parameter then the edge is disabled: the flag EDGEFLAG_ANGLEZERO is added to the result.

    The classification of edges as convex or concave from their extended edge cosines
    is controlled by a caller-supplied tolerance parameter that allows user to broadly
    control the classification of edges. This parameter has legal range [-1, +1], and
    represents (the edge cosine of) the angle of the most concave edge permitted (not
    disabled). A value of +1, corresponding to a planar edge (with parallel triangle
    normals pointing in the same direction), dictates that any even slightly concave
    edge will be disabled. A value of -1, corresponding to a "closed" edge with no
    inner region on the concave side, dictates that even such edges are permitted and
    are not disabled.
    
    \note The concave edge disabling threshold is specified as a conventional edge cosine
    rather than as an extended one.

    \param extendedEdgeCosine       The extended edge cosine value of the edge.
    \param minConcaveEdgeCosine     Threshold edge cosine below which concave edges are disabled.
    \param matched                  Flag indicating whether or not the edge is matched with a neighbor.

    \return Single-byte edge code representing an encoded edge cosine and associated flags.
    */
    static uint8_t GenerateEdgeCode(
        const rwpmath::VecFloat &extendedEdgeCosine,
        const rwpmath::VecFloat &minConcaveEdgeCosine,
        const bool matched);

private:

    typedef detail::TriangleEdgeCodes TriangleEdgeCodes;
    typedef detail::TriangleEdgeCosines TriangleEdgeCosines;
    typedef detail::TriangleNeighbors TriangleNeighbors;
};


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_EDGECODEGENERATOR_H
