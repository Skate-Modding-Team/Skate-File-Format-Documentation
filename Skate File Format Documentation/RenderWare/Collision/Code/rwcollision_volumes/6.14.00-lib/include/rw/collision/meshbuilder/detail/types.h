// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_TYPES_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_TYPES_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU


#include <rw/math/fpu.h>

#include <rw/collision/kdtreebuilder.h>

#include <rw/collision/clusteredmeshcluster.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
\struct Triangle
\brief A struct containing three vertex indices defining a triangle.
*/
struct Triangle
{
    /**
    \brief Default Constructor
    Initializes members to default state.
    */
    Triangle()
    {
    }

    /// Indices of triangle vertices, index into vert list.
    uint32_t vertices[3];
};


/**
Integer ID identifying the surface of which a triangle is a component.
*/
typedef uint32_t TriangleSurfaceID;


/**
Integer ID identifying the face group of which a triangle is a member.
*/
typedef uint32_t TriangleGroupID;


/**
\struct TriangleEdgeCodes
\brief Contains the byte-encoded edge cosines of the three edges of a single triangle.
*/
struct TriangleEdgeCodes
{
    /**
    \brief Default Constructor
    Initializes members to default state.
    */
    TriangleEdgeCodes()
    {
        encodedEdgeCos[0] = 0;
        encodedEdgeCos[1] = 0;
        encodedEdgeCos[2] = 0;
    }

    /// Encoded edge cosines of each triangle edge. See ClusteredMeshBuilderAngleByte(float edgeCos) for more details.
    uint8_t encodedEdgeCos[3];
};


/**
\struct TriangleEdgeCosines
Stores edge cosines for a single triangle
*/
struct TriangleEdgeCosines
{
    /// Edge cosines of each triangle edge
    float edgeCos[3];
};


/**
\struct TriangleNeighbors
Stores neighbor triangle indices for a single triangle
*/
struct TriangleNeighbors
{
    /// Neighboring triangle indices
    uint32_t neighbor[3];
};


/**
\struct TriangleFlags
\brief A class for flag data associated with a triangle.
*/
struct TriangleFlags
{
    /**
    \brief Default Constructor
    Initializes the enabled flag to true, the default value.
    */
    TriangleFlags() : enabled(true)
    {
    }

    /// Flag indicating whether or not triangle is enabled
    bool enabled;
};


/**
\struct Unit
\brief A Unit is a triangle or quad (two joined triangles). However, the design may be
altered to expand the scope of the unit to include fans, strips, lists etc.
*/
struct Unit
{
    /**
    \enum The type of data in the unit.
    \note A quad unit is actually just a pair of triangles sharing an edge. The triangles need not be co-planar.
    */
    enum
    {
        TYPE_TRIANGLE = UNITTYPE_TRIANGLE,
        TYPE_QUAD = UNITTYPE_QUAD
    };

    /// Index of first triangle
    uint32_t tri0;
    /// Index of second triangle, if unit is quad
    uint32_t tri1;
    /// Type of unit, either triangle or quad
    uint32_t type : 2;
    /// Local index of extra vertex on tri1
    uint32_t extraVertex : 2;
    /// Local index of tri0 edge opposing extra vertex on tri1
    uint32_t edgeOpposingExtraVertex : 2;
    /// Local index of longest edge on tri1
    uint32_t longestEdgeOnTri1 : 2;
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


#endif // PUBLIC_RW_COLLISION_MESHBUILDER_TYPES_H
