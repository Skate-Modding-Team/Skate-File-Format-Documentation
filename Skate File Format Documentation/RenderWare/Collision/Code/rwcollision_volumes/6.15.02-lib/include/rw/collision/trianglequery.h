// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_TRIANGLE_QUERY_H
#define PUBLIC_RW_COLLISION_TRIANGLE_QUERY_H

/*************************************************************************************************************

File: rwctriangle.hpp

Purpose: Declaration of the Triangle queries

*/


#include "rw/collision/common.h"
#include "rw/collision/deprecated/linetriangle.h"

namespace rw {
namespace collision {


/**
\internal
\brief A struct to package up the arguments and results of a triangle line test

\importlib rwccore
*/
struct TriangleQuery
{
    TriangleQuery()
    {

    };
    /**
    \internal
    \brief Constructor

    \param v0 The first vertex of the triangle to query
    \param v1 The second vertex of the triangle to query
    \param v2 The third vertex of the triangle to query
    \importlib rwccore
    */
    TriangleQuery(rwpmath::Vector3::InParam v0, rwpmath::Vector3::InParam v1, rwpmath::Vector3::InParam v2)
    {
        this->v0 = v0;
        this->v1 = v1;
        this->v2 = v2;
    }

    rwpmath::Vector3 v0;               ///< The first vertex of the triangle to query
    rwpmath::Vector3 v1;               ///< The second vertex of the triangle to query
    rwpmath::Vector3 v2;               ///< The third vertex of the triangle to query

    rwpmath::Vector3 triParam;         ///< A Vector holding the normaized triangle parameters of the intersection point
    rwpmath::VecFloat lineParam;       ///< A Vector holding the normaized line parameter of the intersection point
    rwpmath::Vector3 position;         ///< The position of the intersection in world space
    rwpmath::MaskScalar intersects;    ///< A mask holding whether the line intersects the triangle
};

RW_COLLISION_FORCE_INLINE rwpmath::Vector4
DotSoA(rwpmath::Vector4::InParam v1x, rwpmath::Vector4::InParam v1y, rwpmath::Vector4::InParam v1z,
       rwpmath::Vector4::InParam v2x, rwpmath::Vector4::InParam v2y, rwpmath::Vector4::InParam v2z)
{
    using namespace rwpmath;
    rwpmath::Vector4 ret = rwpmath::Mult(v1x, v2x);
    ret = rw::physics::mathutils::MultAdd(v1y, v2y, ret);
    ret = rw::physics::mathutils::MultAdd(v1z, v2z, ret);
    return ret;
}

RW_COLLISION_FORCE_INLINE rwpmath::Vector4
DotSoA(rwpmath::Vector3::InParam v1,
       rwpmath::Vector4::InParam v2x, rwpmath::Vector4::InParam v2y, rwpmath::Vector4::InParam v2z)
{
    using namespace rwpmath;
    rwpmath::Vector4 ret = rwpmath::Mult(v2x, v1.GetX());
    ret = rw::physics::mathutils::MultAdd(v2y, rw::physics::mathutils::ScalarToVector4(v1.GetY()), ret);
    ret = rw::physics::mathutils::MultAdd(v2z, rw::physics::mathutils::ScalarToVector4(v1.GetZ()), ret);
    return ret;
}

RW_COLLISION_FORCE_INLINE void
CrossSoA(rwpmath::Vector4::InParam v1x, rwpmath::Vector4::InParam v1y, rwpmath::Vector4::InParam v1z,
         rwpmath::Vector4::InParam v2x, rwpmath::Vector4::InParam v2y, rwpmath::Vector4::InParam v2z,
         rwpmath::Vector4::OutParam outx, rwpmath::Vector4::OutParam outy, rwpmath::Vector4::OutParam outz)
{
    outx = rwpmath::Subtract(rwpmath::Mult(v1y, v2z), rwpmath::Mult(v1z, v2y));
    outy = rwpmath::Subtract(rwpmath::Mult(v1z, v2x), rwpmath::Mult(v1x, v2z));
    outz = rwpmath::Subtract(rwpmath::Mult(v1x, v2y), rwpmath::Mult(v1y, v2x));
}

RW_COLLISION_FORCE_INLINE void
CrossSoA(rwpmath::Vector4::InParam v1x, rwpmath::Vector4::InParam v1y, rwpmath::Vector4::InParam v1z,
         rwpmath::Vector3::InParam v2,
         rwpmath::Vector4::OutParam outx, rwpmath::Vector4::OutParam outy, rwpmath::Vector4::OutParam outz)
{
    outx = rwpmath::Subtract(rwpmath::Mult(v1y, v2.GetZ()), rwpmath::Mult(v1z, v2.GetY()));
    outy = rwpmath::Subtract(rwpmath::Mult(v1z, v2.GetX()), rwpmath::Mult(v1x, v2.GetZ()));
    outz = rwpmath::Subtract(rwpmath::Mult(v1x, v2.GetY()), rwpmath::Mult(v1y, v2.GetX()));
}

RW_COLLISION_FORCE_INLINE void
CrossSoA(rwpmath::Vector3::InParam v1,
         rwpmath::Vector4::InParam v2x, rwpmath::Vector4::InParam v2y, rwpmath::Vector4::InParam v2z,
         rwpmath::Vector4::OutParam outx, rwpmath::Vector4::OutParam outy, rwpmath::Vector4::OutParam outz)
{
    outx = rwpmath::Subtract(rwpmath::Mult(v2z, v1.GetY()), rwpmath::Mult(v2y, v1.GetZ()));
    outy = rwpmath::Subtract(rwpmath::Mult(v2x, v1.GetZ()), rwpmath::Mult(v2z, v1.GetX()));
    outz = rwpmath::Subtract(rwpmath::Mult(v2y, v1.GetX()), rwpmath::Mult(v2x, v1.GetY()));
}


/**
\internal
\brief An un-normalized intersection test between 1 triangle and 1 line

This returns the un normalized position, line param and triangle params if an intersection occours.  To normalize
these values, divide them by 1/Det.  This function is mainly designed for use if you only want to know *if* the line
intersects, not where

\param v0            The first vertex of the triangle to test
\param edge1        The vector from v0 to the first vertex of the triangle
\param edge2        The vector from v0 to the second vertex of the triangle
\param lineStart    The position of the start of the line
\param lineDelta    The vector from the start of the line to the end of the line
\param Det            This gets filled in with the value that can be used to normalize the parameters
\param W1            The first barycentric triangle param
\param W2            The second barycentric triangle param
\param Alpha        The parametric distance along the line where the intersection occurs

\return                A mask which is true if an intersection occurs and false if not

\importlib rwccore
*/
RW_COLLISION_FORCE_INLINE rwpmath::MaskScalar
TriangleLineSegIntersect(rwpmath::Vector3::InParam v0,
                         rwpmath::Vector3::InParam edge1,
                         rwpmath::Vector3::InParam edge2,
                         rwpmath::Vector3::InParam lineStart,
                         rwpmath::Vector3::InParam lineDelta,

                         rwpmath::VecFloat &Det,
                         rwpmath::VecFloat &W1,
                         rwpmath::VecFloat &W2,
                         rwpmath::VecFloat &Alpha
                         )
{
    rwpmath::Vector3 tVec, pVec, qVec;
    rwpmath::VecFloat det, lo, hi, u, v;

    // Begin calculating determinant - also used to calculate u parameter
    pVec = rwpmath::Cross( lineDelta, edge2 );

    // If determinant is near zero, ray lies in plane of triangle, if negative, triangle is backfacing
    det = rwpmath::Dot( edge1, pVec );

    RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEPSILON_VecFloat, RTINTSECEPSILON);
    rwpmath::MaskScalar detValid = rwpmath::CompGreaterThan(det, RTINTSECEPSILON_VecFloat);

    // Calculate bounds for parameters with tolerance
    RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEDGEEPS_VecFloat, RTINTSECEDGEEPS);
    lo = - det * RTINTSECEDGEEPS_VecFloat;
    hi = det - lo;

    // Calculate u parameter and test bounds
    tVec = lineStart - v0;
    u = rwpmath::Dot( tVec, pVec );

    rwpmath::MaskScalar uValid =
        rwpmath::And( rwpmath::CompGreaterEqual(u, lo), rwpmath::CompLessEqual(u, hi) );

    // Calculate v parameter and test bounds
    qVec = rwpmath::Cross( tVec, edge1 );
    v = rwpmath::Dot( lineDelta, qVec );

    rwpmath::MaskScalar vValid =
        rwpmath::And( rwpmath::CompGreaterEqual(v, lo), rwpmath::CompLessEqual(u + v, hi) );

    // Calculate t, and make sure intersection is in bounds of line
    rwpmath::VecFloat t = rwpmath::Dot(edge2, qVec);

    rwpmath::MaskScalar tValid =
        rwpmath::And( rwpmath::CompGreaterEqual(t, lo), rwpmath::CompLessEqual(t, hi) );

    // output unnormalized intersection parameters
    Det   = det;
    W1    = u;
    W2    = v;
    Alpha = t;

    // determine whether intersection occurs from all parameter bound tests
    rwpmath::MaskScalar resultValid = rwpmath::And( rwpmath::And(uValid, vValid), rwpmath::And(detValid, tValid) );

    return  resultValid ;
}


/**
\internal
\brief A normalized intersection test between 1 triangle and 1 line

This returns the normalized position, line param and triangle params if an intersection occours.  This function is slightly
slower than the unnormalized version, so it should be reserved for when you usually get hits and always need the position

\param v0            The first vertex of the triangle to test
\param v1            The seconf vertex of the triangle to test
\param v2            The third vertex of the triangle to test
\param lineStart    The position of the start of the line
\param lineDelta    The vector from the start of the line to the end of the line
\param position        The position in world space of the intersection
\param lineParam    The parametric distance along the line where the intersection occurs
\param triParam        The barycentric parameter showing where on the triangle the line hits

\return                A mask which is true if an intersection occurs and false if not

\importlib rwccore
*/
RW_COLLISION_FORCE_INLINE RwpBool
TriangleLineSegIntersect(rwpmath::Vector3::InParam v0,
                         rwpmath::Vector3::InParam v1,
                         rwpmath::Vector3::InParam v2,
                         rwpmath::Vector3::InParam lineStart,
                         rwpmath::Vector3::InParam lineDelta,

                         rwpmath::Vector3  &position,
                         rwpmath::VecFloat &lineParam,
                         rwpmath::Vector3  &triParam
                         )
{
    // intersection results
    rwpmath::VecFloat det, u, v, t;

    // Find vectors for the two edges sharing v0
    rwpmath::Vector3 edge1 = v1 - v0;
    rwpmath::Vector3 edge2 = v2 - v0;

    // test for intersection - returns unnormalized parameters
    rwpmath::MaskScalar intersects = TriangleLineSegIntersect(v0, edge1, edge2, lineStart, lineDelta, det, u, v, t);

    rwpmath::VecFloat invDet = rwpmath::Reciprocal(det);
    // position in line parameter space
    lineParam = t * invDet;

    // position in world space
    position = lineStart + lineDelta * lineParam;

    // position in barycentric coordinates
    triParam = rwpmath::Vector3(u * invDet, v * invDet, rwpmath::GetVecFloat_Zero());

    return(static_cast<RwpBool>(intersects.GetBool()));
}


/**
\internal
\brief A normalized intersection test between 1 triangle and 1 line

This returns the normalized position, line param and triangle params if an intersection occours.  This function is slightly
slower than the unnormalized version, so it should be reserved for when you usually get hits and always need the position

\param v0            The first vertex of the triangle to test
\param v1            The seconf vertex of the triangle to test
\param v2            The third vertex of the triangle to test
\param lineStart    The position of the start of the line
\param lineDelta    The vector from the start of the line to the end of the line
\param position        The position in world space of the intersection
\param lineParam    The parametric distance along the line where the intersection occurs
\param triParam        The barycentric parameter showing where on the triangle the line hits

\return                A mask which is true if an intersection occurs and false if not

\importlib rwccore
*/

RW_COLLISION_FORCE_INLINE RwpBool
TriangleLineSegIntersect_Branching(rwpmath::Vector3::InParam v0,
                                  rwpmath::Vector3::InParam v1,
                                  rwpmath::Vector3::InParam v2,

                                  rwpmath::Vector3::InParam lineStart,
                                  rwpmath::Vector3::InParam lineDelta,

                                  rwpmath::Vector3  &position,
                                  rwpmath::VecFloat &lineParam,
                                  rwpmath::Vector3  &triParam
                                  )
{
    rwpmath::Vector3  edge1, edge2, tVec, pVec, qVec;
    rwpmath::VecFloat det, lo, hi, u, v;

    // Find vectors for two edges sharing vert0
    edge1 = v1 - v0;
    edge2 = v2 - v0;

    // Begin calculating determinant - also used to calculate u parameter
    pVec = rwpmath::Cross( lineDelta, edge2 );

    // If determinant is near zero, ray lies in plane of triangle, if negative, triangle is backfacing
    det = rwpmath::Dot( edge1, pVec );

    RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEPSILON_VecFloat, RTINTSECEPSILON);
    rwpmath::MaskScalar detValid = rwpmath::CompGreaterThan(det, RTINTSECEPSILON_VecFloat);

    if ( detValid.GetBool() )
    {
        // Calculate bounds for parameters with tolerance
        RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEDGEEPS_VecFloat, RTINTSECEDGEEPS);
        lo = - det * RTINTSECEDGEEPS_VecFloat;
        hi = det - lo;

        // Calculate u parameter and test bounds
        tVec = lineStart - v0;
        u = Dot( tVec, pVec );

        rwpmath::MaskScalar uValid =
            rwpmath::And( rwpmath::CompGreaterEqual(u, lo), rwpmath::CompLessEqual(u, hi) );

        if ( uValid.GetBool() )
        {
            // Calculate v parameter and test bounds
            qVec = Cross( tVec, edge1 );
            v = Dot( lineDelta, qVec );

            rwpmath::MaskScalar vValid =
                rwpmath::And( rwpmath::CompGreaterEqual(v, lo), rwpmath::CompLessEqual(u + v, hi) );

            if( vValid.GetBool() )
            {
                // Calculate t, and make sure intersection is in bounds of line
                rwpmath::VecFloat t = Dot(edge2, qVec);

                rwpmath::MaskScalar tValid =
                    rwpmath::And( rwpmath::CompGreaterEqual(t, lo), rwpmath::CompLessEqual(t, hi) );

                // Within bounds of line?
                if ( tValid.GetBool() )
                {
                    rwpmath::VecFloat invDet = rwpmath::Reciprocal(det);

                    // position in line parameter space
                    lineParam = t * invDet;

                    // position in world space
                    position = lineStart + lineDelta * lineParam;

                    // position in barycentric coordinates
                    triParam = rwpmath::Vector3( u * invDet, v * invDet, rwpmath::GetVecFloat_Zero() );

                    return TRUE;
                }
            }
        }
    }
    return FALSE;
}


/**
\internal
\brief An un-normalized intersection test between 16 triangles and 1 line

This returns the un normalized position, line param and triangle params if an intersection occours.  To normalize
these values, divide them by 1/Det.  This function is mainly designed for use if you only want to know *if* the line
intersects, not where.  Arguments are supplied in an SoA format, which the caller is responsible for formatting.

\param v0x0            The X componants of the first vertices of the 4 input triangles in the first batch
\param v0y0            The Y componants of the first vertices of the 4 input triangles in the first batch
\param v0z0            The Z componants of the first vertices of the 4 input triangles in the first batch

\param v0x1            The X componants of the first vertices of the 4 input triangles in the second batch
\param v0y1            The Y componants of the first vertices of the 4 input triangles in the second batch
\param v0z1            The Z componants of the first vertices of the 4 input triangles in the second batch

\param v0x2            The X componants of the first vertices of the 4 input triangles in the third batch
\param v0y2            The Y componants of the first vertices of the 4 input triangles in the third batch
\param v0z2            The Z componants of the first vertices of the 4 input triangles in the third batch

\param v0x3            The X componants of the first vertices of the 4 input triangles in the fourth batch
\param v0y3            The Y componants of the first vertices of the 4 input triangles in the fourth batch
\param v0z3            The Z componants of the first vertices of the 4 input triangles in the fourth batch

\param edge1x0        The X componants of the vector from v0 to the first vertex of the triangles in the first batch
\param edge1y0        The Y componants of the vector from v0 to the first vertex of the triangles in the first batch
\param edge1z0        The Z componants of the vector from v0 to the first vertex of the triangles in the first batch

\param edge1x1        The X componants of the vector from v0 to the first vertex of the triangles in the second batch
\param edge1y1        The Y componants of the vector from v0 to the first vertex of the triangles in the second batch
\param edge1z1        The Z componants of the vector from v0 to the first vertex of the triangles in the second batch

\param edge1x2        The X componants of the vector from v0 to the first vertex of the triangles in the third batch
\param edge1y2        The Y componants of the vector from v0 to the first vertex of the triangles in the third batch
\param edge1z2        The Z componants of the vector from v0 to the first vertex of the triangles in the third batch

\param edge1x3        The X componants of the vector from v0 to the first vertex of the triangles in the fourth batch
\param edge1y3        The Y componants of the vector from v0 to the first vertex of the triangles in the fourth batch
\param edge1z3        The Z componants of the vector from v0 to the first vertex of the triangles in the fourth batch

\param edge2x0        The X componants of the vector from v0 to the second vertex of the triangles in the first batch
\param edge2y0        The Y componants of the vector from v0 to the second vertex of the triangles in the first batch
\param edge2z0        The Z componants of the vector from v0 to the second vertex of the triangles in the first batch

\param edge2x1        The X componants of the vector from v0 to the second vertex of the triangles in the second batch
\param edge2y1        The Y componants of the vector from v0 to the second vertex of the triangles in the second batch
\param edge2z1        The Z componants of the vector from v0 to the second vertex of the triangles in the second batch

\param edge2x2        The X componants of the vector from v0 to the second vertex of the triangles in the third batch
\param edge2y2        The Y componants of the vector from v0 to the second vertex of the triangles in the third batch
\param edge2z2        The Z componants of the vector from v0 to the second vertex of the triangles in the third batch

\param edge2x3        The X componants of the vector from v0 to the second vertex of the triangles in the fourth batch
\param edge2y3        The Y componants of the vector from v0 to the second vertex of the triangles in the fourth batch
\param edge2z3        The Z componants of the vector from v0 to the second vertex of the triangles in the fourth batch


\param lineStart    The position of the start of the line
\param lineDelta    The vector from the start of the line to the end of the line

\param det0            This gets filled in with the value that can be used to normalize the parameters of the first batch
\param det1            This gets filled in with the value that can be used to normalize the parameters of the second batch
\param det2            This gets filled in with the value that can be used to normalize the parameters of the third batch
\param det3            This gets filled in with the value that can be used to normalize the parameters of the fourth batch

\param tri1Params0    The first barycentric triangle params for the first batch
\param tri1Params1    The first barycentric triangle params for the second batch
\param tri1Params2    The first barycentric triangle params for the third batch
\param tri1Params3    The first barycentric triangle params for the fourth batch

\param tri1Params0    The second barycentric triangle params for the first batch
\param tri1Params1    The second barycentric triangle params for the second batch
\param tri1Params2    The second barycentric triangle params for the third batch
\param tri1Params3    The second barycentric triangle params for the fourth batch

\param lineParams0    The parametric distance along the line where the intersection occurs for the first batch
\param lineParams1    The parametric distance along the line where the intersection occurs for the second batch
\param lineParams2    The parametric distance along the line where the intersection occurs for the third batch
\param lineParams3    The parametric distance along the line where the intersection occurs for the fourth batch

\resultsValid0        A mask which is true if an intersection occurs and false if not for each of the triangles in the first batch
\resultsValid1        A mask which is true if an intersection occurs and false if not for each of the triangles in the second batch
\resultsValid2        A mask which is true if an intersection occurs and false if not for each of the triangles in the third batch
\resultsValid3        A mask which is true if an intersection occurs and false if not for each of the triangles in the fourth batch

\importlib rwccore
*/
RW_COLLISION_FORCE_INLINE void
TriangleLineSegIntersect(
    rwpmath::Vector4::InParam v0x0, rwpmath::Vector4::InParam v0y0, rwpmath::Vector4::InParam v0z0,
    rwpmath::Vector4::InParam v0x1, rwpmath::Vector4::InParam v0y1, rwpmath::Vector4::InParam v0z1,
    rwpmath::Vector4::InParam v0x2, rwpmath::Vector4::InParam v0y2, rwpmath::Vector4::InParam v0z2,
    rwpmath::Vector4::InParam v0x3, rwpmath::Vector4::InParam v0y3, rwpmath::Vector4::InParam v0z3,

    rwpmath::Vector4::InParam edge1x0, rwpmath::Vector4::InParam edge1y0, rwpmath::Vector4::InParam edge1z0,
    rwpmath::Vector4::InParam edge1x1, rwpmath::Vector4::InParam edge1y1, rwpmath::Vector4::InParam edge1z1,
    rwpmath::Vector4::InParam edge1x2, rwpmath::Vector4::InParam edge1y2, rwpmath::Vector4::InParam edge1z2,
    rwpmath::Vector4::InParam edge1x3, rwpmath::Vector4::InParam edge1y3, rwpmath::Vector4::InParam edge1z3,

    rwpmath::Vector4::InParam edge2x0, rwpmath::Vector4::InParam edge2y0, rwpmath::Vector4::InParam edge2z0,
    rwpmath::Vector4::InParam edge2x1, rwpmath::Vector4::InParam edge2y1, rwpmath::Vector4::InParam edge2z1,
    rwpmath::Vector4::InParam edge2x2, rwpmath::Vector4::InParam edge2y2, rwpmath::Vector4::InParam edge2z2,
    rwpmath::Vector4::InParam edge2x3, rwpmath::Vector4::InParam edge2y3, rwpmath::Vector4::InParam edge2z3,

    rwpmath::Vector3::InParam lineStart,
    rwpmath::Vector3::InParam lineDelta,

    rwpmath::Vector4 &det0, rwpmath::Vector4 &det1, rwpmath::Vector4 &det2, rwpmath::Vector4 &det3,
    rwpmath::Vector4 &tri1Params0,rwpmath::Vector4 &tri1Params1,rwpmath::Vector4 &tri1Params2,rwpmath::Vector4 &tri1Params3,
    rwpmath::Vector4 &tri2Params0,rwpmath::Vector4 &tri2Params1,rwpmath::Vector4 &tri2Params2,rwpmath::Vector4 &tri2Params3,

    rwpmath::Vector4 &lineParams0, rwpmath::Vector4 &lineParams1, rwpmath::Vector4 &lineParams2, rwpmath::Vector4 &lineParams3,
    rwpmath::Mask4 &resultsValid0, rwpmath::Mask4 &resultsValid1, rwpmath::Mask4 &resultsValid2, rwpmath::Mask4 &resultsValid3
    )
{
    rwpmath::Vector4 pVecX0, pVecX1, pVecX2, pVecX3, pVecY0, pVecY1, pVecY2, pVecY3, pVecZ0, pVecZ1, pVecZ2, pVecZ3;

    CrossSoA(lineDelta, edge2x0, edge2y0, edge2z0, pVecX0, pVecY0, pVecZ0);
    CrossSoA(lineDelta, edge2x1, edge2y1, edge2z1, pVecX1, pVecY1, pVecZ1);
    CrossSoA(lineDelta, edge2x2, edge2y2, edge2z2, pVecX2, pVecY2, pVecZ2);
    CrossSoA(lineDelta, edge2x3, edge2y3, edge2z3, pVecX3, pVecY3, pVecZ3);

    det0 = DotSoA(edge1x0, edge1y0, edge1z0, pVecX0, pVecY0, pVecZ0);
    det1 = DotSoA(edge1x1, edge1y1, edge1z1, pVecX1, pVecY1, pVecZ1);
    det2 = DotSoA(edge1x2, edge1y2, edge1z2, pVecX2, pVecY2, pVecZ2);
    det3 = DotSoA(edge1x3, edge1y3, edge1z3, pVecX3, pVecY3, pVecZ3);

    RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEPSILON_VecFloat, RTINTSECEPSILON);
    rwpmath::Vector4 rtIntDecEpsilonV4(RTINTSECEPSILON_VecFloat,
        RTINTSECEPSILON_VecFloat,
        RTINTSECEPSILON_VecFloat,
        RTINTSECEPSILON_VecFloat);

    // Calculate u parameter and test bounds
    rwpmath::Vector4 tVecX0, tVecX1, tVecX2, tVecX3, tVecY0, tVecY1, tVecY2, tVecY3, tVecZ0, tVecZ1, tVecZ2, tVecZ3;

    tVecX0 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetX())) - v0x0;
    tVecY0 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetY())) - v0y0;
    tVecZ0 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetZ())) - v0z0;

    tVecX1 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetX())) - v0x1;
    tVecY1 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetY())) - v0y1;
    tVecZ1 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetZ())) - v0z1;

    tVecX2 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetX())) - v0x2;
    tVecY2 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetY())) - v0y2;
    tVecZ2 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetZ())) - v0z2;

    tVecX3 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetX())) - v0x3;
    tVecY3 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetY())) - v0y3;
    tVecZ3 = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetZ())) - v0z3;

    // Calculate bounds for parameters with tolerance
    RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEDGEEPS_VecFloat, RTINTSECEDGEEPS);
    rwpmath::Vector4 lo0, lo1, lo2, lo3, hi0, hi1, hi2, hi3;
    lo0 = - det0 * RTINTSECEDGEEPS_VecFloat;
    rwpmath::Mask4 detValid0 = CompGreaterThan(det0, rtIntDecEpsilonV4);
    lo1 = - det1 * RTINTSECEDGEEPS_VecFloat;
    rwpmath::Mask4 detValid1 = CompGreaterThan(det1, rtIntDecEpsilonV4);
    lo2 = - det2 * RTINTSECEDGEEPS_VecFloat;
    rwpmath::Mask4 detValid2 = CompGreaterThan(det2, rtIntDecEpsilonV4);
    lo3 = - det3 * RTINTSECEDGEEPS_VecFloat;
    rwpmath::Mask4 detValid3 = CompGreaterThan(det3, rtIntDecEpsilonV4);


    hi0 = det0 - lo0;
    hi1 = det1 - lo1;
    hi2 = det2 - lo2;
    hi3 = det3 - lo3;

    rwpmath::Vector4 u0, u1, u2, u3;
    u0 = DotSoA(tVecX0, tVecY0, tVecZ0, pVecX0, pVecY0, pVecZ0);
    u1 = DotSoA(tVecX1, tVecY1, tVecZ1, pVecX1, pVecY1, pVecZ1);
    u2 = DotSoA(tVecX2, tVecY2, tVecZ2, pVecX2, pVecY2, pVecZ2);
    u3 = DotSoA(tVecX3, tVecY3, tVecZ3, pVecX3, pVecY3, pVecZ3);
    rwpmath::Mask4 uValid0 = And( CompGreaterEqual(u0, lo0), CompLessEqual(u0, hi0) );
    rwpmath::Mask4 uValid1 = And( CompGreaterEqual(u1, lo1), CompLessEqual(u1, hi1) );
    rwpmath::Mask4 uValid2 = And( CompGreaterEqual(u2, lo2), CompLessEqual(u2, hi2) );
    rwpmath::Mask4 uValid3 = And( CompGreaterEqual(u3, lo3), CompLessEqual(u3, hi3) );


    rwpmath::Vector4 qVecX0, qVecX1, qVecX2, qVecX3, qVecY0, qVecY1, qVecY2, qVecY3, qVecZ0, qVecZ1, qVecZ2, qVecZ3;

    CrossSoA(tVecX0, tVecY0, tVecZ0, edge1x0, edge1y0, edge1z0, qVecX0, qVecY0, qVecZ0);
    CrossSoA(tVecX1, tVecY1, tVecZ1, edge1x1, edge1y1, edge1z1, qVecX1, qVecY1, qVecZ1);
    CrossSoA(tVecX2, tVecY2, tVecZ2, edge1x2, edge1y2, edge1z2, qVecX2, qVecY2, qVecZ2);
    CrossSoA(tVecX3, tVecY3, tVecZ3, edge1x3, edge1y3, edge1z3, qVecX3, qVecY3, qVecZ3);

    tri1Params0 = u0;
    tri1Params1 = u1;
    tri1Params2 = u2;
    tri1Params3 = u3;

    rwpmath::Vector4 v0, v1, v2, v3;
    v0 = DotSoA(lineDelta, qVecX0, qVecY0, qVecZ0);
    v1 = DotSoA(lineDelta, qVecX1, qVecY1, qVecZ1);
    v2 = DotSoA(lineDelta, qVecX2, qVecY2, qVecZ2);
    v3 = DotSoA(lineDelta, qVecX3, qVecY3, qVecZ3);

    rwpmath::Vector4 t0, t1, t2, t3;
    t0 = DotSoA(edge2x0, edge2y0, edge2z0, qVecX0, qVecY0, qVecZ0);
    t1 = DotSoA(edge2x1, edge2y1, edge2z1, qVecX1, qVecY1, qVecZ1);
    t2 = DotSoA(edge2x2, edge2y2, edge2z2, qVecX2, qVecY2, qVecZ2);
    t3 = DotSoA(edge2x3, edge2y3, edge2z3, qVecX3, qVecY3, qVecZ3);


    rwpmath::Mask4 vValid0 = And( CompGreaterEqual(v0, lo0), CompLessEqual(u0 + v0, hi0) );
    rwpmath::Mask4 vValid1 = And( CompGreaterEqual(v1, lo1), CompLessEqual(u1 + v1, hi1) );
    rwpmath::Mask4 vValid2 = And( CompGreaterEqual(v2, lo2), CompLessEqual(u2 + v2, hi2) );
    rwpmath::Mask4 vValid3 = And( CompGreaterEqual(v3, lo3), CompLessEqual(u3 + v3, hi3) );

    tri2Params0 = v0;
    tri2Params1 = v1;
    tri2Params2 = v2;
    tri2Params3 = v3;

    lineParams0 = t0;
    lineParams1 = t1;
    lineParams2 = t2;
    lineParams3 = t3;

    rwpmath::Mask4 tValid0 = And( CompGreaterEqual(t0, lo0), CompLessEqual(t0, hi0) );
    rwpmath::Mask4 tValid1 = And( CompGreaterEqual(t1, lo1), CompLessEqual(t1, hi1) );
    rwpmath::Mask4 tValid2 = And( CompGreaterEqual(t2, lo2), CompLessEqual(t2, hi2) );
    rwpmath::Mask4 tValid3 = And( CompGreaterEqual(t3, lo3), CompLessEqual(t3, hi3) );

    resultsValid0 = And( And(uValid0, vValid0), And(detValid0, tValid0) );
    resultsValid1 = And( And(uValid1, vValid1), And(detValid1, tValid1) );
    resultsValid2 = And( And(uValid2, vValid2), And(detValid2, tValid2) );
    resultsValid3 = And( And(uValid3, vValid3), And(detValid3, tValid3) );
}


/**
\internal
\brief A normalized intersection test between 16 triangles and 1 line

This returns the normalized position, line param and triangle params if an intersection occours.  This function is slightly
slower than the unnormalized version, so it should be reserved for when you usually get hits and always need the position

\param t0        Triangle query number 1
\param t1        Triangle query number 2
\param t2        Triangle query number 3
\param t3        Triangle query number 4
\param t4        Triangle query number 5
\param t5        Triangle query number 6
\param t6        Triangle query number 7
\param t7        Triangle query number 8
\param t8        Triangle query number 9
\param t9        Triangle query number 10
\param t10        Triangle query number 11
\param t11        Triangle query number 12
\param t12        Triangle query number 13
\param t13        Triangle query number 14
\param t14        Triangle query number 15
\param t15        Triangle query number 16

\return                A mask which is true if an intersection occurs and false if not for each of the triangles

\importlib rwccore
*/
RW_COLLISION_FORCE_INLINE void
TriangleLineSegIntersect(TriangleQuery & t0,
                         TriangleQuery & t1,
                         TriangleQuery & t2,
                         TriangleQuery & t3,
                         TriangleQuery & t4,
                         TriangleQuery & t5,
                         TriangleQuery & t6,
                         TriangleQuery & t7,
                         TriangleQuery & t8,
                         TriangleQuery & t9,
                         TriangleQuery & t10,
                         TriangleQuery & t11,
                         TriangleQuery & t12,
                         TriangleQuery & t13,
                         TriangleQuery & t14,
                         TriangleQuery & t15,

                         rwpmath::Vector3::InParam lineStart,
                         rwpmath::Vector3::InParam lineDelta
                        )
{
    using namespace rw::physics::mathutils;
    //Transpose4to3(v0A, v0B, v0C, v0D, v0x, v0y, v0z);
    rwpmath::Vector4 v0x0 = rwpmath::Vector4(t0.v0.GetX(),  t1.v0.GetX(),  t2.v0.GetX(),  t3.v0.GetX());
    rwpmath::Vector4 v0x1 = rwpmath::Vector4(t4.v0.GetX(),  t5.v0.GetX(),  t6.v0.GetX(),  t7.v0.GetX());
    rwpmath::Vector4 v0x2 = rwpmath::Vector4(t8.v0.GetX(),  t9.v0.GetX(),  t10.v0.GetX(), t11.v0.GetX());
    rwpmath::Vector4 v0x3 = rwpmath::Vector4(t12.v0.GetX(), t13.v0.GetX(), t14.v0.GetX(), t15.v0.GetX());

    rwpmath::Vector4 v0y0 = rwpmath::Vector4(t0.v0.GetY(),  t1.v0.GetY(),  t2.v0.GetY(),  t3.v0.GetY());
    rwpmath::Vector4 v0y1 = rwpmath::Vector4(t4.v0.GetY(),  t5.v0.GetY(),  t6.v0.GetY(),  t7.v0.GetY());
    rwpmath::Vector4 v0y2 = rwpmath::Vector4(t8.v0.GetY(),  t9.v0.GetY(),  t10.v0.GetY(), t11.v0.GetY());
    rwpmath::Vector4 v0y3 = rwpmath::Vector4(t12.v0.GetY(), t13.v0.GetY(), t14.v0.GetY(), t15.v0.GetY());

    rwpmath::Vector4 v0z0 = rwpmath::Vector4(t0.v0.GetZ(),  t1.v0.GetZ(),  t2.v0.GetZ(),  t3.v0.GetZ());
    rwpmath::Vector4 v0z1 = rwpmath::Vector4(t4.v0.GetZ(),  t5.v0.GetZ(),  t6.v0.GetZ(),  t7.v0.GetZ());
    rwpmath::Vector4 v0z2 = rwpmath::Vector4(t8.v0.GetZ(),  t9.v0.GetZ(),  t10.v0.GetZ(), t11.v0.GetZ());
    rwpmath::Vector4 v0z3 = rwpmath::Vector4(t12.v0.GetZ(), t13.v0.GetZ(), t14.v0.GetZ(), t15.v0.GetZ());

    // Find vectors for the two edges sharing v0
    rwpmath::Vector3 edge1A = t0.v1 - t0.v0;
    rwpmath::Vector3 edge1B = t1.v1 - t1.v0;
    rwpmath::Vector3 edge1C = t2.v1 - t2.v0;
    rwpmath::Vector3 edge1D = t3.v1 - t3.v0;
    rwpmath::Vector3 edge1E = t4.v1 - t4.v0;
    rwpmath::Vector3 edge1F = t5.v1 - t5.v0;
    rwpmath::Vector3 edge1G = t6.v1 - t6.v0;
    rwpmath::Vector3 edge1H = t7.v1 - t7.v0;
    rwpmath::Vector3 edge1I = t8.v1 - t8.v0;
    rwpmath::Vector3 edge1J = t9.v1 - t9.v0;
    rwpmath::Vector3 edge1K = t10.v1 - t10.v0;
    rwpmath::Vector3 edge1L = t11.v1 - t11.v0;
    rwpmath::Vector3 edge1M = t12.v1 - t12.v0;
    rwpmath::Vector3 edge1N = t13.v1 - t13.v0;
    rwpmath::Vector3 edge1O = t14.v1 - t14.v0;
    rwpmath::Vector3 edge1P = t15.v1 - t15.v0;

    rwpmath::Matrix44 mtx0 = rwpmath::Matrix44(ToVector4(edge1A), ToVector4(edge1B), ToVector4(edge1C), ToVector4(edge1D));
    rwpmath::Matrix44 mtx1 = rwpmath::Matrix44(ToVector4(edge1E), ToVector4(edge1F), ToVector4(edge1G), ToVector4(edge1H));
    rwpmath::Matrix44 mtx2 = rwpmath::Matrix44(ToVector4(edge1I), ToVector4(edge1J), ToVector4(edge1K), ToVector4(edge1L));
    rwpmath::Matrix44 mtx3 = rwpmath::Matrix44(ToVector4(edge1M), ToVector4(edge1N), ToVector4(edge1O), ToVector4(edge1P));
    mtx0 = rwpmath::Transpose(mtx0);
    mtx1 = rwpmath::Transpose(mtx1);
    mtx2 = rwpmath::Transpose(mtx2);
    mtx3 = rwpmath::Transpose(mtx3);

    rwpmath::Vector4 edge1x0(mtx0.GetX());
    rwpmath::Vector4 edge1x1(mtx1.GetX());
    rwpmath::Vector4 edge1x2(mtx2.GetX());
    rwpmath::Vector4 edge1x3(mtx3.GetX());

    rwpmath::Vector4 edge1y0(mtx0.GetY());
    rwpmath::Vector4 edge1y1(mtx1.GetY());
    rwpmath::Vector4 edge1y2(mtx2.GetY());
    rwpmath::Vector4 edge1y3(mtx3.GetY());

    rwpmath::Vector4 edge1z0(mtx0.GetZ());
    rwpmath::Vector4 edge1z1(mtx1.GetZ());
    rwpmath::Vector4 edge1z2(mtx2.GetZ());
    rwpmath::Vector4 edge1z3(mtx3.GetZ());

    rwpmath::Vector3 edge2A = t0.v2 - t0.v0;
    rwpmath::Vector3 edge2B = t1.v2 - t1.v0;
    rwpmath::Vector3 edge2C = t2.v2 - t2.v0;
    rwpmath::Vector3 edge2D = t3.v2 - t3.v0;
    rwpmath::Vector3 edge2E = t4.v2 - t4.v0;
    rwpmath::Vector3 edge2F = t5.v2 - t5.v0;
    rwpmath::Vector3 edge2G = t6.v2 - t6.v0;
    rwpmath::Vector3 edge2H = t7.v2 - t7.v0;
    rwpmath::Vector3 edge2I = t8.v2 - t8.v0;
    rwpmath::Vector3 edge2J = t9.v2 - t9.v0;
    rwpmath::Vector3 edge2K = t10.v2 - t10.v0;
    rwpmath::Vector3 edge2L = t11.v2 - t11.v0;
    rwpmath::Vector3 edge2M = t12.v2 - t12.v0;
    rwpmath::Vector3 edge2N = t13.v2 - t13.v0;
    rwpmath::Vector3 edge2O = t14.v2 - t14.v0;
    rwpmath::Vector3 edge2P = t15.v2 - t15.v0;

    rwpmath::Vector4 edge2x0 = rwpmath::Vector4(edge2A.GetX(), edge2B.GetX(), edge2C.GetX(), edge2D.GetX());
    rwpmath::Vector4 edge2x1 = rwpmath::Vector4(edge2E.GetX(), edge2F.GetX(), edge2G.GetX(), edge2H.GetX());
    rwpmath::Vector4 edge2x2 = rwpmath::Vector4(edge2I.GetX(), edge2J.GetX(), edge2K.GetX(), edge2L.GetX());
    rwpmath::Vector4 edge2x3 = rwpmath::Vector4(edge2M.GetX(), edge2N.GetX(), edge2O.GetX(), edge2P.GetX());

    rwpmath::Vector4 edge2y0 = rwpmath::Vector4(edge2A.GetY(), edge2B.GetY(), edge2C.GetY(), edge2D.GetY());
    rwpmath::Vector4 edge2y1 = rwpmath::Vector4(edge2E.GetY(), edge2F.GetY(), edge2G.GetY(), edge2H.GetY());
    rwpmath::Vector4 edge2y2 = rwpmath::Vector4(edge2I.GetY(), edge2J.GetY(), edge2K.GetY(), edge2L.GetY());
    rwpmath::Vector4 edge2y3 = rwpmath::Vector4(edge2M.GetY(), edge2N.GetY(), edge2O.GetY(), edge2P.GetY());

    rwpmath::Vector4 edge2z0 = rwpmath::Vector4(edge2A.GetZ(), edge2B.GetZ(), edge2C.GetZ(), edge2D.GetZ());
    rwpmath::Vector4 edge2z1 = rwpmath::Vector4(edge2E.GetZ(), edge2F.GetZ(), edge2G.GetZ(), edge2H.GetZ());
    rwpmath::Vector4 edge2z2 = rwpmath::Vector4(edge2I.GetZ(), edge2J.GetZ(), edge2K.GetZ(), edge2L.GetZ());
    rwpmath::Vector4 edge2z3 = rwpmath::Vector4(edge2M.GetZ(), edge2N.GetZ(), edge2O.GetZ(), edge2P.GetZ());

    rwpmath::Vector4 det0, det1, det2, det3;
    rwpmath::Vector4 tri1Params0, tri1Params1, tri1Params2, tri1Params3;
    rwpmath::Vector4 tri2Params0, tri2Params1, tri2Params2, tri2Params3;
    rwpmath::Vector4 lineParams0, lineParams1, lineParams2, lineParams3;
    rwpmath::Mask4 resultsValid0, resultsValid1, resultsValid2, resultsValid3;

    // test for intersection - returns unnormalized parameters
    TriangleLineSegIntersect(v0x0, v0y0, v0z0,
                             v0x1, v0y1, v0z1,
                             v0x2, v0y2, v0z2,
                             v0x3, v0y3, v0z3,

                             edge1x0, edge1y0, edge1z0,
                             edge1x1, edge1y1, edge1z1,
                             edge1x2, edge1y2, edge1z2,
                             edge1x3, edge1y3, edge1z3,

                             edge2x0, edge2y0, edge2z0,
                             edge2x1, edge2y1, edge2z1,
                             edge2x2, edge2y2, edge2z2,
                             edge2x3, edge2y3, edge2z3,

                             lineStart, lineDelta,
                             det0, det1, det2, det3,
                             tri1Params0, tri1Params1, tri1Params2, tri1Params3,
                             tri2Params0, tri2Params1, tri2Params2, tri2Params3,
                             lineParams0, lineParams1, lineParams2, lineParams3,
                             resultsValid0, resultsValid1, resultsValid2, resultsValid3);

        rwpmath::Vector4 invDet0, invDet1, invDet2, invDet3;
        invDet0 = rwpmath::GetVector4_One() / det0;
        invDet1 = rwpmath::GetVector4_One() / det1;
        invDet2 = rwpmath::GetVector4_One() / det2;
        invDet3 = rwpmath::GetVector4_One() / det3;

        t0.intersects  = resultsValid0.GetX();
        t1.intersects  = resultsValid0.GetY();
        t2.intersects  = resultsValid0.GetZ();
        t3.intersects  = resultsValid0.GetW();
        t4.intersects  = resultsValid1.GetX();
        t5.intersects  = resultsValid1.GetY();
        t6.intersects  = resultsValid1.GetZ();
        t7.intersects  = resultsValid1.GetW();
        t8.intersects  = resultsValid2.GetX();
        t9.intersects  = resultsValid2.GetY();
        t10.intersects = resultsValid2.GetZ();
        t11.intersects = resultsValid2.GetW();
        t12.intersects = resultsValid3.GetX();
        t13.intersects = resultsValid3.GetY();
        t14.intersects = resultsValid3.GetZ();
        t15.intersects = resultsValid3.GetW();

        tri1Params0 *= invDet0;
        tri1Params1 *= invDet1;
        tri1Params2 *= invDet2;
        tri1Params3 *= invDet3;

        tri2Params0 *= invDet0;
        tri2Params1 *= invDet1;
        tri2Params2 *= invDet2;
        tri2Params3 *= invDet3;

        lineParams0 *= invDet0;
        lineParams1 *= invDet1;
        lineParams2 *= invDet2;
        lineParams3 *= invDet3;

        // position in barycentric coordinates
        t0.triParam   = rwpmath::Vector3(tri1Params0.GetX(), tri2Params0.GetX(), rwpmath::GetVecFloat_Zero());
        t1.triParam   = rwpmath::Vector3(tri1Params0.GetY(), tri2Params0.GetY(), rwpmath::GetVecFloat_Zero());
        t2.triParam   = rwpmath::Vector3(tri1Params0.GetZ(), tri2Params0.GetZ(), rwpmath::GetVecFloat_Zero());
        t3.triParam   = rwpmath::Vector3(tri1Params0.GetW(), tri2Params0.GetW(), rwpmath::GetVecFloat_Zero());
        t4.triParam   = rwpmath::Vector3(tri1Params1.GetX(), tri2Params1.GetX(), rwpmath::GetVecFloat_Zero());
        t5.triParam   = rwpmath::Vector3(tri1Params1.GetY(), tri2Params1.GetY(), rwpmath::GetVecFloat_Zero());
        t6.triParam   = rwpmath::Vector3(tri1Params1.GetZ(), tri2Params1.GetZ(), rwpmath::GetVecFloat_Zero());
        t7.triParam   = rwpmath::Vector3(tri1Params1.GetW(), tri2Params1.GetW(), rwpmath::GetVecFloat_Zero());
        t8.triParam   = rwpmath::Vector3(tri1Params2.GetX(), tri2Params2.GetX(), rwpmath::GetVecFloat_Zero());
        t9.triParam   = rwpmath::Vector3(tri1Params2.GetY(), tri2Params2.GetY(), rwpmath::GetVecFloat_Zero());
        t10.triParam  = rwpmath::Vector3(tri1Params2.GetZ(), tri2Params2.GetZ(), rwpmath::GetVecFloat_Zero());
        t11.triParam  = rwpmath::Vector3(tri1Params2.GetW(), tri2Params2.GetW(), rwpmath::GetVecFloat_Zero());
        t12.triParam  = rwpmath::Vector3(tri1Params3.GetX(), tri2Params3.GetX(), rwpmath::GetVecFloat_Zero());
        t13.triParam  = rwpmath::Vector3(tri1Params3.GetY(), tri2Params3.GetY(), rwpmath::GetVecFloat_Zero());
        t14.triParam  = rwpmath::Vector3(tri1Params3.GetZ(), tri2Params3.GetZ(), rwpmath::GetVecFloat_Zero());
        t15.triParam  = rwpmath::Vector3(tri1Params3.GetW(), tri2Params3.GetW(), rwpmath::GetVecFloat_Zero());

        t0.lineParam = lineParams0.GetX();
        t1.lineParam = lineParams0.GetY();
        t2.lineParam = lineParams0.GetZ();
        t3.lineParam = lineParams0.GetW();

        t4.lineParam = lineParams1.GetX();
        t5.lineParam = lineParams1.GetY();
        t6.lineParam = lineParams1.GetZ();
        t7.lineParam = lineParams1.GetW();

        t8.lineParam = lineParams2.GetX();
        t9.lineParam = lineParams2.GetY();
        t10.lineParam = lineParams2.GetZ();
        t11.lineParam = lineParams2.GetW();

        t12.lineParam = lineParams3.GetX();
        t13.lineParam = lineParams3.GetY();
        t14.lineParam = lineParams3.GetZ();
        t15.lineParam = lineParams3.GetW();

        t0.position  = rwpmath::Mult(lineDelta, lineParams0.GetX());
        t1.position  = rwpmath::Mult(lineDelta, lineParams0.GetY());
        t2.position  = rwpmath::Mult(lineDelta, lineParams0.GetZ());
        t3.position  = rwpmath::Mult(lineDelta, lineParams0.GetW());
        t4.position  = rwpmath::Mult(lineDelta, lineParams1.GetX());
        t5.position  = rwpmath::Mult(lineDelta, lineParams1.GetY());
        t6.position  = rwpmath::Mult(lineDelta, lineParams1.GetZ());
        t7.position  = rwpmath::Mult(lineDelta, lineParams1.GetW());
        t8.position  = rwpmath::Mult(lineDelta, lineParams2.GetX());
        t9.position  = rwpmath::Mult(lineDelta, lineParams2.GetY());
        t10.position = rwpmath::Mult(lineDelta, lineParams2.GetZ());
        t11.position = rwpmath::Mult(lineDelta, lineParams2.GetW());
        t12.position = rwpmath::Mult(lineDelta, lineParams3.GetX());
        t13.position = rwpmath::Mult(lineDelta, lineParams3.GetY());
        t14.position = rwpmath::Mult(lineDelta, lineParams3.GetZ());
        t15.position = rwpmath::Mult(lineDelta, lineParams3.GetW());

        t0.position  = rwpmath::Add(lineStart, t0.position);
        t1.position  = rwpmath::Add(lineStart, t1.position);
        t2.position  = rwpmath::Add(lineStart, t2.position);
        t3.position  = rwpmath::Add(lineStart, t3.position);
        t4.position  = rwpmath::Add(lineStart, t4.position);
        t5.position  = rwpmath::Add(lineStart, t5.position);
        t6.position  = rwpmath::Add(lineStart, t6.position);
        t7.position  = rwpmath::Add(lineStart, t7.position);
        t8.position  = rwpmath::Add(lineStart, t8.position);
        t9.position  = rwpmath::Add(lineStart, t9.position);
        t10.position = rwpmath::Add(lineStart, t10.position);
        t11.position = rwpmath::Add(lineStart, t11.position);
        t12.position = rwpmath::Add(lineStart, t12.position);
        t13.position = rwpmath::Add(lineStart, t13.position);
        t14.position = rwpmath::Add(lineStart, t14.position);
        t15.position = rwpmath::Add(lineStart, t15.position);
}


/**
\internal
\brief An un-normalized intersection test between 4 triangles and 1 line

This returns the un normalized position, line param and triangle params if an intersection occours.  To normalize
these values, divide them by 1/Det.  This function is mainly designed for use if you only want to know *if* the line
intersects, not where.  Arguments are supplied in an SoA format, which the caller is responsible for formatting.

\param v0x0            The X componants of the first vertices of the 4 input triangles
\param v0y0            The Y componants of the first vertices of the 4 input triangles
\param v0z0            The Z componants of the first vertices of the 4 input triangles
\param edge1x        The X componants of the vector from v0 to the first vertex of the triangles
\param edge1y        The Y componants of the vector from v0 to the first vertex of the triangles
\param edge1z        The Z componants of the vector from v0 to the first vertex of the triangles
\param edge2x        The X componants of the vector from v0 to the second vertex of the triangles
\param edge2y        The Y componants of the vector from v0 to the second vertex of the triangles
\param edge2z        The Z componants of the vector from v0 to the second vertex of the triangles
\param lineStart    The position of the start of the line
\param lineDelta    The vector from the start of the line to the end of the line
\param Det            This gets filled in with the value that can be used to normalize the parameters
\param W1            The first barycentric triangle params
\param W2            The second barycentric triangle params
\param Alpha        The parametric distance along the line where the intersection occurs

\return                A mask which is true if an intersection occurs and false if not for each of the triangles

\importlib rwccore
*/
RW_COLLISION_FORCE_INLINE rwpmath::Mask4
TriangleLineSegIntersect(rwpmath::Vector4::InParam v0x, rwpmath::Vector4::InParam v0y, rwpmath::Vector4::InParam v0z,
                         rwpmath::Vector4::InParam edge1x, rwpmath::Vector4::InParam edge1y, rwpmath::Vector4::InParam edge1z,
                         rwpmath::Vector4::InParam edge2x, rwpmath::Vector4::InParam edge2y, rwpmath::Vector4::InParam edge2z,
                         rwpmath::Vector3::InParam lineStart, rwpmath::Vector3::InParam lineDelta,
                         rwpmath::Vector4 &det,
                         rwpmath::Vector4 &W1,
                         rwpmath::Vector4 &W2,
                         rwpmath::Vector4 &lineParams
                         )
{
    rwpmath::Vector4 lo, hi, u, v;

    // Begin calculating determinant - also used to calculate u parameter
    rwpmath::Vector4 pVecX, pVecY, pVecZ;
    CrossSoA(lineDelta, edge2x, edge2y, edge2z, pVecX, pVecY, pVecZ);

    // If determinant is near zero, ray lies in plane of triangle, if negative, triangle is backfacing
    det = DotSoA(edge1x, edge1y, edge1z, pVecX, pVecY, pVecZ);

    RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEPSILON_VecFloat, RTINTSECEPSILON);
    rwpmath::Vector4 rtIntDecEpsilonV4(RTINTSECEPSILON_VecFloat,
        RTINTSECEPSILON_VecFloat,
        RTINTSECEPSILON_VecFloat,
        RTINTSECEPSILON_VecFloat);

    rwpmath::Mask4 detValid = CompGreaterThan(det, rtIntDecEpsilonV4);

    // Calculate bounds for parameters with tolerance
    RWPMATH_CONSTRUCT_CONSTANT_VECFLOAT(RTINTSECEDGEEPS_VecFloat, RTINTSECEDGEEPS);
    lo = - det * RTINTSECEDGEEPS_VecFloat;
    hi = det - lo;

    // Calculate u parameter and test bounds
    rwpmath::Vector4 tVecX, tVecY, tVecZ;

    tVecX = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetX())) - v0x;
    tVecY = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetY())) - v0y;
    tVecZ = rwpmath::Vector4(rwpmath::VecFloat(lineStart.GetZ())) - v0z;

    u = DotSoA(tVecX, tVecY, tVecZ, pVecX, pVecY, pVecZ);

    rwpmath::Mask4 uValid =
        And( CompGreaterEqual(u, lo), CompLessEqual(u, hi) );

    // Calculate v parameter and test bounds
    rwpmath::Vector4 qVecX, qVecY, qVecZ;
    CrossSoA(tVecX, tVecY, tVecZ, edge1x, edge1y, edge1z, qVecX, qVecY, qVecZ);

    v = DotSoA(lineDelta, qVecX, qVecY, qVecZ);

    rwpmath::Mask4 vValid =
        And( CompGreaterEqual(v, lo), CompLessEqual(u + v, hi) );

    // Calculate t, and make sure intersection is in bounds of line
    rwpmath::Vector4 t;

    t = DotSoA(edge2x, edge2y, edge2z, qVecX, qVecY, qVecZ);

    lineParams = t;
    W1    = u;
    W2    = v;

    rwpmath::Mask4 tValid = rwpmath::And( rwpmath::CompGreaterEqual(t, lo), rwpmath::CompLessEqual(t, hi) );
    rwpmath::Mask4 resultValid = rwpmath::And( rwpmath::And(uValid, vValid), rwpmath::And(detValid, tValid) );

    return  resultValid ;
}


/**
\internal
\brief A normalized intersection test between 4 triangles and 1 line

This returns the normalized position, line param and triangle params if an intersection occours.  This function is slightly
slower than the unnormalized version, so it should be reserved for when you usually get hits and always need the position

\param v0A            The first vertex of the first triangle to test
\param v0B            The first vertex of the second triangle to test
\param v0C            The first vertex of the third triangle to test
\param v0D            The first vertex of the first triangle to test

\param v1A            The second vertex of the first triangle to test
\param v1B            The second vertex of the second triangle to test
\param v1C            The second vertex of the third triangle to test
\param v1D            The second vertex of the first triangle to test

\param v2A            The third vertex of the first triangle to test
\param v2B            The third vertex of the second triangle to test
\param v2C            The third vertex of the third triangle to test
\param v2D            The third vertex of the first triangle to test

\param lineStart    The position of the start of the line for each of the triangles
\param lineDelta    The vector from the start of the line to the end of the line for each of the triangles
\param position        The position in world space of the intersection for each of the triangles
\param lineParam    The parametric distance along the line where the intersection occurs for each of the triangles
\param triParam        The barycentric parameter showing where on the triangle the line hits for each of the triangles

\return                A mask which is true if an intersection occurs and false if not for each of the triangles

\importlib rwccore
*/
RW_COLLISION_FORCE_INLINE rwpmath::Mask4
TriangleLineSegIntersect(rwpmath::Vector3::InParam v0A,
                             rwpmath::Vector3::InParam v1A,
                             rwpmath::Vector3::InParam v2A,
                             rwpmath::Vector3::InParam v0B,
                             rwpmath::Vector3::InParam v1B,
                             rwpmath::Vector3::InParam v2B,
                             rwpmath::Vector3::InParam v0C,
                             rwpmath::Vector3::InParam v1C,
                             rwpmath::Vector3::InParam v2C,
                             rwpmath::Vector3::InParam v0D,
                             rwpmath::Vector3::InParam v1D,
                             rwpmath::Vector3::InParam v2D,

                             rwpmath::Vector3::InParam lineStart,
                             rwpmath::Vector3::InParam lineDelta,

                             rwpmath::Vector3  &positionA,
                             rwpmath::Vector3  &positionB,
                             rwpmath::Vector3  &positionC,
                             rwpmath::Vector3  &positionD,
                             rwpmath::Vector4 &lineParams,
                             rwpmath::Vector3  &triParamA,
                             rwpmath::Vector3  &triParamB,
                             rwpmath::Vector3  &triParamC,
                             rwpmath::Vector3  &triParamD
                        )
{
    // intersection results
    rwpmath::Vector4 w1, w2;
    rwpmath::Vector4 v0x, v0y, v0z, edge1x, edge1y, edge1z, edge2x, edge2y, edge2z;

    v0x = rwpmath::Vector4(v0A.GetX(), v0B.GetX(), v0C.GetX(), v0D.GetX());
    v0y = rwpmath::Vector4(v0A.GetY(), v0B.GetY(), v0C.GetY(), v0D.GetY());
    v0z = rwpmath::Vector4(v0A.GetZ(), v0B.GetZ(), v0C.GetZ(), v0D.GetZ());

    // Find vectors for the two edges sharing v0
    rwpmath::Vector3 edge1A = v1A - v0A;
    rwpmath::Vector3 edge1B = v1B - v0B;
    rwpmath::Vector3 edge1C = v1C - v0C;
    rwpmath::Vector3 edge1D = v1D - v0D;

    edge1x = rwpmath::Vector4(edge1A.GetX(), edge1B.GetX(), edge1C.GetX(), edge1D.GetX());
    edge1y = rwpmath::Vector4(edge1A.GetY(), edge1B.GetY(), edge1C.GetY(), edge1D.GetY());
    edge1z = rwpmath::Vector4(edge1A.GetZ(), edge1B.GetZ(), edge1C.GetZ(), edge1D.GetZ());

    rwpmath::Vector3 edge2A = v2A - v0A;
    rwpmath::Vector3 edge2B = v2B - v0B;
    rwpmath::Vector3 edge2C = v2C - v0C;
    rwpmath::Vector3 edge2D = v2D - v0D;

    edge2x = rwpmath::Vector4(edge2A.GetX(), edge2B.GetX(), edge2C.GetX(), edge2D.GetX());
    edge2y = rwpmath::Vector4(edge2A.GetY(), edge2B.GetY(), edge2C.GetY(), edge2D.GetY());
    edge2z = rwpmath::Vector4(edge2A.GetZ(), edge2B.GetZ(), edge2C.GetZ(), edge2D.GetZ());


    rwpmath::Vector4 det, invDet;

    // test for intersection - returns unnormalized parameters
    rwpmath::Mask4 intersects = TriangleLineSegIntersect(v0x, v0y, v0z,
                                                          edge1x, edge1y, edge1z,
                                                          edge2x, edge2y, edge2z,
                                                          lineStart, lineDelta, det,
                                                          w1, w2, lineParams);


    invDet = rwpmath::GetVector4_One() / det;

    w1 *= invDet;
    w2 *= invDet;

    // position in barycentric coordinates
    triParamA = rwpmath::Vector3(w1.GetX(), w2.GetX(), rwpmath::GetVecFloat_Zero());
    triParamB = rwpmath::Vector3(w1.GetY(), w2.GetY(), rwpmath::GetVecFloat_Zero());
    triParamC = rwpmath::Vector3(w1.GetZ(), w2.GetZ(), rwpmath::GetVecFloat_Zero());
    triParamD = rwpmath::Vector3(w1.GetW(), w2.GetW(), rwpmath::GetVecFloat_Zero());


    // position in line parameter space
    lineParams = lineParams * invDet;

    // position in world space
    positionA = lineStart + lineDelta * lineParams.GetX();
    positionB = lineStart + lineDelta * lineParams.GetY();
    positionC = lineStart + lineDelta * lineParams.GetZ();
    positionD = lineStart + lineDelta * lineParams.GetW();

    return(intersects);
}
}
}

#endif //PUBLIC_RW_COLLISION_TRIANGLE_QUERY_H
