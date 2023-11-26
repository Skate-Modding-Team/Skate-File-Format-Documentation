// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwctrianglekdtreeprocedural.cpp

 Purpose: Procedural aggregate of triangles with KDTree spatial map.

 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/kdtree.h"
#include "rw/collision/aggregate.h"
#include "rw/collision/volumelinequery.h"
#include "rw/collision/volumebboxquery.h"
#include "rw/collision/triangle.h"

#include "rw/collision/procedural.h"
#include "rw/collision/trianglekdtreeprocedural.h"


using namespace rwpmath;

namespace rw
{
namespace collision
{


// *******************************************************************************************************
// Forward Declarations

// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts

#define rwcTRIANGLEKDTREEPROCEDURALALIGNMENT RWMATH_VECTOR3_ALIGNMENT

// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions

// ***********************************************************************************************************
// Structs + Unions + Classes

/**
\internal
*/
struct TriangleValidityCheckNodeData
{
    uint32_t parent;
    AABBox   bbox;
};

/**
\internal

\brief The initialization of the static member variable that holds the functions pointers.
*/
rw::collision::Procedural::VTable TriangleKDTreeProcedural::sm_vTable =
{
    RWCOBJECTTYPE_TRIANGLEKDTREEPROCEDURAL,
    (GetSizeFn)(&TriangleKDTreeProcedural::GetSizeThis),
    rwcTRIANGLEKDTREEPROCEDURALALIGNMENT,
    TRUE,
    (UpdateFn)(&TriangleKDTreeProcedural::UpdateThis),
    (LineIntersectionQueryFn)(&TriangleKDTreeProcedural::LineIntersectionQueryThis),
    (BBoxOverlapQueryFn)(&TriangleKDTreeProcedural::BBoxOverlapQueryThis),
    0, // rw::collision::Aggregate::GetNextVolumeFn
    0, // rw::collision::Aggregate::ClearAllProcessedFlags
    0  // rw::collision::Aggregate::ApplyUniformScale
};

// ***********************************************************************************************************
// Static Functions

/**
\brief
Constructor for an rw::collision::TriangleKDTreeProcedural. This should only be called from Initialize
or a derived class.

\param    numVerts     The number of vertices in this TriangleKDTreeProcedural.
\param    numTris      The number of triangles in this TriangleKDTreeProcedural.
\param    vTable       Pointer to the function table.
\param    classSize    Size in bytes of the actual class structure (may be derived from
                       TriangleKDTreeProcedural).
*/
TriangleKDTreeProcedural::TriangleKDTreeProcedural(uint32_t numVerts,
                               uint32_t numTris,
                               VTable *vTable,
                               uint32_t classSize)
                               :   Procedural(numTris, vTable),
                                   m_numVerts(numVerts)
{
    uintptr_t addr = (uintptr_t)(this) + classSize;

    //Set the pointer to the data for the vertices
    addr = EA::Physics::SizeAlign<uintptr_t>(addr, RWMATH_VECTOR3_ALIGNMENT);
    m_verts = (Vector3 *)addr;
    addr += numVerts*sizeof(Vector3);

    //set the ptr to the tri indices just after the vtx data
    m_tris = (Triangle *)addr;
    addr += numTris*sizeof(Triangle);
    m_flags = (uint32_t *)addr;
    addr += ((numTris+7)>>3)*sizeof(uint32_t);

    //set the ptr to the spatial map just after the tri indices
    addr = EA::Physics::SizeAlign<uintptr_t>(addr, rwcKDTREE_ALIGNMENT);
    m_map = (KDTree *)addr;

}


/**
\brief Initializes a TriangleKDTreeProcedural into a resource.

\param resource The EA::Physics::MemoryPtr the TriangleKDTreeProcedural is initialized into.
\param numVerts The number of vertices in this TriangleKDTreeProcedural.
\param numTris The number of triangles in this TriangleKDTreeProcedural.
\param numNodes The number of nodes in the indexed KDTree map.
\param bbox Reference to the axis-aligned bounding box.
\param vTable Pointer to the function table.
\param classSize Size in bytes of the actual class structure (may be derived from
TriangleKDTreeProcedural).

\return The new TriangleKDTreeProcedural.
*/
TriangleKDTreeProcedural *
TriangleKDTreeProcedural::Initialize(const EA::Physics::MemoryPtr& resource,
                                     uint32_t numVerts,
                                     uint32_t numTris,
                                     uint32_t numNodes,
                                     const AABBox  &bbox,
                                     VTable *vTable,
                                     uint32_t classSize)
{
    uint32_t i, n = (numTris+7)>>3;

    TriangleKDTreeProcedural *agg = new (resource.GetMemory())
        TriangleKDTreeProcedural(numVerts, numTris, vTable, classSize);
    KDTree::Initialize(EA::Physics::MemoryPtr(agg->m_map), numNodes, numTris, bbox);

    for (i = 0; i < n; ++i)
    {
        agg->m_flags[i] = 0xffffffff;
    }
    return agg;
}


/**
\brief
Releases a block of memory that was being used for a TriangleKDTreeProcedural.
\param pKDT The TriangleKDTreeProcedural to release.
*/
void
TriangleKDTreeProcedural::Release(TriangleKDTreeProcedural *pKDT)
{
    pKDT->m_map->Release();

}

/**
\brief
Releases a block of memory that was being used for a TriangleKDTreeProcedural.
*/
void
TriangleKDTreeProcedural::Release()
{
    m_map->Release();
}

/**
\internal
 */
void
TriangleKDTreeProcedural::UpdateThis(void)
{
    //Update overall BBox
    m_AABB = m_map->GetBBox();

}

// Disable "assignment within conditional" warning, as it is used intentionally below.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4706)
#endif

/**
\internal
\see rw::collision::Aggregate::LineIntersectionQuery.
 */
RwpBool
TriangleKDTreeProcedural::LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                                          const Matrix44Affine *tm)
{
    // Map line into spatial map space
    Matrix44Affine invTm(*tm);
    invTm= InverseOfMatrixWithOrthonormal3x3(invTm);
    Vector3 localLineStart = TransformPoint(lineQuery->m_pt1, invTm);
    Vector3 localLineEnd   = TransformPoint(lineQuery->m_pt2, invTm);
    Vector3 localLineDelta = localLineEnd - localLineStart;

    // See whether to start a new query
    KDTree::LineQuery *mapQuery = (KDTree::LineQuery *)lineQuery->m_curSpatialMapQuery;
    if (!mapQuery)
    {
        mapQuery = new (lineQuery->m_spatialMapQueryMem)
            KDTree::LineQuery(m_map, localLineStart, localLineEnd, lineQuery->m_fatness);
        lineQuery->m_curSpatialMapQuery = (void*)mapQuery;
    }

    // Far clip val might have been set by another volume hit
    if (lineQuery->m_resultsSet != VolumeLineQuery::ALLLINEINTERSECTIONS)
    {
        mapQuery->ClipEnd(lineQuery->m_endClipVal);
    }

    uint32_t index = rwcKDTREE_INVALID_INDEX;
    while ( (lineQuery->m_resCount < lineQuery->m_resMax)
         && (lineQuery->m_instVolCount < lineQuery->m_instVolMax)
           && mapQuery->GetNext(index))
    {
        VolumeLineSegIntersectResult *res = &lineQuery->m_resBuffer[lineQuery->m_resCount];
        TriangleKDTreeProcedural::Triangle &tri = m_tris[index];
        Vector3  v0, v1, v2;

        v0 = m_verts[tri.indices[0]];
        v1 = m_verts[tri.indices[1]];
        v2 = m_verts[tri.indices[2]];


        RwpBool intersectsTriangle = false;

        intersectsTriangle = TriangleLineSegIntersect(*res, localLineStart, localLineDelta , v0, v1, v2, lineQuery->m_fatness);

        if (intersectsTriangle)
        {
            // We have a hit
            lineQuery->m_resCount++;

            // Instance triangle volume
            TriangleVolume *triVol = TriangleVolume::Initialize(
                    EA::Physics::MemoryPtr(&lineQuery->m_instVolPool[lineQuery->m_instVolCount++]), v0, v1, v2);

            triVol->SetGroup(tri.id);
            triVol->SetSurface(tri.id);
            triVol->SetFlags(GetTriangleFlags(index));

            //Clip the line to min distance
            if (lineQuery->m_resultsSet != VolumeLineQuery::ALLLINEINTERSECTIONS)
            {
                if(res->lineParam < lineQuery->m_endClipVal )
                {
                    lineQuery->m_endClipVal = res->lineParam;
                    mapQuery->ClipEnd(lineQuery->m_endClipVal);
                }
            }

            res->inputIndex = lineQuery->m_currInput-1;
            res->v = lineQuery->m_inputVols[res->inputIndex];

            //In future the vref should be in a freelist
            res->vRef.volume = triVol;
            res->vRef.tmContents = *tm;
            res->vRef.tm = &res->vRef.tmContents;
            //Map intersect result back into query space
            res->position = TransformPoint(res->position, *tm);

            // let's set up the triangle's normal in the result
            triVol->GetNormal(res->normal);
            res->normal = TransformVector(res->normal, *tm);

            //Set up tag to this triangle
            res->vRef.tag = lineQuery->m_tag;
            uint32_t numTagBits = lineQuery->m_numTagBits;

            UpdateTagWithChildIndex(res->vRef.tag, numTagBits, index);
            res->vRef.numTagBits = static_cast<uint8_t>(numTagBits);
        }
    }

    // Return false if we failed to complete query due to lack of buffer space (will resume later).
    return static_cast<RwpBool>(lineQuery->m_resCount < lineQuery->m_resMax);
}

/**
\internal
\see rw::collision::Aggregate::BBoxOverlapQuery.
 */
RwpBool
TriangleKDTreeProcedural::BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                                     const Matrix44Affine *tm)
{
    KDTree::BBoxQuery *mapQuery = (KDTree::BBoxQuery *)bboxQuery->m_curSpatialMapQuery;

    //See whether to start a new query
    if (!mapQuery)
    {
        AABBox *localBBox;
        AABBox tempbb;

        //map bb into spatial map space
        if (tm)
        {
            Matrix44Affine invTm(*tm);
            invTm= InverseOfMatrixWithOrthonormal3x3(invTm);
            tempbb = bboxQuery->m_aabb.Transform(&invTm);
            localBBox = &tempbb;
        }
        else
        {
            localBBox = &bboxQuery->m_aabb;
        }

        // Initialize query
        mapQuery = new (bboxQuery->m_spatialMapQueryMem) KDTree::BBoxQuery(m_map, *localBBox);
        bboxQuery->m_curSpatialMapQuery = (void *)mapQuery;
    }
//    bboxQuery->m_instVolCount = 0;
//    bboxQuery->m_primNext = 0;

    uint32_t index = rwcKDTREE_INVALID_INDEX;
    while (  (bboxQuery->m_instVolCount < bboxQuery->m_instVolMax)
          && (bboxQuery->m_primNext <  bboxQuery->m_primBufferSize)
           && mapQuery->GetNext(index))
    {
        Vector3 v[3], vtemp[3], *vw;
        TriangleKDTreeProcedural::Triangle &tri = m_tris[index];

        v[0] = m_verts[tri.indices[0]];
        v[1] = m_verts[tri.indices[1]];
        v[2] = m_verts[tri.indices[2]];

        // Get world space vertices and bbox
        if (tm)
        {
            TransformPoints(v, 3, *tm, vtemp);
            vw = vtemp;
        }
        else
        {
            vw = v;
        }

        AABBox bb;
        bb.m_min = Min(vw[0], Min(vw[1], vw[2]));
        bb.m_max = Max(vw[0], Max(vw[1], vw[2]));

        // We could check here whether the leaf overlaps the query box in
        // query/world space, but this doesn't seem to be worth it.

        //We're overlapping this tri so instance into volume buffer
        //and the add to primitive buffer
        TriangleVolume *vol = TriangleVolume::Initialize(
                EA::Physics::MemoryPtr(&bboxQuery->m_instVolPool[bboxQuery->m_instVolCount++]), v[0], v[1], v[2]);
        vol->SetGroup(tri.id);
        vol->SetSurface(tri.id);
        vol->SetFlags(GetTriangleFlags(index));

        //Set up tag to this triangle
        uint32_t tag = bboxQuery->m_tag;
        uint32_t numTagBits = bboxQuery->m_numTagBits;
        UpdateTagWithChildIndex(tag, numTagBits, index);

        bboxQuery->AddPrimitiveRef(vol, tm, bb, tag, static_cast<uint8_t>(numTagBits));
    }

    RwpBool outOfPrimitiveSpace = static_cast<RwpBool>(bboxQuery->m_primNext >= bboxQuery->m_primBufferSize);
    RwpBool outOfInstanceSpace = static_cast<RwpBool>(bboxQuery->m_instVolCount >= bboxQuery->m_instVolMax);
    if (outOfPrimitiveSpace)
    {
        bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE);
    }
    if (outOfInstanceSpace)
    {
        bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFINSTANCEBUFFERSPACE);
    }
    
    // Return false if we failed to complete query due to lack of buffer space (will resume later).
    RwpBool finished = static_cast<RwpBool>(!outOfInstanceSpace && !outOfPrimitiveSpace);
    return finished;
}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

// ***********************************************************************************************************
// External Functions
// ***********************************************************************************************************

/**
\brief Get the resource requirements of the TriangleKDTreeProcedural.

\param numVerts The number of vertices in this TriangleKDTreeProcedural.
\param numTris The number of triangles in this TriangleKDTreeProcedural.
\param numNodes The number of nodes in the indexed KDTree map.
\param bbox Reference to the axis-aligned bounding box.
\return The EA::Physics::SizeAndAlignment
*/
EA::Physics::SizeAndAlignment
TriangleKDTreeProcedural::GetResourceDescriptor(uint32_t numVerts,
                                                uint32_t numTris,
                                                uint32_t numNodes,
                                                const AABBox  &bbox,
                                                const VTable  * /*vTable*/,
                                                uint32_t /*classSize*/)
{
    uint32_t size;

    // Class structure
    size = sizeof(TriangleKDTreeProcedural);

    // m_verts
    size = EA::Physics::SizeAlign<uint32_t>(size, RWMATH_VECTOR3_ALIGNMENT);
    size += numVerts*sizeof(Vector3);

    // m_tris
    size += numTris*sizeof(Triangle);

    // m_flags
    size += ((numTris+7)>>3)*sizeof(uint32_t);

    // Spatialmap
    EA::Physics::SizeAndAlignment kdTreeResDesc = KDTree::GetResourceDescriptor(numNodes, numTris, bbox);
    size = EA::Physics::SizeAlign<uint32_t>(size, kdTreeResDesc.GetAlignment());
    size += kdTreeResDesc.GetSize();

    return EA::Physics::SizeAndAlignment(size, rwcTRIANGLEKDTREEPROCEDURALALIGNMENT);
}


/**
\see rw::collision::Procedural::GetSize.
 */

uint32_t
TriangleKDTreeProcedural::GetSizeThis() const
{
    return TriangleKDTreeProcedural::GetResourceDescriptor(m_numVerts,
                                                             m_numVolumes,
                                                             m_map->GetNumBranchNodes(),
                                                             m_AABB,
                                                             m_vTable).GetSize();
}

/**
==================================================================================================
        TRIANGLE FLAGS CALCULATION
==================================================================================================
*/

/**
\brief
Compute a sphere which inscribes the given triangle.

See http://mathworld.wolfram.com/Incircle.html

\param center output center of sphere
\param p0 triangle vertex
\param p1 triangle vertex
\param p2 triangle vertex
\param perimeter optional output

\return Radius of inscribed sphere.
*/
static float
SphereInsideTriangle(Vector3 &center,
                     const Vector3 &p0, const Vector3 &p1, const Vector3 &p2,
                     float *perimeter = NULL)
{
    rwpmath::VecFloat s0,s1,s2;   // length of sides
    rwpmath::VecFloat area, semi;

    s0 = Magnitude(p1 - p2);
    s1 = Magnitude(p2 - p0);
    s2 = Magnitude(p0 - p1);

    semi = (s0 + s1 + s2) * rwpmath::VecFloat(0.5f);      // semi perimeter
    area = Sqrt(Abs(semi*(semi-s0)*(semi-s1)*(semi-s2)));  // heron's formula for area

    if (semi < rwpmath::VecFloat(MINIMUM_RECIPROCAL))   // perimeter is zero?
    {
        semi = 1.0f;
        center = p0;
    }
    else
    {
        center = p0*s0 + p1*s1 + p2*s2;
        center *= rwpmath::VecFloat(0.5f) / semi;
    }
    if (perimeter)
    {
        *perimeter = s0 + s1 + s2;
    }
    return area / semi;
}

/**
\brief
Get triangle face normal.

\param i Index of triangle.
\param result The triangle face normal.

\return Two times the area of the triangle.  Or zero on failure.
*/
float
TriangleKDTreeProcedural::GetTriangleNormal(uint32_t i, Vector3 &result) const
{
    Vector3 &v0 = m_verts[m_tris[i].indices[0]];
    Vector3 &v1 = m_verts[m_tris[i].indices[1]];
    Vector3 &v2 = m_verts[m_tris[i].indices[2]];
    float len;

    result.SetX(v0.GetY()*(v1.GetZ()-v2.GetZ()) + v1.GetY()*(v2.GetZ()-v0.GetZ()) + v2.GetY()*(v0.GetZ()-v1.GetZ()));
    result.SetY(v0.GetZ()*(v1.GetX()-v2.GetX()) + v1.GetZ()*(v2.GetX()-v0.GetX()) + v2.GetZ()*(v0.GetX()-v1.GetX()));
    result.SetZ(v0.GetX()*(v1.GetY()-v2.GetY()) + v1.GetX()*(v2.GetY()-v0.GetY()) + v2.GetX()*(v0.GetY()-v1.GetY()));
    len = Magnitude(result);
    if (len > MINIMUM_RECIPROCAL)
    {
        result /= rwpmath::VecFloat(len);
        return len;
    }
    return 0.0f;
}

/**
\internal
\param i,j Triangle indices.
\param inorm Face normal of triangle i.
\return true if triangle j is entirely outside any of the edges of i.
*/
RwpBool
TriangleKDTreeProcedural::TriangleIsOutside(uint32_t i, uint32_t j, const Vector3 &inorm) const
{
    uint32_t i0, i1;
    Vector3 exn;
    float d;
    uint32_t *vi, *vj;

    vi = m_tris[i].indices;
    vj = m_tris[j].indices;

    // Test each edge of i against all vertices of j

    for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
    {
        exn = Cross(m_verts[vi[i1]] - m_verts[vi[i0]], inorm);
        d = Dot(exn, m_verts[vi[i0]]);

        if (static_cast<float>(Dot(exn, m_verts[vj[0]])) > d &&
            static_cast<float>(Dot(exn, m_verts[vj[1]])) > d &&
            static_cast<float>(Dot(exn, m_verts[vj[2]])) > d)
        {
            return TRUE;
        }
    }
    return FALSE;
}


/**
\internal

\brief
Test if any edges of the second triangle mate with any of the first, and if so, set
appropriate flags in the first triangle.
It is assumed that all flags are set to 1 by default (edges enabled, face one sided) so
this only changes the flags to 0.

\param i index of triangle of which the flags are to be changed.
\param j index of a potential mate of the other triangle
\param eps tolerance to test the distance squared between two coincident vertices
\param inradius inscribed radius of triangle i, used for testing back to back.
\param inorm face normal of the i triangle
*/
void
TriangleKDTreeProcedural::MateTriangles(uint32_t i, uint32_t j, float eps, float inradius,
                                        const Vector3 &inorm)
{
    uint32_t i0, j0, i1, j1, vi0, vi1, vj0, vj1;
    Vector3 jnorm, nxn, edge;
    float cos_theta, sin_theta, distance, jarea;

    // The following assignments are to avoid a compiler warning about local variables not initialized
    vi0 = 0;
    vj1 = 0;

    jarea = GetTriangleNormal(j, jnorm);

    if (jarea < MINIMUM_RECIPROCAL)
    {
    }
    cos_theta = Dot(inorm, jnorm);

    // Try to match every edge of tri[i] against every edge of tri[j]

    for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
    {
        // Get the vertex indices for the edge endpoints
        vi0 = m_tris[i].indices[i0];
        vi1 = m_tris[i].indices[i1];

        for (j0 = 2, j1 = 0; j1 < 3; j0 = j1++)
        {
            // Get the vertex indices for the other edge endpoints
            vj0 = m_tris[j].indices[j0];
            vj1 = m_tris[j].indices[j1];

            // Test for match

            if ((vi0==vj1 || static_cast<float>(MagnitudeSquared(m_verts[vi0] - m_verts[vj1])) < eps) &&
                (vi1==vj0 || static_cast<float>(MagnitudeSquared(m_verts[vi1] - m_verts[vj0])) < eps))
            {
                nxn = Cross(inorm, jnorm);
                edge = m_verts[vj1] - m_verts[vj0];
                edge= Normalize(edge);
                sin_theta = Dot(edge, nxn);

                // theta is the angle between the two triangle normals
                // sin_theta < 0 implies convex.  cos_theta < 0 implies acute.

                if (sin_theta > 0.0f || (sin_theta > -0.2f && cos_theta > 0.5f))
                {
                    // edge is mostly reflex, disable it
                    SetTriangleFlags(i, 0ul, 2ul<< i0);
                }
                else if (sin_theta < 0.0f && cos_theta < -0.2f)
                {
                    // edge is very convex, mark triangle two-sided
                    SetTriangleFlags(i, 0ul, 1ul);
                }

                // If not back-to-back, next triangle.  If back-to-back, we have to keep looking
                // because maybe the other two edges are also mates.

                if (cos_theta > -0.99f)
                {
                }
            }
        }
    }

    // One last test, its possible that j is back to back with i, even though none of the edges
    // are matched.  If so, mark i as two sided.   Ignore j if area is very small.

    if (cos_theta < -0.99f && (GetTriangleFlags(i) & VOLUMEFLAG_TRIANGLEONESIDED) &&
        jarea > inradius*inradius*0.01f)
    {
        // ensure j is behind i.

        distance = Dot(m_verts[vj1] - m_verts[vi0], inorm);

        if (distance < eps*0.01f && distance > -inradius)
        {
            if (!TriangleIsOutside(i, j, inorm) && !TriangleIsOutside(j, i, jnorm))
            {
                SetTriangleFlags(i, 0, 1);   // mark triangle two-sided
            }
        }
    }
}



/**
\brief
Automatically set the edge and face flags for all triangles.

\param tolerance is used to decided if two vertices are coincident.  The tolerance is scaled by
the radius of the inscribed sphere of the triangle being tested, thus 0.1 means two vertices match if the
distance between them is less than 10% of the inradius.  If you set tolerance to zero, it means that
only vertices that have the same index in the m_verts table will match.
*/
void
TriangleKDTreeProcedural::AutoSetFlags(float tolerance)
{
    uint32_t i, j = rwcKDTREE_INVALID_INDEX;
    Vector3 center, inorm;
    float radius, eps, perimeter;
    AABBox bbox;

    //  For each triangle, perform a query to find its neighbors.

    for (i = 0; i < m_numVolumes; ++i)
    {
        //  Ignore triangles with zero area
        if (GetTriangleNormal(i, inorm) < MINIMUM_RECIPROCAL)
        {
            continue;
        }

        radius = SphereInsideTriangle(center, m_verts[m_tris[i].indices[0]],
               m_verts[m_tris[i].indices[1]], m_verts[m_tris[i].indices[2]], &perimeter);

        //  Ignore very small or very thin triangles
        if (perimeter < MINIMUM_RECIPROCAL || radius < 1e-3f * perimeter)
        {
            continue;
        }

        eps = radius * tolerance;
        eps *= eps;
        radius = Max(radius * 1.2f, perimeter * 0.05f);
        bbox.Set(center - Vector3(radius,radius,radius), center + Vector3(radius,radius,radius));

        SetTriangleFlags(i, 15);   // default all set

        KDTree::BBoxQuery query(m_map, bbox);

        while (query.GetNext(j))
        {
            if (i != j)
            {
                MateTriangles(i, j, eps, radius, inorm);
            }
        }
    }
}


/**
\brief Check validity of TriangleKDTreeProcedural. Only available in debug library.

\return TRUE if object is internally consistent.
 */
RwpBool
TriangleKDTreeProcedural::IsValid() const
{
    RwpBool isValid = TRUE;

    // Check KDTree
    if (!m_map->IsValid())
    {
        return FALSE;
    }

    // Now check triangles
    TriangleValidityCheckNodeData curData;
    curData.bbox = m_map->GetBBox();
    curData.parent = 0;

    KDTree::Traversal<TriangleValidityCheckNodeData> traversal(m_map, curData);

    uint32_t expectedNextLeafEntryIndex = 0;
    const Vector3 *verts = GetVertices();
    const TriangleKDTreeProcedural::Triangle *tris = GetTriangles();

    while (traversal.PopNode(curData))
    {
        if (traversal.CurrentNodeIsBranch())
        {
            KDTree::BranchNode &branch = m_map->m_branchNodes[traversal.GetBranchIndex()];
            TriangleValidityCheckNodeData childData;

            childData.parent = traversal.GetBranchIndex();

            // Push right
            childData.bbox = curData.bbox;
            childData.bbox.m_min.SetComponent((uint16_t)(branch.m_axis), branch.m_extents[1]);
            traversal.PushChildNode(1, childData);

            // Push left
            childData.bbox = curData.bbox;
            childData.bbox.m_max.SetComponent((uint16_t)(branch.m_axis), branch.m_extents[0]);
            traversal.PushChildNode(0, childData);
        }
        else
        {
            uint32_t first, count;
            traversal.GetLeafNodeEntries(first, count);

            if (first != expectedNextLeafEntryIndex)
            {
                EAPHYSICS_MESSAGE("Invalid leaf entry index (referenced from node %d).", curData.parent);
                isValid = FALSE;
            }
            expectedNextLeafEntryIndex = first + count;

            // For all triangles
            for (uint32_t i = first; i < (first + count); i++)
            {
                Vector3  v0, v1, v2;

                v0 = verts[tris[i].indices[0]];
                v1 = verts[tris[i].indices[1]];
                v2 = verts[tris[i].indices[2]];

                AABBox bb(Min(Min(v0, v1), v2),
                          Max(Max(v0, v1), v2));

                // Check that leaf node bbox contains triangle
                if (!curData.bbox.Contains(bb))
                {
                    EAPHYSICS_MESSAGE("Triangle %d outside of leaf bounding box (internal node %d).",
                        i, curData.parent);
                    isValid = FALSE;
                }
            }
        }
    }

    // Should have found all the triangles
    if (expectedNextLeafEntryIndex != m_numVolumes)
    {
        EAPHYSICS_MESSAGE("KDTree only indexed %d out of %d triangles.", expectedNextLeafEntryIndex, m_numVolumes);
        isValid = FALSE;
    }

    return isValid;
}

// ***********************************************************************************************************
// Class Member Functions


} // namespace collision
} // namespace rw
