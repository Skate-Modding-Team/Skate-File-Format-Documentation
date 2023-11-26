// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_TRIANGLE_H
#define PUBLIC_RW_COLLISION_TRIANGLE_H

/*************************************************************************************************************

 File: rwctriangle.hpp

 Purpose: Declaration of the Triangle primitive class

 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volume.h"

/* Include Deprecated API */
#include "rw/collision/deprecated/linetriangle.h"


namespace rw
{
namespace collision
{

class TriangleVolume;

extern Volume::VTable globalTriangleVTable;


/**
\brief The TriangleVolume represents a simple collision shape for a triangle with optional fatness.

The triangle volume is a flat shape with three sides and three corners.  As the other primitive shapes,
the triangle may also have a radius.  The effect of the radius on the triangle is to make it thicker and
have rounded edges and corners.  It is as it you put a sphere around each triangle corner and then wrapped
it in shrink wrap.  The thickness of the triangle is two times the radius.

The triangle is defined by the location of its three corners, and the corner points can be at any location
relative to the center of the volume.  However, for numerical precision reasons, you should not make the
corners unnecessarily far away from the volume origin.

\warning The triangle volume is unique because the volume relative transform is disabled.  The volume
relative transform is not necessary, because you can transform the triangle by simply changing its
corners.  But the real reason is that the memory of the relative transform is hijacked to store the
triangle corners and face normal.

\importlib rwccore
*/
class TriangleVolume : public Volume
{
protected:
    inline TriangleVolume(rwpmath::Vector3::InParam p1,
                          rwpmath::Vector3::InParam p2,
                          rwpmath::Vector3::InParam p3,
                          float r = 0.0f);

public:
    /**

    Gets the resource requirements of the triangle volume.
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor()
    {
        return EA::Physics::SizeAndAlignment(sizeof(Volume), rwcVOLUMEALIGNMENT);
    }

    /**
        \internal
    Gets the resource requirements of the volume.  This method is provided with three parameters so that
    you can use the Creator template.
    \param p1  ignored
    \param p2  ignored
    \param p3  ignored
    \return The EA::Physics::SizeAndAlignment.
    */
    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(rwpmath::Vector3::InParam /*p1*/,
                          rwpmath::Vector3::InParam /*p2*/,
                          rwpmath::Vector3::InParam /*p3*/,
                          float /*radius*/ = 0.0f)
    {
        return GetResourceDescriptor();
    }

    static inline TriangleVolume *
    Initialize(const EA::Physics::MemoryPtr& resource);

    static inline TriangleVolume *
    Initialize(const EA::Physics::MemoryPtr& resource,
               rwpmath::Vector3::InParam p1, rwpmath::Vector3::InParam p2, rwpmath::Vector3::InParam p3, 
               float radius = 0.0f);

    void
    GetNormal(rwpmath::Vector3 &normal, const rwpmath::Matrix44Affine *tm = 0) const;

    void
    GetPoints(rwpmath::Vector3 &p1,
              rwpmath::Vector3 &p2,
              rwpmath::Vector3 &p3,
              const rwpmath::Matrix44Affine *tm = 0) const;

    void
    SetPoints(rwpmath::Vector3::InParam p1,
              rwpmath::Vector3::InParam p2,
              rwpmath::Vector3::InParam p3);

    float GetEdgeCos(uint32_t i) const;

    rwpmath::Vector3 GetEdgeCosVector() const;

    void SetEdgeCos(float ec0, float ec1, float ec2);

    RwpBool
    GetBBox(const rwpmath::Matrix44Affine *tm, RwpBool tight, AABBox &bBox) const;

    rwpmath::Vector3
        GetBBoxDiag() const;

    RwpBool
    CreateGPInstance(GPInstance &instance, const rwpmath::Matrix44Affine *tm) const;

    RwpBool
    LineSegIntersect(rwpmath::Vector3::InParam pt1,
                     rwpmath::Vector3::InParam pt2,
                     const rwpmath::Matrix44Affine *tm,
                     VolumeLineSegIntersectResult &result,
                     const float fatness = 0.0f) const;

    struct ObjectDescriptor
    {
        template <class Archive>
            void Serialize(Archive & /*ar*/, uint32_t /*version*/)
        {}
    };

    static TriangleVolume *
        Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & /*objDesc*/)
    {
        return (Initialize(resource));
    }

    static EA::Physics::SizeAndAlignment
        GetResourceDescriptor(const ObjectDescriptor & /*objDesc*/)
    {
        return (GetResourceDescriptor());
    }

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const
    {
        return ObjectDescriptor();
    }

    void Release() {}

    void ApplyUniformScale(float scale, bool useProcessedFlags = false);
private:
};

// ***********************************************************************************************************
// External functions

RwpBool
TriangleLineSegIntersect(VolumeLineSegIntersectResult &result,
                         rwpmath::Vector3::InParam lineStart,
                         rwpmath::Vector3::InParam lineDelta,
                         rwpmath::Vector3::InParam v0,
                         rwpmath::Vector3::InParam v1,
                         rwpmath::Vector3::InParam v2,
                         const float lineFatness = 0.0f,
                         const float triFatness = 0.0f);

RwpBool
TriangleLineSegIntersectTwoSided(VolumeLineSegIntersectResult &result,
                                 rwpmath::Vector3::InParam lineStart,
                                 rwpmath::Vector3::InParam lineDelta,
                                 rwpmath::Vector3::InParam v0,
                                 rwpmath::Vector3::InParam v1,
                                 rwpmath::Vector3::InParam v2,
                                 const float lineFatness = 0.0f,
                                 const float triFatness = 0.0f);

RwpBool
FatTriangleLineSegIntersect(VolumeLineSegIntersectResult &result,
                            rwpmath::Vector3::InParam lineStart,
                            rwpmath::Vector3::InParam lineDelta,
                            rwpmath::Vector3::InParam v0,
                            rwpmath::Vector3::InParam v1,
                            rwpmath::Vector3::InParam v2,
                            float radius);

// ***********************************************************************************************************
// Inline functions


/**
Gets the triangle's normal.

The normal is a unit vector that is perpendicular to all the triangle edges and is
pointing out of the counter-clockwise face.  In mathematical terms, the normal is the
cross product of (p1 - p0) x (p2 - p0) and scaled to unit length.
\param normal   Reference to the output structure
\param tm optional parent transformation to be applied to the result.  May be NULL.
*/
inline void
TriangleVolume::GetNormal(rwpmath::Vector3 &normal, const rwpmath::Matrix44Affine *tm) const
{
    // If the normal is invalid, then compute the normal using the cross product of the edges
    // and store the normal for future reference so we don't have to compute it again.

    rwpmath::Vector3 tmpNormal = transform.GetW();
    if (m_flags & VOLUMEFLAG_TRIANGLENORMALISDIRTY)
    {
        rwpmath::Vector3 n = rwpmath::Cross(transform.YAxis() - transform.XAxis(),
                                            transform.ZAxis() - transform.XAxis());
        //rwpmath::VecFloat len2 = rwpmath::MagnitudeSquared(transform.WAxis());
        rwpmath::VecFloat len2 = rwpmath::MagnitudeSquared(n);
        EA_ASSERT(len2 > VEC_EPSILON_SQUARED);
        n = rwpmath::Select(rwpmath::CompGreaterThan(len2, VEC_EPSILON_SQUARED), n * rwpmath::InvSqrtFast(len2), n);

        const_cast<TriangleVolume*>(this)->m_flags &= ~VOLUMEFLAG_TRIANGLENORMALISDIRTY;
        transform.SetW(n);
        tmpNormal=n;
    }

    if (tm)
    {
        tmpNormal = TransformVector(tmpNormal, *tm);
    }
    normal = tmpNormal;
}

/*
\toedit
\summary Gets the triangle's vertices
\param p1      Reference to the output structure for the first vertex
\param p2      Reference to the output structure for the second vertex
\param p3      Reference to the output structure for the third vertex
\param tm optional parent transformation to be applied to the result.  May be NULL.
\see TriangleVolume::SetPoints
*/
inline void
TriangleVolume::GetPoints(rwpmath::Vector3 &p1,
            rwpmath::Vector3 &p2,
            rwpmath::Vector3 &p3,
            const rwpmath::Matrix44Affine *tm) const
{
    if (tm)
    {
        p1 = TransformPoint(transform.XAxis(), *tm);
        p2 = TransformPoint(transform.YAxis(), *tm);
        p3 = TransformPoint(transform.ZAxis(), *tm);
    }
    else
    {
        p1 = transform.XAxis();
        p2 = transform.YAxis();
        p3 = transform.ZAxis();
    }
}

/**
\brief Initializes a TriangleVolume at the given memory location.
The radius and vertices of the triangle will be zero.  You can change these after initialization.
\param resource Memory resource

\return a pointer to the new TriangleVolume.
*/
inline TriangleVolume *
TriangleVolume::Initialize(const EA::Physics::MemoryPtr& resource)
{
    rwcASSERTALIGN(resource.memory, rwcVOLUMEALIGNMENT);
    return new (resource.memory) TriangleVolume(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
}

/**
\brief Initializes a TriangleVolume at the given memory location.
\param resource Memory resource
\param p1       First vertex.
\param p2       Second vertex.
\param p3       Third vertex.
\param radius   Triangle Fatness
\return a pointer to the new TriangleVolume.
*/
inline TriangleVolume *
TriangleVolume::Initialize(const EA::Physics::MemoryPtr& resource, rwpmath::Vector3::InParam p1,
                           rwpmath::Vector3::InParam p2,
                           rwpmath::Vector3::InParam p3,
                           float r)
{
    rwcASSERTALIGN(resource.memory, rwcVOLUMEALIGNMENT);
    return new (resource.memory) TriangleVolume(p1, p2, p3, r);
}

/**
\brief Sets the triangle's vertices and recalculates the normal


The triangle is defined by the location of its three corners, and the corner points can be at any
location relative to the center of the volume.  However, for numerical precision reasons, you should
not make the corners unnecessarily far away from the volume origin.
\param p1 the first vertex
\param p2 the second vertex
\param p3 the third vertex in counter clockwise order
\see TriangleVolume::GetPoints
*/
inline void
TriangleVolume::SetPoints(rwpmath::Vector3::InParam p1,
            rwpmath::Vector3::InParam p2,
            rwpmath::Vector3::InParam p3)
{
    transform.XAxis() = p1;
    transform.YAxis() = p2;
    transform.ZAxis() = p3;

    m_flags |= VOLUMEFLAG_TRIANGLENORMALISDIRTY;
}

/**
Get the edgecos value for an edge.

The edgecos is only useful for triangles in a mesh.  The edgecos of the edge indicates the cosine
of the angle between the normals of the two triangles that share this edge in the mesh.
\param i the edge index, e0 is the edge between p0 and p1, e1 is p1 to p2, e2 is p2 to p0.
\return the edgecos of the edge.
\see TriangleVolume::GetEdgeCosVector, TriangleVolume::SetEdgeCos
*/
inline float
TriangleVolume::GetEdgeCos(uint32_t i) const
{
    EA_ASSERT_MSG(i < 3, ("Bad parameter to TriangleVolume::GetEdgeCos"));
    return i==0 ? triangleData.edgeCos0 : i==1 ? triangleData.edgeCos1 : triangleData.edgeCos2;
}

/**
Get the edgecos values for all edges.

The edgecos is only useful for triangles in a mesh.  The edgecos of the edge indicates the cosine
of the angle between the normals of the two triangles that share this edge in the mesh.
\return the edgecos values for the edges.
\see TriangleVolume::GetEdgeCos, TriangleVolume::SetEdgeCos
*/
inline rwpmath::Vector3
TriangleVolume::GetEdgeCosVector() const
{
    return rwpmath::Vector3(triangleData.edgeCos0, triangleData.edgeCos1, triangleData.edgeCos2);
}

/**
Set the edgecos values for the edges.

The edgecos is only useful for triangles in a mesh.  The edgecos of the edge indicates the cosine
of the angle between the normals of the two triangles that share this edge in the mesh.
The edgecos is used for culling edge contacts.  The permissible range for the edge contact normal
is between the two face normals that share the edge.  If the dot product of the edge contact with
the face normal is less than the "edgecos" value of the edge, then the contact is culled.
If you set edgeCos to 1 then all edge contacts are culled.  If you set edgeCos to -1 then no contacts
and culled.  To enable the edgecos you must have set the flag VOLUMEFLAG_TRIANGLEUSEEDGECOS.
Also when the flag VOLUMEFLAG_TRIANGLEEDGE0CONVEX is true, the permissible angle is in front of
the triangle, otherwise it is in back of the triangle.

\param ec0 the edgecos value for edge0 (p0 to p1)
\param ec1 the edgecos value for edge1 (p1 to p2)
\param ec2 the edgecos value for edge2 (p2 to p0)

\see TriangleVolume::GetEdgeCosVector, TriangleVolume::GetEdgeCos
*/
inline void
TriangleVolume::SetEdgeCos(float ec0, float ec1, float ec2)
{
    triangleData.edgeCos0 = ec0;
    triangleData.edgeCos1 = ec1;
    triangleData.edgeCos2 = ec2;
}


/**
\brief    Triangle constructor.
\param p1   First vertex.
\param p2   Second vertex.
\param p3   Third vertex.
\param r    Triangle fatness
*/
inline
TriangleVolume::TriangleVolume(rwpmath::Vector3::InParam p1,
               rwpmath::Vector3::InParam p2,
               rwpmath::Vector3::InParam p3,
               float r)
               : Volume(rw::collision::VOLUMETYPETRIANGLE, r)
{
    m_flags = VOLUMEFLAG_TRIANGLEDEFAULT;
    SetPoints(p1, p2, p3);
    SetEdgeCos(-1.0f,-1.0f,-1.0f);
}


} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_TRIANGLE_H
